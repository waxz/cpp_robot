//
// Created by waxz on 5/9/24.
//

// 10000000 Byte = 10 MB
// 50000000 Byte = 50 MB
#include <atomic>


#include "cxxopts/cxxopts.hpp"


#include "common/functions.h"
#include "common/signals.h"
#include "common/task.h"


#include "message_center_handler.h"

#include "dds_helper.h"
#include "config/dds_builder_config_gen.hpp"
#include "common/string_logger.h"

#include "pointcloud_pallet_detect_impl.h"
#include "pointcloud_process.h"

#include "message_center_types.h"

#define STATIC_MEMORY_SIZE 200000000
static unsigned char memory_pool[STATIC_MEMORY_SIZE];
static const ta_cfg_t memory_pool_cfg = {
        memory_pool,
        &memory_pool[sizeof(memory_pool)],
        1024,
        128,
        8,
};


int main(int argc, char** argv){

    ta_init(&memory_pool_cfg);
    cxxopts::Options options("MyProgram", "One line description of MyProgram");
    options.add_options()
            ("h,help", "Print usage")
            ("d,dds_config", "dds  and detector config file", cxxopts::value<std::string>()->default_value("/home/waxz/RustroverProjects/rust_practice/crates/nx_gui/config/gui_config.toml"))
            ;

    std::string dds_toml;
    try{
        auto result = options.parse(argc, argv);

        if (result.count("help"))
        {
            std::cout << options.help() << std::endl;
            return 0;
        }
        dds_toml = result["d"].as<std::string>();

    }catch (std::exception & e){

        std::cout << "parse fail: " << e.what() << "\n";
        std::cout << options.help() << std::endl;

        return 0;
    }

    std::cout << "dds_toml: " << dds_toml << "\n";

    message_handler_t dds_handler = dds_handler_create();
    bool dds_rt = dds_handler.create(&dds_handler, dds_toml.c_str(), &memory_pool_cfg);
    if (!dds_rt) {

        dds_handler.close(&dds_handler);
        return 0;
    }

    perception::PalletDetector palletDetector;
    int detector_ok = palletDetector.create(dds_toml.c_str(),&memory_pool_cfg);

    if(detector_ok < 0){
        dds_handler.close(&dds_handler);
        return 0;

    }


    common::TaskManager taskManager{20};
    taskManager.set_loop(100.0, 0);

    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig) {
        std::cout << "get sig " << sig;
        program_run = false;
    });
    set_signal_handler(my_handler);



    auto& detector_config = palletDetector.config;
    f32_t* raw_cloud_buffer = ( f32_t*)ta_alloc(&memory_pool_cfg, detector_config.cloud.dim.height*detector_config.cloud.dim.width*3*4);
    f32_t* filter_cloud_buffer = ( f32_t*)ta_alloc(&memory_pool_cfg, detector_config.cloud.dim.height*detector_config.cloud.dim.width*3*4);
    f32_t* transform_cloud_buffer = ( f32_t*)ta_alloc(&memory_pool_cfg, detector_config.cloud.dim.height*detector_config.cloud.dim.width*3*4);

    bool mean_window_filter_count_reach = !detector_config.cloud.filter.enable_mean_window;

    u32_t mean_window_filter_count = 0;
    u32_t mean_window_filter_max = detector_config.cloud.filter.mean_window_len;


    bool new_cloud_update = false;

    bool task_start_detect = true;




    if(detector_config.cloud.filter.enable_mean_window){

        taskManager.add_task("recv_cloud",[&palletDetector, &dds_handler,detector_config,&transform_cloud_buffer, &raw_cloud_buffer,&filter_cloud_buffer,&mean_window_filter_count,&mean_window_filter_count_reach,mean_window_filter_max, &new_cloud_update]{

            ChannelBuffer_ptr recv_cloud =  dds_handler.read_data(&dds_handler,"cloud_sub");

            if(recv_cloud && recv_cloud->buffer_size >0){

                for(int i = 0 ; i < recv_cloud->buffer_size; i++){
                    PointCloud2_ptr data = PointCloud2_ptr(recv_cloud->buffer[i]);
                    std::cout << "recv data: " << data->frame_id << " height: " << data->height << ", width: " << data->width << "\n";

                    // reallocate
                    raw_cloud_buffer = ( f32_t*)ta_realloc(&memory_pool_cfg, raw_cloud_buffer,  data->height* data->width*3*4);
                    filter_cloud_buffer = ( f32_t*)ta_realloc(&memory_pool_cfg, filter_cloud_buffer,  data->height* data->width*3*4);
                    transform_cloud_buffer = ( f32_t*)ta_realloc(&memory_pool_cfg, transform_cloud_buffer,  data->height* data->width*3*4);

                    // clip
                    u64_t clip_height = detector_config.cloud.filter.filter_height_max - detector_config.cloud.filter.filter_height_min;
                    u64_t clip_width = detector_config.cloud.filter.filter_width_max - detector_config.cloud.filter.filter_width_min;

                    pointcloud_clip(data->buffer, data->height,data->width,filter_cloud_buffer,detector_config.cloud.filter.filter_height_min,detector_config.cloud.filter.filter_height_max, detector_config.cloud.filter.filter_width_min,detector_config.cloud.filter.filter_width_max );

                    // mean window filter
                    pointcloud_mean_filter(filter_cloud_buffer,  clip_height* clip_width*3,raw_cloud_buffer, &mean_window_filter_count,detector_config.cloud.filter.mean_window_jump_max );


                    mean_window_filter_count_reach =
                            mean_window_filter_count == mean_window_filter_max;

                    if (mean_window_filter_count_reach) {
                        mean_window_filter_count = 0;
                    }

                    if(mean_window_filter_count_reach){
                        pointcloud_transform( raw_cloud_buffer,clip_height* clip_width*3,
                                              transform_cloud_buffer,detector_config.extrinsic.pose.tx, detector_config.extrinsic.pose.ty,detector_config.extrinsic.pose.tz,
                                      detector_config.extrinsic.pose.roll,detector_config.extrinsic.pose.pitch,detector_config.extrinsic.pose.yaw);
                        new_cloud_update = true;

                        palletDetector.set_input(transform_cloud_buffer,clip_height, clip_width, detector_config.extrinsic.pose.tx, detector_config.extrinsic.pose.ty,detector_config.extrinsic.pose.tz );

                    }


                }


            }

            return true;
        },100.0);
    }else{

    }


    taskManager.add_task("detect",[&new_cloud_update,&palletDetector](){

        if(new_cloud_update){
            new_cloud_update = false;

            palletDetector.filter_ground(1);
            palletDetector.filter_vertical(1);
            palletDetector.filter_pallet(10);


            std::cout << "need detect" << "\n";


        }

        return true;
        },100.0);



    while (program_run) {

        taskManager.run();


    }



    dds_handler.close(&dds_handler);

}