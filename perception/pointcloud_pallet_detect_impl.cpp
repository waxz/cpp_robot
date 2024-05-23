//
// Created by waxz on 5/21/24.
//
#include "pointcloud_pallet_detect_impl.h"
#include "math/geometry/normal_estimation_3d.h"

#include <iostream>
#include "common/string_logger.h"
#include "common/clock_time.h"

namespace perception{
    int PalletDetector::create(const char *filename, const ta_cfg_t *cfg) {
        if(!cfg){
            std::cout << "PalletDetector::create: cfg" << cfg << std::endl;

            return -1;
        }
        mem_cfg = *cfg;


        // load toml file

        std::cout << "PalletDetector::create:" << filename << std::endl;
        toml::basic_value<toml::discard_comments, std::unordered_map, std::vector> toml_data;

        try{
            toml_data = toml::parse(filename);
        }catch (std::runtime_error & e){
            MLOGW("PalletDetector::create:toml::parse(%s) failed, error: %s\n",filename, e.what());
            return -1;
        }catch (toml::syntax_error& e){
            MLOGW("PalletDetector::create:toml::parse(%s) failed, error: %s\n",filename, e.what());
            return -1;
        }catch (...){
            MLOGW("PalletDetector::create:toml::parse(%s) failed, error: %s\n",filename, "Unknown");
            return -1;
        }

        try{
            config = perception::DetectorConfig(toml_data.at("pallet_detector"));
            toml::value config_data = config;
            std::cout << "config_data:\n" << config_data << "\n";
        }catch (std::exception& e){
            MLOGW("PalletDetector::create:toml::parse(%s) failed, error: %s\n",filename, e.what());
            return -1;
        }


        std::cout << "[PalletDetector]: configured done" << std::endl;

        ground_init_buffer = (f32_t*)ta_alloc(&mem_cfg,1000*3*4);

        ground_output_buffer = (f32_t*)ta_alloc(&mem_cfg,1000*3*4);

        cloud_label_table = (i8_t*) ta_alloc(&mem_cfg,1000);
        if( !ground_init_buffer || !ground_output_buffer){
            MLOGW("PalletDetector: create buffer fail: %p, %p", ground_init_buffer, ground_output_buffer);
            return -1;
        }
        return 0;
    }

    void PalletDetector::stop() {

    }

    void
    PalletDetector::set_input(f32_t *buffer, u64_t height,
                                       u64_t width, f32_t vx, f32_t vy, f32_t vz) {

        cloud_buffer = buffer;
        config.cloud.dim.height = height;
        config.cloud.dim.width = width;
        viewpoint_x = vx;
        viewpoint_y = vy;
        viewpoint_z = vz;

    }

    void PalletDetector::set_ground_init_dim(u64_t height_min, u64_t height_max, u64_t width_min, u64_t width_max) {
        config.filter_ground.init_ground_height_min = height_min;
        config.filter_ground.init_ground_height_max = height_max;
        config.filter_ground.init_ground_width_min = width_min;
        config.filter_ground.init_ground_width_max = width_max;

    }

    void PalletDetector::set_ground_adaptive_thresh(f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max) {

        MLOGI("set_ground_adaptive_thresh: [%f, %f], [%f, %f], [%f, %f]", x_min, x_max, y_min, y_max, z_min, z_max)
        config.filter_ground.adaptive_x_min  =x_min;
        config.filter_ground.adaptive_x_max = x_max;
        config.filter_ground.adaptive_y_min  =y_min;
        config.filter_ground.adaptive_y_max = y_max;
        config.filter_ground.adaptive_z_min  =z_min;
        config.filter_ground.adaptive_z_max = z_max;
    }
    void PalletDetector::set_ground_init_thresh(f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min,
                                                f32_t z_max, f32_t nz_min) {
        config.filter_ground.init_ground_cx_min = x_min;
        config.filter_ground.init_ground_cx_max = x_max;
        config.filter_ground.init_ground_cy_min = y_min;
        config.filter_ground.init_ground_cy_max = y_max;
        config.filter_ground.init_ground_cz_min = z_min;
        config.filter_ground.init_ground_cz_max = z_max;
        config.filter_ground.init_ground_nz_min = nz_min;

    }

    PointCloudBuffer_ptr PalletDetector::filter_ground(u32_t output_mode) {

        common::Time start_time = common::FromUnixNow();
        // computer center and norm for given initial ground window, within thresh

        int cloud_dim_height = config.cloud.dim.height;
        int cloud_dim_width = config.cloud.dim.width;

        int init_ground_height_min =  config.filter_ground.init_ground_height_min;
        int init_ground_height_max = config.filter_ground.init_ground_height_max > cloud_dim_height ? cloud_dim_height : config.filter_ground.init_ground_height_max;

        int init_ground_width_min =  config.filter_ground.init_ground_width_min;
        int init_ground_width_max = config.filter_ground.init_ground_width_max > cloud_dim_width ? cloud_dim_width : config.filter_ground.init_ground_width_max ;

        float init_ground_nz_min = config.filter_ground.init_ground_nz_min;

        int init_ground_height = init_ground_height_max - init_ground_height_min;
        int init_ground_width = init_ground_width_max - init_ground_width_min;

        float adaptive_z_thresh_min = config.filter_ground.adaptive_z_min;
        float adaptive_z_thresh_max = config.filter_ground.adaptive_z_max;


        MLOGI("init_ground_height  : [%i, %i]", init_ground_height_min ,init_ground_height_max);
        MLOGI("init_ground_width  : [%i, %i]", init_ground_width_min,init_ground_width_max );

        MLOGI("init_ground_dim : [%i, %i}]",init_ground_height, init_ground_width );
        MLOGI("cloud_dim : [%i, %i]",cloud_dim_height, cloud_dim_width );
        if( init_ground_height == 0
        || init_ground_width == 0
        ){
            MLOGI("init_ground_dim contains zero value: [%i, %i}]",init_ground_height, init_ground_width );

            return 0;
        }


        ground_init_buffer = (f32_t*)ta_realloc(&mem_cfg,ground_init_buffer, init_ground_height*init_ground_width*3*4);
        ground_output_buffer = (f32_t*)ta_realloc(&mem_cfg,ground_output_buffer, cloud_dim_height*cloud_dim_width*3*4);

        cloud_label_table = (i8_t*) ta_realloc(&mem_cfg,cloud_label_table, cloud_dim_height*cloud_dim_width);


        if(
                !ground_init_buffer
                || !ground_output_buffer
                || !cloud_label_table

        ){
            MLOGW("PalletDetector: create buffer fail: %p, %p, %p", ground_init_buffer, ground_output_buffer,cloud_label_table);
            return nullptr;
        }

        memset(cloud_label_table,0,cloud_dim_height*cloud_dim_width);


        f32_t init_ground_x_min = config.filter_ground.init_ground_cx_min;
        f32_t init_ground_x_max = config.filter_ground.init_ground_cx_max;
        f32_t init_ground_y_min = config.filter_ground.init_ground_cy_min;
        f32_t init_ground_y_max = config.filter_ground.init_ground_cy_max;
        f32_t init_ground_z_min = config.filter_ground.init_ground_cz_min;
        f32_t init_ground_z_max = config.filter_ground.init_ground_cz_max;

        MLOGI("init_ground_thresh : [%f, %f],[%f, %f],[%f, %f] ",init_ground_x_min, init_ground_x_max,init_ground_y_min, init_ground_y_max,init_ground_z_min, init_ground_z_max );

        int valid_num = 0;
        for( int i = init_ground_height_min; i < init_ground_height_max;i++){

            for(int j = init_ground_width_min ; j < init_ground_width_max;j++ ){

                int k = (i * cloud_dim_width + j)*3;
                f32_t *p = cloud_buffer + k;
                bool valid = p[0] > init_ground_x_min && p[0] < init_ground_x_max
                             && p[1] > init_ground_y_min && p[1] < init_ground_y_max
                             && p[2] > init_ground_z_min && p[2] < init_ground_z_max;

                ground_init_buffer[valid_num*3] = p[0];
                ground_init_buffer[valid_num*3 + 1] = p[1];
                ground_init_buffer[valid_num*3 + 2] = p[2];
                valid_num += valid;
            }
        }
        MLOGW("valid_num: %i ", valid_num );

        if(valid_num < config.filter_ground.init_valid_num_min){
            MLOGW("valid_num: %i < %i", valid_num, config.filter_ground.init_valid_num_min);

            return nullptr;
        }

        f32_t init_ground_x_sum = 0.0;
        f32_t init_ground_y_sum = 0.0;
        f32_t init_ground_z_sum = 0.0;

        f32_t valid_num_inv = 1.0 /f32_t(valid_num);
        for(int i = 0 ; i < valid_num;i++){
            init_ground_x_sum += ground_init_buffer[i*3];
            init_ground_y_sum += ground_init_buffer[i*3 + 1];
            init_ground_z_sum += ground_init_buffer[i*3 + 2];
        }
        f32_t  cx = init_ground_x_sum * valid_num_inv;
        f32_t  cy = init_ground_y_sum * valid_num_inv;
        f32_t  cz = init_ground_z_sum * valid_num_inv;


        MLOGI("find center: [%f, %f, %f]", cx, cy, cz);

        perception::NormalEst3d normalEst3D;
        normalEst3D.setViewerPoint(viewpoint_x,viewpoint_y,viewpoint_z);
        normalEst3D.addCenter(cx,cy,cz);
        for(int i = 0 ; i < valid_num;i++){
            normalEst3D.addPoint(ground_init_buffer[i*3],ground_init_buffer[i*3 + 1],ground_init_buffer[i*3 +2]);
        }

        f32_t cnx, cny, cnz, cnd;
        normalEst3D.compute(cnx, cny, cnz, cnd);
        Eigen::Matrix <float, 3, 1> normal (cnx, cny, cnz);

        MLOGI("find norm: [%f, %f, %f, %f]", cnx, cny, cnz,cnd);


        if (cnz < init_ground_nz_min
        ){
            MLOGI("find norm: [%f, %f, %f, %f] error with nz_limit [%f, %f]", cnx, cny, cnz,cnd,init_ground_nz_min );

        }


        for(int i = 0 ; i < cloud_dim_height;i++){
            for(int j = 0 ;j < cloud_dim_width;j++){
                int k = (i * cloud_dim_width + j)*3;
                f32_t *p = cloud_buffer + k;
                f32_t *p_dst = ground_output_buffer + k;

                Eigen::Matrix <float, 3, 1> vp (p[0] -cx, p[1] - cy, p[2] - cz);
                float cos_theta = vp.dot (normal);
                p_dst[0] = p[0];
                p_dst[1] = p[1];
                p_dst[2] = cos_theta;//std::abs(cos_theta) < 0.05 ? 0.0 : cos_theta;

            }
        }




        if (output_mode == 0){

            ground_cloud_buffer.buffer = ground_init_buffer;
            ground_cloud_buffer.float_num = valid_num*3;
            MLOGI("filter_ground use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));

            return &ground_cloud_buffer;

        }else if (output_mode == 1){

            ground_cloud_buffer.buffer = ground_output_buffer;
            ground_cloud_buffer.float_num = cloud_dim_height*cloud_dim_width*3;
            MLOGI("filter_ground use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));

            return &ground_cloud_buffer;

        }else if(output_mode == 2){
            MLOGI("adaptive_z  : [%f, %f]", adaptive_z_thresh_min ,adaptive_z_thresh_max);


            for(int i = 0 ; i < cloud_dim_height;i++){
                int i_w = i * cloud_dim_width;
                for(int j = 0 ;j < cloud_dim_width;j++){
                    int k = (i_w + j);
                    int l = (i_w + j)*3;
                    f32_t *p = cloud_buffer + l;
                    f32_t *p_dst = ground_output_buffer + l;

                    p_dst[2] = ((p_dst[2]> adaptive_z_thresh_min) && (p_dst[2] < adaptive_z_thresh_max) ) ? p_dst[2] - 0.5: p_dst[2];
//                    p_dst[2] = 0.0;
                }
            }


            ground_cloud_buffer.buffer = ground_output_buffer;
            ground_cloud_buffer.float_num = cloud_dim_height*cloud_dim_width*3;
            MLOGI("filter_ground use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));
            return &ground_cloud_buffer;
        }


        return 0;
    }
}