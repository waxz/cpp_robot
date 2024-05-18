//
// Created by waxz on 4/26/24.
//

#include "config/dds_builder_config_gen.hpp"

#include "dds_handler.hpp"
#include "dds_handler_variant.hpp"

#include "common/functions.h"
#include "common/signals.h"

#include "lyra/lyra.hpp"
#include "common/task.h"

#include "message_center_types.h"
// 10000000 Byte = 10 MB
// 100000000 Byte = 100 MB

#define STATIC_MEMORY_SIZE 100000000
static unsigned char memory_pool[STATIC_MEMORY_SIZE];
static const ta_cfg_t memory_pool_cfg = {
        memory_pool,
        &memory_pool[sizeof(memory_pool)],
        512,
        16,
        8,
};


void ta_print(){
    printf("ta_num_used : %lu\n",ta_num_used(&memory_pool_cfg));
    printf("ta_num_free : %lu\n",ta_num_free(&memory_pool_cfg));
    printf("ta_num_fresh : %lu\n",ta_num_fresh(&memory_pool_cfg));
    printf("ta_check : %i\n",ta_check(&memory_pool_cfg));
}



int main(int argc, char** argv){

    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig){ std::cout << "get sig " << sig;program_run = false;});
    set_signal_handler(my_handler);

    bool get_help = false;
    std::string exe_name;
//    std::string name;

    std::string xml_file;



    // get topic xml
    auto cli
            = lyra::exe_name(exe_name)
              | lyra::help(get_help)
              | lyra::opt(xml_file, "xml_file")["-x"]["--xml_file"]("xml_file")
//                | lyra::opt(name, "name")["-n"]["--name"]("name")


    ;

    auto result = cli.parse({argc, argv});
    if (get_help) {
        std::cout << cli << std::endl;
        return 0;

    }


    if (!result) {
        std::cerr << "Error in command line: " << result.message() << std::endl;
        return 0;
    }



    ta_init(&memory_pool_cfg);


    dds_helper::DdsHandlerVariant dds_handler;
    dds_handler.create("dds_config_send.toml", &memory_pool_cfg);
    std::cout << "destroy DdsHandlerVariant" << std::endl;


    auto cloud = PointCloud2_alloc(480,640,3,&memory_pool_cfg);
    cloud->frame_id;

    void* cloud_buffer[] = {cloud};

    common::TaskManager taskManager(10);


    taskManager.set_loop(100.0,0);

    taskManager.add_task("send", [&dds_handler,&cloud_buffer]{
        dds_handler.write_data("cloud_pub",cloud_buffer,1);
        return true;
        },10.0);

    while (program_run){

        taskManager.run();

    }

    std::cout<< "start Exit" << std::endl;


//    dds_handler.read_data("test");
//
//    dds_handler.write_data("test",0,0);




    ta_print();
    std::cout<< "Exit" << std::endl;

}