//
// Created by waxz on 5/9/24.
//

// 10000000 Byte = 10 MB
// 50000000 Byte = 50 MB
#include <atomic>

#include "lyra/lyra.hpp"

#include "common/functions.h"
#include "common/signals.h"
#include "common/task.h"


#include "message_center_handler.h"

#include "ros_helper.h"
#include "dds_helper.h"
#include "config/bridge_config_gen.hpp"
#include "config/ros_helper_config_gen.hpp"
#include "config/dds_builder_config_gen.hpp"

#define STATIC_MEMORY_SIZE 50000000
static unsigned char memory_pool[STATIC_MEMORY_SIZE];
static const ta_cfg_t memory_pool_cfg = {
        .base = memory_pool,
        .limit = &memory_pool[sizeof(memory_pool)],
        .max_blocks = 512,
        .split_thresh = 16,
        .alignment = 8,
};

int main(int argc, char **argv) {


    bool get_help = false;
    std::string exe_name;
    std::string dds_toml;
    std::string ros_toml;
    std::string bridge_toml;

    float frequency = 100.0;

    // get topic xml
    auto cli
            = lyra::exe_name(exe_name)
              | lyra::help(get_help)
              | lyra::opt(dds_toml, "dds_toml")["-d"]["--dds_toml"]("dds_toml")
              | lyra::opt(ros_toml, "ros_toml")["-r"]["--ros_toml"]("ros_toml")
              | lyra::opt(bridge_toml, "bridge_toml")["-b"]["--bridge_toml"]("bridge_toml")

              | lyra::opt(frequency, "frequency")["-f"]["--frequency"]("frequency");
    auto result = cli.parse({argc, argv});
    if (get_help) {
        std::cout << cli << std::endl;
        return 0;

    }


    if (!result) {
        std::cerr << "Error in command line: " << result.message() << std::endl;
        return 0;
    }
    if (dds_toml.empty()
        || ros_toml.empty()
        || bridge_toml.empty()
            ) {
        std::cout << cli << std::endl;
        return 0;

    }

    toml::value bridge_config_toml;
    toml::value ros_config_toml;
    toml::value dds_config_toml;

    try {

        bridge_config_toml = toml::parse(bridge_toml);

        ros_config_toml = toml::parse(ros_toml);
        dds_config_toml = toml::parse(dds_toml);

    } catch (std::exception &e) {
        std::cout << "parse error: " << e.what() << std::endl;
        return 0;
    } catch (...) {
        std::cout << "parse error: unknown " << std::endl;
        return 0;
    }

    bridge_helper::BridgeConfig bridgeConfig;


    ros_helper::Config rosConfig;
    dds_helper::DdsConfig ddsConfig;


    try {
        std::cout << "deserialize bridgeConfig" << std::endl;
        bridgeConfig = bridge_helper::BridgeConfig(bridge_config_toml.at("bridge"));
        std::cout << "deserialize bridgeConfig done" << std::endl;

        std::cout << "deserialize rosConfig" << std::endl;
        rosConfig = ros_helper::Config(ros_config_toml.at("ros"));
        std::cout << "deserialize rosConfig done" << std::endl;

        std::cout << "deserialize ddsConfig" << std::endl;
        ddsConfig = dds_helper::DdsConfig(dds_config_toml.at("dds"));
        std::cout << "deserialize ddsConfig done" << std::endl;

    } catch (std::exception &e) {
        std::cout << "deserialize error: " << e.what() << std::endl;
        return 0;
    } catch (...) {
        std::cout << "deserialize error: unknown " << std::endl;
        return 0;
    }


    ta_init(&memory_pool_cfg);

    // ros
    message_handler_t ros_handler = ros_handler_create();

    bool ros_rt = ros_handler.create(&ros_handler, ros_toml.c_str(), &memory_pool_cfg);
    if (!ros_rt) {
        ros_handler.close(&ros_handler);
        return 0;
    }

    message_handler_t dds_handler = dds_handler_create();
    bool dds_rt = dds_handler.create(&dds_handler, dds_toml.c_str(), &memory_pool_cfg);
    if (!dds_rt) {
        ros_handler.close(&ros_handler);

        dds_handler.close(&dds_handler);
        return 0;
    }


    common::TaskManager taskManager{20};
    taskManager.set_loop(100.0, 0);


    for (auto &c: bridgeConfig.bridge) {
        const auto &name = c.first;

        const auto &from = c.second.from;
        const auto &from_channel = c.second.from_channel;

        const auto &to = c.second.to;
        const auto &to_channel = c.second.to_channel;

        auto interval = c.second.interval;

        if (((
                     ("ros" == from) || ("dds" == from)
             )
             && (
                     ("ros" == to) || ("dds" == to))
            )
            && (from != to)
                ) {

            if ("ros" == from) {

                // check channel
                auto it1 = rosConfig.readers.find(from_channel);
                if(it1 == rosConfig.readers.end()){
                    continue;
                }
                auto it2 = ddsConfig.writers.find(to_channel);
                if(it2 == ddsConfig.writers.end()){
                    continue;
                }

                auto type_it = std::find(bridgeConfig.ros_dds_type.begin(), bridgeConfig.ros_dds_type.end(), std::array<std::string,2>{it1->second.topic_type,it2->second.topic_type});

                // check type
                if(  type_it != bridgeConfig.ros_dds_type.end()
                ){
                    std::cout << "create task: " << name << std::endl;

                    const char* ros_channel = from_channel.c_str();
                    const char* dds_channel = to_channel.c_str();

                    taskManager.add_task(name.c_str(),[&ros_handler,&dds_handler,ros_channel,dds_channel]{

                        auto recv = ros_handler.read_data(&ros_handler, ros_channel);

                        if(recv && recv->buffer_size>0){
                            std::cout << "recv data send to dds " << std::endl;
                            dds_handler.write_data(&dds_handler,dds_channel, recv->buffer,recv->buffer_size );
                        }

                        {

                        }

                        return true;
                    }, interval);

                }





            } else {
                auto it1 = rosConfig.writers.find(from_channel);
                if(it1 == rosConfig.writers.end()){
                    continue;
                }
                auto it2 = ddsConfig.readers.find(to_channel);
                if(it2 == ddsConfig.readers.end()){
                    continue;
                }

                auto type_it = std::find(bridgeConfig.ros_dds_type.begin(), bridgeConfig.ros_dds_type.end(), std::array<std::string,2>{it1->second.topic_type,it2->second.topic_type});

                // check type
                if(  type_it != bridgeConfig.ros_dds_type.end()
                        ){
                    std::cout << "create task: " << name << std::endl;


                    const char* ros_channel =  to_channel.c_str();
                    const char* dds_channel = from_channel.c_str();

                    taskManager.add_task(name.c_str(),[&ros_handler,&dds_handler,ros_channel,dds_channel]{

                        auto recv =  dds_handler.read_data(&dds_handler, dds_channel);

                        if(recv && recv->buffer_size>0){
                            std::cout << "recv data send to ros " << std::endl;

                            ros_handler.write_data(&ros_handler,ros_channel, recv->buffer,recv->buffer_size );
                        }

                        return true;
                    }, interval);

                }
            }
        }




    }




    std::atomic_bool program_run(true);
    auto my_handler = common::fnptr<void(int)>([&](int sig) {
        std::cout << "get sig " << sig;
        program_run = false;
    });
    set_signal_handler(my_handler);
    while (program_run) {
        taskManager.run();
    }


    ros_handler.close(&ros_handler);
    dds_handler.close(&dds_handler);

}