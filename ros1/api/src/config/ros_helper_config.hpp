//
// Created by waxz on 4/15/24.
//

#ifndef LIBROSCPP_ROS_HELPER_CONFIG_HPP
#define LIBROSCPP_ROS_HELPER_CONFIG_HPP

#include <vector>
#include <unordered_map>
#include <iostream>

namespace ros_helper{

    /// GEN[TOML] GEN[JSON]
    struct Channel{
        std::string channel_type ;
        std::string  topic_name;
        std::string topic_type;
        int qos_queue_size = 10;

    };

    /// GEN[TOML] GEN[JSON]
    struct Config{
        std::unordered_map<std::string, std::string> config;
        std::unordered_map<std::string, Channel> channel;
    };
}



#endif //LIBROSCPP_ROS_HELPER_CONFIG_HPP
