#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include <toml.hpp>
namespace ros_helper{
/// GEN[TOML]
struct Channel{
 //        std::string channel_type ;
     std::string topic_name;
     std::string topic_type;
     int qos_queue_size = 10;
 

    explicit Channel(const toml::value& value) {
        topic_name= toml::get<decltype(topic_name)>(value.at("topic_name"));
        topic_type= toml::get<decltype(topic_type)>(value.at("topic_type"));
        qos_queue_size= toml::get<decltype(qos_queue_size)>(value.at("qos_queue_size"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"topic_name", this->topic_name},
        {"topic_type", this->topic_type},
        {"qos_queue_size", this->qos_queue_size}};
    }
   Channel() = default;

};
}
namespace ros_helper{
/// GEN[TOML]
struct Config{
     std::unordered_map < std::string , std::string > config;
     std::unordered_map < std::string , Channel > readers;
     std::unordered_map < std::string , Channel > writers;
 

    explicit Config(const toml::value& value) {
        config= toml::get<decltype(config)>(value.at("config"));
        readers= toml::get<decltype(readers)>(value.at("readers"));
        writers= toml::get<decltype(writers)>(value.at("writers"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"config", this->config},
        {"readers", this->readers},
        {"writers", this->writers}};
    }
   Config() = default;

};
}
