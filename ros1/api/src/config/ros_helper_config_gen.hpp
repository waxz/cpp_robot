#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include <toml.hpp>
#include <nlohmann/json.hpp>
namespace ros_helper{
/// GEN[TOML] GEN[JSON]
struct Channel{
     std::string channel_type;
     std::string topic_name;
     std::string topic_type;
     int qos_queue_size = 10;
 

    explicit Channel(const toml::value& value) {
        channel_type= toml::get<decltype(channel_type)>(value.at("channel_type"));
        topic_name= toml::get<decltype(topic_name)>(value.at("topic_name"));
        topic_type= toml::get<decltype(topic_type)>(value.at("topic_type"));
        qos_queue_size= toml::get<decltype(qos_queue_size)>(value.at("qos_queue_size"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"channel_type", this->channel_type},
        {"topic_name", this->topic_name},
        {"topic_type", this->topic_type},
        {"qos_queue_size", this->qos_queue_size}};
    }
   Channel() = default;

};

 void from_json(const nlohmann::json & value,Channel& object ){
    from_json(value["channel_type"], object.channel_type);
    from_json(value["topic_name"], object.topic_name);
    from_json(value["topic_type"], object.topic_type);
    from_json(value["qos_queue_size"], object.qos_queue_size);

 }


 void to_json( nlohmann::json & value,const Channel& object ){
    to_json(value["channel_type"], object.channel_type);
    to_json(value["topic_name"], object.topic_name);
    to_json(value["topic_type"], object.topic_type);
    to_json(value["qos_queue_size"], object.qos_queue_size);

 }
}
namespace ros_helper{
/// GEN[TOML] GEN[JSON]
struct Config{
     std::unordered_map < std::string , std::string > config;
     std::unordered_map < std::string , Channel > channel;
 

    explicit Config(const toml::value& value) {
        config= toml::get<decltype(config)>(value.at("config"));
        channel= toml::get<decltype(channel)>(value.at("channel"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"config", this->config},
        {"channel", this->channel}};
    }
   Config() = default;

};

 void from_json(const nlohmann::json & value,Config& object ){
    from_json(value["config"], object.config);
    from_json(value["channel"], object.channel);

 }


 void to_json( nlohmann::json & value,const Config& object ){
    to_json(value["config"], object.config);
    to_json(value["channel"], object.channel);

 }
}
