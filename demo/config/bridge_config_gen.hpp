#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include <toml.hpp>
namespace bridge_helper{
/// GEN[TOML]
struct Channel{
     std::string from;
     std::string from_channel;
     std::string to;
     std::string to_channel;
     float interval = 100.0;
 

    explicit Channel(const toml::value& value) {
        from= toml::get<decltype(from)>(value.at("from"));
        from_channel= toml::get<decltype(from_channel)>(value.at("from_channel"));
        to= toml::get<decltype(to)>(value.at("to"));
        to_channel= toml::get<decltype(to_channel)>(value.at("to_channel"));
        interval= toml::get<decltype(interval)>(value.at("interval"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"from", this->from},
        {"from_channel", this->from_channel},
        {"to", this->to},
        {"to_channel", this->to_channel},
        {"interval", this->interval}};
    }
   Channel() = default;

};
}
namespace bridge_helper{
/// GEN[TOML]
struct BridgeConfig{
     std::unordered_map < std::string , Channel > bridge;
     std::vector < std::array < std::string , 2 > > ros_dds_type;
 

    explicit BridgeConfig(const toml::value& value) {
        bridge= toml::get<decltype(bridge)>(value.at("bridge"));
        ros_dds_type= toml::get<decltype(ros_dds_type)>(value.at("ros_dds_type"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"bridge", this->bridge},
        {"ros_dds_type", this->ros_dds_type}};
    }
   BridgeConfig() = default;

};
}
