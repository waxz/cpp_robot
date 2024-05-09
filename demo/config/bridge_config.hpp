//
// Created by waxz on 5/9/24.
//

#ifndef LIBROSCPP_BRIDGE_CONFIG_HPP
#define LIBROSCPP_BRIDGE_CONFIG_HPP

#include <vector>
#include <unordered_map>

namespace bridge_helper{

/// GEN[TOML]
    struct Channel{

        std::string from;
        std::string from_channel;
        std::string to;
        std::string to_channel;

        float interval = 100.0;

    };

/// GEN[TOML]
    struct BridgeConfig{
        std::unordered_map<std::string,Channel> bridge;
        std::vector<std::array<std::string,2>  > ros_dds_type;
    };
}


#endif //LIBROSCPP_BRIDGE_CONFIG_HPP
