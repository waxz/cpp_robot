//
// Created by waxz on 4/18/24.
//

#ifndef CMAKE_SUPER_BUILD_DDS_BUILDER_CONFIG_HPP
#define CMAKE_SUPER_BUILD_DDS_BUILDER_CONFIG_HPP

#include <vector>
#include <string>
#include <unordered_map>

namespace dds_helper{
    /// GEN[TOML] GEN[JSON]
    struct ParticipantConfig{
        std::string xml;
        std::string profile;
        std::string name;
        std::vector<std::string> interface_allowlist;
        std::vector<std::string> interface_blocklist;

    };

/// GEN[TOML] GEN[JSON]
    struct ReaderConfig{

        std::string topic_name;
        std::string topic_type;
        std::string sub_profile;
        std::string topic_profile;
        std::string reader_profile;
        int qos_queue_size = 3;
    };


/// GEN[TOML] GEN[JSON]
    struct WriterConfig{

        std::string topic_name;
        std::string topic_type;
        std::string pub_profile;
        std::string topic_profile;
        std::string writer_profile;
        int qos_queue_size = 3;

    };
/// GEN[TOML] GEN[JSON]
    struct DdsConfig{
        ParticipantConfig participant;
        std::unordered_map<std::string, ReaderConfig> readers;
        std::unordered_map<std::string, WriterConfig> writers;

    };

}



#endif //CMAKE_SUPER_BUILD_DDS_BUILDER_CONFIG_HPP
