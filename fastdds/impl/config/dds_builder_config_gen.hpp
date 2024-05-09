#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include <toml.hpp>


#include <nlohmann/json.hpp>
/// GEN[TOML] GEN[JSON]
struct ParticipantConfig{
     std::string xml;
     std::string profile;
     std::string name;
     std::vector < std::string > interface_allowlist;
     std::vector < std::string > interface_blocklist;
 

    explicit ParticipantConfig(const toml::value& value) {
        xml= toml::get<decltype(xml)>(value.at("xml"));
        profile= toml::get<decltype(profile)>(value.at("profile"));
        name= toml::get<decltype(name)>(value.at("name"));
        interface_allowlist= toml::get<decltype(interface_allowlist)>(value.at("interface_allowlist"));
        interface_blocklist= toml::get<decltype(interface_blocklist)>(value.at("interface_blocklist"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"xml", this->xml},
        {"profile", this->profile},
        {"name", this->name},
        {"interface_allowlist", this->interface_allowlist},
        {"interface_blocklist", this->interface_blocklist}};
    }
   ParticipantConfig() = default;

};

 inline void from_json(const nlohmann::json & value,ParticipantConfig& object ){
    from_json(value["xml"], object.xml);
    from_json(value["profile"], object.profile);
    from_json(value["name"], object.name);
    from_json(value["interface_allowlist"], object.interface_allowlist);
    from_json(value["interface_blocklist"], object.interface_blocklist);

 }


 inline void to_json( nlohmann::json & value,const ParticipantConfig& object ){
    to_json(value["xml"], object.xml);
    to_json(value["profile"], object.profile);
    to_json(value["name"], object.name);
    to_json(value["interface_allowlist"], object.interface_allowlist);
    to_json(value["interface_blocklist"], object.interface_blocklist);

 }
/// GEN[TOML] GEN[JSON]
struct ReaderConfig{
     std::string topic_name;
     std::string topic_type;
     std::string sub_profile;
     std::string topic_profile;
     std::string reader_profile;
     int qos_queue_size = 3;
 

    explicit ReaderConfig(const toml::value& value) {
        topic_name= toml::get<decltype(topic_name)>(value.at("topic_name"));
        topic_type= toml::get<decltype(topic_type)>(value.at("topic_type"));
        sub_profile= toml::get<decltype(sub_profile)>(value.at("sub_profile"));
        topic_profile= toml::get<decltype(topic_profile)>(value.at("topic_profile"));
        reader_profile= toml::get<decltype(reader_profile)>(value.at("reader_profile"));
        qos_queue_size= toml::get<decltype(qos_queue_size)>(value.at("qos_queue_size"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"topic_name", this->topic_name},
        {"topic_type", this->topic_type},
        {"sub_profile", this->sub_profile},
        {"topic_profile", this->topic_profile},
        {"reader_profile", this->reader_profile},
        {"qos_queue_size", this->qos_queue_size}};
    }
   ReaderConfig() = default;

};

 inline void from_json(const nlohmann::json & value,ReaderConfig& object ){
    from_json(value["topic_name"], object.topic_name);
    from_json(value["topic_type"], object.topic_type);
    from_json(value["sub_profile"], object.sub_profile);
    from_json(value["topic_profile"], object.topic_profile);
    from_json(value["reader_profile"], object.reader_profile);
    from_json(value["qos_queue_size"], object.qos_queue_size);

 }


 inline void to_json( nlohmann::json & value,const ReaderConfig& object ){
    to_json(value["topic_name"], object.topic_name);
    to_json(value["topic_type"], object.topic_type);
    to_json(value["sub_profile"], object.sub_profile);
    to_json(value["topic_profile"], object.topic_profile);
    to_json(value["reader_profile"], object.reader_profile);
    to_json(value["qos_queue_size"], object.qos_queue_size);

 }
/// GEN[TOML] GEN[JSON]
struct WriterConfig{
     std::string topic_name;
     std::string topic_type;
     std::string pub_profile;
     std::string topic_profile;
     std::string writer_profile;
     int qos_queue_size = 3;
 

    explicit WriterConfig(const toml::value& value) {
        topic_name= toml::get<decltype(topic_name)>(value.at("topic_name"));
        topic_type= toml::get<decltype(topic_type)>(value.at("topic_type"));
        pub_profile= toml::get<decltype(pub_profile)>(value.at("pub_profile"));
        topic_profile= toml::get<decltype(topic_profile)>(value.at("topic_profile"));
        writer_profile= toml::get<decltype(writer_profile)>(value.at("writer_profile"));
        qos_queue_size= toml::get<decltype(qos_queue_size)>(value.at("qos_queue_size"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"topic_name", this->topic_name},
        {"topic_type", this->topic_type},
        {"pub_profile", this->pub_profile},
        {"topic_profile", this->topic_profile},
        {"writer_profile", this->writer_profile},
        {"qos_queue_size", this->qos_queue_size}};
    }
   WriterConfig() = default;

};

 inline void from_json(const nlohmann::json & value,WriterConfig& object ){
    from_json(value["topic_name"], object.topic_name);
    from_json(value["topic_type"], object.topic_type);
    from_json(value["pub_profile"], object.pub_profile);
    from_json(value["topic_profile"], object.topic_profile);
    from_json(value["writer_profile"], object.writer_profile);
    from_json(value["qos_queue_size"], object.qos_queue_size);

 }


 inline void to_json( nlohmann::json & value,const WriterConfig& object ){
    to_json(value["topic_name"], object.topic_name);
    to_json(value["topic_type"], object.topic_type);
    to_json(value["pub_profile"], object.pub_profile);
    to_json(value["topic_profile"], object.topic_profile);
    to_json(value["writer_profile"], object.writer_profile);
    to_json(value["qos_queue_size"], object.qos_queue_size);

 }
/// GEN[TOML] GEN[JSON]
struct DdsConfig{
     ParticipantConfig participant;
     std::unordered_map < std::string , ReaderConfig > readers;
     std::unordered_map < std::string , WriterConfig > writers;
 

    explicit DdsConfig(const toml::value& value) {
        participant= toml::get<decltype(participant)>(value.at("participant"));
        readers= toml::get<decltype(readers)>(value.at("readers"));
        writers= toml::get<decltype(writers)>(value.at("writers"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"participant", this->participant},
        {"readers", this->readers},
        {"writers", this->writers}};
    }
   DdsConfig() = default;

};

 inline void from_json(const nlohmann::json & value,DdsConfig& object ){
    from_json(value["participant"], object.participant);
    from_json(value["readers"], object.readers);
    from_json(value["writers"], object.writers);

 }


 inline void to_json( nlohmann::json & value,const DdsConfig& object ){
    to_json(value["participant"], object.participant);
    to_json(value["readers"], object.readers);
    to_json(value["writers"], object.writers);

 }
