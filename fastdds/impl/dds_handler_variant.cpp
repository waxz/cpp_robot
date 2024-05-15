//
// Created by waxz on 4/28/24.
//

#include "dds_handler_variant.hpp"
#include "common/string_logger.h"


#define BIND_DDS(TYPE)      { \
const char* bind_name = (char*)XSTRING(TYPE);                              \
std::function<std::shared_ptr<DdsSimpleReader<TYPE,TYPE##PubSubType>>(ta_cfg_t* ,const  std::shared_ptr<DdsSimpleParticipant>&, const ReaderConfig& )> reader_creator = \
[](ta_cfg_t* cfg,const  std::shared_ptr<DdsSimpleParticipant>& participant, const ReaderConfig& c) \
-> std::shared_ptr<DdsSimpleReader<TYPE,TYPE##PubSubType>> { \
return std::make_shared<DdsSimpleReader<TYPE,TYPE##PubSubType>>(cfg, participant,c);\
};\
reader_builder_functions.emplace(bind_name,reader_creator);\
\
std::function<std::shared_ptr<DdsSimpleWriter<TYPE,TYPE##PubSubType>>(const  std::shared_ptr<DdsSimpleParticipant>&, const WriterConfig& )> writer_creator =\
[](const  std::shared_ptr<DdsSimpleParticipant>& participant,const WriterConfig& c)\
-> std::shared_ptr<DdsSimpleWriter<TYPE,TYPE##PubSubType>>{ \
return std::make_shared<DdsSimpleWriter<TYPE,TYPE##PubSubType>>(participant,c); \
};\
writer_builder_functions.emplace(bind_name,writer_creator);                              \
\
}\

#define mem_bar asm volatile("": : :"memory")
namespace dds_helper {
    DdsHandlerVariant::DdsHandlerVariant() : participant_(nullptr) {

        setup();
        std::cout << "DdsHandlerVariant::setup finished" << std::endl;

    }


    int DdsHandlerVariant::create_handler() {


        try {

            // create participant

            if (config.participant.name.empty() || config.participant.xml.empty() ||
                config.participant.profile.empty()) {
                std::cout << "config.participant is empty" << std::endl;

                return -1;
            } else {


                participant_ = std::make_shared<DdsSimpleParticipant>();
                int rt = participant_->init(config.participant);

                if (rt < 0) {

                    return -1;
                }


                for (auto &r: config.readers) {
                    MLOGI("create reader %s", r.first.c_str());

                    if (reader_builder_functions.end() == reader_builder_functions.find(r.second.topic_type)) {
                        MLOGW("topic type %s not found", r.second.topic_type.c_str());
                    } else {

                        auto reader = reader_builder_functions[r.second.topic_type](
                                &mem_cfg, participant_, r.second
                        );

                        readers_.emplace(r.first, std::move(reader));

                    }
                }

                // create writer
                for (auto &r: config.writers) {
                    MLOGI("create writers %s", r.first.c_str());


                    if (writer_builder_functions.end() == writer_builder_functions.find(r.second.topic_type)) {
                        MLOGW("topic type %s not found", r.second.topic_type.c_str());
                    } else {
#if 0
                        writers_.emplace( std::piecewise_construct , std::forward_as_tuple(r.first) ,
                                          std::forward_as_tuple(writer_builder_functions[r.second.topic_type](
                                                  participant_,r.second))
                        );
#endif

#if 1

                        auto writer = writer_builder_functions[r.second.topic_type](
                                participant_, r.second);

                        writers_.emplace(r.first, std::move(writer));

#endif

                    }


                }


            }


        } catch (const std::runtime_error &e) {
            printf("DdsHandlerVariant::create participant/reader/writer, runtime_error: %s\n", e.what());
            return -1;
        } catch (...) {
            printf("DdsHandlerVariant::create participant/reader/writer, runtime_error: %s\n", "Unknown");
            return -1;
        }
        return 0;
    }

    int DdsHandlerVariant::create(const char *filename, const ta_cfg_t *cfg) {

        mem_cfg = *cfg;

        // load toml file
        std::cout << "DdsHandlerVariant::create:" << filename << std::endl;
        toml::value toml_data;


        try {
            toml_data = toml::parse(filename);
        } catch (const std::runtime_error &e) {
            MLOGW("DdsHandlerVariant::create:toml::parse(%s) failed, runtime_error: %s\n", filename, e.what());
            return -1;
        } catch (const toml::syntax_error &e) {
            MLOGW("DdsHandlerVariant::create:toml::parse(%s) failed, syntax_error: %s\n", filename, e.what());
            return -1;
        } catch (...) {
            MLOGW("DdsHandlerVariant::create:toml::parse(%s) failed, error: %s\n", filename, "Unknown");
            return -1;
        }

        // get config

        std::cout << "toml_data:\n" << toml_data << std::endl;

        //
        bool config_is_valid = false;


        try {
            config = DdsConfig(toml_data.at("dds"));
            toml::value config_data = config;
            std::cout << "config_data:\n" << config_data << "\n";
            config_is_valid = true;
        } catch (const std::exception &e) {
            printf("DdsHandlerVariant::create:toml::parse_config(%s) failed, error: %s\n", filename, e.what());
            return -1;
        } catch (...) {
            printf("DdsHandlerVariant::create:toml::parse_config(%s) failed, error: %s\n", filename, "Unknown");
            return -1;
        }


        std::cout << "create participant ret" << std::endl;

        int create_handler_rt = -1;
        if (config_is_valid) {
            create_handler_rt = create_handler();
        }

        return create_handler_rt;
    }

    int DdsHandlerVariant::write_data(const char *name, void **buffer, size_t buffer_size) {
        if(buffer == nullptr || buffer_size == 0){
            MLOGW("channel buffer_size [%zu] wrong", buffer_size);

            return 0;
        }
        auto it = writers_.find(name);

        if (it == writers_.end()) {
            MLOGW("channel [%s] not found", name);

            return -1;
        }
        int rt = 0;

        absl::visit([buffer, buffer_size, &rt](auto &w) { rt = w->write_data(buffer, buffer_size); }, it->second);

        return rt;
    }

    ChannelBuffer_ptr DdsHandlerVariant::read_data(const char *name) {

        ChannelBuffer_ptr ret_buffer = nullptr;

        auto it = readers_.find(name);

        if (it == readers_.end()) {
            MLOGW("channel [%s] not found", name);
            return ret_buffer;
        }

        absl::visit([&ret_buffer](auto &r) {
            ret_buffer = r->read_data();
        }, it->second);

        return ret_buffer;
    }

    void DdsHandlerVariant::stop() {

    }

    bool DdsHandlerVariant::is_ok() {
        return true;
    }

    void DdsHandlerVariant::setup() {


        BIND_DDS(Message::Laserscan1500)
        BIND_DDS(Message::Pointcloud1920x1080x3)
        BIND_DDS(Message::Pointcloud1200x800x4)

    }


}
