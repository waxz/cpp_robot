//
// Created by waxz on 4/28/24.
//

#include "dds_handler_variant.h"
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

        MLOGI("run %i", 0);

        try {

            // create participant

            if (config.participant.name.empty() || config.participant.xml.empty() ||
                config.participant.profile.empty()) {
                std::cout << "config.participant is empty" << std::endl;

                return -1;
            } else {


                participant_ = std::make_shared<DdsSimpleParticipant>();
                int rt = participant_->init(config.participant);
                MLOGI("participant_->init %i", rt);

                if (rt < 0) {
                    MLOGI("cancel create %i\n", rt);

                    return -1;
                }
                MLOGI("participant_->init %i", rt);


                for (auto &r: config.readers) {
                    MLOGI("create reader %s", r.first.c_str());

                    if (reader_builder_functions.end() == reader_builder_functions.find(r.second.topic_type)) {
                        MLOGW("topic type %s not found", r.second.topic_type.c_str());
                    } else {

                        MLOGW("check drop %i", 0);
                        auto reader = reader_builder_functions[r.second.topic_type](
                                &mem_cfg, participant_, r.second
                        );
                        MLOGW("check drop %i", 0);

                        readers_.emplace(r.first, std::move(reader));
                        MLOGW("check drop %i", 0);

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
                        MLOGW("check drop %i", 0);

                        auto writer = writer_builder_functions[r.second.topic_type](
                                participant_, r.second);
                        MLOGW("check drop %i", 0);

                        writers_.emplace(r.first, std::move(writer));
                        MLOGW("check drop %i", 0);


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
            printf("DdsHandlerVariant::create:toml::parse(%s) failed, runtime_error: %s\n", filename, e.what());
            return -1;
        } catch (const toml::syntax_error &e) {
            printf("DdsHandlerVariant::create:toml::parse(%s) failed, syntax_error: %s\n", filename, e.what());
            return -1;
        } catch (...) {
            printf("DdsHandlerVariant::create:toml::parse(%s) failed, error: %s\n", filename, "Unknown");
            return -1;
        }

        // get config
        MLOGI("run %i", 0);

        std::cout << "toml_data:\n" << toml_data << std::endl;

        //
        bool config_is_valid = false;
        MLOGI("run %i", 0);


        try {
            if (toml_data.contains("dds")) {
                config = DdsConfig(toml_data.at("dds"));
                toml::value config_data = config;
                std::cout << "config_data:\n" << config_data << "\n";
                config_is_valid = true;
            } else {
                MLOGW("run %i", 0);

            }
        } catch (const std::exception &e) {
            printf("DdsHandlerVariant::create:toml::parse_config(%s) failed, error: %s\n", filename, e.what());
            return -1;
        } catch (...) {
            printf("DdsHandlerVariant::create:toml::parse_config(%s) failed, error: %s\n", filename, "Unknown");
            return -1;
        }


        MLOGI("run %i", 0);
        std::cout << "create participant ret" << std::endl;
        MLOGI("config_is_valid %i", config_is_valid);

        int create_handler_rt = -1;
        if (config_is_valid) {
            create_handler_rt = create_handler();
        }

        std::cout << "create_handler_rt: " << create_handler_rt << std::endl;
        return create_handler_rt;
    }

    int DdsHandlerVariant::write_data(const char *name, void **buffer, size_t buffer_size) {

        auto it = writers_.find(name);

        if (it == writers_.end()) {
            return -1;
        }
        int rt = 0;

        absl::visit([buffer, buffer_size, &rt](auto &w) { rt = w->write_data(buffer, buffer_size); }, it->second);

        return rt;
    }

    ChannelBuffer_ptr DdsHandlerVariant::read_data(const char *name) {

        ChannelBuffer_ptr ret_buffer = 0;

        auto it = readers_.find(name);

        if (it == readers_.end()) {
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
