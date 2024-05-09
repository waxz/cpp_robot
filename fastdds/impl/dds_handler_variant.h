//
// Created by waxz on 4/28/24.
//

#ifndef CMAKE_SUPER_BUILD_DDS_HANDLER_VARIANT_H
#define CMAKE_SUPER_BUILD_DDS_HANDLER_VARIANT_H

#include "dds_handler.h"

#include <absl/types/variant.h>


#include "common/variant_builder.h"
#include "config/dds_builder_config_gen.hpp"


namespace dds_helper {


    using DdsReaderVariant = absl::variant<
            std::shared_ptr<DdsSimpleReader<Message::Laserscan1500, Message::Laserscan1500PubSubType>>,
            std::shared_ptr<DdsSimpleReader<Message::Pointcloud1200x800x4, Message::Pointcloud1200x800x4PubSubType>>,
            std::shared_ptr<DdsSimpleReader<Message::Pointcloud1920x1080x3, Message::Pointcloud1920x1080x3PubSubType>>
    >;

    using DdsWriterVariant = absl::variant<
            std::shared_ptr<DdsSimpleWriter<Message::Laserscan1500, Message::Laserscan1500PubSubType>>,
            std::shared_ptr<DdsSimpleWriter<Message::Pointcloud1200x800x4, Message::Pointcloud1200x800x4PubSubType>>,
            std::shared_ptr<DdsSimpleWriter<Message::Pointcloud1920x1080x3, Message::Pointcloud1920x1080x3PubSubType>>
    >;

//    using DdsReaderVariantBuilder = generic::TypeRegistry<DdsReaderVariant, ta_cfg_t* ,const  std::shared_ptr<DdsSimpleParticipant>&, const char*, const char*, const char*, const char*>;

//    using DdsWriterVariantBuilder = generic::TypeRegistry<DdsWriterVariant,const  std::shared_ptr<DdsSimpleParticipant>&, const char*, const char*, const char*, const char*>;

    struct DdsHandlerVariant {

        std::unordered_map<std::string, DdsReaderVariant> readers_;
        std::unordered_map<std::string, DdsWriterVariant> writers_;
        ta_cfg_t mem_cfg = {nullptr, nullptr, 0, 0, 0};


        std::shared_ptr<DdsSimpleParticipant> participant_ = nullptr;

//        DdsReaderVariantBuilder reader_builder_;
//        DdsWriterVariantBuilder writer_builder_;

        std::unordered_map<std::string, std::function<DdsReaderVariant(ta_cfg_t *,
                                                                       const std::shared_ptr<DdsSimpleParticipant> &,
                                                                       const ReaderConfig &)>> reader_builder_functions;
        std::unordered_map<std::string, std::function<DdsWriterVariant(
                const std::shared_ptr<DdsSimpleParticipant> &, const WriterConfig &)>> writer_builder_functions;

        DdsHandlerVariant();

        DdsConfig config;

        int create(const char *filename, const ta_cfg_t *cfg);

        void stop();

        bool is_ok();

        int create_handler();

        void setup();

        ChannelBuffer_ptr read_data(const char *name);

        int write_data(const char *name, void **buffer, size_t buffer_size);

    };
}


#endif //CMAKE_SUPER_BUILD_DDS_HANDLER_VARIANT_H
