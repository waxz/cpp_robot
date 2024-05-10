//
// Created by waxz on 4/28/24.
//

#ifndef CMAKE_SUPER_BUILD_DDS_MESSAGE_CONVERT_H
#define CMAKE_SUPER_BUILD_DDS_MESSAGE_CONVERT_H


#include "Message.h"
#include "MessagePubSubTypes.h"
#include "message_center_types.h"
#include "message_center_handler.h"
#include "memory_pool_handler.hpp"

//#include <absl/container/inlined_vector.h>
//
//#include "tinyalloc/tinyalloc.h"
//
//struct MemPoolHandler {
//
//    static constexpr size_t BUFFER_SIZE = 100;
//    absl::InlinedVector<void *, BUFFER_SIZE> buffer;
//    ta_cfg_t cfg = {nullptr,nullptr,0,0,0};
//    size_t count = 0;
//    size_t max_size = BUFFER_SIZE;
//
//    MemPoolHandler() {
//        buffer.clear();
//    }
//};

namespace dds_helper {
    // to_dds
    void from_dds(const Message::Laserscan1500 &dds_value, MemPoolHandler *mem_pool);


    int to_dds(Message::Laserscan1500 *dds_value, void *common_ptr) ;

// from_dds


    void from_dds(const Message::Pointcloud1200x800x4 &dds_value, MemPoolHandler *mem_pool) ;

    void from_dds(const Message::Pointcloud1920x1080x3 &dds_value, MemPoolHandler *mem_pool);
    int to_dds(Message::Pointcloud1200x800x4 *dds_value, void *common_ptr);

    int to_dds(Message::Pointcloud1920x1080x3 *dds_value, void *common_ptr);

}


#endif //CMAKE_SUPER_BUILD_DDS_MESSAGE_CONVERT_H
