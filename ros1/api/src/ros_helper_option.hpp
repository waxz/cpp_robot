//
// Created by waxz on 10/19/23.
//

#ifndef CMAKE_SUPER_BUILD_ROS_HELPER_OPTION_H
#define CMAKE_SUPER_BUILD_ROS_HELPER_OPTION_H


//#include <absl/container/inlined_vector.h>

#include "memory_pool_handler.hpp"


//struct MemPoolHandler {
//
//    static constexpr size_t BUFFER_SIZE = 100;
//    absl::InlinedVector<void *, BUFFER_SIZE> buffer;
//    ta_cfg_t cfg = {0};
//    size_t count = 0;
//    size_t max_size = BUFFER_SIZE;
//
//    MemPoolHandler() {
//        buffer.resize(BUFFER_SIZE, 0);
//        buffer.clear();
//    }
//};


    // qos

    struct reader_option{
    char topic_name[200];
    char topic_type[200];

    int queue_size;
    int wait_time_s;
    MemPoolHandler* mem_pool;

};


    struct writer_option{
    char topic_name[200];
    char topic_type[200];

    int queue_size;
    bool keep_last;

        MemPoolHandler* mem_pool;

};




#endif //CMAKE_SUPER_BUILD_ROS_HELPER_OPTION_H
