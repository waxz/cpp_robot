//
// Created by waxz on 4/26/24.
//

#ifndef CMAKE_SUPER_BUILD_MESSAGE_CENTER_HANDLER_H
#define CMAKE_SUPER_BUILD_MESSAGE_CENTER_HANDLER_H

#include "common/c_style.h"
#include "tinyalloc/tinyalloc.h"


#ifdef __cplusplus
extern "C" {
#endif



typedef struct ChannelBuffer {
    void** buffer;
    size_t buffer_size;
} ChannelBuffer,* ChannelBuffer_ptr;



typedef struct message_handler_t{

    void* handler;

    /// crete ros node and subscriber and publisher from toml file
    /// \param h
    /// \param filename
    bool(*create)(struct message_handler_t* h, const char* filename, const ta_cfg_t* cfg);
    /// close all resource
    /// \param h
    void(*close)(struct message_handler_t* h);

//    void(*set_mem_pool)(struct ros_handler_t* h, const ta_cfg_t* cfg);
    /// get ros::ok
    /// \param h
    /// \return
    bool(*is_ok)(struct message_handler_t* h);

    /// read data from channel
    /// \param h
    /// \param channel_name
    /// \return
    int(*read)(struct message_handler_t* h, const char* channel_name);

    /// write data to channel
    /// \param h
    /// \param channel_name
    /// \return
    int(*write)(struct message_handler_t* h, const char* channel_name);
//    int(*write_another_pool)(struct ros_handler_t* h, const char* channel_name, MemPool_ptr pool);

    int(*write_data)(struct message_handler_t* h, const char* channel_name, void** buffer, u32_t buffer_size);

    ChannelBuffer_ptr (*read_data)(struct message_handler_t* h, const char* channel_name);


    /// get memory pool
    /// \param h
    /// \param channel_name
    /// \return
//    MemPool_ptr(*pool)(struct ros_handler_t* h, const char* channel_name);


}message_handler_t, *message_handler_ptr_t;



#ifdef __cplusplus
}
#endif


#endif //CMAKE_SUPER_BUILD_MESSAGE_CENTER_HANDLER_H
