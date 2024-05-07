//
// Created by waxz on 9/13/23.
//

#ifndef CMAKE_SUPER_BUILD_ROS_HELPER_H
#define CMAKE_SUPER_BUILD_ROS_HELPER_H

//#include "common/data_holder.h"
//#include "ros_helper_message.h"

#include "message_center_handler.h"


#ifdef __cplusplus
extern "C" {
#endif

// crete
// clean
// is_running
// read_all
// read_channel
// write_all
// write_channel
// write_channel_data





//void ros_create(ros_handler_ptr_t h, const char* filename,const ta_cfg_t* cfg);


//void ros_close(ros_handler_ptr_t h);
//bool ros_is_ok(struct ros_handler_t* h);


//int ros_read(ros_handler_ptr_t h, const char* channel_name);
//int ros_write(ros_handler_ptr_t h, const char* channel_name);
//int ros_write_another_pool(ros_handler_ptr_t h, const char* channel_name,MemPool_ptr pool);

//MemPool_ptr ros_get_pool(ros_handler_ptr_t h, const char* channel_name);




message_handler_t ros_handler_create();

#ifdef __cplusplus
};
#endif

#endif //CMAKE_SUPER_BUILD_ROS_HELPER_H
