//
// Created by waxz on 10/19/23.
//

#ifndef CMAKE_SUPER_BUILD_ROS_HELPER_TOPIC_H
#define CMAKE_SUPER_BUILD_ROS_HELPER_TOPIC_H


#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "ros_helper_option.h"
//#include "ros_helper_message.h"
#include "message_center_types.h"

// map topic string identifier to actual type


struct ROSTopicReader{
    ros::Subscriber reader;
    bool valid = false;
};

struct ROSTopicWriter{
    ros::Publisher writer;
    bool valid = false;
//    int(*func)(ros::Publisher& pub, MemPool_ptr pool);
    int(*write_data)(ros::Publisher& pub, void** buffer, uint32_t buffer_size);

};

ROSTopicReader create_reader(ros::NodeHandle& nh, reader_option* option);

ROSTopicWriter create_writer(ros::NodeHandle& nh, writer_option* option);

//int ros_write_tf(std::shared_ptr<tf::TransformBroadcaster> tfb, tf::StampedTransform& target, MemPoolHandler_ptr mem_pool );
int ros_write_tf_data(std::shared_ptr<tf::TransformBroadcaster> tfb, tf::StampedTransform& target, void** buffer, uint32_t buffer_size );

PoseStamped* ros_read_tf(std::shared_ptr<tf::TransformListener> tfl, tf::StampedTransform& target);

#endif //CMAKE_SUPER_BUILD_ROS_HELPER_TOPIC_H
