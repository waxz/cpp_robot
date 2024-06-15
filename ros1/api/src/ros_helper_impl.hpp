//
// Created by waxz on 10/19/23.
//

#ifndef CMAKE_SUPER_BUILD_ROS_HELPER_IMPL_H
#define CMAKE_SUPER_BUILD_ROS_HELPER_IMPL_H

#include "common/functions.h"

//#include "mem_pool_manager.h"

// toml
#include <toml.hpp>
// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>


//absl

// types
#include "absl/types/any.h"
#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "absl/types/span.h"

// string
#include "absl/strings/str_format.h"
#include "absl/strings/escaping.h"
#include "absl/strings/string_view.h"
#include "absl/strings/match.h"

// container
#include "absl/container/fixed_array.h"
#include "absl/container/inlined_vector.h"
#include "absl/container/flat_hash_map.h"

// common
//#include "ros_helper_message.h"
#include "message_center_types.h"

// option
#include "ros_helper_option.hpp"
//#include "ros_helper_message.h"
#include "ros_helper_topic.hpp"
#include "config/ros_helper_config_gen.hpp"

//#include "message_center.h"
#include "message_center_types.h"
#include "message_center_handler.h"

// dynamic function depends on specific message type
typedef ros::Subscriber(*new_subscriber_t)(ros::NodeHandle &, const char*, reader_option ,MemPoolHandler* mem_pool_handler ) ;
typedef ros::Publisher(*new_publisher_t)(ros::NodeHandle &, const char*, writer_option ) ;
typedef void(*publish_msg_t)(ros::Publisher&, MemPoolHandler* mem_pool_handler);




struct ROSTFReader{
    std::shared_ptr<tf::TransformListener> m_tfl;
//    MemPoolManager& mem_pool_manager;
//    MemPoolHandler mem_pool_handler;
//    MemPool pool;
    ChannelBuffer channel_buffer;
    void* buffer[1] = {0};
    PoseStamped pose{};
    tf::StampedTransform target;
    std::string parent_frame;
    std::string child_frame;
    float wait_time_s = 0.0f;

    const char* config_key_frame = "parent_frame";

    const char* m_config_key_topic_type = "topic_type";
    const char* m_config_key_topic_name = "topic_name";
    const char* m_config_key_qos_queue_size = "qos_queue_size";


    explicit ROSTFReader(std::shared_ptr<tf::TransformListener> tfl);
    //
//    int create_from_toml(const toml::basic_value<toml::discard_comments>& config);
    int create(const ros_helper::Channel& channel);

    // receive and store data in pool
//    int process();

    ChannelBuffer_ptr read_data();
};


struct ROSTFWriter{
    std::shared_ptr<tf::TransformBroadcaster> m_tfb;
    MemPoolHandler mem_pool_handler;

    //    MemPool pool;
    tf::StampedTransform target;

//    PoseStampedT sample;
    std::string parent_frame;
    std::string child_frame;
    float wait_time_s = 0.1f;

    const char* m_config_key_topic_type = "topic_type";
    const char* m_config_key_topic_name = "topic_name";
    const char* m_config_key_qos_queue_size = "qos_queue_size";

    explicit ROSTFWriter(std::shared_ptr<tf::TransformBroadcaster> tfb);
    //
//    int create_from_toml(const toml::basic_value<toml::discard_comments>& config);
    int create(const ros_helper::Channel& channel);
    // process data in pool
//    int process();
    int write_data(const void** buffer, u32_t buffer_len);

};

struct ROSReader{
    MemPoolHandler mem_pool_handler;

//    MemPool pool;

    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;
    ROSTopicReader reader;
    std::shared_ptr<ros::CallbackQueue> queue;
    ChannelBuffer channel_buffer;
    reader_option option;

    const char* m_config_key_topic_type = "topic_type";
    const char* m_config_key_topic_name = "topic_name";
    const char* m_config_key_qos_queue_size = "qos_queue_size";


    explicit ROSReader(ta_cfg_t * cfg);
    //
//    int create_from_toml(const toml::basic_value<toml::discard_comments>& config);
    int create(const ros_helper::Channel& channel);

    // process data in pool
//    int process();

    ChannelBuffer_ptr read_data();

};

struct ROSWriter{
    MemPoolHandler mem_pool_handler;

    //    MemPool pool;
    ros::NodeHandle m_nh;
    ros::NodeHandle m_nh_private;

    ROSTopicWriter writer;
    writer_option option;

    const char* m_config_key_topic_type = "topic_type";
    const char* m_config_key_topic_name = "topic_name";
    const char* m_config_key_qos_queue_size = "qos_queue_size";
    const char* m_config_key_qos_keep_last = "qos_keep_last";


    ROSWriter();
    //
//    int create_from_toml(const toml::basic_value<toml::discard_comments>& config);
    int create(const ros_helper::Channel& channel);

    // process data in pool
//    int process();
//    int process(MemPool_ptr another_pool);

    int write_data(const void** buffer, u32_t buffer_len);

};


struct RosHandler{
    using TopicHandlerType = absl::variant< ROSTFWriter,ROSTFReader,ROSWriter,ROSReader >;

    using TopicReaderType = absl::variant< ROSTFReader, ROSReader >;
    using TopicWriterType = absl::variant< ROSTFWriter, ROSWriter  >;


//    absl::flat_hash_map
    std::unordered_map<std::string,TopicHandlerType> channel_holder_map;

    std::unordered_map<std::string,TopicReaderType> channel_reader_map;
    std::unordered_map<std::string,TopicWriterType> channel_writer_map;


    std::shared_ptr<tf::TransformListener> m_tfl;
    std::shared_ptr<tf::TransformBroadcaster> m_tfb;

    ta_cfg_t mem_cfg = {0};



    // toml filed key
    const char* m_config_key_root = "ros";
    const char* m_config_key_config = "config";
    const char* m_config_key_channel = "channel";
    const char* m_config_key_channel_type = "channel_type";
    const char* m_config_key_topic_type = "topic_type";
    const char* m_config_key_topic_name = "topic_name";
    const char* m_config_key_qos_queue_size = "qos_queue_size";
    const char* m_config_key_qos_keep_last = "qos_keep_last";

    const char* m_config_predefine_channel_type_pub = "pub";
    const char* m_config_predefine_channel_type_sub = "sub";
    const char* m_config_predefine_topic_type_tf = "tf";
    const char* m_config_predefine_topic_name_tf = "tf";


    int create(const char* filename,const ta_cfg_t* cfg);

    int stop();
//    MemPool_ptr get_pool(const char* channel_name);
//    int read(const char* channel_name);
//    int write(const char* channel_name);
//    int write(const char* channel_name, MemPool_ptr another_pool);

    int write_data(const char* channel_name, const void** buffer, u32_t buffer_size);
    ChannelBuffer_ptr read_data(const char* channel_name);

};


#endif //CMAKE_SUPER_BUILD_ROS_HELPER_IMPL_H
