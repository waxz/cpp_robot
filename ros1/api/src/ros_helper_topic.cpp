//
// Created by waxz on 10/19/23.
//

#include "ros_helper_topic.hpp"
#include <string>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>


#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt8MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <rospy_tutorials/HeaderString.h>
#include <string.h>

//#include "ros_helper_message.h"
#include "message_center_types.h"
#include "common/clock_time.h"
#include "common/string_logger.h"
//LaserScan
int from_common_LaserScan(ros::Publisher& publisher,void** buffer, uint32_t buffer_size);

void to_common_LaserScan(sensor_msgs::LaserScanConstPtr msg, reader_option* option);

ROSTopicWriter create_writer_LaserScan(ros::NodeHandle& nh, writer_option* option);

ROSTopicReader create_reader_LaserScan(ros::NodeHandle& nh, reader_option* option);

//Twist
int from_common_Twist(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_Twist(ros::NodeHandle& nh, writer_option* option);

void to_common_Twist(geometry_msgs::TwistConstPtr msg, reader_option* option);

ROSTopicReader create_reader_Twist(ros::NodeHandle& nh, reader_option* option);

//UInt8MultiArray
int from_common_UInt8MultiArray(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_UInt8MultiArray(ros::NodeHandle& nh, writer_option* option);

void to_common_UInt8MultiArray(std_msgs::UInt8MultiArrayConstPtr msg, reader_option* option);

ROSTopicReader create_reader_UInt8MultiArray(ros::NodeHandle& nh, reader_option* option);

//UInt16MultiArray
int from_common_UInt16MultiArray(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_UInt16MultiArray(ros::NodeHandle& nh, writer_option* option);

void to_common_UInt16MultiArray(std_msgs::UInt16MultiArrayConstPtr msg, reader_option* option);

ROSTopicReader create_reader_UInt16MultiArray(ros::NodeHandle& nh, reader_option* option);

//OccupancyGrid
int from_common_OccupancyGrid(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_OccupancyGrid(ros::NodeHandle& nh, writer_option* option);

void to_common_OccupancyGrid(nav_msgs::OccupancyGridConstPtr msg, reader_option* option);

ROSTopicReader create_reader_OccupancyGrid(ros::NodeHandle& nh, reader_option* option);

// Odometry
int from_common_Odometry(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_Odometry(ros::NodeHandle& nh, writer_option* option);

void to_common_Odometry(nav_msgs::OdometryConstPtr msg, reader_option* option);

ROSTopicReader create_reader_Odometry(ros::NodeHandle& nh, reader_option* option);

// Path
int from_common_Path(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_Path(ros::NodeHandle& nh, writer_option* option);

void to_common_Path(nav_msgs::PathConstPtr msg, reader_option* option);

ROSTopicReader create_reader_Path(ros::NodeHandle& nh, reader_option* option);

// Pose
int from_common_PoseStamped(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_PoseStamped(ros::NodeHandle& nh, writer_option* option);

void to_common_PoseStamped(geometry_msgs::PoseStampedConstPtr msg,reader_option* option);

ROSTopicReader create_reader_PoseStamped(ros::NodeHandle& nh, reader_option* option);
// HeaderString
int from_common_HeaderString(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_HeaderString(ros::NodeHandle& nh, writer_option* option);

void to_common_HeaderString(rospy_tutorials::HeaderStringConstPtr msg, reader_option* option);

ROSTopicReader create_reader_HeaderString(ros::NodeHandle& nh, reader_option* option);

// PointCloud2
int from_common_PointCloud2(ros::Publisher& publisher, void** buffer, uint32_t buffer_size);

ROSTopicWriter create_writer_PointCloud2(ros::NodeHandle& nh, writer_option* option);

void to_common_PointCloud2(sensor_msgs::PointCloud2ConstPtr msg, reader_option* option);

ROSTopicReader create_reader_PointCloud2(ros::NodeHandle& nh, reader_option* option);


// LaserScan

int from_common_LaserScan(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){

    static thread_local  sensor_msgs::LaserScan msg;

    msg.header.stamp = ros::Time::now();

    for(int i = 0; i < buffer_size ;i++){
        LaserScan* scan_ptr =  (LaserScan* )buffer[i] ;

        msg.header.frame_id.assign(scan_ptr->frame_id);
        msg.range_min = scan_ptr->range_min;
        msg.range_max = scan_ptr->range_max;

        msg.angle_min = scan_ptr->angle_min;
        msg.angle_max = scan_ptr->angle_max;
        msg.angle_increment = scan_ptr->angle_increment;

        msg.ranges.resize(scan_ptr->ranges_size);
        msg.intensities.resize(scan_ptr->ranges_size);

        memcpy( msg.ranges.data(),LaserScan_get_ranges(scan_ptr), scan_ptr->ranges_size*sizeof (float));
        memcpy( msg.intensities.data(),LaserScan_get_intensities(scan_ptr), scan_ptr->ranges_size*sizeof (float));

        publisher.publish(msg);
    }


    return 0;
}
void to_common_LaserScan(sensor_msgs::LaserScanConstPtr msg, reader_option* option){
    size_t ranges_size = msg->ranges.size();


    LaserScan* ptr_target = nullptr;
    if (option->mem_pool->count >= option->mem_pool->buffer.size() ){
        ptr_target = LaserScan_alloc( ranges_size,& option->mem_pool->cfg );
        if(ptr_target)
        option->mem_pool->buffer.push_back(ptr_target);
    }else{
        ptr_target = (LaserScan*)option->mem_pool->buffer[option->mem_pool->count];
        ptr_target = LaserScan_realloc( ranges_size,ptr_target,& option->mem_pool->cfg );

    }
    if (!ptr_target){
        std::cout << __FUNCTION__  << "fail allocate" << std::endl;
        return;
    }

    option->mem_pool->count+=1;

//    LaserScanT scan = LaserScanT_create();
//    LaserScanT_set_buffer(&scan, ranges_size);

//    LaserScan* scan_ptr =  (LaserScan*)MemPool_get(pool,&scan);
    // frame_id
    strncpy(ptr_target->frame_id,msg->header.frame_id.c_str(),MSG_STRUCT_MAX_FRAME_ID_LEN);
    ptr_target->stamp = msg->header.stamp.toNSec();

    // basic info
    ptr_target->range_min = msg->range_min;
    ptr_target->range_max = msg->range_max;
    ptr_target->angle_min = msg->angle_min;
    ptr_target->angle_max = msg->angle_max;
    ptr_target->angle_increment = msg->angle_increment;


    // ranges

    memcpy(LaserScan_get_ranges(ptr_target), msg->ranges.data(),ptr_target->ranges_size*sizeof (float));
    if(!msg->intensities.empty()){
        memcpy(LaserScan_get_intensities(ptr_target), msg->intensities.data(),ptr_target->ranges_size*sizeof (float));
    }

}

ROSTopicWriter create_writer_LaserScan(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<sensor_msgs::LaserScan>(option->topic_name, option->queue_size,option->keep_last);
     target.write_data = from_common_LaserScan;

    return target;
}

ROSTopicReader create_reader_LaserScan(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<sensor_msgs::LaserScan>(option->topic_name, option->queue_size, [option](sensor_msgs::LaserScanConstPtr msg){

        to_common_LaserScan(msg, option);
    });

    return target;
}

// Twist
int from_common_Twist(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){

    static thread_local  geometry_msgs::Twist msg;

    for(int i = 0; i < buffer_size;i++){
        Twist* ptr =  (Twist* )buffer[i] ;

        msg.angular.x = ptr->angular.x;
        msg.angular.y = ptr->angular.y;
        msg.angular.z = ptr->angular.z;

        msg.linear.x = ptr->linear.x;
        msg.linear.y = ptr->linear.y;
        msg.linear.z = ptr->linear.z;

        publisher.publish(msg);
    }
    return 0;
}

ROSTopicWriter create_writer_Twist(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<geometry_msgs::Twist>(option->topic_name, option->queue_size,option->keep_last);
    target.write_data = from_common_Twist;

    return target;
}

void to_common_Twist(geometry_msgs::TwistConstPtr msg, reader_option* option){



    Twist* ptr_target = nullptr;
    if (option->mem_pool->count >= option->mem_pool->buffer.size() ){
        ptr_target = Twist_alloc( & option->mem_pool->cfg );
        if(ptr_target)
        option->mem_pool->buffer.push_back(ptr_target);
    }else{
        ptr_target = (Twist*)option->mem_pool->buffer[option->mem_pool->count];
        ptr_target = Twist_realloc( ptr_target,& option->mem_pool->cfg );

    }
    if (!ptr_target){
        std::cout << __FUNCTION__  << "fail allocate" << std::endl;
        return;
    }

    option->mem_pool->count+=1;

    ptr_target->angular.x =  msg->angular.x;
    ptr_target->angular.y =  msg->angular.y;
    ptr_target->angular.z =  msg->angular.z;

    ptr_target->linear.x =  msg->linear.x;
    ptr_target->linear.y =  msg->linear.y;
    ptr_target->linear.z =  msg->linear.z;

}

ROSTopicReader create_reader_Twist(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<geometry_msgs::Twist>(option->topic_name, option->queue_size, [option](geometry_msgs::TwistConstPtr msg){

        to_common_Twist(msg, option);
    });

    return target;
}


// Odometry


int from_common_Odometry(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){
    static thread_local  nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();

    for(int i = 0; i < buffer_size;i++){
        Odometry * ptr =  (Odometry * )buffer[i] ;

        msg.header.frame_id.assign(ptr->frame_id);
        msg.child_frame_id.assign(ptr->child_frame_id);


        msg.pose.pose.position.x = ptr->pose.position.x;
        msg.pose.pose.position.y = ptr->pose.position.y;
        msg.pose.pose.position.z = ptr->pose.position.z;

        msg.pose.pose.orientation.w = ptr->pose.quaternion.w;
        msg.pose.pose.orientation.x = ptr->pose.quaternion.x;
        msg.pose.pose.orientation.y = ptr->pose.quaternion.y;
        msg.pose.pose.orientation.z = ptr->pose.quaternion.z;

        std::memcpy(msg.pose.covariance.data(), ptr->pose_cov, 36*sizeof(double ) );


        msg.twist.twist.linear.x = ptr->twist.linear.x;
        msg.twist.twist.linear.y = ptr->twist.linear.y;
        msg.twist.twist.linear.z = ptr->twist.linear.z;

        msg.twist.twist.angular.x = ptr->twist.angular.x;
        msg.twist.twist.angular.y = ptr->twist.angular.y;
        msg.twist.twist.angular.z = ptr->twist.angular.z;

        std::memcpy(msg.twist.covariance.data(), ptr->twist_cov, 36*sizeof(double ) );

        publisher.publish(msg);
    }
    return 0;
}

ROSTopicWriter create_writer_Odometry(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<nav_msgs::Odometry>(option->topic_name, option->queue_size,option->keep_last);
    target.write_data = from_common_Odometry;

    return target;
}

void to_common_Odometry(nav_msgs::OdometryConstPtr msg, reader_option* option){


    Odometry* ptr_target = nullptr;
    if (option->mem_pool->count >= option->mem_pool->buffer.size() ){
        ptr_target = Odometry_alloc( & option->mem_pool->cfg );
        if(ptr_target)
        option->mem_pool->buffer.push_back(ptr_target);
    }else{
        ptr_target = (Odometry*)option->mem_pool->buffer[option->mem_pool->count];
        ptr_target = Odometry_realloc( ptr_target,& option->mem_pool->cfg );

    }
    if (!ptr_target){
        std::cout << __FUNCTION__  << "fail allocate" << std::endl;
        return;
    }

    option->mem_pool->count+=1;



    ptr_target->stamp = msg->header.stamp.toNSec();


    strncpy(ptr_target->frame_id, msg->header.frame_id.c_str(),MSG_STRUCT_MAX_FRAME_ID_LEN);

    strncpy(ptr_target->child_frame_id, msg->child_frame_id.c_str(),MSG_STRUCT_MAX_FRAME_ID_LEN);

    ptr_target->pose.position.x =  msg->pose.pose.position.x;
    ptr_target->pose.position.y =  msg->pose.pose.position.y;
    ptr_target->pose.position.z =  msg->pose.pose.position.z;

    ptr_target->pose.quaternion.w =  msg->pose.pose.orientation.w;
    ptr_target->pose.quaternion.x =  msg->pose.pose.orientation.x;
    ptr_target->pose.quaternion.y =  msg->pose.pose.orientation.y;
    ptr_target->pose.quaternion.z =  msg->pose.pose.orientation.z;
    std::memcpy(ptr_target->pose_cov, msg->pose.covariance.data(), 36*sizeof(double ) );

    ptr_target->twist.linear.x = msg->twist.twist.linear.x;
    ptr_target->twist.linear.y = msg->twist.twist.linear.y;
    ptr_target->twist.linear.z = msg->twist.twist.linear.z;

    ptr_target->twist.angular.x = msg->twist.twist.angular.x;
    ptr_target->twist.angular.y = msg->twist.twist.angular.y;
    ptr_target->twist.angular.z = msg->twist.twist.angular.z;

    std::memcpy(ptr_target->twist_cov, msg->twist.covariance.data(), 36*sizeof(double) );
}

ROSTopicReader create_reader_Odometry(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<nav_msgs::Odometry>(option->topic_name, option->queue_size, [option](nav_msgs::OdometryConstPtr msg){

        to_common_Odometry(msg, option);
    });

    return target;
}




// UInt8MultiArray

int from_common_UInt8MultiArray(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){

    static thread_local  std_msgs::UInt8MultiArray msg;

    for(int i = 0; i < buffer_size;i++){
        UInt8MultiArray* ptr =  (UInt8MultiArray* )buffer[i] ;
        msg.data.resize(ptr->element_size);
        memcpy(msg.data.data(), ptr->buffer, ptr->buffer_size);
        publisher.publish(msg);
    }
    return 0;
}

ROSTopicWriter create_writer_UInt8MultiArray(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<std_msgs::UInt8MultiArray>(option->topic_name, option->queue_size,option->keep_last);
    target.write_data = from_common_UInt8MultiArray;
    return target;
}

void to_common_UInt8MultiArray(std_msgs::UInt8MultiArrayConstPtr msg, reader_option* option){

    size_t size = msg->data.size();

    UInt8MultiArray* ptr_target = nullptr;
    if (option->mem_pool->count >= option->mem_pool->buffer.size() ){
        ptr_target = UInt8MultiArray_alloc( size,& option->mem_pool->cfg );
        if(ptr_target)
        option->mem_pool->buffer.push_back(ptr_target);
    }else{
        ptr_target = (UInt8MultiArray*)option->mem_pool->buffer[option->mem_pool->count];
        ptr_target = UInt8MultiArray_realloc( size,ptr_target,& option->mem_pool->cfg );

    }
    if (!ptr_target){
        std::cout << __FUNCTION__  << "fail allocate" << std::endl;
        return;
    }

    option->mem_pool->count+=1;


    memcpy(ptr_target->buffer, msg->data.data(),ptr_target->buffer_size );
}


ROSTopicReader create_reader_UInt8MultiArray(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<std_msgs::UInt8MultiArray>(option->topic_name, option->queue_size, [option](std_msgs::UInt8MultiArrayConstPtr msg){

        to_common_UInt8MultiArray(msg, option);
    });

    return target;
}

// UInt16MultiArray
int from_common_UInt16MultiArray(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){

    static thread_local  std_msgs::UInt16MultiArray msg;

    for(int i = 0; i < buffer_size;i++){
        UInt16MultiArray* ptr =  (UInt16MultiArray* )buffer[i] ;
        msg.data.resize(ptr->element_size);
        memcpy(msg.data.data(), ptr->buffer, ptr->buffer_size);
        publisher.publish(msg);
    }
    return 0;
}

ROSTopicWriter create_writer_UInt16MultiArray(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<std_msgs::UInt16MultiArray>(option->topic_name, option->queue_size,option->keep_last);
    target.write_data = from_common_UInt16MultiArray;
    return target;
}

void to_common_UInt16MultiArray(std_msgs::UInt16MultiArrayConstPtr msg, reader_option* option){

    size_t size = msg->data.size();

    UInt16MultiArray* ptr_target = nullptr;
    if (option->mem_pool->count >= option->mem_pool->buffer.size() ){
        ptr_target = UInt16MultiArray_alloc( size,& option->mem_pool->cfg );
        if(ptr_target)
        option->mem_pool->buffer.push_back(ptr_target);
    }else{
        ptr_target = (UInt16MultiArray*)option->mem_pool->buffer[option->mem_pool->count];
        ptr_target = UInt16MultiArray_realloc( size,ptr_target,& option->mem_pool->cfg );

    }
    if (!ptr_target){
        std::cout << __FUNCTION__  << "fail allocate" << std::endl;
        return;
    }

    option->mem_pool->count+=1;

    memcpy(ptr_target->buffer, msg->data.data(),ptr_target->buffer_size );
}

ROSTopicReader create_reader_UInt16MultiArray(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<std_msgs::UInt16MultiArray>(option->topic_name, option->queue_size, [option](std_msgs::UInt16MultiArrayConstPtr msg){

        to_common_UInt16MultiArray(msg, option);
    });

    return target;
}

//===============
int from_common_OccupancyGrid(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){
    static thread_local  nav_msgs::OccupancyGrid msg;
    msg.header.stamp = ros::Time::now();

    for(int i = 0; i < buffer_size;i++){
        OccupancyGrid* ptr =  (OccupancyGrid* )buffer[i] ;

        msg.header.frame_id.assign(ptr->frame_id);
        msg.info.width = ptr->width;
        msg.info.height = ptr->height;
        msg.info.resolution = ptr->resolution;


        msg.info.origin.position.x = ptr->origin.position.x;
        msg.info.origin.position.y = ptr->origin.position.y;
        msg.info.origin.position.z = ptr->origin.position.z;

        msg.info.origin.orientation.w = ptr->origin.quaternion.w;
        msg.info.origin.orientation.x = ptr->origin.quaternion.x;
        msg.info.origin.orientation.y = ptr->origin.quaternion.y;
        msg.info.origin.orientation.z = ptr->origin.quaternion.z;

        msg.data.resize(ptr->width * ptr->height);
        memcpy(msg.data.data(), ptr->data, ptr->buffer_size);
        publisher.publish(msg);
    }
    return 0;
}

ROSTopicWriter create_writer_OccupancyGrid(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<nav_msgs::OccupancyGrid>(option->topic_name, option->queue_size,option->keep_last);
    target.write_data = from_common_OccupancyGrid;
    return target;
}

void to_common_OccupancyGrid(nav_msgs::OccupancyGridConstPtr msg, reader_option* option){



    auto width = msg->info.width;
    auto height = msg->info.height;

    OccupancyGrid* ptr_target = nullptr;
    if (option->mem_pool->count >= option->mem_pool->buffer.size() ){
        ptr_target = OccupancyGrid_alloc( width,height ,& option->mem_pool->cfg );
        if(ptr_target)
        option->mem_pool->buffer.push_back(ptr_target);
    }else{
        ptr_target = (OccupancyGrid*)option->mem_pool->buffer[option->mem_pool->count];
        ptr_target = OccupancyGrid_realloc( width,height,ptr_target,& option->mem_pool->cfg );

    }
    if (!ptr_target){
        std::cout << __FUNCTION__  << "fail allocate" << std::endl;
        return;
    }

    option->mem_pool->count+=1;

    ptr_target->stamp = msg->header.stamp.toNSec();

    //
    strncpy(ptr_target->frame_id, msg->header.frame_id.c_str(),MSG_STRUCT_MAX_FRAME_ID_LEN);
    ptr_target->width = msg->info.width;
    ptr_target->height = msg->info.height;
    ptr_target->resolution = msg->info.resolution;

    ptr_target->origin.position.x = msg->info.origin.position.x;
    ptr_target->origin.position.y = msg->info.origin.position.y;
    ptr_target->origin.position.z = msg->info.origin.position.z;

    ptr_target->origin.quaternion.w = msg->info.origin.orientation.w;
    ptr_target->origin.quaternion.x = msg->info.origin.orientation.x;
    ptr_target->origin.quaternion.y = msg->info.origin.orientation.y;
    ptr_target->origin.quaternion.z = msg->info.origin.orientation.z;

    memcpy(ptr_target->data, msg->data.data(),ptr_target->buffer_size );
}

ROSTopicReader create_reader_OccupancyGrid(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<nav_msgs::OccupancyGrid>(option->topic_name, option->queue_size, [option](nav_msgs::OccupancyGridConstPtr msg){
        to_common_OccupancyGrid(msg, option);
    });

    return target;
}
////
int from_common_PoseStamped(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){
    static thread_local  geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();

    for(int i = 0; i < buffer_size;i++){
        PoseStamped* ptr =  (PoseStamped* )buffer[i] ;

        msg.header.frame_id.assign(ptr->frame_id);
        msg.pose.position.x = ptr->position.x;
        msg.pose.position.y = ptr->position.y;
        msg.pose.position.z = ptr->position.z;

        msg.pose.orientation.w = ptr->quaternion.w;
        msg.pose.orientation.x = ptr->quaternion.x;
        msg.pose.orientation.y = ptr->quaternion.y;
        msg.pose.orientation.z = ptr->quaternion.z;

        publisher.publish(msg);
    }
    return 0;
}

ROSTopicWriter create_writer_PoseStamped(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<geometry_msgs::PoseStamped>(option->topic_name, option->queue_size,option->keep_last);
    target.write_data = from_common_PoseStamped;

    return target;
}

void to_common_PoseStamped(geometry_msgs::PoseStampedConstPtr msg,reader_option* option){


    PoseStamped* ptr_target = nullptr;
    if (option->mem_pool->count >= option->mem_pool->buffer.size() ){
        ptr_target = PoseStamped_alloc( & option->mem_pool->cfg );
        if(ptr_target)
        option->mem_pool->buffer.push_back(ptr_target);
    }else{
        ptr_target = (PoseStamped*)option->mem_pool->buffer[option->mem_pool->count];
        ptr_target = PoseStamped_realloc( ptr_target ,& option->mem_pool->cfg);

    }
    if (!ptr_target){
        std::cout << __FUNCTION__  << "fail allocate" << std::endl;
        return;
    }

    option->mem_pool->count+=1;

    //
    ptr_target->stamp = msg->header.stamp.toNSec();


    strncpy(ptr_target->frame_id,msg->header.frame_id.c_str(), MSG_STRUCT_MAX_FRAME_ID_LEN);
    ptr_target->position.x =  msg->pose.position.x;
    ptr_target->position.y =  msg->pose.position.y;
    ptr_target->position.z =  msg->pose.position.z;

    ptr_target->quaternion.w =  msg->pose.orientation.w;
    ptr_target->quaternion.x =  msg->pose.orientation.x;
    ptr_target->quaternion.y =  msg->pose.orientation.y;
    ptr_target->quaternion.z =  msg->pose.orientation.z;
}

ROSTopicReader create_reader_PoseStamped(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<geometry_msgs::PoseStamped>(option->topic_name, option->queue_size, [option](geometry_msgs::PoseStampedConstPtr msg){

        to_common_PoseStamped(msg, option);
    });

    return target;
}

////

int from_common_Path(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){
    static thread_local  nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();

    for(int i = 0; i < buffer_size;i++){
        Path* ptr =  (Path* )buffer[i] ;
        msg.header.frame_id.assign(ptr->frame_id);
        msg.poses.resize(ptr->element_size);

        for(int j = 0 ; j < ptr->element_size;j++){
            PoseStamped* p = & ptr->data[j];
            msg.poses[j].header.frame_id.assign(p->frame_id);
            msg.poses[j].pose.position.x = p->position.x;
            msg.poses[j].pose.position.y = p->position.y;
            msg.poses[j].pose.position.z = p->position.z;

            msg.poses[j].pose.orientation.w = p->quaternion.w;
            msg.poses[j].pose.orientation.x = p->quaternion.x;
            msg.poses[j].pose.orientation.y = p->quaternion.y;
            msg.poses[j].pose.orientation.z = p->quaternion.z;
        }
        publisher.publish(msg);
    }
    return 0;
}

ROSTopicWriter create_writer_Path(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<nav_msgs::Path>(option->topic_name, option->queue_size,option->keep_last);
    target.write_data = from_common_Path;
    return target;
}

void to_common_Path(nav_msgs::PathConstPtr msg, reader_option* option){

    int pose_num = msg->poses.size();



    Path* ptr_target = nullptr;
    if (option->mem_pool->count >= option->mem_pool->buffer.size() ){
        ptr_target = Path_alloc( pose_num,& option->mem_pool->cfg );
        if(ptr_target)
        option->mem_pool->buffer.push_back(ptr_target);
    }else{
        ptr_target = (Path*)option->mem_pool->buffer[option->mem_pool->count];
        ptr_target = Path_realloc( pose_num,ptr_target,& option->mem_pool->cfg );

    }
    if (!ptr_target){
        std::cout << __FUNCTION__  << "fail allocate" << std::endl;
        return;
    }

    option->mem_pool->count+=1;



    strncpy(ptr_target->frame_id, msg->header.frame_id.c_str(),MSG_STRUCT_MAX_FRAME_ID_LEN );

    ptr_target->stamp = msg->header.stamp.toNSec();

    for(int j = 0 ; j < pose_num; j++){
        PoseStamped* p = & ptr_target->data[j];
        auto& p_msg = msg->poses[j];
        p->position.x = p_msg.pose.position.x;
        p->position.y = p_msg.pose.position.y;
        p->position.z = p_msg.pose.position.z;

        p->quaternion.w = p_msg.pose.orientation.w;
        p->quaternion.x = p_msg.pose.orientation.x;
        p->quaternion.y = p_msg.pose.orientation.y;
        p->quaternion.z = p_msg.pose.orientation.z;
    }

}

ROSTopicReader create_reader_Path(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<nav_msgs::Path>(option->topic_name, option->queue_size, [option](nav_msgs::PathConstPtr msg){

        to_common_Path(msg, option);
    });

    return target;
}
//

int from_common_HeaderString(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){
    static thread_local  rospy_tutorials::HeaderString msg;

    msg.header.stamp = ros::Time::now();

    for(int i = 0; i < buffer_size;i++){
        HeaderString* ptr =  (HeaderString* )buffer[i] ;

        msg.header.frame_id.assign(ptr->frame_id);
        msg.data.assign(ptr->data);

        publisher.publish(msg);
    }
    return 0;
}

ROSTopicWriter create_writer_HeaderString(ros::NodeHandle& nh, writer_option* option){
    ROSTopicWriter target;
    target.valid = true;

    target.writer =nh.advertise<rospy_tutorials::HeaderString>(option->topic_name, option->queue_size,option->keep_last);
    target.write_data = from_common_HeaderString;

    return target;
}

void to_common_HeaderString(rospy_tutorials::HeaderStringConstPtr msg, reader_option* option){


    size_t size = msg->data.size();


    HeaderString* ptr_target = nullptr;
    if (option->mem_pool->count >= option->mem_pool->buffer.size() ){
        ptr_target = HeaderString_alloc( size,& option->mem_pool->cfg );
        if(ptr_target)
        option->mem_pool->buffer.push_back(ptr_target);
    }else{
        ptr_target = (HeaderString*)option->mem_pool->buffer[option->mem_pool->count];
        ptr_target = HeaderString_realloc( size,ptr_target,& option->mem_pool->cfg );

    }
    if (!ptr_target){
        std::cout << __FUNCTION__  << "fail allocate" << std::endl;
        return;
    }

    option->mem_pool->count+=1;


    ptr_target->stamp = msg->header.stamp.toNSec();


    strncpy(ptr_target->frame_id, msg->header.frame_id.c_str(),MSG_STRUCT_MAX_FRAME_ID_LEN);
    strcpy(ptr_target->data, msg->data.c_str());
}

ROSTopicReader create_reader_HeaderString(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<rospy_tutorials::HeaderString>(option->topic_name, option->queue_size, [option](rospy_tutorials::HeaderStringConstPtr msg){

        to_common_HeaderString(msg, option);
    });

    return target;
}

//====
//int from_common_PointCloud2(ros::Publisher& publisher, void** buffer, uint32_t buffer_size){
//
//}
//
//ROSTopicWriter create_writer_PointCloud2(ros::NodeHandle& nh, writer_option* option){
//
//}

void to_common_PointCloud2(sensor_msgs::PointCloud2ConstPtr msg, reader_option* option){


    bool valid_data_type = true;
    bool valid_filed_x = false, valid_filed_y = false,valid_filed_z = false;
    size_t valid_filed_x_id = 0, valid_filed_y_id = 0, valid_filed_z_id = 0;

    size_t point_step = msg->point_step;

    for( auto& p : msg->fields){

        if(p.name[0] == 'x'){
            if (p.datatype != 7){
                valid_data_type = false;
                std::cout << "to_common_PointCloud2, invalid datatype, " << p.name << ", " << p.datatype << std::endl;
                break;
            }
            valid_filed_x = true;
            valid_filed_x_id = p.offset;
        }else if(p.name[0] == 'y'){
            if (p.datatype != 7){
                valid_data_type = false;
                std::cout << "to_common_PointCloud2, invalid datatype, " << p.name << ", " << p.datatype << std::endl;
                break;
            }
            valid_filed_y = true;
            valid_filed_y_id = p.offset;
        }else if(p.name[0] == 'z'){
            if (p.datatype != 7){
                valid_data_type = false;
                std::cout << "to_common_PointCloud2, invalid datatype, " << p.name << ", " << p.datatype << std::endl;
                break;
            }
            valid_filed_z = true;
            valid_filed_z_id = p.offset;
        }
    }

    if( ! (valid_data_type && valid_filed_x && valid_filed_y && valid_filed_z)){
        return;
    }


    size_t height = msg->height;
    size_t width = msg->width;
    size_t channel = msg->fields.size();

    size_t point_num = height*width;
    size_t point_num_use_omp = 1000*1000;

    channel = 3;

//    std::cout << "to_common_PointCloud2 allocate buffer, point_num: " << point_num << std::endl;
//    std::cout << "1 to_common_PointCloud2 allocate buffer, option->mem_pool->buffer.size(): " << option->mem_pool->buffer.size() << std::endl;

    PointCloud2* ptr_target = nullptr;
    if (option->mem_pool->count >= option->mem_pool->buffer.size() ){
        ptr_target = PointCloud2_alloc( height,width,channel,& option->mem_pool->cfg );
        if(ptr_target)
        option->mem_pool->buffer.push_back(ptr_target);
    }else{
        ptr_target = (PointCloud2*)option->mem_pool->buffer[option->mem_pool->count];
        ptr_target = PointCloud2_realloc( height,width,channel,ptr_target,& option->mem_pool->cfg );

    }
//    std::cout << "2 to_common_PointCloud2 allocate buffer, option->mem_pool->buffer.size(): " << option->mem_pool->buffer.size() << std::endl;

    if (!ptr_target){

        std::cout << __FUNCTION__  << " fail to allocate, need "<< point_num*3*4*1e-6 << " MB\n"
        << "free: " << ta_num_free(&option->mem_pool->cfg)<< "\n"
        << "used: " << ta_num_used(&option->mem_pool->cfg)<< "\n"
        ;
        return;
    }

    ptr_target->stamp = msg->header.stamp.toNSec();
    strncpy(ptr_target->frame_id, msg->header.frame_id.c_str(),MSG_STRUCT_MAX_FRAME_ID_LEN);

    ptr_target->is_dense = msg->is_dense;




//    std::cout << "to_common_PointCloud2: point_num: " << point_num
//    <<", valid_data_type: " << valid_data_type
//    << ", valid_filed_x: " << valid_filed_x
//            << ", valid_filed_y: " << valid_filed_y
//            << ", valid_filed_z: " << valid_filed_z
//            << ", valid_filed_x_id: " << valid_filed_x_id
//            << ", valid_filed_y_id: " << valid_filed_y_id
//            << ", valid_filed_z_id: " << valid_filed_z_id
//
//            << "\n"
//;


    {



        common::Time t1 = common::FromUnixNow();
        int is_invalid_num = 0;
//        float x = 0.0, y = 0.0, z = 0.0;
#if 1
        if(point_num < point_num_use_omp){

            const unsigned char * src_data = msg->data.data();
            float* target_data = ptr_target->buffer;
            for(size_t i = 0 ; i < point_num;i++){
                float x = 0.0,y = 0.0,z = 0.0;
                x = *((float *) (src_data + (i*point_step + valid_filed_x_id)));
                y = *((float *) (src_data + (i*point_step + valid_filed_y_id)));
                z = *((float *) (src_data + (i*point_step + valid_filed_z_id)));
                bool valid = (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
                 && !std::isnan(x) && !std::isnan(y) && !std::isnan(z)
                        );
                is_invalid_num += !valid;
                target_data[i*3 + 0 ] = valid ? x:0.0f;
                target_data[i*3 + 1 ] = valid ? y:0.0f;
                target_data[i*3 + 2 ] = valid ? z:0.0f;

//                ptr_target->buffer[i*3 + 0 ] = *((float *) (msg->data.data() + (i*point_step + valid_filed_x_id)));
//                ptr_target->buffer[i*3 + 1 ] = *((float *) (msg->data.data() + (i*point_step + valid_filed_y_id)));
//                ptr_target->buffer[i*3 + 2 ] = *((float *) (msg->data.data() + (i*point_step + valid_filed_z_id)));
            }
        }else{
            std::cout << __FUNCTION__  <<" use omp\n";
            size_t i = 0;
            const unsigned char * src_data = msg->data.data();
            float* target_data = ptr_target->buffer;
            //schedule(static,2048)
#ifdef _OPENMP
#pragma omp parallel for default(none) private(i) shared(point_num,target_data, src_data,point_step,valid_filed_x_id, valid_filed_y_id,  valid_filed_z_id) schedule(static,point_num/2) num_threads(2)
#endif
            for(i = 0 ; i < point_num;i++){
                float x = 0.0,y = 0.0,z = 0.0;
                x = *((float *) (src_data + (i*point_step + valid_filed_x_id)));
                y = *((float *) (src_data + (i*point_step + valid_filed_y_id)));
                z = *((float *) (src_data + (i*point_step + valid_filed_z_id)));
                bool valid = (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)
                              && !std::isnan(x) && !std::isnan(y) && !std::isnan(z)
                );
//                is_invalid_num += !valid;

                target_data[i*3 + 0 ] = valid ? x:0.0f;
                target_data[i*3 + 1 ] = valid ? y:0.0f;
                target_data[i*3 + 2 ] = valid ? z:0.0f;

//                target_data[i*3 + 0 ] = *((float *) (src_data + (i*point_step + valid_filed_x_id)));
//                target_data[i*3 + 1 ] = *((float *) (src_data + (i*point_step + valid_filed_y_id)));
//                target_data[i*3 + 2 ] = *((float *) (src_data + (i*point_step + valid_filed_z_id)));
            }

        }


#endif

#if 0
        {
            size_t i = 0;
            const unsigned char * src_data = msg->data.data();
            float* target_data = ptr_target->buffer;
            //schedule(static,2048)
#ifdef _OPENMP
#pragma omp parallel for default(none) private(i) shared(point_num,target_data, src_data,point_step,valid_filed_x_id, valid_filed_y_id,  valid_filed_z_id) schedule(static,1024)
#endif
            for(i = 0 ; i < point_num;i++){
                float x = 0.0,y = 0.0,z = 0.0;
                x = *((float *) (src_data + (i*point_step + valid_filed_x_id)));
                y = *((float *) (src_data + (i*point_step + valid_filed_y_id)));
                z = *((float *) (src_data + (i*point_step + valid_filed_z_id)));

                bool valid = (isfinite(x) && isfinite(y) && isfinite(z));
                target_data[i*3 + 0 ] = valid ? x:0.0f;
                target_data[i*3 + 1 ] = valid ? y:0.0f;
                target_data[i*3 + 2 ] = valid ? z:0.0f;

//                target_data[i*3 + 0 ] = *((float *) (src_data + (i*point_step + valid_filed_x_id)));
//                target_data[i*3 + 1 ] = *((float *) (src_data + (i*point_step + valid_filed_y_id)));
//                target_data[i*3 + 2 ] = *((float *) (src_data + (i*point_step + valid_filed_z_id)));
            }

        }
#endif





        std::cout << __FUNCTION__  << " for loop use time: " << common::ToMillSeconds(common::FromUnixNow() - t1) << " ms, is_invalid_num: " << is_invalid_num << "\n";
//        std::cout << "\n";

        option->mem_pool->count+=1;
    }


}

ROSTopicReader create_reader_PointCloud2(ros::NodeHandle& nh, reader_option* option){
    ROSTopicReader target;
    target.valid = true;

    target.reader = nh.subscribe<sensor_msgs::PointCloud2>(option->topic_name, 1, [option](sensor_msgs::PointCloud2ConstPtr msg){

        to_common_PointCloud2(msg, option);
    });

    return target;
}

//=====

//================


ROSTopicReader create_reader(ros::NodeHandle& nh, reader_option* option){


    if(std::strcmp(option->topic_type,"LaserScan") == 0 ){
        return create_reader_LaserScan(nh,option);
    } else if(std::strcmp(option->topic_type,"Twist") == 0 ){
        return create_reader_Twist(nh,option);
    }else if(std::strcmp(option->topic_type,"UInt8MultiArray") ==0){
        return create_reader_UInt8MultiArray(nh,option);

    }else if(std::strcmp(option->topic_type,"UInt16MultiArray") ==0){
        return create_reader_UInt16MultiArray(nh,option);

    }else if(std::strcmp(option->topic_type,"OccupancyGrid") ==0){
        return create_reader_OccupancyGrid(nh,option);

    }else if(std::strcmp(option->topic_type,"Odometry") ==0){
        return create_reader_Odometry(nh,option);

    }else if(std::strcmp(option->topic_type,"Path") ==0){
        return create_reader_Path(nh,option);

    }else if(std::strcmp(option->topic_type,"HeaderString") ==0){
        return create_reader_HeaderString(nh,option);

    }else if(std::strcmp(option->topic_type,"PoseStamped") ==0){
        return create_reader_PoseStamped(nh,option);

    }else if(std::strcmp(option->topic_type,"PointCloud2") ==0){
        return create_reader_PointCloud2(nh,option);
    }

    ROSTopicReader reader;
    reader.valid = false;
    return reader;

}

ROSTopicWriter create_writer(ros::NodeHandle& nh, writer_option* option){

    if(std::strcmp(option->topic_type,"LaserScan") == 0 ){
        return create_writer_LaserScan(nh,option);
    } else if(std::strcmp(option->topic_type,"Twist") == 0 ){
        return create_writer_Twist(nh,option);
    }else if(std::strcmp(option->topic_type,"UInt8MultiArray") ==0){
        return create_writer_UInt8MultiArray(nh,option);

    }else if(std::strcmp(option->topic_type,"UInt16MultiArray") ==0){
        return create_writer_UInt16MultiArray(nh,option);
    }else if(std::strcmp(option->topic_type,"OccupancyGrid") ==0){
        return create_writer_OccupancyGrid(nh,option);
    }else if(std::strcmp(option->topic_type,"Odometry") ==0){
        return create_writer_Odometry(nh,option);
    }else if(std::strcmp(option->topic_type,"Path") ==0){
        return create_writer_Path(nh,option);
    }else if(std::strcmp(option->topic_type,"PoseStamped") ==0){
        return create_writer_PoseStamped(nh,option);
    }else if(std::strcmp(option->topic_type,"HeaderString") ==0){
        return create_writer_HeaderString(nh,option);
    }
//    else if(std::strcmp(option->topic_type,"PointCloud2") ==0){
//        return create_writer_PointCloud2(nh,option);
//    }

    //pth, pose, headstring




    ROSTopicWriter reader;
    reader.valid = false;
    return reader;
}


//int ros_write_tf(std::shared_ptr<tf::TransformBroadcaster> tfb, tf::StampedTransform& target,MemPoolHandler_ptr mem_pool ){
//
//    for(int i = 0; i < pool->valid_len;i++){
////        if(pool->nodes[i].ptr == nullptr || pool->nodes[i].count ==0){ break;}
//
//        PoseStampedT_ptr data = (PoseStampedT_ptr)pool->nodes[i];
//        target.setOrigin(tf::Vector3(data->position.x,
//                                     data->position.y,
//                                     data->position.z));
//        target.setRotation(tf::Quaternion(data->quaternion.x,
//                                          data->quaternion.y,
//                                          data->quaternion.z,
//                                          data->quaternion.w));
//
//        tfb->sendTransform(target);
//
//    }
//    return 0;
//
//}

int ros_write_tf_data(std::shared_ptr<tf::TransformBroadcaster> tfb, tf::StampedTransform& target, void** buffer, uint32_t buffer_size ){

    for(int i = 0; i < buffer_size;i++){
//        if(pool->nodes[i].ptr == nullptr || pool->nodes[i].count ==0){ break;}

        PoseStamped_ptr data = (PoseStamped_ptr)buffer[i];
        target.stamp_ = ros::Time::now();
        target.setOrigin(tf::Vector3(data->position.x,
                                     data->position.y,
                                     data->position.z));
        target.setRotation(tf::Quaternion(data->quaternion.x,
                                          data->quaternion.y,
                                          data->quaternion.z,
                                          data->quaternion.w));

        tfb->sendTransform(target);

    }
    return 0;
}

PoseStamped* ros_read_tf(std::shared_ptr<tf::TransformListener> tfl, tf::StampedTransform& target ){
    tfl->lookupTransform(target.frame_id_, target.child_frame_id_,ros::Time(0), target);


    static thread_local  PoseStamped sample = PoseStamped_create();
    sample.position.x = target.getOrigin().x();
    sample.position.y = target.getOrigin().y();
    sample.position.z = target.getOrigin().z();

    sample.quaternion.w = target.getRotation().w();
    sample.quaternion.x = target.getRotation().x();
    sample.quaternion.y = target.getRotation().y();
    sample.quaternion.z = target.getRotation().z();


    sample.stamp = target.stamp_.toNSec();

    return &sample;
}

