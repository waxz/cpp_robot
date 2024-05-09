//
// Created by waxz on 4/28/24.
//

#ifndef CMAKE_SUPER_BUILD_DDS_MESSAGE_CONVERT_H
#define CMAKE_SUPER_BUILD_DDS_MESSAGE_CONVERT_H


#include "Message.h"
#include "MessagePubSubTypes.h"
#include "message_center_types.h"
#include "message_center_handler.h"
#include "memory_pool_handler.h"

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

namespace dds_helper{
    // to_dds
    inline void from_dds(const Message::Laserscan1500& dds_value, MemPoolHandler* mem_pool){

        u32_t size = dds_value.ranges_size();


        LaserScan_ptr ptr_target = nullptr;
        if (mem_pool->count >= mem_pool->buffer.size() ){
            ptr_target = LaserScan_alloc( size,& mem_pool->cfg );
            if (ptr_target)
                mem_pool->buffer.push_back(ptr_target);
        }else{
            ptr_target = (LaserScan_ptr)mem_pool->buffer[mem_pool->count];
            ptr_target = LaserScan_realloc( size,ptr_target,& mem_pool->cfg );

        }
        if(!ptr_target){
            return;
        }

        mem_pool->count+=1;

        ptr_target->stamp = dds_value.stamp();

        ptr_target->angle_increment = dds_value.angle_increment();
        ptr_target->angle_min = dds_value.angle_min();
        ptr_target->angle_max = dds_value.angle_max();

        ptr_target->range_min = dds_value.range_min();
        ptr_target->range_max = dds_value.range_max();



        auto& framed_id = dds_value.frame_id();
        auto& ranges = dds_value.ranges();
        auto& intensities = dds_value.intensities();

        std::strcpy(ptr_target->frame_id, framed_id.data());
        std::copy( ranges.data(), ranges.data() + size, ptr_target->buffer);
        std::copy( intensities.data(), intensities.data() + size, ptr_target->buffer + size);

    }


    inline int to_dds( Message::Laserscan1500* dds_value,  void* common_ptr){
        LaserScan_ptr target_ptr = (LaserScan_ptr)common_ptr;

        u32_t size = target_ptr->ranges_size;

        auto& ranges = dds_value->ranges();
        auto& intensities = dds_value->intensities();
        if(size > ranges.size()){
            return -1;
        }

        auto& framed_id = dds_value->frame_id();

        std::strcpy(framed_id.data(), target_ptr->frame_id);
        dds_value->stamp(target_ptr->stamp);

        dds_value->angle_increment(target_ptr->angle_increment);
        dds_value->angle_min(target_ptr->angle_min);
        dds_value->angle_max(target_ptr->angle_max);

        dds_value->range_min(target_ptr->range_min);
        dds_value->range_max(target_ptr->range_max);


        std::copy(target_ptr->buffer, target_ptr->buffer + size, ranges.data());
        std::copy(target_ptr->buffer + size, target_ptr->buffer + size + size , intensities.data());



        return 0;

    }

// from_dds


    inline void from_dds(const Message::Pointcloud1200x800x4& dds_value, MemPoolHandler* mem_pool){

        static_cast<void>(dds_value);
        static_cast<void>(mem_pool);

        u32_t height = dds_value.height();
        u32_t width = dds_value.width();
        u32_t channel = dds_value.channel();
        size_t stamp = dds_value.stamp();

        size_t float_num = height*width*channel;

        PointCloud2_ptr ptr_target = nullptr;
        if (mem_pool->count >= mem_pool->buffer.size() ){
            ptr_target = PointCloud2_alloc( height,width,channel ,& mem_pool->cfg );
            if(ptr_target)
                mem_pool->buffer.push_back(ptr_target);
        }else{
            ptr_target = (PointCloud2_ptr)mem_pool->buffer[mem_pool->count];
            ptr_target = PointCloud2_realloc( height,width,channel,ptr_target,& mem_pool->cfg );
        }
        if(!ptr_target){
            return;
        }
        mem_pool->count+=1;

        ptr_target->stamp = stamp;
        std::array<char,50> frame_id =  dds_value.frame_id();
        auto& data = dds_value.data();
        std::strcpy(ptr_target->frame_id, frame_id.data());
        std::copy(data.begin(), data.begin() + float_num, ptr_target->buffer );
    }
    inline void from_dds(const Message::Pointcloud1920x1080x3& dds_value, MemPoolHandler* mem_pool){

        static_cast<void>(dds_value);
        static_cast<void>(mem_pool);

        u32_t height = dds_value.height();
        u32_t width = dds_value.width();
        u32_t channel = dds_value.channel();
        size_t stamp = dds_value.stamp();

        size_t float_num = height*width*channel;

        PointCloud2_ptr ptr_target = nullptr;
        if (mem_pool->count >= mem_pool->buffer.size() ){
            ptr_target = PointCloud2_alloc( height,width,channel ,& mem_pool->cfg );
            if(ptr_target)
                mem_pool->buffer.push_back(ptr_target);
        }else{
            ptr_target = (PointCloud2_ptr)mem_pool->buffer[mem_pool->count];
            ptr_target = PointCloud2_realloc( height,width,channel,ptr_target,& mem_pool->cfg );
        }
        if(!ptr_target){
            return;
        }
        mem_pool->count+=1;

        ptr_target->stamp = stamp;
        std::array<char,50> frame_id =  dds_value.frame_id();
        auto& data = dds_value.data();
        std::strcpy(ptr_target->frame_id, frame_id.data());
        std::copy(data.begin(), data.begin() + float_num, ptr_target->buffer );
    }
    inline int to_dds( Message::Pointcloud1200x800x4* dds_value, void* common_ptr){
        PointCloud2_ptr target_ptr = (PointCloud2_ptr)common_ptr;
        dds_value->channel(target_ptr->channel);
        dds_value->height(target_ptr->height);
        dds_value->width(target_ptr->width);
        size_t float_num = target_ptr->channel * target_ptr->height * target_ptr->width;
        auto& data = dds_value->data();
        if (float_num >  data.size()){
            return -1;
        }
        std::array<char,50> frame_id =  dds_value->frame_id();
        std::copy(target_ptr->frame_id,target_ptr->frame_id+50, frame_id.begin() );

        std::copy(target_ptr->buffer,target_ptr->buffer+float_num, data.begin() );
        dds_value->stamp(target_ptr->stamp);

        return 0;
    }

    inline int to_dds( Message::Pointcloud1920x1080x3* dds_value, void* common_ptr){
        PointCloud2_ptr target_ptr = (PointCloud2_ptr)common_ptr;
        dds_value->channel(target_ptr->channel);
        dds_value->height(target_ptr->height);
        dds_value->width(target_ptr->width);
        size_t float_num = target_ptr->channel * target_ptr->height * target_ptr->width;
        auto& data = dds_value->data();
        if (float_num >  data.size()){
            return -1;
        }
        std::array<char,50> frame_id =  dds_value->frame_id();
        std::copy(target_ptr->frame_id,target_ptr->frame_id+50, frame_id.begin() );

        std::copy(target_ptr->buffer,target_ptr->buffer+float_num, data.begin() );
        dds_value->stamp(target_ptr->stamp);

        return 0;
    }

}



#endif //CMAKE_SUPER_BUILD_DDS_MESSAGE_CONVERT_H
