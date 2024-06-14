//
// Created by waxz on 5/9/24.
//

#include "dds_message_convert.hpp"
#include "common/string_logger.h"
#include "common/clock_time.h"
namespace dds_helper {
    // to_dds
    void from_dds(const Message::Laserscan1500 &dds_value, MemPoolHandler *mem_pool) {

        u32_t size = dds_value.ranges_size();


        LaserScan_ptr target_ptr = nullptr;
        if (mem_pool->count >= mem_pool->buffer.size()) {
            target_ptr = LaserScan_alloc(size, &mem_pool->cfg);
            if (target_ptr)
                mem_pool->buffer.push_back(target_ptr);
        } else {
            target_ptr = (LaserScan_ptr) mem_pool->buffer[mem_pool->count];
            target_ptr = LaserScan_realloc(size, target_ptr, &mem_pool->cfg);

        }
        if (!target_ptr) {
            return;
        }

        mem_pool->count += 1;

        target_ptr->stamp = dds_value.stamp();

        target_ptr->angle_increment = dds_value.angle_increment();
        target_ptr->angle_min = dds_value.angle_min();
        target_ptr->angle_max = dds_value.angle_max();

        target_ptr->range_min = dds_value.range_min();
        target_ptr->range_max = dds_value.range_max();


        auto &framed_id = dds_value.frame_id();
        auto &ranges = dds_value.ranges();
        auto &intensities = dds_value.intensities();

        std::strcpy(target_ptr->frame_id, framed_id.data());
        std::copy(ranges.data(), ranges.data() + size, target_ptr->buffer);
        std::copy(intensities.data(), intensities.data() + size, target_ptr->buffer + size);

    }


    int to_dds(Message::Laserscan1500 *dds_value, void *common_ptr) {
        LaserScan_ptr target_ptr = (LaserScan_ptr) common_ptr;

        u32_t size = target_ptr->ranges_size;

        auto &ranges = dds_value->ranges();
        auto &intensities = dds_value->intensities();
        if (size > ranges.size()) {
            return -1;
        }

        auto &framed_id = dds_value->frame_id();

        std::strcpy(framed_id.data(), target_ptr->frame_id);
        dds_value->stamp(target_ptr->stamp);

        dds_value->angle_increment(target_ptr->angle_increment);
        dds_value->angle_min(target_ptr->angle_min);
        dds_value->angle_max(target_ptr->angle_max);

        dds_value->range_min(target_ptr->range_min);
        dds_value->range_max(target_ptr->range_max);


        std::copy(target_ptr->buffer, target_ptr->buffer + size, ranges.data());
        std::copy(target_ptr->buffer + size, target_ptr->buffer + size + size, intensities.data());


        return 0;

    }
    int to_dds(Message::HeaderString1024 *dds_value, void *common_ptr){
        HeaderString_ptr target_ptr = (HeaderString_ptr) common_ptr;
        u32_t size = target_ptr->element_size;

        auto &data = dds_value->data();
        if (size > data.size()) {
            return -1;
        }

        auto &framed_id = dds_value->frame_id();
        std::strcpy(framed_id.data(), target_ptr->frame_id);
        dds_value->stamp(target_ptr->stamp);
        std::copy(target_ptr->data, target_ptr->data + size, data.data());

        return 0;


    }
    int to_dds(Message::Path1024 *dds_value, void *common_ptr){
        Path_ptr target_ptr = (Path_ptr) common_ptr;
        u32_t size = target_ptr->element_size;

        auto &data = dds_value->poses();
        if (size > data.size()) {
            return -1;
        }

        auto &framed_id = dds_value->frame_id();
        std::strcpy(framed_id.data(), target_ptr->frame_id);
        dds_value->stamp(target_ptr->stamp);

        for(u32_t i = 0 ; i < size; i++){
            auto& to_data = data[i];
            auto& from_data = target_ptr->data[i];
            std::strcpy(to_data.frame_id().data(), from_data.frame_id);

            to_data.position().x(from_data.position.x) ;
            to_data.position().y(from_data.position.y) ;
            to_data.position().z(from_data.position.z) ;

            to_data.quaternion().w(from_data.quaternion.w) ;
            to_data.quaternion().x(from_data.quaternion.x) ;
            to_data.quaternion().y(from_data.quaternion.y) ;
            to_data.quaternion().z(from_data.quaternion.z) ;
        }
        return 0;

    }


// from_dds
    void from_dds(const Message::HeaderString1024 &dds_value, MemPoolHandler *mem_pool){

        static_cast<void>(dds_value);
        static_cast<void>(mem_pool);

        u32_t size = dds_value.data_size();


        HeaderString_ptr target_ptr = nullptr;
        if (mem_pool->count >= mem_pool->buffer.size()) {
            target_ptr = HeaderString_alloc(size, &mem_pool->cfg);
            if (target_ptr)
                mem_pool->buffer.push_back(target_ptr);
        } else {
            target_ptr = (HeaderString_ptr) mem_pool->buffer[mem_pool->count];
            target_ptr = HeaderString_realloc(size, target_ptr, &mem_pool->cfg);

        }
        if (!target_ptr) {
            return;
        }


        mem_pool->count += 1;

        target_ptr->stamp = dds_value.stamp();
        auto &framed_id = dds_value.frame_id();
        auto &data = dds_value.data();

        std::strcpy(target_ptr->frame_id, framed_id.data());
        std::strcpy(target_ptr->data, data.data());
    }
    void from_dds(const Message::Path1024 &dds_value, MemPoolHandler *mem_pool){
        static_cast<void>(dds_value);
        static_cast<void>(mem_pool);

        u32_t size = dds_value.poses_size();


        Path_ptr target_ptr = nullptr;
        if (mem_pool->count >= mem_pool->buffer.size()) {
            target_ptr = Path_alloc(size, &mem_pool->cfg);
            if (target_ptr)
                mem_pool->buffer.push_back(target_ptr);
        } else {
            target_ptr = (Path_ptr) mem_pool->buffer[mem_pool->count];
            target_ptr = Path_realloc(size, target_ptr, &mem_pool->cfg);

        }
        if (!target_ptr) {
            return;
        }


        mem_pool->count += 1;
        target_ptr->stamp = dds_value.stamp();
        auto &framed_id = dds_value.frame_id();
        auto &data = dds_value.poses();
        std::strcpy(target_ptr->frame_id, framed_id.data());

        for(u32_t i = 0 ; i < size;i++){
            auto& from_data = data[i];
            auto& to_data = target_ptr->data[i];

            std::strcpy(to_data.frame_id, from_data.frame_id().data());

            to_data.position.x = from_data.position().x();
            to_data.position.y = from_data.position().y();
            to_data.position.z = from_data.position().z();

            to_data.quaternion.w = from_data.quaternion().w();
            to_data.quaternion.x = from_data.quaternion().x();
            to_data.quaternion.y = from_data.quaternion().y();
            to_data.quaternion.z = from_data.quaternion().z();

        }


    }


    void from_dds(const Message::Pointcloud1200x800x4 &dds_value, MemPoolHandler *mem_pool) {

        static_cast<void>(dds_value);
        static_cast<void>(mem_pool);

        u32_t height = dds_value.height();
        u32_t width = dds_value.width();
        u32_t channel = dds_value.channel();
        size_t stamp = dds_value.stamp();

        size_t float_num = height * width * channel;

        PointCloud2_ptr target_ptr = nullptr;
        if (mem_pool->count >= mem_pool->buffer.size()) {
            target_ptr = PointCloud2_alloc(height, width, channel, &mem_pool->cfg);
            if (target_ptr)
                mem_pool->buffer.push_back(target_ptr);
        } else {
            target_ptr = (PointCloud2_ptr) mem_pool->buffer[mem_pool->count];
            target_ptr = PointCloud2_realloc(height, width, channel, target_ptr, &mem_pool->cfg);
        }
        if (!target_ptr) {
            return;
        }
        mem_pool->count += 1;

        target_ptr->stamp = stamp;
        auto& frame_id = dds_value.frame_id();
        auto &data = dds_value.data();
        std::strcpy(target_ptr->frame_id, frame_id.data());
        std::copy(data.begin(), data.begin() + float_num, target_ptr->buffer);
    }
    void from_dds(const Message::Pointcloud640x480x3 &dds_value, MemPoolHandler *mem_pool){

        static_cast<void>(dds_value);
        static_cast<void>(mem_pool);

        u32_t height = dds_value.height();
        u32_t width = dds_value.width();
        u32_t channel = dds_value.channel();
        size_t stamp = dds_value.stamp();

        size_t float_num = height * width * channel;

        PointCloud2_ptr target_ptr = nullptr;
        if (mem_pool->count >= mem_pool->buffer.size()) {
            target_ptr = PointCloud2_alloc(height, width, channel, &mem_pool->cfg);
            if (target_ptr)
                mem_pool->buffer.push_back(target_ptr);
        } else {
            target_ptr = (PointCloud2_ptr) mem_pool->buffer[mem_pool->count];
            target_ptr = PointCloud2_realloc(height, width, channel, target_ptr, &mem_pool->cfg);
        }
        if (!target_ptr) {
            MLOGW("allocate memory fail, [%u, %u, %u], mem: [%p, %p, %zu, %zu, %zu]", height, width,channel,  mem_pool->cfg.base, mem_pool->cfg.limit, mem_pool->cfg.alignment,  ta_num_free(&mem_pool->cfg), ta_num_used(&mem_pool->cfg));

            return;
        }
        mem_pool->count += 1;

        target_ptr->stamp = stamp;
        auto& frame_id = dds_value.frame_id();


        auto &data = dds_value.data();
        std::strcpy(target_ptr->frame_id, frame_id.data());
        std::copy(data.begin(), data.begin() + float_num, target_ptr->buffer);
    }

    void from_dds(const Message::Pointcloud1920x1080x3 &dds_value, MemPoolHandler *mem_pool) {

        static_cast<void>(dds_value);
        static_cast<void>(mem_pool);

        u32_t height = dds_value.height();
        u32_t width = dds_value.width();
        u32_t channel = dds_value.channel();
        size_t stamp = dds_value.stamp();

        size_t float_num = height * width * channel;

        PointCloud2_ptr target_ptr = nullptr;
        if (mem_pool->count >= mem_pool->buffer.size()) {
            target_ptr = PointCloud2_alloc(height, width, channel, &mem_pool->cfg);
            if (target_ptr)
                mem_pool->buffer.push_back(target_ptr);
        } else {
            target_ptr = (PointCloud2_ptr) mem_pool->buffer[mem_pool->count];
            target_ptr = PointCloud2_realloc(height, width, channel, target_ptr, &mem_pool->cfg);
        }
        if (!target_ptr) {
            MLOGW("allocate memory fail, [%u, %u, %u], mem: [%p, %p, %zu, %zu, %zu]", height, width,channel,  mem_pool->cfg.base, mem_pool->cfg.limit, mem_pool->cfg.alignment,  ta_num_free(&mem_pool->cfg), ta_num_used(&mem_pool->cfg));

            return;
        }
        mem_pool->count += 1;

        target_ptr->stamp = stamp;
        auto& frame_id = dds_value.frame_id();


        auto &data = dds_value.data();
        std::strcpy(target_ptr->frame_id, frame_id.data());
        std::copy(data.begin(), data.begin() + float_num, target_ptr->buffer);
    }

    int to_dds(Message::Pointcloud1200x800x4 *dds_value, void *common_ptr) {
        PointCloud2_ptr target_ptr = (PointCloud2_ptr) common_ptr;
        dds_value->channel(target_ptr->channel);
        dds_value->height(target_ptr->height);
        dds_value->width(target_ptr->width);
        size_t float_num = target_ptr->channel * target_ptr->height * target_ptr->width;
        auto &data = dds_value->data();
        if (float_num > data.size()) {
            return -1;
        }
        auto& frame_id = dds_value->frame_id();

        std::copy(target_ptr->frame_id, target_ptr->frame_id + MSG_STRUCT_MAX_FRAME_ID_LEN, frame_id.begin());

        std::copy(target_ptr->buffer, target_ptr->buffer + float_num, data.begin());
        dds_value->stamp(target_ptr->stamp);

        return 0;
    }
    int to_dds(Message::Pointcloud640x480x3 *dds_value, void *common_ptr){
        PointCloud2_ptr target_ptr = (PointCloud2_ptr) common_ptr;
        dds_value->channel(target_ptr->channel);
        dds_value->height(target_ptr->height);
        dds_value->width(target_ptr->width);
        size_t float_num = target_ptr->channel * target_ptr->height * target_ptr->width;
        auto &data = dds_value->data();
        if (float_num > data.size()) {
            return -1;
        }

        common::Time t1 = common::FromUnixNow();
        auto& frame_id = dds_value->frame_id();
        std::copy(target_ptr->frame_id, target_ptr->frame_id + 50, frame_id.begin());

        std::copy(target_ptr->buffer, target_ptr->buffer + float_num, data.begin());
        dds_value->stamp(target_ptr->stamp);

//        MLOGI("to_dds Pointcloud640x480x3 use time %ld ms", common::ToMillSeconds( common::FromUnixNow() -t1 ));
        return 0;
    }


    int to_dds(Message::Pointcloud1920x1080x3 *dds_value, void *common_ptr) {
        PointCloud2_ptr target_ptr = (PointCloud2_ptr) common_ptr;
        dds_value->channel(target_ptr->channel);
        dds_value->height(target_ptr->height);
        dds_value->width(target_ptr->width);
        size_t float_num = target_ptr->channel * target_ptr->height * target_ptr->width;
        auto &data = dds_value->data();
        if (float_num > data.size()) {
            return -1;
        }
        auto& frame_id = dds_value->frame_id();
        std::copy(target_ptr->frame_id, target_ptr->frame_id + 50, frame_id.begin());

        std::copy(target_ptr->buffer, target_ptr->buffer + float_num, data.begin());
        dds_value->stamp(target_ptr->stamp);

        return 0;
    }

}
