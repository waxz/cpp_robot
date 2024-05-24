//
// Created by waxz on 5/21/24.
//

#ifndef LIBROSCPP_POINTCLOUD_PALLET_DETECT_IMPL_H
#define LIBROSCPP_POINTCLOUD_PALLET_DETECT_IMPL_H
#include "tinyalloc/tinyalloc.h"
#include "common/c_style.h"
#include "config/pallet_detect_config_gen.hpp"
#include "pointcloud_pallet_detect.h"

namespace perception{

    struct StableFilter{
        f32_t resolution = 0.01f;

    };



    struct PalletDetector{

        PalletDetector()=default;
        perception::DetectorConfig config;
        ta_cfg_t mem_cfg = {0};
        PointCloudBuffer ground_cloud_buffer;
        f32_t * ground_init_buffer = nullptr;
        f32_t * ground_output_buffer = nullptr;

        // 0: not define
        // 1: ground
        // -1 noise
        // 2 vertical
        // 3: pallet
        // 4: cargo
        i8_t * cloud_label_table = nullptr;

        int create(const char* filename, const ta_cfg_t* cfg);
        void stop();
        f32_t *cloud_buffer = nullptr;
        f32_t viewpoint_x = 0.0;
        f32_t viewpoint_y = 0.0;
        f32_t viewpoint_z = 0.0;
        void set_input(  f32_t * buffer,u64_t height, u64_t width, f32_t vx, f32_t vy, f32_t vz);
        void set_ground_init_dim( u64_t height_min , u64_t height_max ,u64_t width_min, u64_t width_max);
        void set_ground_init_thresh( f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max, f32_t nz_min);
        void set_ground_adaptive_thresh(  f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max);
        void set_ground_uncertain_thresh(  f32_t far_uncertain_z_max, f32_t far_uncertain_x_change_min,f32_t far_uncertain_adaptive_z_max,i32_t far_uncertain_row);


        i8_t filter_ground_status = 0;
        PointCloudBuffer_ptr filter_ground(u32_t output_mode);
        PointCloudBuffer_ptr filter_vertical(u32_t output_mode);
        PointCloudBuffer_ptr filter_pallet(u32_t output_mode);

    };
}


#endif //LIBROSCPP_POINTCLOUD_PALLET_DETECT_IMPL_H
