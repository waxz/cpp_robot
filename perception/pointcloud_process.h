//
// Created by waxz on 5/14/24.
//

#ifndef LIBROSCPP_POINTCLOUD_PROCESS_H
#define LIBROSCPP_POINTCLOUD_PROCESS_H

#include "common/c_style.h"
#include "pointcloud_calib_optim.h"



#ifdef __cplusplus
extern "C" {
#endif



    // pointcloud:  f32_t[height*width*3]

    int se3_inverse(f32_t tx, f32_t ty, f32_t tz,  f32_t roll, f32_t pitch, f32_t yaw,f32_t* tx_inv, f32_t* ty_inv, f32_t* tz_inv,  f32_t* roll_inv, f32_t* pitch_inv, f32_t* yaw_inv);

    int pointcloud_norm(f32_t* src_buffer, u64_t point_num, u64_t* index_buffer, u64_t index_num, f32_t vx, f32_t vy, f32_t vz, f32_t* cx, f32_t* cy, f32_t* cz, f32_t* nx, f32_t* ny, f32_t* nz, f32_t* nd);


    int pointcloud_norm2d(f32_t* src_buffer, u64_t point_num, u64_t* index_buffer, u64_t index_num, f32_t vx, f32_t vy, f32_t vz, f32_t* cx, f32_t* cy, f32_t* cz, f32_t* nx, f32_t* ny, f32_t* nz, f32_t* nd);

    // process srcbuffer, store filter result in index_buffer
    int pointcloud_filter_along_line(f32_t* src_buffer, u64_t point_num, u64_t* index_buffer, u64_t* index_num, f32_t cx, f32_t cy, f32_t cz, f32_t dx, f32_t dy, f32_t dz, f32_t radius, f32_t a_min, f32_t a_max);

    int pointcloud_clip(f32_t* src_buffer, u64_t height, u64_t width,f32_t* dst_buffer ,u64_t filter_height_min, u64_t filter_height_max, u64_t filter_width_min, u64_t filter_width_max  );

    int pointcloud_filter_count(f32_t* src_buffer, u64_t point_num, u64_t* count, f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max);

    int pointcloud_transform(f32_t* src_buffer, u64_t point_num, f32_t* dst_buffer , f32_t tx, f32_t ty, f32_t tz,  f32_t roll, f32_t pitch, f32_t yaw);

    int pointcloud_mean_filter(f32_t* src_buffer, u64_t point_num, f32_t* dst_buffer , u32_t* count, f32_t jump_max);


#ifdef __cplusplus
}
#endif


#endif //LIBROSCPP_POINTCLOUD_PROCESS_H
