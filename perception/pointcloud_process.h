//
// Created by waxz on 5/14/24.
//

#ifndef LIBROSCPP_POINTCLOUD_PROCESS_H
#define LIBROSCPP_POINTCLOUD_PROCESS_H

#include "common/c_style.h"

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct PointIndex{
        float * buffer;
        u64_t point_num ;


    }PointIndex, *PointIndex_ptr;


    // pointcloud:  float[height*width*3]

    int se3_inverse(f32_t tx, f32_t ty, f32_t tz,  f32_t roll, f32_t pitch, f32_t yaw,f32_t* tx_inv, f32_t* ty_inv, f32_t* tz_inv,  f32_t* roll_inv, f32_t* pitch_inv, f32_t* yaw_inv);

    int pointcloud_norm(float* src_buffer, u64_t point_num, u64_t* index_buffer, u64_t index_num, f32_t vx, f32_t vy, f32_t vz, f32_t* cx, f32_t* cy, f32_t* cz, f32_t* nx, f32_t* ny, f32_t* nz, f32_t* nd);

    int pointcloud_clip(float* src_buffer, u64_t height, u64_t width,float* dst_buffer ,u64_t filter_height_min, u64_t filter_height_max, u64_t filter_width_min, u64_t filter_width_max  );

    int pointcloud_transform(float* src_buffer, u64_t point_num, float* dst_buffer , f32_t tx, f32_t ty, f32_t tz,  f32_t roll, f32_t pitch, f32_t yaw);



#ifdef __cplusplus
}
#endif


#endif //LIBROSCPP_POINTCLOUD_PROCESS_H
