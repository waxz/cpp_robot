//
// Created by waxz on 5/14/24.
//

#ifndef LIBROSCPP_POINTCLOUD_PROCESS_H
#define LIBROSCPP_POINTCLOUD_PROCESS_H

#include "common/c_style.h"

#ifdef __cplusplus
extern "C" {
#endif

    // pointcloud:  float[height*width*3]


    int pointcloud_clip(float* src_buffer, u32_t height, u32_t width,float* dst_buffer ,u32_t filter_height_min, u32_t filter_height_max, u32_t filter_width_min, u32_t filter_width_max  );

    int pointcloud_transform(float* src_buffer, u32_t point_num, float* dst_buffer , f32_t tx, f32_t ty, f32_t tz,  f32_t roll, f32_t pitch, f32_t yaw);



#ifdef __cplusplus
}
#endif


#endif //LIBROSCPP_POINTCLOUD_PROCESS_H
