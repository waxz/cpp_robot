//
// Created by waxz on 5/14/24.
//
#include "pointcloud_process.h"

#include "math/transform/eigen_transform.h"

#include "common/clock_time.h"

#include "math/transform/transform3d.h"


#include <iostream>
typedef Eigen::Matrix<float,3,Eigen::Dynamic,Eigen::ColMajor> Matrix3xf;
typedef Eigen::Map<Matrix3xf> Matrix3xfMap;
typedef Eigen::Map<const Matrix3xf> Matrix3xfMapConst;   // a read-only map

int pointcloud_clip(float* src_buffer, u64_t height, u64_t width,float* dst_buffer ,u64_t filter_height_min, u64_t filter_height_max, u64_t filter_width_min, u64_t filter_width_max  ){

    if(filter_height_min >=filter_height_max
    || filter_height_max >  height

    || filter_width_min >= filter_width_max
    || filter_width_max > width
    ){
        return -1;
    }

    size_t width_offset = width*3;
//
    size_t clip_width = filter_width_max - filter_width_min;
    size_t clip_width_offset = clip_width*3;

    size_t clip_height = filter_height_max - filter_height_min;
    size_t clip_width_byte = clip_width_offset* sizeof(float);
    size_t filter_width_min_offset= filter_width_min*3;


    for(size_t i = 0 ; i < clip_height; i++){
        float* src = src_buffer + (filter_height_min + i )*width_offset + filter_width_min_offset;
        float* dst = dst_buffer + i*clip_width_offset;
        memcpy(dst, src, clip_width_byte);
    }


    return 0;
}


int pointcloud_transform(float* src_buffer, u64_t point_num, float* dst_buffer , f32_t tx, f32_t ty, f32_t tz,  f32_t roll, f32_t pitch, f32_t yaw){
    auto transform = transform::createSe3(tx,ty,tz,roll,pitch,yaw);
    std::cout << "transform:\n" << transform.matrix() << "\n";
    auto src_mat = Matrix3xfMapConst(src_buffer, 3, point_num);
    std::cout << "src_mat:\n" << src_mat << "\n";
    auto dst_mat = Matrix3xfMap(dst_buffer, 3, point_num);
    dst_mat = transform * src_mat;
    std::cout << "dst_mat:\n" << dst_mat << "\n";

    return 0;
}
