//
// Created by waxz on 5/21/24.
//

#ifndef LIBROSCPP_POINTCLOUD_PALLET_DETECT_H
#define LIBROSCPP_POINTCLOUD_PALLET_DETECT_H

#include "common/c_style.h"
#include "tinyalloc/tinyalloc.h"


#ifdef __cplusplus
extern "C" {
#endif


    typedef struct PointCloudBuffer{
        f32_t * buffer;
        u64_t float_num;
    }PointCloudBuffer, *PointCloudBuffer_ptr;

    typedef struct pointcloud_pallet_detector_t{

        void* handler;

        /// crete ros node and subscriber and publisher from toml file
        /// \param h
        /// \param filename
        bool(*create)(struct pointcloud_pallet_detector_t* h, const char* filename, const ta_cfg_t* cfg);
        /// close all resource
        /// \param h
        void(*close)(struct pointcloud_pallet_detector_t* h);


        void(*set_input)(struct pointcloud_pallet_detector_t* h, f32_t * buffer,u64_t height, u64_t width, f32_t vx, f32_t vy, f32_t vz);

        //filter ground
        PointCloudBuffer_ptr (*filter_ground)(struct pointcloud_pallet_detector_t* h, u32_t output_mode);
        void(*set_ground_init_dim)(struct pointcloud_pallet_detector_t* h, u64_t height_min , u64_t height_max ,u64_t width_min, u64_t width_max);
        void(*set_ground_init_thresh)(struct pointcloud_pallet_detector_t* h,  f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max, f32_t nz_min);
        void(*set_ground_adaptive_thresh)(struct pointcloud_pallet_detector_t* h,  f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max);



        //find cargo
        PointCloudBuffer_ptr(*filter_cargo)(struct pointcloud_pallet_detector_t* h);

        PointCloudBuffer_ptr(*filter_pallet)(struct pointcloud_pallet_detector_t* h);

        PointCloudBuffer_ptr(*find_pallet)(struct pointcloud_pallet_detector_t* h);

        // get pose
        void(*compute_pallet)(struct pointcloud_pallet_detector_t* h);

    }pointcloud_pallet_detector_t, *pointcloud_pallet_detector_t_ptr;



    pointcloud_pallet_detector_t pointcloud_pallet_detector_create();


#ifdef __cplusplus
}
#endif

#endif //LIBROSCPP_POINTCLOUD_PALLET_DETECT_H
