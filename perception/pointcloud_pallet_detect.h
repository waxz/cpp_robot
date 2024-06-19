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

typedef struct PalletInfo{
    f32_t confidence;
    i32_t info;
    f64_t tx;
    f64_t ty;
    f64_t tz;
    f64_t roll;
    f64_t pitch;
    f64_t yaw;

}PalletInfo, *PalletInfo_ptr;

typedef struct PalletInfoBuffer{
    PalletInfo_ptr  buffer;
    u64_t pallet_num;
}PalletInfoBuffer, *PalletInfoBuffer_ptr;

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

        void(*set_vertical_init_dim)(struct pointcloud_pallet_detector_t* h, u64_t height_min , u64_t height_max ,u64_t width_min, u64_t width_max);
        void(*set_vertical_init_thresh)(struct pointcloud_pallet_detector_t* h,  f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max,  f32_t jx_max, f32_t jy_max, f32_t jz_max);

        //    far_uncertain_z_max :f32,
        //    far_uncertain_x_change_min : f32,
        //    far_uncertain_row : u64
        void(*set_ground_uncertain_thresh)(struct pointcloud_pallet_detector_t* h,  f32_t far_uncertain_z_max, f32_t far_uncertain_x_change_min, f32_t far_uncertain_adaptive_z_max, i32_t far_uncertain_row);


        void(*set_pallet_row)(struct pointcloud_pallet_detector_t* h, i32_t row_high, i32_t row_low);
        void(*set_pallet_thresh)(struct pointcloud_pallet_detector_t* h,  f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max,  f32_t jx_max, f32_t jy_max, f32_t jz_max);


        //find cargo
        PointCloudBuffer_ptr(*filter_vertical)(struct pointcloud_pallet_detector_t* h, u32_t output_mode);

        PointCloudBuffer_ptr(*filter_pallet)(struct pointcloud_pallet_detector_t* h, u32_t output_mode);

        PointCloudBuffer_ptr(*find_pallet)(struct pointcloud_pallet_detector_t* h);

        PalletInfoBuffer_ptr(*get_pallet)(struct pointcloud_pallet_detector_t* h, u32_t output_mode);


        // get pose
        void(*compute_pallet)(struct pointcloud_pallet_detector_t* h);

    }pointcloud_pallet_detector_t, *pointcloud_pallet_detector_t_ptr;



    pointcloud_pallet_detector_t pointcloud_pallet_detector_create();


#ifdef __cplusplus
}
#endif

#endif //LIBROSCPP_POINTCLOUD_PALLET_DETECT_H
