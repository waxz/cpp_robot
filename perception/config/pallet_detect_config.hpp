//
// Created by waxz on 5/21/24.
//

#ifndef LIBROSCPP_PALLET_DETECT_CONFIG_HPP
#define LIBROSCPP_PALLET_DETECT_CONFIG_HPP



#include <vector>
#include <unordered_map>
#include <iostream>

namespace perception{

    /// GEN[TOML]
    struct CloudDimConfig{
        int height;
        int width;

    };
    /// GEN[TOML]
    struct CloudFilterConfig{
        bool enable_dim; 
        int filter_height_min;
        int filter_height_max;
        int filter_width_min;
        int filter_width_max;
        bool enable_range;

        float filter_x_min;
        float filter_y_min;
        float filter_z_min;
        float filter_x_max;
        float filter_y_max;
        float filter_z_max;
    };

    
    /// GEN[TOML]
    struct CloudConfig{
        CloudDimConfig dim;
        CloudFilterConfig filter;

    };
    /// GEN[TOML]
    struct Pose{
        float tx;
        float ty;
        float tz;
        float roll;
        float pitch;
        float yaw;

    };
    /// GEN[TOML]
    struct Extrinsic{
        bool enable;
        Pose pose;

    };

    /// GEN[TOML]
    struct GroundFilter{
        int output_mode;
        // 0 height min to max
        // 1 height max to min
        // 2 width min to max
        // 3 width max to min

        int search_direction;

        int init_valid_num_min;
        // initial ground region
        int init_ground_height_min;
        int init_ground_height_max;
        int init_ground_width_min;
        int init_ground_width_max;

        // check initial ground is ground
        float init_ground_cx_min;
        float init_ground_cx_max;
        float init_ground_cy_min;
        float init_ground_cy_max;
        float init_ground_cz_min;
        float init_ground_cz_max;

        // ground region grow to intersect vertical region

        float init_ground_nz_min;

        // search region
        int ray_height_min;
        int ray_height_max;
        int ray_width_min;
        int ray_width_max;


        // detect vertical plane, pallet or cargo or wall, or outlier shadow
        int scan_width_step;
        // find stable center[cx,cy,cz],reject outlier

        // adaptive thresh
        float adaptive_x_min;
        float adaptive_x_max;
        float adaptive_y_min;
        float adaptive_y_max;
        float adaptive_z_min;
        float adaptive_z_max;

        float far_uncertain_z_max;
        float far_uncertain_adaptive_z_max;
        float far_uncertain_x_change_min;
        int far_uncertain_row;


    };
    /// GEN[TOML]
    struct VerticalFilter{
        int output_mode;
        // 0 height min to max
        // 1 height max to min
        // 2 width min to max
        // 3 width max to min

        int search_direction;

        int init_valid_num_min;
        int init_row_valid_num_min;

        // initial center region
        int init_center_height_min;
        int init_center_height_max;
        int init_center_width_min;
        int init_center_width_max;

        // continuous mean window
        int init_center_mean_window_len;
        int init_center_mean_search_len;
        int init_center_line_search_start_index;
        // center to line dist max
        float init_center_line_center_dist_max;
        // line dist min
        float init_center_line_length_min;

        float init_center_mean_window_jx_max;
        float init_center_mean_window_jy_max;
        float init_center_mean_window_jz_max;

        // use pointcloud_filter_alone_line to filter points
        float init_center_line_filter_dist;
        int init_center_line_filter_relative_row;
        int init_center_line_filter_early_stop_num_change;

        float init_center_line_filter_len_step;
        float init_center_line_filter_len_valid_min;
        float init_center_line_filter_len_search_min;
        float init_center_line_filter_len_search_max;


        // filter range
        float init_center_cx_min;
        float init_center_cx_max;
        float init_center_cy_min;
        float init_center_cy_max;
        float init_center_cz_min;
        float init_center_cz_max;

        float init_center_jx_max;
        float init_center_jy_max;
        float init_center_jz_max;




        //
        float pallet_edge_z_min;
        float pallet_edge_z_max;


    };

    /// GEN[TOML]
    struct DetectorConfig{
        GroundFilter filter_ground;
        VerticalFilter filter_vertical;

        Extrinsic extrinsic;
        CloudConfig cloud;
    };
}

#endif //LIBROSCPP_PALLET_DETECT_CONFIG_HPP
