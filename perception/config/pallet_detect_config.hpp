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
        bool enable_mean_window;
        int mean_window_len;
        float mean_window_jump_max;

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
    struct PalletFilter{
        int output_mode;
        // 0 height min to max
        // 1 height max to min
        // 2 width min to max
        // 3 width max to min

        int search_direction;

        // split candidates to cluster
        float cluster_filter_split_last_dist;
        float cluster_filter_split_start_dist;

        float cluster_filter_split_last_z;
        float cluster_filter_split_start_z;
        float cluster_filter_split_last_x;
        float cluster_filter_split_start_x;

        float cluster_filter_split_last_yaw;
        float cluster_filter_split_start_yaw;

        float cluster_filter_pose_weight;


        int cluster_filter_num_min;

        // filter pocket region
        // check pocket hole existence
        int filter_pallet_row_high;
        int filter_pallet_row_low;

        // filter center line
        float filter_pallet_line_x_min;
        float filter_pallet_line_x_max;
        float filter_pallet_line_y_min;
        float filter_pallet_line_y_max;
        float filter_pallet_line_z_min;
        float filter_pallet_line_z_max;
        float filter_pallet_line_continuous_min;

        // space filter
        float filter_space_continuous_resolution;
        float filter_space_move_window_x_search_start;
        float filter_space_move_window_x_search_end;
        float filter_space_move_window_x_search_step;
        float filter_space_move_window_y_len;

        float filter_space_move_window_x_thresh;
        float filter_space_continuous_len_min;

        float filter_space_second_height_high;
        float filter_space_second_move_mean_thresh;
        float filter_space_width_slip_len;

        float projector_dir_weight_p3;
        float projector_dir_weight_p3t;
        float projector_dir_weight_t;
        float projector_dir_weight_0;

        float projector_offset_y;
        float projector_offset_z;
        float projector_resolution;
        float projector_search_y_range;
        float projector_search_z_range;
        float projector_search_maximum_distance;
        float projector_similarity_min;



        // filter
        float filter_pallet_x_min;
        float filter_pallet_x_max;
        float filter_pallet_y_min;
        float filter_pallet_y_max;
        float filter_pallet_z_min;
        float filter_pallet_z_max;
        // check between [ z_pocket, z_pocket + filter_pallet_z_pocket]
        float filter_pallet_z_pocket;

        float filter_pallet_jx;
        float filter_pallet_jy;
        float filter_pallet_jz;

        float filter_space_continuous_dist;
        int filter_space_continuous_num;
        float filter_space_continuous_thresh;
        float pallet_space_direction_diff_max;
        float pallet_space_center_to_line_dist_max;
        float pallet_pocket_empty_x;
        int pallet_space_valid_num;
        int pallet_pocket_max_num;




        // check if a line is pallet marker ot not
        // 1. check if the line continuous, compute center position
        // not real fork size, just for checking cloud shape
        float fork_shape_width;
        float fork_shape_height;
        float fork_pos_y;





        // 2. check pocket hole

        // transform cloud to pallet frame
        // filter with x,y,z limit
//        float pocket_hole_x_min;
//        float pocket_hole_x_max;
//        float pocket_hole_y_min;
//        float pocket_hole_y_max;
//        float pocket_hole_z_min;
//        float pocket_hole_z_max;



        // filter for each pixel
//        float pocket_pixel_x_min;
//        float pocket_pixel_x_max;
//        float pocket_pixel_y_min;
//        float pocket_pixel_y_max;
//        float pocket_pixel_z_min;
//        float pocket_pixel_z_max;

        //
//        float pocket_continuous_dist;
//        int pocket_pixel_valid_num;






        /*

         |----------------------------------------------|
         ph2                                            |
         |----------------------------------------------|
         |     |              |     |             |     |
         |     |              |     |             |     ph1
         |-pw1-|    -pw2-     |-pw3-|             |     |
                                 |
                                -O

         pw = 2*(pw1 + pw2) + pw3
         ph = ph1 + ph2

         */


        float pallet_space_width_left;
        float pallet_pocket_width;
        float pallet_space_width_center;
        float pallet_space_height;
        float pallet_top_height;
        float pallet_top_tail_height_ratio;


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
        // check valid line,
        // 1. continuous, resolution, fill all point to count array buffer based on resolution
        float init_center_line_filter_continuous_valid_resolution;
        float init_center_line_filter_continuous_buffer_len;
        float init_center_line_filter_continuous_len_min;
        float init_center_line_filter_continuous_len_max;


        // 2. length
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
        PalletFilter filter_pallet;

        Extrinsic extrinsic;
        CloudConfig cloud;
    };
}

#endif //LIBROSCPP_PALLET_DETECT_CONFIG_HPP
