#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include <toml.hpp>
namespace perception{
/// GEN[TOML]
struct CloudDimConfig{
     int height;
     int width;
 

    explicit CloudDimConfig(const toml::value& value) {
        height= toml::get<decltype(height)>(value.at("height"));
        width= toml::get<decltype(width)>(value.at("width"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"height", this->height},
        {"width", this->width}};
    }
   CloudDimConfig() = default;

};
}
namespace perception{
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
 

    explicit CloudFilterConfig(const toml::value& value) {
        enable_mean_window= toml::get<decltype(enable_mean_window)>(value.at("enable_mean_window"));
        mean_window_len= toml::get<decltype(mean_window_len)>(value.at("mean_window_len"));
        mean_window_jump_max= toml::get<decltype(mean_window_jump_max)>(value.at("mean_window_jump_max"));
        enable_dim= toml::get<decltype(enable_dim)>(value.at("enable_dim"));
        filter_height_min= toml::get<decltype(filter_height_min)>(value.at("filter_height_min"));
        filter_height_max= toml::get<decltype(filter_height_max)>(value.at("filter_height_max"));
        filter_width_min= toml::get<decltype(filter_width_min)>(value.at("filter_width_min"));
        filter_width_max= toml::get<decltype(filter_width_max)>(value.at("filter_width_max"));
        enable_range= toml::get<decltype(enable_range)>(value.at("enable_range"));
        filter_x_min= toml::get<decltype(filter_x_min)>(value.at("filter_x_min"));
        filter_y_min= toml::get<decltype(filter_y_min)>(value.at("filter_y_min"));
        filter_z_min= toml::get<decltype(filter_z_min)>(value.at("filter_z_min"));
        filter_x_max= toml::get<decltype(filter_x_max)>(value.at("filter_x_max"));
        filter_y_max= toml::get<decltype(filter_y_max)>(value.at("filter_y_max"));
        filter_z_max= toml::get<decltype(filter_z_max)>(value.at("filter_z_max"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"enable_mean_window", this->enable_mean_window},
        {"mean_window_len", this->mean_window_len},
        {"mean_window_jump_max", this->mean_window_jump_max},
        {"enable_dim", this->enable_dim},
        {"filter_height_min", this->filter_height_min},
        {"filter_height_max", this->filter_height_max},
        {"filter_width_min", this->filter_width_min},
        {"filter_width_max", this->filter_width_max},
        {"enable_range", this->enable_range},
        {"filter_x_min", this->filter_x_min},
        {"filter_y_min", this->filter_y_min},
        {"filter_z_min", this->filter_z_min},
        {"filter_x_max", this->filter_x_max},
        {"filter_y_max", this->filter_y_max},
        {"filter_z_max", this->filter_z_max}};
    }
   CloudFilterConfig() = default;

};
}
namespace perception{
/// GEN[TOML]
struct CloudConfig{
     CloudDimConfig dim;
     CloudFilterConfig filter;
 

    explicit CloudConfig(const toml::value& value) {
        dim= toml::get<decltype(dim)>(value.at("dim"));
        filter= toml::get<decltype(filter)>(value.at("filter"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"dim", this->dim},
        {"filter", this->filter}};
    }
   CloudConfig() = default;

};
}
namespace perception{
/// GEN[TOML]
struct Pose{
     float tx;
     float ty;
     float tz;
     float roll;
     float pitch;
     float yaw;
 

    explicit Pose(const toml::value& value) {
        tx= toml::get<decltype(tx)>(value.at("tx"));
        ty= toml::get<decltype(ty)>(value.at("ty"));
        tz= toml::get<decltype(tz)>(value.at("tz"));
        roll= toml::get<decltype(roll)>(value.at("roll"));
        pitch= toml::get<decltype(pitch)>(value.at("pitch"));
        yaw= toml::get<decltype(yaw)>(value.at("yaw"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"tx", this->tx},
        {"ty", this->ty},
        {"tz", this->tz},
        {"roll", this->roll},
        {"pitch", this->pitch},
        {"yaw", this->yaw}};
    }
   Pose() = default;

};
}
namespace perception{
/// GEN[TOML]
struct Extrinsic{
     bool enable;
     Pose pose;
 

    explicit Extrinsic(const toml::value& value) {
        enable= toml::get<decltype(enable)>(value.at("enable"));
        pose= toml::get<decltype(pose)>(value.at("pose"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"enable", this->enable},
        {"pose", this->pose}};
    }
   Extrinsic() = default;

};
}
namespace perception{
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
 

    explicit GroundFilter(const toml::value& value) {
        output_mode= toml::get<decltype(output_mode)>(value.at("output_mode"));
        search_direction= toml::get<decltype(search_direction)>(value.at("search_direction"));
        init_valid_num_min= toml::get<decltype(init_valid_num_min)>(value.at("init_valid_num_min"));
        init_ground_height_min= toml::get<decltype(init_ground_height_min)>(value.at("init_ground_height_min"));
        init_ground_height_max= toml::get<decltype(init_ground_height_max)>(value.at("init_ground_height_max"));
        init_ground_width_min= toml::get<decltype(init_ground_width_min)>(value.at("init_ground_width_min"));
        init_ground_width_max= toml::get<decltype(init_ground_width_max)>(value.at("init_ground_width_max"));
        init_ground_cx_min= toml::get<decltype(init_ground_cx_min)>(value.at("init_ground_cx_min"));
        init_ground_cx_max= toml::get<decltype(init_ground_cx_max)>(value.at("init_ground_cx_max"));
        init_ground_cy_min= toml::get<decltype(init_ground_cy_min)>(value.at("init_ground_cy_min"));
        init_ground_cy_max= toml::get<decltype(init_ground_cy_max)>(value.at("init_ground_cy_max"));
        init_ground_cz_min= toml::get<decltype(init_ground_cz_min)>(value.at("init_ground_cz_min"));
        init_ground_cz_max= toml::get<decltype(init_ground_cz_max)>(value.at("init_ground_cz_max"));
        init_ground_nz_min= toml::get<decltype(init_ground_nz_min)>(value.at("init_ground_nz_min"));
        ray_height_min= toml::get<decltype(ray_height_min)>(value.at("ray_height_min"));
        ray_height_max= toml::get<decltype(ray_height_max)>(value.at("ray_height_max"));
        ray_width_min= toml::get<decltype(ray_width_min)>(value.at("ray_width_min"));
        ray_width_max= toml::get<decltype(ray_width_max)>(value.at("ray_width_max"));
        scan_width_step= toml::get<decltype(scan_width_step)>(value.at("scan_width_step"));
        adaptive_x_min= toml::get<decltype(adaptive_x_min)>(value.at("adaptive_x_min"));
        adaptive_x_max= toml::get<decltype(adaptive_x_max)>(value.at("adaptive_x_max"));
        adaptive_y_min= toml::get<decltype(adaptive_y_min)>(value.at("adaptive_y_min"));
        adaptive_y_max= toml::get<decltype(adaptive_y_max)>(value.at("adaptive_y_max"));
        adaptive_z_min= toml::get<decltype(adaptive_z_min)>(value.at("adaptive_z_min"));
        adaptive_z_max= toml::get<decltype(adaptive_z_max)>(value.at("adaptive_z_max"));
        far_uncertain_z_max= toml::get<decltype(far_uncertain_z_max)>(value.at("far_uncertain_z_max"));
        far_uncertain_adaptive_z_max= toml::get<decltype(far_uncertain_adaptive_z_max)>(value.at("far_uncertain_adaptive_z_max"));
        far_uncertain_x_change_min= toml::get<decltype(far_uncertain_x_change_min)>(value.at("far_uncertain_x_change_min"));
        far_uncertain_row= toml::get<decltype(far_uncertain_row)>(value.at("far_uncertain_row"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"output_mode", this->output_mode},
        {"search_direction", this->search_direction},
        {"init_valid_num_min", this->init_valid_num_min},
        {"init_ground_height_min", this->init_ground_height_min},
        {"init_ground_height_max", this->init_ground_height_max},
        {"init_ground_width_min", this->init_ground_width_min},
        {"init_ground_width_max", this->init_ground_width_max},
        {"init_ground_cx_min", this->init_ground_cx_min},
        {"init_ground_cx_max", this->init_ground_cx_max},
        {"init_ground_cy_min", this->init_ground_cy_min},
        {"init_ground_cy_max", this->init_ground_cy_max},
        {"init_ground_cz_min", this->init_ground_cz_min},
        {"init_ground_cz_max", this->init_ground_cz_max},
        {"init_ground_nz_min", this->init_ground_nz_min},
        {"ray_height_min", this->ray_height_min},
        {"ray_height_max", this->ray_height_max},
        {"ray_width_min", this->ray_width_min},
        {"ray_width_max", this->ray_width_max},
        {"scan_width_step", this->scan_width_step},
        {"adaptive_x_min", this->adaptive_x_min},
        {"adaptive_x_max", this->adaptive_x_max},
        {"adaptive_y_min", this->adaptive_y_min},
        {"adaptive_y_max", this->adaptive_y_max},
        {"adaptive_z_min", this->adaptive_z_min},
        {"adaptive_z_max", this->adaptive_z_max},
        {"far_uncertain_z_max", this->far_uncertain_z_max},
        {"far_uncertain_adaptive_z_max", this->far_uncertain_adaptive_z_max},
        {"far_uncertain_x_change_min", this->far_uncertain_x_change_min},
        {"far_uncertain_row", this->far_uncertain_row}};
    }
   GroundFilter() = default;

};
}
namespace perception{
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
 

    explicit PalletFilter(const toml::value& value) {
        output_mode= toml::get<decltype(output_mode)>(value.at("output_mode"));
        search_direction= toml::get<decltype(search_direction)>(value.at("search_direction"));
        cluster_filter_split_last_dist= toml::get<decltype(cluster_filter_split_last_dist)>(value.at("cluster_filter_split_last_dist"));
        cluster_filter_split_start_dist= toml::get<decltype(cluster_filter_split_start_dist)>(value.at("cluster_filter_split_start_dist"));
        cluster_filter_split_last_z= toml::get<decltype(cluster_filter_split_last_z)>(value.at("cluster_filter_split_last_z"));
        cluster_filter_split_start_z= toml::get<decltype(cluster_filter_split_start_z)>(value.at("cluster_filter_split_start_z"));
        cluster_filter_split_last_x= toml::get<decltype(cluster_filter_split_last_x)>(value.at("cluster_filter_split_last_x"));
        cluster_filter_split_start_x= toml::get<decltype(cluster_filter_split_start_x)>(value.at("cluster_filter_split_start_x"));
        cluster_filter_split_last_yaw= toml::get<decltype(cluster_filter_split_last_yaw)>(value.at("cluster_filter_split_last_yaw"));
        cluster_filter_split_start_yaw= toml::get<decltype(cluster_filter_split_start_yaw)>(value.at("cluster_filter_split_start_yaw"));
        cluster_filter_pose_weight= toml::get<decltype(cluster_filter_pose_weight)>(value.at("cluster_filter_pose_weight"));
        cluster_filter_num_min= toml::get<decltype(cluster_filter_num_min)>(value.at("cluster_filter_num_min"));
        filter_pallet_row_high= toml::get<decltype(filter_pallet_row_high)>(value.at("filter_pallet_row_high"));
        filter_pallet_row_low= toml::get<decltype(filter_pallet_row_low)>(value.at("filter_pallet_row_low"));
        filter_pallet_x_min= toml::get<decltype(filter_pallet_x_min)>(value.at("filter_pallet_x_min"));
        filter_pallet_x_max= toml::get<decltype(filter_pallet_x_max)>(value.at("filter_pallet_x_max"));
        filter_pallet_y_min= toml::get<decltype(filter_pallet_y_min)>(value.at("filter_pallet_y_min"));
        filter_pallet_y_max= toml::get<decltype(filter_pallet_y_max)>(value.at("filter_pallet_y_max"));
        filter_pallet_z_min= toml::get<decltype(filter_pallet_z_min)>(value.at("filter_pallet_z_min"));
        filter_pallet_z_max= toml::get<decltype(filter_pallet_z_max)>(value.at("filter_pallet_z_max"));
        filter_pallet_z_pocket= toml::get<decltype(filter_pallet_z_pocket)>(value.at("filter_pallet_z_pocket"));
        filter_pallet_jx= toml::get<decltype(filter_pallet_jx)>(value.at("filter_pallet_jx"));
        filter_pallet_jy= toml::get<decltype(filter_pallet_jy)>(value.at("filter_pallet_jy"));
        filter_pallet_jz= toml::get<decltype(filter_pallet_jz)>(value.at("filter_pallet_jz"));
        filter_space_continuous_dist= toml::get<decltype(filter_space_continuous_dist)>(value.at("filter_space_continuous_dist"));
        filter_space_continuous_num= toml::get<decltype(filter_space_continuous_num)>(value.at("filter_space_continuous_num"));
        filter_space_continuous_thresh= toml::get<decltype(filter_space_continuous_thresh)>(value.at("filter_space_continuous_thresh"));
        pallet_space_direction_diff_max= toml::get<decltype(pallet_space_direction_diff_max)>(value.at("pallet_space_direction_diff_max"));
        pallet_space_center_to_line_dist_max= toml::get<decltype(pallet_space_center_to_line_dist_max)>(value.at("pallet_space_center_to_line_dist_max"));
        pallet_pocket_empty_x= toml::get<decltype(pallet_pocket_empty_x)>(value.at("pallet_pocket_empty_x"));
        pallet_space_valid_num= toml::get<decltype(pallet_space_valid_num)>(value.at("pallet_space_valid_num"));
        fork_shape_width= toml::get<decltype(fork_shape_width)>(value.at("fork_shape_width"));
        fork_shape_height= toml::get<decltype(fork_shape_height)>(value.at("fork_shape_height"));
        fork_pos_y= toml::get<decltype(fork_pos_y)>(value.at("fork_pos_y"));
        pallet_space_width_left= toml::get<decltype(pallet_space_width_left)>(value.at("pallet_space_width_left"));
        pallet_pocket_width= toml::get<decltype(pallet_pocket_width)>(value.at("pallet_pocket_width"));
        pallet_space_width_center= toml::get<decltype(pallet_space_width_center)>(value.at("pallet_space_width_center"));
        pallet_space_height= toml::get<decltype(pallet_space_height)>(value.at("pallet_space_height"));
        pallet_top_height= toml::get<decltype(pallet_top_height)>(value.at("pallet_top_height"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"output_mode", this->output_mode},
        {"search_direction", this->search_direction},
        {"cluster_filter_split_last_dist", this->cluster_filter_split_last_dist},
        {"cluster_filter_split_start_dist", this->cluster_filter_split_start_dist},
        {"cluster_filter_split_last_z", this->cluster_filter_split_last_z},
        {"cluster_filter_split_start_z", this->cluster_filter_split_start_z},
        {"cluster_filter_split_last_x", this->cluster_filter_split_last_x},
        {"cluster_filter_split_start_x", this->cluster_filter_split_start_x},
        {"cluster_filter_split_last_yaw", this->cluster_filter_split_last_yaw},
        {"cluster_filter_split_start_yaw", this->cluster_filter_split_start_yaw},
        {"cluster_filter_pose_weight", this->cluster_filter_pose_weight},
        {"cluster_filter_num_min", this->cluster_filter_num_min},
        {"filter_pallet_row_high", this->filter_pallet_row_high},
        {"filter_pallet_row_low", this->filter_pallet_row_low},
        {"filter_pallet_x_min", this->filter_pallet_x_min},
        {"filter_pallet_x_max", this->filter_pallet_x_max},
        {"filter_pallet_y_min", this->filter_pallet_y_min},
        {"filter_pallet_y_max", this->filter_pallet_y_max},
        {"filter_pallet_z_min", this->filter_pallet_z_min},
        {"filter_pallet_z_max", this->filter_pallet_z_max},
        {"filter_pallet_z_pocket", this->filter_pallet_z_pocket},
        {"filter_pallet_jx", this->filter_pallet_jx},
        {"filter_pallet_jy", this->filter_pallet_jy},
        {"filter_pallet_jz", this->filter_pallet_jz},
        {"filter_space_continuous_dist", this->filter_space_continuous_dist},
        {"filter_space_continuous_num", this->filter_space_continuous_num},
        {"filter_space_continuous_thresh", this->filter_space_continuous_thresh},
        {"pallet_space_direction_diff_max", this->pallet_space_direction_diff_max},
        {"pallet_space_center_to_line_dist_max", this->pallet_space_center_to_line_dist_max},
        {"pallet_pocket_empty_x", this->pallet_pocket_empty_x},
        {"pallet_space_valid_num", this->pallet_space_valid_num},
        {"fork_shape_width", this->fork_shape_width},
        {"fork_shape_height", this->fork_shape_height},
        {"fork_pos_y", this->fork_pos_y},
        {"pallet_space_width_left", this->pallet_space_width_left},
        {"pallet_pocket_width", this->pallet_pocket_width},
        {"pallet_space_width_center", this->pallet_space_width_center},
        {"pallet_space_height", this->pallet_space_height},
        {"pallet_top_height", this->pallet_top_height}};
    }
   PalletFilter() = default;

};
}
namespace perception{
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
 

    explicit VerticalFilter(const toml::value& value) {
        output_mode= toml::get<decltype(output_mode)>(value.at("output_mode"));
        search_direction= toml::get<decltype(search_direction)>(value.at("search_direction"));
        init_valid_num_min= toml::get<decltype(init_valid_num_min)>(value.at("init_valid_num_min"));
        init_row_valid_num_min= toml::get<decltype(init_row_valid_num_min)>(value.at("init_row_valid_num_min"));
        init_center_height_min= toml::get<decltype(init_center_height_min)>(value.at("init_center_height_min"));
        init_center_height_max= toml::get<decltype(init_center_height_max)>(value.at("init_center_height_max"));
        init_center_width_min= toml::get<decltype(init_center_width_min)>(value.at("init_center_width_min"));
        init_center_width_max= toml::get<decltype(init_center_width_max)>(value.at("init_center_width_max"));
        init_center_mean_window_len= toml::get<decltype(init_center_mean_window_len)>(value.at("init_center_mean_window_len"));
        init_center_mean_search_len= toml::get<decltype(init_center_mean_search_len)>(value.at("init_center_mean_search_len"));
        init_center_line_search_start_index= toml::get<decltype(init_center_line_search_start_index)>(value.at("init_center_line_search_start_index"));
        init_center_line_center_dist_max= toml::get<decltype(init_center_line_center_dist_max)>(value.at("init_center_line_center_dist_max"));
        init_center_line_length_min= toml::get<decltype(init_center_line_length_min)>(value.at("init_center_line_length_min"));
        init_center_mean_window_jx_max= toml::get<decltype(init_center_mean_window_jx_max)>(value.at("init_center_mean_window_jx_max"));
        init_center_mean_window_jy_max= toml::get<decltype(init_center_mean_window_jy_max)>(value.at("init_center_mean_window_jy_max"));
        init_center_mean_window_jz_max= toml::get<decltype(init_center_mean_window_jz_max)>(value.at("init_center_mean_window_jz_max"));
        init_center_line_filter_dist= toml::get<decltype(init_center_line_filter_dist)>(value.at("init_center_line_filter_dist"));
        init_center_line_filter_relative_row= toml::get<decltype(init_center_line_filter_relative_row)>(value.at("init_center_line_filter_relative_row"));
        init_center_line_filter_early_stop_num_change= toml::get<decltype(init_center_line_filter_early_stop_num_change)>(value.at("init_center_line_filter_early_stop_num_change"));
        init_center_line_filter_len_step= toml::get<decltype(init_center_line_filter_len_step)>(value.at("init_center_line_filter_len_step"));
        init_center_line_filter_continuous_valid_resolution= toml::get<decltype(init_center_line_filter_continuous_valid_resolution)>(value.at("init_center_line_filter_continuous_valid_resolution"));
        init_center_line_filter_continuous_buffer_len= toml::get<decltype(init_center_line_filter_continuous_buffer_len)>(value.at("init_center_line_filter_continuous_buffer_len"));
        init_center_line_filter_continuous_len_min= toml::get<decltype(init_center_line_filter_continuous_len_min)>(value.at("init_center_line_filter_continuous_len_min"));
        init_center_line_filter_continuous_len_max= toml::get<decltype(init_center_line_filter_continuous_len_max)>(value.at("init_center_line_filter_continuous_len_max"));
        init_center_line_filter_len_valid_min= toml::get<decltype(init_center_line_filter_len_valid_min)>(value.at("init_center_line_filter_len_valid_min"));
        init_center_line_filter_len_search_min= toml::get<decltype(init_center_line_filter_len_search_min)>(value.at("init_center_line_filter_len_search_min"));
        init_center_line_filter_len_search_max= toml::get<decltype(init_center_line_filter_len_search_max)>(value.at("init_center_line_filter_len_search_max"));
        init_center_cx_min= toml::get<decltype(init_center_cx_min)>(value.at("init_center_cx_min"));
        init_center_cx_max= toml::get<decltype(init_center_cx_max)>(value.at("init_center_cx_max"));
        init_center_cy_min= toml::get<decltype(init_center_cy_min)>(value.at("init_center_cy_min"));
        init_center_cy_max= toml::get<decltype(init_center_cy_max)>(value.at("init_center_cy_max"));
        init_center_cz_min= toml::get<decltype(init_center_cz_min)>(value.at("init_center_cz_min"));
        init_center_cz_max= toml::get<decltype(init_center_cz_max)>(value.at("init_center_cz_max"));
        init_center_jx_max= toml::get<decltype(init_center_jx_max)>(value.at("init_center_jx_max"));
        init_center_jy_max= toml::get<decltype(init_center_jy_max)>(value.at("init_center_jy_max"));
        init_center_jz_max= toml::get<decltype(init_center_jz_max)>(value.at("init_center_jz_max"));
        pallet_edge_z_min= toml::get<decltype(pallet_edge_z_min)>(value.at("pallet_edge_z_min"));
        pallet_edge_z_max= toml::get<decltype(pallet_edge_z_max)>(value.at("pallet_edge_z_max"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"output_mode", this->output_mode},
        {"search_direction", this->search_direction},
        {"init_valid_num_min", this->init_valid_num_min},
        {"init_row_valid_num_min", this->init_row_valid_num_min},
        {"init_center_height_min", this->init_center_height_min},
        {"init_center_height_max", this->init_center_height_max},
        {"init_center_width_min", this->init_center_width_min},
        {"init_center_width_max", this->init_center_width_max},
        {"init_center_mean_window_len", this->init_center_mean_window_len},
        {"init_center_mean_search_len", this->init_center_mean_search_len},
        {"init_center_line_search_start_index", this->init_center_line_search_start_index},
        {"init_center_line_center_dist_max", this->init_center_line_center_dist_max},
        {"init_center_line_length_min", this->init_center_line_length_min},
        {"init_center_mean_window_jx_max", this->init_center_mean_window_jx_max},
        {"init_center_mean_window_jy_max", this->init_center_mean_window_jy_max},
        {"init_center_mean_window_jz_max", this->init_center_mean_window_jz_max},
        {"init_center_line_filter_dist", this->init_center_line_filter_dist},
        {"init_center_line_filter_relative_row", this->init_center_line_filter_relative_row},
        {"init_center_line_filter_early_stop_num_change", this->init_center_line_filter_early_stop_num_change},
        {"init_center_line_filter_len_step", this->init_center_line_filter_len_step},
        {"init_center_line_filter_continuous_valid_resolution", this->init_center_line_filter_continuous_valid_resolution},
        {"init_center_line_filter_continuous_buffer_len", this->init_center_line_filter_continuous_buffer_len},
        {"init_center_line_filter_continuous_len_min", this->init_center_line_filter_continuous_len_min},
        {"init_center_line_filter_continuous_len_max", this->init_center_line_filter_continuous_len_max},
        {"init_center_line_filter_len_valid_min", this->init_center_line_filter_len_valid_min},
        {"init_center_line_filter_len_search_min", this->init_center_line_filter_len_search_min},
        {"init_center_line_filter_len_search_max", this->init_center_line_filter_len_search_max},
        {"init_center_cx_min", this->init_center_cx_min},
        {"init_center_cx_max", this->init_center_cx_max},
        {"init_center_cy_min", this->init_center_cy_min},
        {"init_center_cy_max", this->init_center_cy_max},
        {"init_center_cz_min", this->init_center_cz_min},
        {"init_center_cz_max", this->init_center_cz_max},
        {"init_center_jx_max", this->init_center_jx_max},
        {"init_center_jy_max", this->init_center_jy_max},
        {"init_center_jz_max", this->init_center_jz_max},
        {"pallet_edge_z_min", this->pallet_edge_z_min},
        {"pallet_edge_z_max", this->pallet_edge_z_max}};
    }
   VerticalFilter() = default;

};
}
namespace perception{
/// GEN[TOML]
struct DetectorConfig{
     GroundFilter filter_ground;
     VerticalFilter filter_vertical;
     PalletFilter filter_pallet;
     Extrinsic extrinsic;
     CloudConfig cloud;
 

    explicit DetectorConfig(const toml::value& value) {
        filter_ground= toml::get<decltype(filter_ground)>(value.at("filter_ground"));
        filter_vertical= toml::get<decltype(filter_vertical)>(value.at("filter_vertical"));
        filter_pallet= toml::get<decltype(filter_pallet)>(value.at("filter_pallet"));
        extrinsic= toml::get<decltype(extrinsic)>(value.at("extrinsic"));
        cloud= toml::get<decltype(cloud)>(value.at("cloud"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"filter_ground", this->filter_ground},
        {"filter_vertical", this->filter_vertical},
        {"filter_pallet", this->filter_pallet},
        {"extrinsic", this->extrinsic},
        {"cloud", this->cloud}};
    }
   DetectorConfig() = default;

};
}
