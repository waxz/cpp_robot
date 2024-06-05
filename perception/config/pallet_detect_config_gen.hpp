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
     Extrinsic extrinsic;
     CloudConfig cloud;
 

    explicit DetectorConfig(const toml::value& value) {
        filter_ground= toml::get<decltype(filter_ground)>(value.at("filter_ground"));
        filter_vertical= toml::get<decltype(filter_vertical)>(value.at("filter_vertical"));
        extrinsic= toml::get<decltype(extrinsic)>(value.at("extrinsic"));
        cloud= toml::get<decltype(cloud)>(value.at("cloud"));

    }

    toml::value into_toml() const {
        return toml::value{
        {"filter_ground", this->filter_ground},
        {"filter_vertical", this->filter_vertical},
        {"extrinsic", this->extrinsic},
        {"cloud", this->cloud}};
    }
   DetectorConfig() = default;

};
}
