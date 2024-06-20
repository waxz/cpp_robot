//
// Created by waxz on 5/21/24.
//

#ifndef LIBROSCPP_POINTCLOUD_PALLET_DETECT_IMPL_H
#define LIBROSCPP_POINTCLOUD_PALLET_DETECT_IMPL_H
#include "tinyalloc/tinyalloc.h"
#include "common/c_style.h"
#include "config/pallet_detect_config_gen.hpp"
#include "pointcloud_pallet_detect.h"
#include "math/geometry/point_vector3.h"
#include "math/transform/eigen_transform.h"

#include <list>
#include <set>
#include <forward_list>
namespace perception{

    struct StableFilter{
        f32_t resolution = 0.01f;

    };

    int compute_points_vec_norm2d(std::vector<geometry::float3> data[], size_t data_len, float vx, float vy, float & cx, float &cy, float &cz, float &nx, float& ny, float& nz, float& nd);

    struct LineMark{
        geometry::float3 point_left;
        geometry::float3 point_center;
        geometry::float3 point_right;
        geometry::float3 dir_left_to_right;
        geometry::float3 intersection_to_a_axis;

        geometry::float3 filtered_line_center;
        geometry::float3 filtered_line_dir;

        u32_t index_center;
        u32_t index_left;
        u32_t index_right;

        int valid_status;

        u32_t raw_index_center[2];
        u32_t raw_index_left[2];
        u32_t raw_index_right[2];

    };

    struct PalletCandidate{

//        LineMark line;
        u32_t raw_index_center[2];

        geometry::float3 pallet_center;
        f32_t pallet_direction;
        //
         Eigen::Transform<double,3,Eigen::Isometry> pallet_pose;
         Eigen::Transform<double,3,Eigen::Isometry> pallet_pose_inv;
         Eigen::Transform<double,3,Eigen::Isometry> pallet_pose_in_start;

        int valid_status;

        int cluster_id;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct PalletCluster{
        std::vector<PalletCandidate,Eigen::aligned_allocator<PalletCandidate>> candidates;

         Eigen::Transform<double,3,Eigen::Isometry> est_pose;
         Eigen::Transform<double,3,Eigen::Isometry> est_pose_inv;

        Eigen::Transform<double,3,Eigen::Isometry> est_pose_project;
        Eigen::Transform<double,3,Eigen::Isometry> est_pose_project_inv;

        i32_t valid_status;
        f32_t refined_y;
        f32_t continuous_len;
        f32_t confidence;
        geometry::float3 pallet_pose_center;
        f32_t pallet_pose_yaw;


        u32_t top_row_high;
        u32_t top_row_low;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct LineMarkMatcherPair{
        int left_id;
        int right_id;
        int accumulate_count;
        // valid
        int valid_status;
        // line direction vec
        geometry::float3 line_direction_vec;
        // line direction yaw = atan2(y,x)
        float line_direction_xy;

        // line_direction_vec intersect with the x-axis
        geometry::float3 intersection_point;

    };
    struct LineMarkMatcher{
        // left_id, right_id, accumulate_count
        std::vector <LineMarkMatcherPair> marker_pairs;
        float intersect_dist = 0.02f;
        float direction_yaw_diff = 0.02f;

        int accumulate_count_min = 1;

        void clear();
        void add_marker();
        bool match();
        void set(float intersect_dist_, int accumulate_count_min_);
        LineMarkMatcher()=default;

    };

    struct Cluster{

        f32_t center[3] = {0.0,0.0,0.0};
        std::vector<u64_t> index;
        Cluster()=default;

    };

    struct PalletDetector{

        PalletDetector()=default;
        perception::DetectorConfig config;
        ta_cfg_t mem_cfg = {0};
        PointCloudBuffer output_cloud_buffer;
        PalletInfoBuffer output_pallet_info_buffer;
        std::vector<PalletInfo> output_pallet_vec;
//        std::vector<PalletInfo_ptr > output_pallet_ptr_vec;

        f32_t * ground_init_buffer = nullptr;
        f32_t * ground_output_buffer = nullptr;
        i32_t * ground_pixel_count_buffer = nullptr;
        f32_t * vertical_center_buffer = nullptr;
        f32_t * vertical_output_buffer = nullptr;
        u32_t * vertical_center_index_buffer = nullptr;
        u32_t * vertical_center_index_row_valid_num_buffer = nullptr;
        u32_t * vertical_center_vertical_index_buffer = nullptr;
        u64_t * vertical_filter_index_buffer = nullptr;
        u32_t * vertical_output_index_buffer = nullptr;

        u32_t * vertical_line_continuous_count_buffer = nullptr;
        f32_t * vertical_line_continuous_mean_buffer = nullptr;

//        u32_t * vertical_line_continuous_right_buffer;

        f32_t * pallet_pocket_buffer = nullptr;
        f32_t * pallet_center_line_buffer = nullptr;
        u32_t * pallet_space_continuous_count_buffer = nullptr;
        f32_t * pallet_space_continuous_mean_buffer = nullptr;

        f32_t * pallet_project_buffer = nullptr;
        u32_t * pallet_project_index_buffer = nullptr;
        f32_t * pallet_project_template_score_buffer = nullptr;
        u8_t* projector_output_img_buffer = nullptr;
        std::vector<unsigned char> projector_output_img_encode_buffer;

        f32_t * pallet_output_buffer = nullptr;


//        std::list<u64_t> vertical_filter_index_list;
        std::vector<u64_t> vertical_filter_index_vec;

//        std::set<std::array<u64_t ,2>> pallet_projector_index_buffer_list;
        std::vector<std::array<i32_t,2>> pallet_projector_index_buffer_vec;

        std::vector<LineMark> center_line_markers;
        std::vector<PalletCandidate,Eigen::aligned_allocator<PalletCandidate>> pallet_candidates;
        std::vector<PalletCluster,Eigen::aligned_allocator<PalletCluster> > pallet_cluster;

        std::vector<geometry::float3> pallet_space_left;
        std::vector<geometry::float3> pallet_pocket_left;
        std::vector<geometry::float3> pallet_space_center;
        std::vector<geometry::float3> pallet_pocket_right;
        std::vector<geometry::float3> pallet_space_right;

        std::vector<geometry::float3> pallet_top_line;

        // 0: not define
        // 1: ground
        // -1 noise
        // 2 vertical
        // 3: pallet
        // 4: cargo
        i8_t * cloud_label_table = nullptr;

        int create(const char* filename, const ta_cfg_t* cfg);
        void stop();
        f32_t *cloud_buffer = nullptr;
        f32_t viewpoint_x = 0.0;
        f32_t viewpoint_y = 0.0;
        f32_t viewpoint_z = 0.0;
        void set_input( f32_t * buffer,u64_t height, u64_t width, f32_t vx, f32_t vy, f32_t vz);
        void reset();

        void set_ground_init_dim( u64_t height_min , u64_t height_max ,u64_t width_min, u64_t width_max);
        void set_ground_init_thresh( f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max, f32_t nz_min);
        void set_ground_adaptive_thresh(  f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max);
        void set_ground_uncertain_thresh(  f32_t far_uncertain_z_max, f32_t far_uncertain_x_change_min,f32_t far_uncertain_adaptive_z_max,i32_t far_uncertain_row);

        void set_vertical_init_dim( u64_t height_min , u64_t height_max ,u64_t width_min, u64_t width_max);
        void set_vertical_init_thresh( f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max,  f32_t jx_max, f32_t jy_max, f32_t jz_max );

        void set_pallet_row(i32_t row_high, i32_t row_low);
        void set_pallet_thresh( f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max,  f32_t jx_max, f32_t jy_max, f32_t jz_max);

//        std::unordered_map<u64_t, std::vector<Cluster>> ray_filter;

        i8_t filter_ground_status = 0;
        PointCloudBuffer_ptr filter_ground(u32_t output_mode);
        i8_t filter_vertical_status = 0;
        PointCloudBuffer_ptr filter_vertical(u32_t output_mode);
        i8_t filter_pallet_status = 0;

        PointCloudBuffer_ptr filter_pallet(u32_t output_mode);
        PalletInfoBuffer_ptr  get_pallet(u32_t output_mode);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };
}


#endif //LIBROSCPP_POINTCLOUD_PALLET_DETECT_IMPL_H
