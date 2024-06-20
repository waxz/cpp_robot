//
// Created by waxz on 5/21/24.
//
#include "pointcloud_pallet_detect_impl.h"
#include "math/geometry/normal_estimation_3d.h"
#include "math/geometry/normal_estimation_2d.h"

#include "pointcloud_process.h"

#include <iostream>
#include "common/string_logger.h"
#include "common/clock_time.h"
#include "math/math_basic.h"
#include "math/geometry.h"
#include "math/transform/eigen_transform.h"

#include "lodepng.h"


namespace perception{

    int compute_points_vec_norm2d(std::vector<geometry::float3> data[], size_t data_len, float vx, float vy, float & cx, float &cy, float &cz, float &nx, float& ny, float& nz, float& nd){

        float sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        size_t sum_num = 0;
        for(size_t i = 0 ; i < data_len;i++){
            auto& v = data[i];
            for( auto&p : v){
                sum_x += p.x;
                sum_y += p.y;
                sum_z += p.z;
            }
            sum_num += v.size();
        }

        float mean_x  = sum_x/static_cast<float>(sum_num);
        float mean_y  = sum_y/static_cast<float>(sum_num);
        float mean_z  = sum_z/static_cast<float>(sum_num);
        NormalEst2d normalEst2D;
        normalEst2D.setViewerPoint(vx,vy);
        normalEst2D.addCenter(mean_x,mean_y);
        for(size_t i = 0 ; i < data_len;i++){
            auto& v = data[i];
            for( auto&p : v){
                normalEst2D.addPoint(p.x,p.y);
            }
        }
        f32_t cnx,cny,cnz,cnd ;
        normalEst2D.compute(cnx,cny,cnz,cnd );
        cx = mean_x;
        cy = mean_y;
        cz = mean_z;
        nx = cnx;
        ny = cny;
        nz = cnz;
        nd = cnd;


        return 0;
    }


    void LineMarkMatcher::clear() {

        marker_pairs.clear();
    }

    void LineMarkMatcher::set(float intersect_dist_, int accumulate_count_min_) {
        intersect_dist = intersect_dist_;
        accumulate_count_min = accumulate_count_min_;

    }
    bool LineMarkMatcher::match() {
        if(marker_pairs.empty()){
            return false;
        }

        int marker_pairs_len = marker_pairs.size();


        // check cluster
        // line direction
        // intersect point distance

        for(int i = 0 ; i <marker_pairs_len;i++ ){

            auto& p1 = marker_pairs[i];
            p1.accumulate_count = 1;
            for(int j = 0 ; j <marker_pairs_len;j++ ){


                if(i == j )continue;
                auto& p2 = marker_pairs[j];

                float dist = (p1.intersection_point - p2.intersection_point).norm();
                p2.accumulate_count = dist < intersect_dist;
            }
        }


        // filter

        for(int i = 0 ; i <marker_pairs_len;i++ ){
            auto& p1 = marker_pairs[i];
            p1.valid_status = p1.accumulate_count > accumulate_count_min;

        }
        // remove



        auto remove_it = std::remove_if(marker_pairs.begin(), marker_pairs.end(),[](auto &x){
            return x.valid_status == 0;
        });


        marker_pairs.erase(remove_it, marker_pairs.end());


        // sort

        // filter
        for(int i = 0 ; i <marker_pairs_len;i++ ){
            auto& p1 = marker_pairs[i];
            if(p1.valid_status == 0) continue;


        }

        return false;
    }

    int PalletDetector::create(const char *filename, const ta_cfg_t *cfg) {
        if(!cfg){
            std::cout << "PalletDetector::create: cfg" << cfg << std::endl;

            return -1;
        }
        mem_cfg = *cfg;


        // load toml file

        std::cout << "PalletDetector::create:" << filename << std::endl;
        toml::basic_value<toml::discard_comments, std::unordered_map, std::vector> toml_data;

        try{
            toml_data = toml::parse(filename);
        }catch (std::runtime_error & e){
            MLOGW("PalletDetector::create:toml::parse(%s) failed, error: %s\n",filename, e.what());
            return -1;
        }catch (toml::syntax_error& e){
            MLOGW("PalletDetector::create:toml::parse(%s) failed, error: %s\n",filename, e.what());
            return -1;
        }catch (...){
            MLOGW("PalletDetector::create:toml::parse(%s) failed, error: %s\n",filename, "Unknown");
            return -1;
        }

        try{
            config = perception::DetectorConfig(toml_data.at("pallet_detector"));
            toml::value config_data = config;
            std::cout << "config_data:\n" << config_data << "\n";
        }catch (std::exception& e){
            MLOGW("PalletDetector::create:toml::parse(%s) failed, error: %s\n",filename, e.what());
            return -1;
        }


        std::cout << "[PalletDetector]: configured done" << std::endl;

        ground_init_buffer = (f32_t*)ta_realloc(&mem_cfg,ground_init_buffer,1000*3*4);

        ground_output_buffer = (f32_t*)ta_realloc(&mem_cfg,ground_output_buffer,1000*3*4);

        cloud_label_table = (i8_t*) ta_realloc(&mem_cfg,cloud_label_table, 1000);

        ground_pixel_count_buffer = (i32_t*)ta_realloc(&mem_cfg,ground_pixel_count_buffer,1000);

        vertical_center_buffer = (f32_t*)ta_realloc(&mem_cfg,vertical_center_buffer,1000*3*4);
        vertical_output_buffer = (f32_t*)ta_realloc(&mem_cfg,vertical_output_buffer,1000*3*4);


        vertical_center_index_buffer = (u32_t *)ta_realloc(&mem_cfg,vertical_center_index_buffer,1000);
        vertical_center_vertical_index_buffer = (u32_t *)ta_realloc(&mem_cfg,vertical_center_vertical_index_buffer,1000);

        ;
        vertical_center_index_row_valid_num_buffer = (u32_t *)ta_realloc(&mem_cfg, vertical_center_index_row_valid_num_buffer, 1000);
        vertical_filter_index_buffer = (u64_t *)ta_realloc(&mem_cfg,vertical_filter_index_buffer,1000);
        vertical_output_index_buffer = (u32_t *)ta_realloc(&mem_cfg,vertical_output_index_buffer,1000);

        vertical_line_continuous_count_buffer =  (u32_t *)ta_realloc(&mem_cfg, vertical_line_continuous_count_buffer, 1000);

        vertical_line_continuous_mean_buffer=  (f32_t *)ta_realloc(&mem_cfg, vertical_line_continuous_mean_buffer, 1000);
        pallet_space_continuous_count_buffer =  (u32_t *)ta_realloc(&mem_cfg, pallet_space_continuous_count_buffer, 1000);

        pallet_space_continuous_mean_buffer=  (f32_t *)ta_realloc(&mem_cfg, pallet_space_continuous_mean_buffer, 1000);


        pallet_pocket_buffer = (f32_t*)ta_realloc(&mem_cfg,pallet_pocket_buffer,1000*3*4);
        pallet_output_buffer = (f32_t*)ta_realloc(&mem_cfg,pallet_output_buffer,1000*3*4);

        pallet_project_index_buffer = (u32_t *)ta_realloc(&mem_cfg,pallet_project_index_buffer,1000);

        ;
        pallet_project_template_score_buffer = (f32_t*)ta_realloc(&mem_cfg,pallet_project_template_score_buffer,1000*3*4);

        pallet_project_buffer = (f32_t*)ta_realloc(&mem_cfg,pallet_project_buffer,1000*3*4);


        pallet_center_line_buffer = (f32_t*)ta_realloc(&mem_cfg,pallet_pocket_buffer,1000*3*4);


        if( !ground_init_buffer || !ground_output_buffer){
            MLOGW("PalletDetector: create buffer fail: %p, %p", ground_init_buffer, ground_output_buffer);
            return -1;
        }
        return 0;
    }

    void PalletDetector::stop() {

    }

    void PalletDetector::reset() {

        filter_ground_status = 0;
        filter_vertical_status = 0;
        filter_pallet_status = 0 ;

    }
    void
    PalletDetector::set_input(f32_t *buffer, u64_t height,
                                       u64_t width, f32_t vx, f32_t vy, f32_t vz) {

        cloud_buffer = buffer;
        config.cloud.dim.height = height;
        config.cloud.dim.width = width;
        viewpoint_x = vx;
        viewpoint_y = vy;
        viewpoint_z = vz;

        filter_ground_status = 0;
        filter_vertical_status = 0;
        filter_pallet_status = 0 ;
    }

    void PalletDetector::set_vertical_init_dim(u64_t height_min, u64_t height_max, u64_t width_min, u64_t width_max) {
        config.filter_vertical.init_center_height_min = height_min;
        config.filter_vertical.init_center_height_max = height_max;
        config.filter_vertical.init_center_width_min = width_min;
        config.filter_vertical.init_center_width_max = width_max;
    }

    void PalletDetector::set_pallet_row(i32_t row_high, i32_t row_low) {
        config.filter_pallet.filter_pallet_row_high  = row_high;
        config.filter_pallet.filter_pallet_row_low  = row_low;

    }

    void PalletDetector::set_pallet_thresh(f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max,
                                           f32_t jx_max, f32_t jy_max, f32_t jz_max) {
        config.filter_pallet.filter_pallet_x_min = x_min;
        config.filter_pallet.filter_pallet_x_max = x_max;
        config.filter_pallet.filter_pallet_y_min = y_min;
        config.filter_pallet.filter_pallet_y_max = y_max;
        config.filter_pallet.filter_pallet_z_min = z_min;
        config.filter_pallet.filter_pallet_z_max = z_max;

        config.filter_pallet.filter_pallet_jx = jx_max;
        config.filter_pallet.filter_pallet_jy = jy_max;
        config.filter_pallet.filter_pallet_jz = jz_max;

    }

    void PalletDetector::set_vertical_init_thresh(f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min,
                                                  f32_t z_max, f32_t jx_max, f32_t jy_max, f32_t jz_max ) {
        config.filter_vertical.init_center_cx_min = x_min;
        config.filter_vertical.init_center_cx_max = x_max;
        config.filter_vertical.init_center_cy_min = y_min;
        config.filter_vertical.init_center_cy_max = y_max;
        config.filter_vertical.init_center_cz_min = z_min;
        config.filter_vertical.init_center_cz_max = z_max;
        config.filter_vertical.init_center_jx_max = jx_max;
        config.filter_vertical.init_center_jy_max = jy_max;
        config.filter_vertical.init_center_jz_max = jz_max;

    }

    void PalletDetector::set_ground_init_dim(u64_t height_min, u64_t height_max, u64_t width_min, u64_t width_max) {
        config.filter_ground.init_ground_height_min = height_min;
        config.filter_ground.init_ground_height_max = height_max;
        config.filter_ground.init_ground_width_min = width_min;
        config.filter_ground.init_ground_width_max = width_max;

    }

    void PalletDetector::set_ground_uncertain_thresh(f32_t far_uncertain_z_max, f32_t far_uncertain_x_change_min,
                                                     f32_t far_uncertain_adaptive_z_max,
                                                     i32_t far_uncertain_row) {
        config.filter_ground.far_uncertain_row = far_uncertain_row;
        config.filter_ground.far_uncertain_z_max =far_uncertain_z_max;
        config.filter_ground.far_uncertain_adaptive_z_max = far_uncertain_adaptive_z_max;
        config.filter_ground.far_uncertain_x_change_min =far_uncertain_x_change_min;

    }
    void PalletDetector::set_ground_adaptive_thresh(f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max) {

        MLOGI("set_ground_adaptive_thresh: [%f, %f], [%f, %f], [%f, %f]", x_min, x_max, y_min, y_max, z_min, z_max)
        config.filter_ground.adaptive_x_min  =x_min;
        config.filter_ground.adaptive_x_max = x_max;
        config.filter_ground.adaptive_y_min  =y_min;
        config.filter_ground.adaptive_y_max = y_max;
        config.filter_ground.adaptive_z_min  =z_min;
        config.filter_ground.adaptive_z_max = z_max;
    }
    void PalletDetector::set_ground_init_thresh(f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min,
                                                f32_t z_max, f32_t nz_min) {
        config.filter_ground.init_ground_cx_min = x_min;
        config.filter_ground.init_ground_cx_max = x_max;
        config.filter_ground.init_ground_cy_min = y_min;
        config.filter_ground.init_ground_cy_max = y_max;
        config.filter_ground.init_ground_cz_min = z_min;
        config.filter_ground.init_ground_cz_max = z_max;
        config.filter_ground.init_ground_nz_min = nz_min;

    }

    PointCloudBuffer_ptr PalletDetector::filter_vertical(u32_t output_mode) {

        //
        common::Time start_time = common::FromUnixNow();
        // computer center and norm for given initial ground window, within thresh

        int cloud_dim_height = config.cloud.dim.height;
        int cloud_dim_width = config.cloud.dim.width;
        int search_direction = config.filter_vertical.search_direction;

        int init_center_height_min = config.filter_vertical.init_center_height_min;
        int init_center_height_max = config.filter_vertical.init_center_height_max  > cloud_dim_height ? cloud_dim_height : config.filter_vertical.init_center_height_max ;

        int init_center_width_min = config.filter_vertical.init_center_width_min;
        int init_center_width_max = config.filter_vertical.init_center_height_max  > cloud_dim_width ? cloud_dim_width : config.filter_vertical.init_center_width_max ;


        int init_center_height = init_center_height_max - init_center_height_min;
        int init_center_width = init_center_width_max - init_center_width_min;

        int init_center_mean_search_len = config.filter_vertical.init_center_mean_search_len;
        int init_center_mean_window_len = config.filter_vertical.init_center_mean_window_len;
        f32_t init_center_mean_window_jx_max = config.filter_vertical.init_center_mean_window_jx_max;
        f32_t init_center_mean_window_jy_max = config.filter_vertical.init_center_mean_window_jy_max;
        f32_t init_center_mean_window_jz_max = config.filter_vertical.init_center_mean_window_jz_max;

        int init_center_line_search_start_index = config.filter_vertical.init_center_line_search_start_index;

        f32_t init_center_line_center_dist_max = config.filter_vertical.init_center_line_center_dist_max;
        f32_t init_center_line_length_min = config.filter_vertical.init_center_line_length_min;

        // line
        f32_t init_center_line_filter_continuous_valid_resolution = config.filter_vertical.init_center_line_filter_continuous_valid_resolution;
        f32_t init_center_line_filter_continuous_buffer_len = config.filter_vertical.init_center_line_filter_continuous_buffer_len;
        f32_t init_center_line_filter_continuous_len_min = config.filter_vertical.init_center_line_filter_continuous_len_min;

        f32_t init_center_line_filter_continuous_len_max = config.filter_vertical.init_center_line_filter_continuous_len_max;

        int init_center_line_filter_continuous_buffer_size = int(init_center_line_filter_continuous_buffer_len/init_center_line_filter_continuous_valid_resolution);



        f32_t init_center_line_filter_dist = config.filter_vertical.init_center_line_filter_dist;
        f32_t init_center_line_filter_len_step = config.filter_vertical.init_center_line_filter_len_step;
        float init_center_line_filter_len_valid_min = config.filter_vertical.init_center_line_filter_len_valid_min;
        f32_t init_center_line_filter_len_search_min = config.filter_vertical.init_center_line_filter_len_search_min;
        f32_t init_center_line_filter_len_search_max = config.filter_vertical.init_center_line_filter_len_search_max;

        int init_center_line_filter_relative_row = config.filter_vertical.init_center_line_filter_relative_row;
        int init_center_line_filter_early_stop_num_change = config.filter_vertical.init_center_line_filter_early_stop_num_change;



        center_line_markers.clear();


        MLOGI("init_center_height  : [%i, %i]", init_center_height_min ,init_center_height_max);
        MLOGI("init_center_width  : [%i, %i]", init_center_width_min,init_center_width_max );

        MLOGI("init_center_dim : [%i, %i}]",init_center_height, init_center_width );
        MLOGI("cloud_dim : [%i, %i]",cloud_dim_height, cloud_dim_width );

        if( filter_ground_status < 0 ){
            MLOGW("filter_ground_status error : [%i}]",filter_ground_status );
            filter_vertical_status = -1;
            return 0;
        }

        if( init_center_height < 3
            || init_center_width < 3
                ){
            MLOGW("init_center_dim contains wrong value: [%i, %i}]",init_center_height, init_center_width );

            filter_vertical_status = -1;
            return 0;
        }

        // reallocate buffer

        vertical_center_buffer = (f32_t*)ta_realloc(&mem_cfg,vertical_center_buffer,init_center_height*init_center_width*3*4);
        vertical_output_buffer = (f32_t*)ta_realloc(&mem_cfg,vertical_output_buffer,cloud_dim_height*cloud_dim_width*3*4);

        vertical_center_index_buffer = (u32_t*) ta_realloc(&mem_cfg,vertical_center_index_buffer,init_center_height*init_center_width*2*4);

        vertical_center_vertical_index_buffer = (u32_t*) ta_realloc(&mem_cfg,vertical_center_vertical_index_buffer,init_center_height*2*4);

        vertical_center_index_row_valid_num_buffer = (u32_t*) ta_realloc(&mem_cfg,vertical_center_index_row_valid_num_buffer,init_center_height*4);

        vertical_filter_index_buffer = (u64_t *)ta_realloc(&mem_cfg,vertical_filter_index_buffer,cloud_dim_height*cloud_dim_width*8);
        vertical_output_index_buffer = (u32_t *)ta_realloc(&mem_cfg,vertical_output_index_buffer,cloud_dim_height*cloud_dim_width*4);

        vertical_line_continuous_count_buffer =  (u32_t *)ta_realloc(&mem_cfg, vertical_line_continuous_count_buffer, 2 * (init_center_line_filter_continuous_buffer_size + 1) * 4  );
        vertical_line_continuous_mean_buffer=  (f32_t *)ta_realloc(&mem_cfg, vertical_line_continuous_mean_buffer,  2 * (init_center_line_filter_continuous_buffer_size + 1) * 4  );


        if(
                !ground_init_buffer
                || !ground_output_buffer
                || !cloud_label_table
                || !ground_pixel_count_buffer
                   || !vertical_center_buffer
                      || !vertical_output_buffer
                      ||!vertical_center_index_buffer
                      || !vertical_center_index_row_valid_num_buffer
                      ||!vertical_center_vertical_index_buffer
                        ||!vertical_filter_index_buffer
                          ||!vertical_output_index_buffer
                          ||!vertical_line_continuous_count_buffer
                          || !vertical_line_continuous_mean_buffer


                ){
            MLOGW("PalletDetector: create buffer fail: %p, %p, %p, %p, %p, %p, %p, %p, %p, %p, %p, %p,%p",
                  ground_init_buffer, ground_output_buffer, cloud_label_table, ground_pixel_count_buffer, vertical_center_buffer, vertical_output_buffer, vertical_center_index_buffer, vertical_center_index_row_valid_num_buffer, vertical_center_vertical_index_buffer, vertical_filter_index_buffer, vertical_output_index_buffer, vertical_line_continuous_count_buffer,vertical_line_continuous_mean_buffer );
            filter_ground_status = -2;
            return nullptr;
        }

//        memset(vertical_center_buffer,0,cloud_dim_height*cloud_dim_width);
//        memset(vertical_output_buffer,0,cloud_dim_height*row_scan_fold);

        float init_center_cx_min = config.filter_vertical.init_center_cx_min;
        float init_center_cx_max = config.filter_vertical.init_center_cx_max;
        float init_center_cy_min = config.filter_vertical.init_center_cy_min;
        float init_center_cy_max = config.filter_vertical.init_center_cy_max;
        float init_center_cz_min = config.filter_vertical.init_center_cz_min;
        float init_center_cz_max = config.filter_vertical.init_center_cz_max;

        float init_center_jx_max = config.filter_vertical.init_center_jx_max;
        float init_center_jy_max = config.filter_vertical.init_center_jy_max;
        float init_center_jz_max = config.filter_vertical.init_center_jz_max;

        int init_row_valid_num_min = config.filter_vertical.init_row_valid_num_min;

        pointcloud_clip(ground_output_buffer, cloud_dim_height, cloud_dim_width, vertical_center_buffer,init_center_height_min ,init_center_height_max,  init_center_width_min,init_center_width_max );




        MLOGI("init_center_thresh : [%f, %f],[%f, %f],[%f, %f] ",init_center_cx_min, init_center_cx_max,init_center_cy_min, init_center_cy_max,init_center_cz_min, init_center_cz_max );
        MLOGI("init_center_thresh jump : [%f, %f, %f] ",init_center_jx_max, init_center_jy_max,init_center_jz_max );

        int valid_num = 0;
        int valid_num_last = 0;

        int row_offset = init_center_width*3;

        for( int i = 1; i < init_center_height -1 ;i++){
            int row_valid_num = 0;
            for(int j = 0 ; j < init_center_width;j++ ){

                int k = (i * init_center_width + j)*3;

                f32_t *p = vertical_center_buffer + k;
                f32_t *p_pre = p - row_offset;
                f32_t *p_nxt = p + row_offset;

                bool valid = p[0] > init_center_cx_min && p[0] < init_center_cx_max
                             && p[1] > init_center_cy_min && p[1] < init_center_cy_max
                             && p[2] > init_center_cz_min && p[2] < init_center_cz_max
                             && ( (
                                     abs(p[0] - p_pre[0]) < init_center_jx_max
                                     &&  abs(p[1] - p_pre[1]) < init_center_jy_max
                                         &&  abs(p[2] - p_pre[2]) < init_center_jz_max

                                     ) || (abs(p[0] - p_nxt[0]) < init_center_jx_max
                                           &&  abs(p[1] - p_nxt[1]) < init_center_jy_max
                                           &&  abs(p[2] - p_nxt[2]) < init_center_jz_max) )

                             ;

                vertical_center_buffer[valid_num*3] = p[0];
                vertical_center_buffer[valid_num*3 + 1] = p[1];
                vertical_center_buffer[valid_num*3 + 2] = p[2];
                vertical_center_index_buffer[valid_num*2 ] = i;
                vertical_center_index_buffer[valid_num*2 +1  ] = j;

                valid_num += valid;
                row_valid_num += valid;
            }
            vertical_center_index_row_valid_num_buffer[i] = row_valid_num;
            bool reject_this_row = row_valid_num < init_row_valid_num_min;



            if ( reject_this_row){
                valid_num -= row_valid_num;
                vertical_center_index_row_valid_num_buffer[i] = 0;
            }
        }
        MLOGW("valid_num: %i ", valid_num );

        if(valid_num < config.filter_vertical.init_valid_num_min){
            MLOGW("valid_num: %i < %i", valid_num, config.filter_vertical.init_valid_num_min);
            filter_vertical_status = -2;
            return nullptr;
        }
        {
#if 0
            std::cout << " check vertical_center_index_buffer:\n";
        for(int i = 0 ; i < valid_num; i++){
            std::cout << "[ " << vertical_center_index_buffer[2*i] << ", " << vertical_center_index_buffer[2*i + 1] << " ] , ";
        }
        std::cout << " finish vertical_center_index_buffer:\n";


#endif
        }

        {
#if 0
            std::cout << " check vertical_center_index_row_valid_num_buffer:\n";
            for( int i = 1; i < init_center_height -1 ;i++){
                std::cout <<i  << ": [ " << vertical_center_index_row_valid_num_buffer[i] <<  " ] , ";
            }
            std::cout << " finish vertical_center_index_row_valid_num_buffer:\n";
#endif

        }
        /*
        perform line check for each row
         1. choose 5 continuous points cluster at left , center, right of each row
         2. in each cluster, distance between neighbor points should be less than 0.02, or shift index range
         3. if left , center, right cluster cannot be determined, mark this row as invalid
         4. check point to line distance, if distance between point, center , and line , lest to right, is less than 0.03, mark it as line, or mark it as invalid


         get valid line index of row
         */

        /*
         match left cluster to right cluster with minimum z difference
         compute line function y = f(x), compute reference point at f(0), compute [0, f(0)] for each line function, choose vertical plain,  f1(0), f2(0), f3(0) should be located in narrow region



         */
#if 1
        int center_vertical_valid_row_num = 0;
        {
//            std::cout << " check row range in vertical_center_index_row_valid_num_buffer:\n";
            int row_start = 0;
            int row_end = 0;

            for( int i = 1; i < init_center_height -1 ;i++){
                std::cout <<i  << ": [ " << vertical_center_index_row_valid_num_buffer[i] <<  " ] , ";
                if (vertical_center_index_row_valid_num_buffer[i] == 0){
                    continue;
                }
                row_end += vertical_center_index_row_valid_num_buffer[i];

                vertical_center_vertical_index_buffer[center_vertical_valid_row_num*2] = row_start;
                vertical_center_vertical_index_buffer[center_vertical_valid_row_num*2 + 1] = row_end;
                center_vertical_valid_row_num+=1;

//                std::cout << "get row: " << i << ", range: " << row_start << ", " << row_end<< "\n";


                //
                row_start = row_end;

            }
//            std::cout << " finish check row range in vertical_center_index_row_valid_num_buffer\n";

        }

        {
            std::cout << "center_vertical_valid_row_num: " << center_vertical_valid_row_num << "\n";
            for(int i = 0 ; i < center_vertical_valid_row_num; i ++){

                int row_start = vertical_center_vertical_index_buffer[i*2] ;
                int row_end = vertical_center_vertical_index_buffer[i*2 + 1]  ;
                int row_center = (row_start + row_end)/2;
//                MLOGI("row_start: %i, row_end: %i, row_center: %i\n",row_start,row_end,row_center);
//                std::cout << "get row: " << i << ", range: " << row_start << ", " << row_end<< "\n index: ";
#if 0
                for(int j = row_start; j < row_end;j++){
                    std::cout << "[ " << vertical_center_index_buffer[2*j] << ", " << vertical_center_index_buffer[2*j + 1] << " ], ";
                    std::cout << "[ " << vertical_center_buffer[j*3]  << ", " << vertical_center_buffer[j*3+1]  << ", " << vertical_center_buffer[j*3+2]  << " ]\n";

                }
#endif


                /*
                 1. find continuous 5 point
                 2.
                 */
                {

                    bool find_valid_center_mark = true;
                    int find_valid_center_mark_index = row_center;
                    f32_t find_valid_center_mark_x = 0.0;
                    f32_t find_valid_center_mark_y = 0.0;
                    f32_t find_valid_center_mark_z = 0.0;

//                    MLOGI("init_center_mean_window_jx_max: %f, init_center_mean_window_jy_max: %f, init_center_mean_window_jz_max: %f\n",init_center_mean_window_jx_max,init_center_mean_window_jy_max,init_center_mean_window_jz_max);

                    int search_len = std::min(std::min(init_center_mean_search_len, (row_center - row_start - init_center_mean_window_len)), (row_end - row_center - init_center_mean_window_len));
                    for(int j1 = 1 ; j1 <search_len *2;j1++){
                        int j = row_center +( (j1%2 == 0) ? j1/2 : -j1/2);
                        find_valid_center_mark_index = j;
                        f32_t * p1 = vertical_center_buffer + 3*j;
                        f32_t row_center_marker_x = p1[0];
                        f32_t row_center_marker_y = p1[1];
                        f32_t row_center_marker_z = p1[2];
//                        MLOGI("p1: [%f, %f, %f]\n",p1[0],p1[1],p1[2]);

                        for(int k = 1; k< init_center_mean_window_len;k++){
                            f32_t * p2 = p1 - 3*k;
                            f32_t * p3 = p1 + 3*k;
//                            MLOGI("p2: [%f, %f, %f]\n",p2[0],p2[1],p2[2]);
//                            MLOGI("p3: [%f, %f, %f]\n",p3[0],p3[1],p3[2]);

                            row_center_marker_x += p2[0] + p3[0];
                            row_center_marker_y += p2[1] + p3[1];
                            row_center_marker_z += p2[2] + p3[2];

                            f32_t  cx = find_valid_center_mark_x =  (row_center_marker_x )/f32_t (1 + 2* k);
                            f32_t  cy = find_valid_center_mark_y =  (row_center_marker_y)/f32_t (1 + 2* k);
                            f32_t  cz = find_valid_center_mark_z = (row_center_marker_z)/f32_t (1 + 2* k);
//                            MLOGI("cx: %f, cy: %f, cz: %f\n",cx,cy,cz);
                            f32_t dx =  abs(p1[0] - cx);
                            f32_t dy =  abs(p1[1] - cy);
                            f32_t dz =  abs(p1[2] - cz);
//                            MLOGI("dx: %f, dy: %f, dz: %f\n",dx,dy,dz);

                            if(
                                   dx > init_center_mean_window_jx_max
                              ||  dy> init_center_mean_window_jy_max
                            || dz> init_center_mean_window_jz_max   ){
                                find_valid_center_mark = false;
                                break;
                            }
                        }
                        if(find_valid_center_mark){
                            break;
                        }

                    }

//                    MLOGI("find_valid_center_mark: %i",find_valid_center_mark);
//                    MLOGI("find_valid_center_mark_index: %i",find_valid_center_mark_index);

//                    MLOGI("find_valid_center_mark_x: %f, find_valid_center_mark_y: %f, find_valid_center_mark_z: %f\n",  find_valid_center_mark_x,find_valid_center_mark_y,find_valid_center_mark_z);

                    bool find_valid_center_line = false;

                    int final_line_start = 0;
                    int final_line_end = 0;
                    if(find_valid_center_mark){



                        int center_line_search_window_len = std::min((find_valid_center_mark_index - row_start),(row_end - find_valid_center_mark_index) );
//                        MLOGI("center_line_search_window_len: %i",center_line_search_window_len);
//                        MLOGI("init_center_line_search_start_index: %i",init_center_line_search_start_index);

                        //==
                        geometry::float3 p_center = {vertical_center_buffer[find_valid_center_mark_index*3] ,
                                                     vertical_center_buffer[find_valid_center_mark_index*3 + 1] ,
                                                     vertical_center_buffer[find_valid_center_mark_index*3 + 2] };

                        int error_try_count = 0;
                        int max_error_try_count = 5;
                        for(int j1 = init_center_line_search_start_index ; j1 <center_line_search_window_len;j1++){
                            int j_left = find_valid_center_mark_index - j1;
                            int j_right = find_valid_center_mark_index +j1;


//                            int j_left_h = vertical_center_index_buffer[j_left*2 ]  ;
//                            int j_left_w = vertical_center_index_buffer[j_left*2 +1  ] ;
//
//                            int j_right_h = vertical_center_index_buffer[j_right*2 ]  ;
//                            int j_right_w = vertical_center_index_buffer[j_right*2 +1  ] ;
//                            vertical_center_buffer;


                            geometry::float3 p_left = {vertical_center_buffer[j_left*3] ,
                                                         vertical_center_buffer[j_left*3 + 1] ,
                                                         vertical_center_buffer[j_left*3 + 2] };

                            geometry::float3 p_right = {vertical_center_buffer[j_right*3] ,
                                                       vertical_center_buffer[j_right*3 + 1] ,
                                                       vertical_center_buffer[j_right*3 + 2] };
                            geometry::float3 p_left_right_dir = (p_right - p_left);
                            f32_t  p_left_right_dir_len = p_left_right_dir.norm();
                            p_left_right_dir.normalize();

                            float dist_center_to_line = geometry::distanceToLineWithNormDir(p_left,p_left_right_dir,p_center );
//                            MLOGI("j_left: %i, j_right: %i, dist_center_to_line: %f, p_left_right_dir_len: %f, init_center_line_center_dist_max: %f, init_center_line_length_min: %f ",   j_left,j_right , dist_center_to_line, p_left_right_dir_len, init_center_line_center_dist_max,init_center_line_length_min);

                            if(dist_center_to_line < init_center_line_center_dist_max){
                                final_line_start = j_left;
                                final_line_end = j_right;
                                find_valid_center_line = p_left_right_dir_len > init_center_line_length_min;
                            }else{
                                error_try_count ++;

                            }

                            if(error_try_count > max_error_try_count){
                                break;
                            }
                        }
                        //==

                    }
//                    MLOGI("find_valid_center_line: %i, final_line_start: %i, final_line_end: %i",  find_valid_center_line,final_line_start,final_line_end);

                    if(find_valid_center_line){
                        geometry::float3 p_center = {vertical_center_buffer[find_valid_center_mark_index*3] ,
                                                     vertical_center_buffer[find_valid_center_mark_index*3 + 1] ,
                                                     vertical_center_buffer[find_valid_center_mark_index*3 + 2] };
                        geometry::float3 p_left = {vertical_center_buffer[final_line_start*3] ,
                                                     vertical_center_buffer[final_line_start*3 + 1] ,
                                                     vertical_center_buffer[final_line_start*3 + 2] };
                        geometry::float3 p_right = {vertical_center_buffer[final_line_end*3] ,
                                                     vertical_center_buffer[final_line_end*3 + 1] ,
                                                     vertical_center_buffer[final_line_end*3 + 2] };

                        geometry::float3 p_left_right_dir = (p_right - p_left);
                        p_left_right_dir.normalize();


                        f32_t intersect_x, intersect_y;
                        bool intersect_ok = math::LineLineIntersect(p_left.x,p_left.y, p_right.x,p_right.y,
                                                                    0.0,0.0,
                                                                    1.0,0.0,
                                                                    intersect_x, intersect_y);
                        geometry::float3 p_intersection = {
                                intersect_x,
                                intersect_y,
                                p_center.z
                        };
                        LineMark line = {
                                p_left, p_center, p_right, p_left_right_dir,p_intersection,
                                p_center,p_left_right_dir,
                                static_cast<u32_t>(find_valid_center_mark_index), static_cast<u32_t>(final_line_start), static_cast<u32_t>(final_line_end),
                                0,{0,0},{0,0},{0,0}
                        };

                        center_line_markers.emplace_back(
                                line
                                );


                    }

                }


            }
        }
#endif

        {
//            vertical_filter_index_list.clear();
            vertical_filter_index_vec.clear();

            int center_line_markers_len = center_line_markers.size();


            if(center_line_markers_len == 0 ){
                filter_vertical_status = -3;
                return 0;
            }


//            MLOGI("center_line_markers_len: %i",center_line_markers_len);
            for(int i = 0; i < center_line_markers_len;i++){
                auto& l1 = center_line_markers[i];
                MLOGI("get center_line_markers[%i], point_left:[%f, %f, %f], point_center:[%f, %f, %f], point_right:[%f, %f, %f], dir_left_to_right:[%f, %f, %f], intersection:[%f, %f, %f]",i,l1.point_left.x,l1.point_left.y,l1.point_left.z ,l1.point_center.x,l1.point_center.y,l1.point_center.z ,l1.point_right.x,l1.point_right.y,l1.point_right.z,l1.dir_left_to_right.x,l1.dir_left_to_right.y,l1.dir_left_to_right.z,l1.intersection_to_a_axis.x,l1.intersection_to_a_axis.y,l1.intersection_to_a_axis.z );

                int index_left = l1.index_left;
                int index_center = l1.index_center;
                int index_right = l1.index_right;
                int index_left_h = vertical_center_index_buffer[2*index_left + 0];
                int index_left_w = vertical_center_index_buffer[2*index_left + 1];
                int index_right_h = vertical_center_index_buffer[2*index_right + 0];
                int index_right_w = vertical_center_index_buffer[2*index_right + 1];
                int index_center_h = vertical_center_index_buffer[2*index_center + 0];
                int index_center_w = vertical_center_index_buffer[2*index_center + 1];
                int index_left_h_raw = index_left_h + init_center_height_min;
                int index_left_w_raw = index_left_w + init_center_width_min;
                int index_right_h_raw = index_right_h + init_center_height_min;
                int index_right_w_raw = index_right_w + init_center_width_min;
                int index_center_h_raw = index_center_h + init_center_height_min;
                int index_center_w_raw = index_center_w + init_center_width_min;

                l1.raw_index_left[0] = index_left_h_raw;
                l1.raw_index_left[1] = index_left_w_raw;

                l1.raw_index_center[0] = index_center_h_raw;
                l1.raw_index_center[1] = index_center_w_raw;

                l1.raw_index_right[0] = index_right_h_raw;
                l1.raw_index_right[1] = index_right_w_raw;


//                MLOGI("index_in_center: [%i, %i], [%i, %i], [%i, %i],index_in_full: [%i, %i], [%i, %i], [%i, %i],",index_left_h, index_left_w,index_center_h, index_center_w, index_right_h, index_right_w,index_left_h_raw, index_left_w_raw,index_center_h_raw, index_center_w_raw, index_right_h_raw, index_right_w_raw);

                f32_t filter_line_cx = (l1.point_left.x +  l1.point_center.x + l1.point_right.x)/3.0f;
                f32_t filter_line_cy = (l1.point_left.y +  l1.point_center.y + l1.point_right.y)/3.0f;
                f32_t filter_line_cz = (l1.point_left.z +  l1.point_center.z + l1.point_right.z)/3.0f;

//                MLOGI("filter_line: [%f, %f, %f]", filter_line_cx, filter_line_cy, filter_line_cz);
                geometry::float3 filter_line_center {
                        filter_line_cx,filter_line_cy,filter_line_cz
                };

                geometry::float3 filter_line_dir = l1.dir_left_to_right;

                filter_line_dir.z = 0.0;
                filter_line_dir.normalize();
//                MLOGI("filter_line_dir: [%f, %f, %f]", filter_line_dir.x, filter_line_dir.y, filter_line_dir.z);

                // check point dist to dir

                f32_t  dist_left = geometry::distanceToLineWithNormDir( filter_line_center,filter_line_dir, l1.point_left );
                f32_t  dist_center = geometry::distanceToLineWithNormDir( filter_line_center,filter_line_dir, l1.point_center );
                f32_t  dist_right = geometry::distanceToLineWithNormDir( filter_line_center,filter_line_dir, l1.point_right );

                f32_t dist_alone_dir_left = filter_line_dir.dot(l1.point_left - filter_line_center);
                f32_t dist_alone_dir_center = filter_line_dir.dot(l1.point_center - filter_line_center);
                f32_t dist_alone_dir_right = filter_line_dir.dot(l1.point_right - filter_line_center);

//                MLOGI("dist_alone_dir: [%f, %f, %f]", dist_alone_dir_left, dist_alone_dir_center, dist_alone_dir_right);

//                MLOGI("filter_line_dst: [%f, %f, %f]", dist_left, dist_center, dist_right);

                // compute relative index
                // for a given line, only rows nearby need filter

                int row_start = std::max(0, index_center_h_raw - init_center_line_filter_relative_row);
                int row_end = std::min( cloud_dim_height, index_center_h_raw + init_center_line_filter_relative_row + 1);
                u64_t src_buffer_point_num = static_cast<u64_t>((row_end - row_start) * cloud_dim_width);


                int point_in_raw_buffer_index = index_center_h_raw* cloud_dim_width + index_center_w_raw;
                f32_t* src_buffer = ground_output_buffer + point_in_raw_buffer_index *3;
//                MLOGI("src_buffer: %p, buffer_point_num: %i, row_start: %i, row_end: %i ", src_buffer, src_buffer_point_num, row_start, row_end);

                geometry::float3 point_in_raw_buffer {
                        src_buffer [0],
                        src_buffer [1],
                        src_buffer [2]
                };

//                MLOGI("point_in_raw_buffer: [%f, %f, %f]", point_in_raw_buffer.x, point_in_raw_buffer.y, point_in_raw_buffer.z);

                u64_t vertical_filter_index_buffer_num = 0;
                u64_t  src_buffer_offset = row_start* cloud_dim_width;
                src_buffer = ground_output_buffer + src_buffer_offset *3 ;
                f32_t line_cx = filter_line_center.x , line_cy = filter_line_center.y , line_cz = filter_line_center.z ,
                line_dx = filter_line_dir.x , line_dy = filter_line_dir.y, line_dz = filter_line_dir.z ,
                line_nx, line_ny, line_nz, line_nd;

                f32_t  start_a = init_center_line_filter_len_search_min;

                while (start_a < init_center_line_filter_len_search_max){


                    pointcloud_filter_along_line(src_buffer,src_buffer_point_num ,vertical_filter_index_buffer,&vertical_filter_index_buffer_num,
                                                 line_cx, line_cy, line_cz,
                                                 line_dx, line_dy,line_dz,
                                                 init_center_line_filter_dist,-start_a,start_a
                    );

                    pointcloud_norm2d(src_buffer,src_buffer_point_num ,vertical_filter_index_buffer,vertical_filter_index_buffer_num,
                                      viewpoint_x,viewpoint_y,viewpoint_z,
                                      &line_cx, &line_cy, &line_cz, &line_nx, &line_ny, &line_nz, &line_nd );

                    // convert norm to direction

                    f32_t line_dir = std::atan2(line_ny, line_nx) + M_PI_2f32;
                    line_dx = std::cos(line_dir);
                    line_dy = std::sin(line_dir);
                    line_dz = 0.0;
                    start_a += init_center_line_filter_len_step;
                }


//                MLOGI("pointcloud_norm2d: [%f, %f, %f], [%f, %f, %f, %f]", line_cx, line_cy, line_cz, line_nx, line_ny, line_nz, line_nd )

                geometry::float3 filter_line_norm_dir {
                        line_nx, line_ny, line_nz
                };

                geometry::float3 filter_line_norm_center {
                        line_cx, line_cy, line_cz
                };
                f32_t dist_alone_norm_left = filter_line_norm_dir.dot(l1.point_left - filter_line_norm_center);
                f32_t dist_alone_norm_center = filter_line_norm_dir.dot(l1.point_center - filter_line_norm_center);
                f32_t dist_alone_norm_right = filter_line_norm_dir.dot(l1.point_right - filter_line_norm_center);
//                MLOGI("dist_alone_norm: [%f, %f, %f]", dist_alone_norm_left, dist_alone_norm_center, dist_alone_norm_right);

                l1.filtered_line_center.x = line_cx;
                l1.filtered_line_center.y = line_cy;
                l1.filtered_line_center.z = line_cz;
                f32_t line_dir = std::atan2(line_ny, line_nx) + M_PI_2f32;
                f32_t line_norm = filter_line_norm_dir.norm();
                l1.filtered_line_dir.x = std::cos(line_dir);
                l1.filtered_line_dir.y = std::sin(line_dir);
                l1.filtered_line_dir.z = 0.0;


                f32_t intersect_x, intersect_y;
                bool intersect_ok = math::LineLineIntersect(  l1.filtered_line_center.x,  l1.filtered_line_center.y, l1.filtered_line_dir.x,l1.filtered_line_dir.y,
                                                            0.0,0.0,
                                                            1.0,0.0,
                                                            intersect_x, intersect_y);

                l1.intersection_to_a_axis.x = intersect_x;
                l1.intersection_to_a_axis.y = intersect_y;
                l1.intersection_to_a_axis.z = line_cz;

                // filter by counting point distribution

                f32_t dist_alone_norm_intersect = l1.filtered_line_dir.dot( l1.intersection_to_a_axis - filter_line_norm_center);
//                MLOGI("intersection_to_a_axis: [%f, %f, %f], dist_alone_norm_intersect: %f", l1.intersection_to_a_axis.x,l1.intersection_to_a_axis.y,l1.intersection_to_a_axis.z, dist_alone_norm_intersect);

                const int count_size = 200;
                float dist_alone_norm_max = -100.0;
                float dist_alone_norm_min = 100.0;
                // check valid line
                // 1. continuous
                // 2. length

                memset(vertical_line_continuous_count_buffer, 0, 2 * (init_center_line_filter_continuous_buffer_size + 1) * 4 );

//                MLOGI("init_center_line_filter_continuous_buffer_size: %i",init_center_line_filter_continuous_buffer_size);

                for(int j = 0 ; j < vertical_filter_index_buffer_num;j++){
                    float *ptr = src_buffer + vertical_filter_index_buffer[j]*3;

                    geometry::float3 point {
                        ptr[0],
                        ptr[1],
                        ptr[2]
                    };
                    f32_t dist_alone_norm = l1.filtered_line_dir.dot( point  - filter_line_norm_center);

                    int continuous_index = dist_alone_norm/init_center_line_filter_continuous_valid_resolution + init_center_line_filter_continuous_buffer_size;
                    continuous_index = std::max(0,std::min(continuous_index, init_center_line_filter_continuous_buffer_size+ init_center_line_filter_continuous_buffer_size));

                    vertical_line_continuous_count_buffer[continuous_index] ++;


                    dist_alone_norm_min = std::min(dist_alone_norm_min , dist_alone_norm);
                    dist_alone_norm_max = std::max(dist_alone_norm_max , dist_alone_norm);
//                    MLOGI("point: [%f, %f, %f], dist_alone_norm: %f, continuous_index: %i, cmp [%i, %i]", point.x,point.y,point.z, dist_alone_norm,continuous_index,(continuous_index < init_center_line_filter_continuous_buffer_size)  ,(continuous_index > -init_center_line_filter_continuous_buffer_size));
                }

                std::cout << "\nvertical_line_continuous_buffer: ";
                for(int j = 0 ; j < 2*init_center_line_filter_continuous_buffer_size;j++){
                    std::cout << vertical_line_continuous_count_buffer[j] << ", ";
                }

                int dist_alone_norm_continuous_max = init_center_line_filter_continuous_buffer_size;
                int dist_alone_norm_continuous_min = init_center_line_filter_continuous_buffer_size;
                for(int j = init_center_line_filter_continuous_buffer_size; j >=0 ; j--){
                    dist_alone_norm_continuous_min = j;
                    if(vertical_line_continuous_count_buffer[j] == 0 ){
                        break;
                    }
                }
                for(int j = init_center_line_filter_continuous_buffer_size ;j < init_center_line_filter_continuous_buffer_size+init_center_line_filter_continuous_buffer_size; j++){
                    dist_alone_norm_continuous_max = j;
                    if(vertical_line_continuous_count_buffer[j] == 0 ){
                        break;
                    }
                }

                f32_t dist_alone_norm_continuous_dist = (dist_alone_norm_continuous_max - dist_alone_norm_continuous_min) *init_center_line_filter_continuous_valid_resolution;

                int dist_alone_norm_continuous_center =  (dist_alone_norm_continuous_max + dist_alone_norm_continuous_min)/2 - init_center_line_filter_continuous_buffer_size;

                float dist_alone_norm_continuous_center_len = dist_alone_norm_continuous_center*init_center_line_filter_continuous_valid_resolution;
                MLOGI("dist_alone_norm_continuous: [%i, %i, %i] , dist: %f, center: %f ",dist_alone_norm_continuous_min, dist_alone_norm_continuous_max, dist_alone_norm_continuous_center,dist_alone_norm_continuous_dist,dist_alone_norm_continuous_center_len);

                l1.filtered_line_center  -= l1.filtered_line_dir * dist_alone_norm_continuous_center_len;



                float dist_alone_norm_max_len = dist_alone_norm_max - dist_alone_norm_min;
//                MLOGI("dist_alone_norm_max_len: %f, dist_alone_norm_min: %f, dist_alone_norm_max: %f ",dist_alone_norm_max_len, dist_alone_norm_min,dist_alone_norm_max);

                bool valid =  dist_alone_norm_max_len > init_center_line_filter_len_valid_min
                        && dist_alone_norm_continuous_dist > init_center_line_filter_len_valid_min
                           && dist_alone_norm_continuous_dist > init_center_line_filter_continuous_len_min
                           && dist_alone_norm_continuous_dist < init_center_line_filter_continuous_len_max

                        ;
                if(!valid){
                    l1.valid_status = -1;
                }

                if (output_mode == 2){
                    for(int j = 0 ; j < vertical_filter_index_buffer_num;j++){
                        vertical_filter_index_vec.push_back(vertical_filter_index_buffer[j] + src_buffer_offset);
                    }
                }


            }

            // filter

            auto center_line_markers_it = std::remove_if(center_line_markers.begin(), center_line_markers.end(),[](auto& x){
                return x.valid_status < 0;
            });
            center_line_markers.erase(center_line_markers_it, center_line_markers.end());
            std::sort(center_line_markers.begin(), center_line_markers.end(), [](auto& v1, auto &v2){
                return v1.filtered_line_center.z < v2.filtered_line_center.z;
            });



        }

#if 0
        {

            int center_line_markers_len = center_line_markers.size();
            MLOGI("center_line_markers_len: %i",center_line_markers_len);
            for(int i = 0; i < center_line_markers_len;i++){
                auto& l1 = center_line_markers[i];

                MLOGI("get center_line_markers[%i], point_center:[%f, %f, %f], point_left:[%f, %f, %f], point_right:[%f, %f, %f], dir_left_to_right:[%f, %f, %f]",i,
                      l1.point_center.x,l1.point_center.y,l1.point_center.z ,
                      l1.point_left.x,l1.point_left.y,l1.point_left.z ,
                      l1.point_right.x,l1.point_right.y,l1.point_right.z,
                      l1.dir_left_to_right.x,l1.dir_left_to_right.y,l1.dir_left_to_right.z
                );

                f32_t best_z_abs = 1000.0;
                int best_z_index = i;
                for( int j = 0 ; j < center_line_markers_len; j++){
                    auto& l2 = center_line_markers[j];
                    f32_t z_diff = abs(l1.point_left.z - l2.point_right.z );
                    if( z_diff < best_z_abs){
                        best_z_abs = z_diff;
                        best_z_index = j;
                    }
                }
                MLOGI("best_z_abs: %f, best_z_index: %i ",best_z_abs, best_z_index);
                auto& l3 = center_line_markers[best_z_index];

                // find intersection

                f32_t intersect_x, intersect_y;
                bool intersect_ok = math::LineLineIntersect(l1.point_left.x,l1.point_left.y, l3.point_right.x,l3.point_right.y,
                                        0.0,0.0,
                                        1.0,0.0,
                                        intersect_x, intersect_y);
                MLOGI("intersect_x: %f, intersect_y: %f ",intersect_x, intersect_y);


            }
        }
#endif


#if 0
        {

            int start_check_line_height_index = vertical_center_index_buffer[0];
            int start_check_line_width_index = vertical_center_index_buffer[1];
            for(int i = 0 ; i < valid_num; i++){



                if (vertical_center_index_buffer[2*i] != start_check_line_height_index){


                    start_check_line_height_index = vertical_center_index_buffer[2*i];
                }


            }

        }


#endif

        filter_vertical_status = center_line_markers.size();


        if (output_mode == 0){

            output_cloud_buffer.buffer = vertical_center_buffer;
            output_cloud_buffer.float_num = init_center_height * init_center_width * 3;
            MLOGI("filter_vertical use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));
            return &output_cloud_buffer;

        }

        if (output_mode == 1){

            output_cloud_buffer.buffer = vertical_center_buffer;
            output_cloud_buffer.float_num = valid_num * 3;
            MLOGI("filter_vertical use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));

            return &output_cloud_buffer;

        }




        if (output_mode == 2){

//            vertical_filter_index_vec.clear();
//            std::copy(vertical_filter_index_list.begin(), vertical_filter_index_list.end(),std::back_inserter(vertical_filter_index_vec));

            std::sort(vertical_filter_index_vec.begin(), vertical_filter_index_vec.end());
            vertical_filter_index_vec.erase(std::unique(vertical_filter_index_vec.begin(),
                                                        vertical_filter_index_vec.end()),vertical_filter_index_vec.end());


            valid_num = vertical_filter_index_vec.size();

//            MLOGI("valid_num: %u",valid_num);
//            for(auto i : vertical_filter_index_list){
//                MLOGI("vertical_filter_index_list i : %llu", i);
//
//            }

            for(int i = 0 ; i < valid_num;i++){
                u64_t ii = vertical_filter_index_vec[i];
//                MLOGI("vertical_filter_index_vec ii: %llu", ii);
                f32_t *p = ground_output_buffer + ii*3;

                vertical_output_buffer[i*3] = p[0];
                vertical_output_buffer[i*3 + 1] = p[1];
                vertical_output_buffer[i*3 + 2] = p[2];
            }


//            vertical_filter_index_vec.reserve(vertical_filter_index_list.size());


            output_cloud_buffer.buffer = vertical_output_buffer;
            output_cloud_buffer.float_num = valid_num * 3;
            MLOGI("filter_vertical use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));

            return &output_cloud_buffer;

        }




        if(output_mode == 3){

            memcpy( vertical_output_buffer,ground_output_buffer,cloud_dim_height*cloud_dim_width*3*4);

            int step_num = 800;
            f32_t step_resolution = 2.0/step_num;

            int center_line_markers_len = center_line_markers.size();
            f32_t * marker_buffer = vertical_output_buffer + (cloud_dim_height*cloud_dim_width - step_num* center_line_markers_len)*3;
//            MLOGI("center_line_markers_len: %i",center_line_markers_len);
             float marker = 0.05f;
            for(int i = 0; i < center_line_markers_len;i++) {
                auto &l1 = center_line_markers[i];

                auto& filtered_line_center = l1.filtered_line_center;
                auto & filtered_line_dir = l1.filtered_line_dir;
                for(int j = 0 ; j < step_num;j++){
                    float jj = j*step_resolution - 1.0f;

                    auto p = filtered_line_center + filtered_line_dir*jj;

                    if( std::abs(jj) <marker){
                        p.x += marker - std::abs(jj);
                    }

                    marker_buffer[(i * step_num + j)*3 + 0 ] = p.x;
                    marker_buffer[(i * step_num + j)*3 + 1 ] = p.y;
                    marker_buffer[(i * step_num + j)*3 + 2 ] = p.z;

                }

            }

            valid_num = cloud_dim_height*cloud_dim_width;


            output_cloud_buffer.buffer = vertical_output_buffer;
            output_cloud_buffer.float_num = valid_num * 3;
            MLOGI("filter_vertical use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));

            return &output_cloud_buffer;


        }




        return 0;
    }

    PalletInfoBuffer_ptr PalletDetector::get_pallet(u32_t output_mode) {

        if(filter_pallet_status <= 0){
            return 0;
        }

        output_pallet_vec.resize(pallet_cluster.size());
        for(size_t i = 0 ; i < output_pallet_vec.size();i++){
            output_pallet_vec[i].confidence = pallet_cluster[i].confidence;
            output_pallet_vec[i].tx = pallet_cluster[i].pallet_pose_center.x;
            output_pallet_vec[i].ty = pallet_cluster[i].pallet_pose_center.y;
            output_pallet_vec[i].tz = pallet_cluster[i].pallet_pose_center.z;
            output_pallet_vec[i].yaw = pallet_cluster[i].pallet_pose_yaw;
            output_pallet_vec[i].roll = 0.0;
            output_pallet_vec[i].pitch = 0.0;
            output_pallet_vec[i].info = 1;
        }

        output_pallet_info_buffer.buffer = output_pallet_vec.data();
        output_pallet_info_buffer.pallet_num = output_pallet_vec.size();
        return &output_pallet_info_buffer;
    }
    PointCloudBuffer_ptr PalletDetector::filter_pallet(u32_t output_mode) {
        /*
         1. split line markers to cluster
         2. remove invalid cluster
         3. computer mean pose for each cluster
         4. find pocket for each cluster
         5. remove invalid cluster
         */
        common::Time start_time = common::FromUnixNow();


        int center_line_markers_len = center_line_markers.size();
        MLOGI("filter_pallet output_mode: %u,filter_vertical_status: %i, center_line_markers_len: %i", output_mode,filter_vertical_status,center_line_markers_len);


        int cloud_dim_height = config.cloud.dim.height;
        int cloud_dim_width = config.cloud.dim.width;

        float filter_pallet_x_min = config.filter_pallet.filter_pallet_x_min;
        float filter_pallet_x_max = config.filter_pallet.filter_pallet_x_max;
        float filter_pallet_y_min = config.filter_pallet.filter_pallet_y_min;
        float filter_pallet_y_max = config.filter_pallet.filter_pallet_y_max;
        float filter_pallet_z_min = config.filter_pallet.filter_pallet_z_min;
        float filter_pallet_z_max = config.filter_pallet.filter_pallet_z_max;
        float filter_pallet_jx = config.filter_pallet.filter_pallet_jx;
        float filter_pallet_jy = config.filter_pallet.filter_pallet_jy;
        float filter_pallet_jz = config.filter_pallet.filter_pallet_jz;


        float filter_pallet_line_x_min = config.filter_pallet.filter_pallet_line_x_min;
        float filter_pallet_line_x_max= config.filter_pallet.filter_pallet_line_x_max;
        float filter_pallet_line_y_min= config.filter_pallet.filter_pallet_line_y_min;
        float filter_pallet_line_y_max= config.filter_pallet.filter_pallet_line_y_max;
        float filter_pallet_line_z_min= config.filter_pallet.filter_pallet_line_z_min;
        float filter_pallet_line_z_max= config.filter_pallet.filter_pallet_line_z_max;
        float filter_pallet_line_continuous_min = config.filter_pallet.filter_pallet_line_continuous_min;


        float filter_space_second_height_high = config.filter_pallet.filter_space_second_height_high;

        float filter_space_second_move_mean_thresh = config.filter_pallet.filter_space_second_move_mean_thresh;


        float cluster_filter_pose_weight = config.filter_pallet.cluster_filter_pose_weight;
        cluster_filter_pose_weight = std::min(1.0f, cluster_filter_pose_weight);

        float filter_pallet_z_pocket_low = config.filter_pallet.filter_pallet_z_pocket;

        float fork_shape_width = config.filter_pallet.fork_shape_width;
        float fork_shape_width_2 = 0.5*fork_shape_width;
        float fork_shape_height = config.filter_pallet.fork_shape_height;
        float fork_pos_y = config.filter_pallet.fork_pos_y;
        float filter_pallet_z_pocket_high = filter_pallet_z_pocket_low + fork_shape_height;

        float pallet_space_width_left = config.filter_pallet.pallet_space_width_left;
        float pallet_pocket_width = config.filter_pallet.pallet_pocket_width;
        float pallet_space_width_center = config.filter_pallet.pallet_space_width_center;
        float pallet_space_width_center_2 = 0.5*pallet_space_width_center;
        float pallet_space_height = config.filter_pallet.pallet_space_height;
        float pallet_top_height = config.filter_pallet.pallet_top_height;
        float pallet_top_tail_height_ratio = config.filter_pallet.pallet_top_tail_height_ratio;

        float pallet_top_tail_height = (pallet_top_height + pallet_space_height)*pallet_top_tail_height_ratio;


        float filter_space_width_slip_len = config.filter_pallet.filter_space_width_slip_len;


        float filter_pallet_space_left_y_min = pallet_space_width_center_2 + pallet_pocket_width -filter_space_width_slip_len;
        float filter_pallet_space_left_y_max = pallet_space_width_center_2 + pallet_pocket_width + pallet_space_width_left + filter_space_width_slip_len;

        float filter_pallet_space_center_y_min = -pallet_space_width_center_2 - filter_space_width_slip_len;
        float filter_pallet_space_center_y_max = pallet_space_width_center_2 + filter_space_width_slip_len;

        float filter_pallet_space_right_y_min = - filter_pallet_space_left_y_max;
        float filter_pallet_space_right_y_max = - filter_pallet_space_left_y_min;

        float filter_pallet_pocket_left_y_min = fork_pos_y - fork_shape_width_2;
        float filter_pallet_pocket_left_y_max = fork_pos_y + fork_shape_width_2;

        float filter_pallet_pocket_right_y_min = -filter_pallet_pocket_left_y_max;
        float filter_pallet_pocket_right_y_max = -filter_pallet_pocket_left_y_min;

        MLOGI("filter pocket :filter_pallet_z_pocket: [%f, %f] pallet_space_left [%f, %f], pallet_pocket_left [%f, %f], pallet_space_center [%f, %f], pallet_pocket_right [%f, %f], pallet_space_right [%f, %f]",
              filter_pallet_z_pocket_low, filter_pallet_z_pocket_high   ,filter_pallet_space_left_y_min, filter_pallet_space_left_y_max,
              filter_pallet_pocket_left_y_min, filter_pallet_pocket_left_y_max, filter_pallet_space_center_y_min, filter_pallet_space_center_y_max, filter_pallet_pocket_right_y_min, filter_pallet_pocket_right_y_max,
              filter_pallet_space_right_y_min, filter_pallet_space_right_y_max
        )

        float filter_space_continuous_dist = config.filter_pallet.filter_space_continuous_dist;
        int filter_space_continuous_num = config.filter_pallet.filter_space_continuous_num;
        float filter_space_continuous_thresh = config.filter_pallet.filter_space_continuous_thresh;
        float pallet_space_direction_diff_max = config.filter_pallet.pallet_space_direction_diff_max;

        int pallet_pocket_max_num= config.filter_pallet.pallet_pocket_max_num;


        // line
        f32_t init_center_line_filter_continuous_valid_resolution = config.filter_vertical.init_center_line_filter_continuous_valid_resolution;
        f32_t init_center_line_filter_continuous_buffer_len = config.filter_vertical.init_center_line_filter_continuous_buffer_len;
        f32_t init_center_line_filter_continuous_len_min = config.filter_vertical.init_center_line_filter_continuous_len_min;

        f32_t init_center_line_filter_continuous_len_max = config.filter_vertical.init_center_line_filter_continuous_len_max;

        int init_center_line_filter_continuous_buffer_size = int(init_center_line_filter_continuous_buffer_len/init_center_line_filter_continuous_valid_resolution);

        // filter space
        float filter_space_continuous_resolution  = config.filter_pallet.filter_space_continuous_resolution;
        float filter_space_move_window_x_search_start = config.filter_pallet.filter_space_move_window_x_search_start;
        float filter_space_move_window_x_search_end = config.filter_pallet.filter_space_move_window_x_search_end;
        float filter_space_move_window_x_search_step = config.filter_pallet.filter_space_move_window_x_search_step;
        float filter_space_move_window_y_len = config.filter_pallet.filter_space_move_window_y_len;

        float filter_space_move_window_x_thresh = config.filter_pallet.filter_space_move_window_x_thresh;
        float filter_space_continuous_len_min = config.filter_pallet.filter_space_continuous_len_min;


        float projector_dir_weight_p3 = config.filter_pallet.projector_dir_weight_p3;

        float projector_dir_weight_p3t = config.filter_pallet.projector_dir_weight_p3t;

        float projector_dir_weight_t = config.filter_pallet.projector_dir_weight_t;

        float projector_dir_weight_0 = config.filter_pallet.projector_dir_weight_0;


        float projector_offset_y= config.filter_pallet.projector_offset_y;
        float projector_offset_z= config.filter_pallet.projector_offset_z;
        float projector_resolution= config.filter_pallet.projector_resolution;
        float projector_resolution_inv = 1.0f/projector_resolution;

        float projector_search_y_range= config.filter_pallet.projector_search_y_range;
        float projector_search_z_range= config.filter_pallet.projector_search_z_range;


        int projector_search_y_range_len = static_cast<int>( projector_search_y_range * projector_resolution_inv);

        int projector_search_z_range_len = static_cast<int>( projector_search_z_range * projector_resolution_inv);

        int projector_search_y_range_num = static_cast<int>(2 * projector_search_y_range * projector_resolution_inv +
                                                            1);

        int projector_search_z_range_num = static_cast<int>(2 * projector_search_z_range * projector_resolution_inv +
                                                            1);



        float projector_search_maximum_distance= config.filter_pallet.projector_search_maximum_distance;
        float projector_similarity_min= config.filter_pallet.projector_similarity_min;

        float project_template_top_range[] = {-(pallet_space_width_center_2 + pallet_pocket_width + pallet_space_width_left ),
                                              (pallet_space_width_center_2 + pallet_pocket_width + pallet_space_width_left ),
                                              0.0,
                                              pallet_top_height};
        float project_template_left_range[] = {pallet_space_width_center_2 + pallet_pocket_width,
                                               pallet_space_width_center_2 + pallet_pocket_width + pallet_space_width_left,
                                               -pallet_space_height,
                                               0.0};
        float project_template_center_range[] = {-pallet_space_width_center_2,
                                                 pallet_space_width_center_2,
                                                 -pallet_space_height,
                                                 0.0};
        float project_template_right_range[] = {-(pallet_space_width_center_2 + pallet_pocket_width + pallet_space_width_left),
                                                -(pallet_space_width_center_2 + pallet_pocket_width),
                                                -pallet_space_height,
                                                0.0};

        int project_template_top_index[] = {static_cast<int>((project_template_top_range[0] + projector_offset_y) *
                                                                   projector_resolution_inv),
                                                  static_cast<int>((project_template_top_range[1] + projector_offset_y) *
                                                                   projector_resolution_inv),
                                                  static_cast<int>((project_template_top_range[2] + projector_offset_z) *
                                                                   projector_resolution_inv),
                                                  static_cast<int>((project_template_top_range[3] + projector_offset_z) *
                                                                   projector_resolution_inv)};
        int project_template_left_index[] = {static_cast<int>((project_template_left_range[0] + projector_offset_y) *
                                                               projector_resolution_inv),
                                              static_cast<int>((project_template_left_range[1] + projector_offset_y) *
                                                               projector_resolution_inv),
                                              static_cast<int>((project_template_left_range[2] + projector_offset_z) *
                                                               projector_resolution_inv),
                                              static_cast<int>((project_template_left_range[3] + projector_offset_z) *
                                                               projector_resolution_inv)};

        int project_template_center_index[] = {static_cast<int>((project_template_center_range[0] + projector_offset_y) *
                                                              projector_resolution_inv),
                                             static_cast<int>((project_template_center_range[1] + projector_offset_y) *
                                                              projector_resolution_inv),
                                             static_cast<int>((project_template_center_range[2] + projector_offset_z) *
                                                              projector_resolution_inv),
                                             static_cast<int>((project_template_center_range[3] + projector_offset_z) *
                                                              projector_resolution_inv)};


        int project_template_right_index[] = {static_cast<int>((project_template_right_range[0] + projector_offset_y) *
                                                                projector_resolution_inv),
                                               static_cast<int>((project_template_right_range[1] + projector_offset_y) *
                                                                projector_resolution_inv),
                                               static_cast<int>((project_template_right_range[2] + projector_offset_z) *
                                                                projector_resolution_inv),
                                               static_cast<int>((project_template_right_range[3] + projector_offset_z) *
                                                                projector_resolution_inv)};



        int projector_template_white_pixel_count = (project_template_top_index[1] - project_template_top_index[0]) *(project_template_top_index[3] - project_template_top_index[2])
                + (project_template_left_index[1] - project_template_left_index[0]) *(project_template_left_index[3] - project_template_left_index[2])
                + (project_template_center_index[1] - project_template_center_index[0]) *(project_template_center_index[3] - project_template_center_index[2])
                + (project_template_right_index[1] - project_template_right_index[0]) *(project_template_right_index[3] - project_template_right_index[2])
                ;
        int projector_template_pixel_count =  ( (project_template_top_index[1] - project_template_right_index[0]) *(project_template_top_index[3] - project_template_right_index[2]));

        float projector_template_pixel_count_inv = 1.0f/static_cast<float>(projector_template_pixel_count);

        float projector_template_q = (static_cast<float>(projector_template_white_pixel_count) /
                static_cast<float>(projector_template_pixel_count));
        float projector_template_q_inv = 1.0f /(1.0f - projector_template_q);

        MLOGI("projector_template_white_pixel_count: %i, projector_template_pixel_count: %i, projector_template_q: %f",projector_template_white_pixel_count,projector_template_pixel_count,projector_template_q)



        // get filter boxes
        // pocket boxes
        // space boxes

        f32_t cluster_filter_split_last_dist = config.filter_pallet.cluster_filter_split_last_dist;
        f32_t cluster_filter_split_last_yaw = config.filter_pallet.cluster_filter_split_last_yaw;

        f32_t cluster_filter_split_start_dist = config.filter_pallet.cluster_filter_split_start_dist;
        f32_t cluster_filter_split_start_yaw = config.filter_pallet.cluster_filter_split_start_yaw;
        f32_t cluster_filter_split_last_z = config.filter_pallet.cluster_filter_split_last_z;
        f32_t cluster_filter_split_start_z = config.filter_pallet.cluster_filter_split_start_z;

        f32_t cluster_filter_split_last_x = config.filter_pallet.cluster_filter_split_last_x;
        f32_t cluster_filter_split_start_x = config.filter_pallet.cluster_filter_split_start_x;
        int cluster_filter_num_min = config.filter_pallet.cluster_filter_num_min;

        int search_direction = config.filter_pallet.search_direction;
        int filter_pallet_row_high = config.filter_pallet.filter_pallet_row_high;
        int filter_pallet_row_low = config.filter_pallet.filter_pallet_row_low;


        float pallet_space_center_to_line_dist_max = config.filter_pallet.pallet_space_center_to_line_dist_max;
        float pallet_pocket_empty_x = config.filter_pallet.pallet_pocket_empty_x;
        int pallet_space_valid_num = config.filter_pallet.pallet_space_valid_num;


        if(output_mode == 0){

            filter_pallet_status = -1;
            output_cloud_buffer.buffer = ground_output_buffer;
            output_cloud_buffer.float_num = cloud_dim_height * cloud_dim_width * 3;
            MLOGI("filter_ground use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));

            return &output_cloud_buffer;

        }


        if(filter_vertical_status < 0 || center_line_markers_len <cluster_filter_num_min ){
            filter_pallet_status = -1;
            return 0;
        }


        //
        pallet_pocket_buffer = (f32_t*)ta_realloc(&mem_cfg, pallet_pocket_buffer, cloud_dim_height * cloud_dim_width * 3 * 4);
        pallet_center_line_buffer = (f32_t*)ta_realloc(&mem_cfg, pallet_pocket_buffer, cloud_dim_height * cloud_dim_width * 3 * 4);
        pallet_project_buffer = (f32_t*)ta_realloc(&mem_cfg, pallet_project_buffer, cloud_dim_height * cloud_dim_width * 3 * 4);

        pallet_project_template_score_buffer = (f32_t*)ta_realloc(&mem_cfg, pallet_project_template_score_buffer, projector_search_y_range_num * projector_search_z_range_num*4);

        projector_output_img_buffer = (u8_t *)ta_realloc(&mem_cfg, projector_output_img_buffer, projector_search_y_range_num * projector_search_z_range_num);


        if(!pallet_pocket_buffer
           || ! pallet_center_line_buffer
           || !pallet_project_buffer
           || !projector_output_img_buffer
                ){
            MLOGW("allocate buffer fail: %p, %p, %p, %p",pallet_pocket_buffer,pallet_center_line_buffer,pallet_project_buffer,projector_output_img_buffer );

            filter_pallet_status = -2;
            return 0;
        }
        memset(pallet_project_template_score_buffer, 0,projector_search_y_range_num * projector_search_z_range_num*4 );


        MLOGI("center_line_markers_len: %i ",center_line_markers_len);

        pallet_candidates.resize(center_line_markers_len);
        pallet_cluster.clear();


        for(int i = 0 ; i < center_line_markers_len;i++){
            auto& l = center_line_markers[i];
            auto& line_center = l.filtered_line_center;
            auto& line_dir = l.filtered_line_dir;
            auto& candidate = pallet_candidates[i];

            MLOGI("marker[%i], center: [%f, %f, %f], dir: [%f, %f, %f]",i,line_center.x, line_center.y, line_center.z,line_dir.x, line_dir.y, line_dir.z  );

            MLOGI("index: [%u, %u], [%u, %u], [%u, %u]", l.raw_index_left[0], l.raw_index_left[1],
                  l.raw_index_center[0], l.raw_index_center[1],
                  l.raw_index_right[0], l.raw_index_right[1]);
            float center_in_base_yaw = std::atan2(line_center.y,line_center.x);

            float line_dir_yaw =  std::atan2(line_dir.y, line_dir.x);
            line_dir_yaw = angle_normalise(line_dir_yaw, center_in_base_yaw);
            float diff = line_dir_yaw - center_in_base_yaw ;
            float pallet_direction = diff > 0.0 ? line_dir_yaw - M_PI_2: line_dir_yaw + M_PI_2;

            MLOGI("center_in_base_yaw: %f, pallet_direction: %f", center_in_base_yaw,pallet_direction);

            candidate.pallet_center = line_center;
            candidate.pallet_direction = angle_normalise_zero( pallet_direction) ;

            candidate.pallet_pose = transform::createSe3<double>( line_center.x, line_center.y, line_center.z,0.0,0.0,pallet_direction );
            candidate.pallet_pose_inv = candidate.pallet_pose.inverse();
            candidate.valid_status = 0;
            candidate.cluster_id = 0;
            candidate.raw_index_center[0] = l.raw_index_center[0];
            candidate.raw_index_center[1] = l.raw_index_center[1];



             std::cout << "pose:\n" << candidate.pallet_pose.matrix()<< "\n";
        }


        if(pallet_candidates.size() ==0 ){
            MLOGI("filter_pallet use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));
            filter_pallet_status = -3;
            return 0;
        }



        {
            /*
             split to cluster
             */


            PalletCluster cluster;

            int start_id = 0;
            int pallet_candidates_len = pallet_candidates.size();
            pallet_candidates[0].pallet_pose_in_start.setIdentity();

            for(int i = 1 ; i < pallet_candidates_len;i++){
                auto& candidate = pallet_candidates[i];
                auto& last_candidate = pallet_candidates[i-1];

                f64_t tx, ty, tz, roll, pitch,yaw;
                auto last_relative_pose = last_candidate.pallet_pose_inv *candidate.pallet_pose;
                transform::extractSe3(last_relative_pose,tx, ty, tz, roll, pitch,yaw );
                MLOGI("last_relative_pose[%i]: tx: %f, ty: %f, tz: %f, roll: %f, pitch: %f,yaw: %f",i, tx, ty, tz, roll, pitch,yaw )
                auto  __attribute__ ((aligned (32)))  start_relative_pose = pallet_candidates[start_id].pallet_pose_inv *candidate.pallet_pose;
                candidate.pallet_pose_in_start = start_relative_pose;
                transform::extractSe3(start_relative_pose,tx, ty, tz, roll, pitch,yaw );
                MLOGI("start_relative_pose[%i]: tx: %f, ty: %f, tz: %f, roll: %f, pitch: %f,yaw: %f",i, tx, ty, tz, roll, pitch,yaw )

                f32_t last_dist_change = (candidate.pallet_center - last_candidate.pallet_center).norm();
                f32_t start_dist_change = (candidate.pallet_center - pallet_candidates[start_id].pallet_center).norm();

                f32_t last_z_change = (candidate.pallet_center.z - last_candidate.pallet_center.z);
                f32_t start_z_change = (candidate.pallet_center.z - pallet_candidates[start_id].pallet_center.z);
                f32_t last_x_change = (candidate.pallet_center.x - last_candidate.pallet_center.x);
                f32_t start_x_change = (candidate.pallet_center.x - pallet_candidates[start_id].pallet_center.x);


                f32_t last_yaw_change = std::abs(angle_normalise(candidate.pallet_direction , last_candidate.pallet_direction)  - last_candidate.pallet_direction );
                f32_t start_yaw_change = std::abs(angle_normalise(candidate.pallet_direction , pallet_candidates[start_id].pallet_direction)  - pallet_candidates[start_id].pallet_direction );

                MLOGI("change: x: [%f, %f], z: [%f, %f], dist: [%f, %f], yaw: [%f, %f]",
                      last_x_change, start_x_change,
                      last_z_change, start_z_change,
                      last_dist_change,start_dist_change, last_yaw_change, start_yaw_change );
                // if new cluster is detected
                bool is_new_cluster = (i == (pallet_candidates_len - 1))
                        || last_dist_change > cluster_filter_split_last_dist
                           || start_dist_change > cluster_filter_split_start_dist
                           || last_z_change > cluster_filter_split_last_z
                           || start_z_change > cluster_filter_split_start_z
                              || last_x_change > cluster_filter_split_last_x
                              || start_x_change > cluster_filter_split_start_x
                              || last_yaw_change > cluster_filter_split_last_yaw
                                 || start_yaw_change > cluster_filter_split_start_yaw

                        ;
                if (is_new_cluster){
                    int end_id = i;

                    if((i == (pallet_candidates_len - 1))){
                        end_id = pallet_candidates_len;
                    }

                    MLOGI("get cluster from [%i, %i]", start_id, end_id);

                    for(int j =start_id; j < end_id; j++ ){
                        std::cout << "[" << j << "] pose:\n"<< pallet_candidates[j].pallet_pose.matrix() << "\n";

//                        cluster.candidates.emplace_back(pallet_candidates[j]);


                    }
                    cluster.candidates.assign(pallet_candidates.begin() + start_id, pallet_candidates.begin() + end_id);
                    pallet_cluster.emplace_back(std::move(cluster));
                    start_id = end_id;
                }

            }

        }



        {
            /*
             check validation
             1. check minimum number limitation
             2. remove invalid cluster
             3. compute mean pose
             4. get index range
             */

            auto it = std::remove_if(pallet_cluster.begin(),pallet_cluster.end(),[cluster_filter_num_min](auto& v){
                return v.candidates.size() < cluster_filter_num_min;
            });
            pallet_cluster.erase(it, pallet_cluster.end());
            int pallet_cluster_len = pallet_cluster.size();
            MLOGI("valid pallet_cluster_len %i",pallet_cluster_len);

            for(int i = 0 ; i < pallet_cluster_len;i++){
                f64_t sum_x = 0.0,sum_y = 0.0, sum_z = 0.0, sum_yaw = 0.0;
                auto& cluster = pallet_cluster[i];
                auto& candidates = cluster.candidates;
                f64_t yaw_start = candidates[0].pallet_direction;
                f64_t num_inv = cluster_filter_pose_weight/(static_cast<float>(candidates.size()));
                for(int j = 0 ; j <candidates.size();j++ ){
                    f64_t tx, ty, tz, roll, pitch,yaw;
                    transform::extractSe3(candidates[j].pallet_pose_in_start,tx, ty, tz, roll, pitch,yaw );
                    sum_x += tx;
                    sum_y += ty;
                    sum_z += tz;
                    sum_yaw += angle_normalise_zero( yaw);

                }

                f64_t mean_x = sum_x *num_inv;
                f64_t mean_y = sum_y *num_inv;
                f64_t mean_z = sum_z *num_inv;
                f64_t mean_yaw = sum_yaw *num_inv;

                MLOGI("mean pose: [%f, %f, %f, %f]", mean_x, mean_y, mean_z, mean_yaw);
                __attribute__ ((aligned (32))) auto relative_pose = transform::createSe3(mean_x, mean_y, 0.0 ,0.0,0.0,mean_yaw );

                cluster.est_pose = candidates[0].pallet_pose * relative_pose;
                cluster.est_pose_inv = cluster.est_pose.inverse();
                cluster.top_row_low =candidates.front().raw_index_center[0];

                cluster.top_row_high =candidates.back().raw_index_center[0];

                std::cout << "final_pose: \n" << cluster.est_pose.matrix() << "\n";


            }


        }


        {
            int pallet_cluster_len = pallet_cluster.size();

            if( pallet_cluster_len == 0 ){
                filter_pallet_status = -3;

                return 0;
            }
        }



        {
            /*
             check pocket
             0. transform cloud to pallet origin
             1. filter outlier, clip range
             2. create 5 box filter base on a pallet shape and slide search_x origin
             3. increase search_x util heating two space box

             */

            if(search_direction == 1){

                int pallet_cluster_len = pallet_cluster.size();

                for(int i = 0 ; i < pallet_cluster_len;i++){

                    pallet_space_left.clear();
                    pallet_pocket_left.clear();
                    pallet_space_center.clear();
                    pallet_pocket_right.clear();
                    pallet_space_right.clear();


                    auto& cluster = pallet_cluster[i];
                    auto& candidates = cluster.candidates;

                    int row_start = filter_pallet_row_high < 0 ? cluster.top_row_high + filter_pallet_row_high : cluster.top_row_low + filter_pallet_row_high;
                    int row_end = cluster.top_row_low + filter_pallet_row_low ;

                    row_start = std::min(std::max(0, row_start), cloud_dim_height);
                    row_end = std::min(std::max(row_start+1, row_end), cloud_dim_height);



                    MLOGI("filter row: [%i, %i] ", row_start, row_end);

                    int float_num  = (row_end - row_start) * cloud_dim_width * 3;

                    {

                        {
                            int point_num = (row_end - row_start) * cloud_dim_width;

                            f64_t tx, ty, tz, roll, pitch,yaw;

                            transform::extractSe3(cluster.est_pose_inv,tx, ty, tz, roll, pitch,yaw );
                            MLOGI("est_pose_inv[%i]: tx: %f, ty: %f, tz: %f, roll: %f, pitch: %f,yaw: %f",i, tx, ty, tz, roll, pitch,yaw )

                            pointcloud_transform(ground_output_buffer + row_start * cloud_dim_width * 3, point_num, pallet_pocket_buffer, tx, ty, tz, roll, pitch, yaw );

                        }


                        f32_t refined_y_offset = 0.0;
                        f32_t continuous_len = 0.0;
                        f32_t continuous_len_limit = 0.0;
                        {
                            int point_num = (row_end - cluster.top_row_low) * cloud_dim_width;

                            int valid_num = 0;
                            int row_offset = cloud_dim_width * 3;

                            for(int j = cloud_dim_width ; j < point_num - cloud_dim_width; j++){
                                const f32_t *p = pallet_pocket_buffer + j*3;
                                const f32_t *p_pre = p - row_offset;
                                const f32_t *p_nxt = p + row_offset;
                                const f32_t *p_left = p - 3;
                                const f32_t *p_right = p + 3;


                                bool valid = p[0] > filter_pallet_line_x_min && p[0] < filter_pallet_line_x_max
                                             && p[1] > filter_pallet_line_y_min && p[1] < filter_pallet_line_y_max
                                             && p[2] > filter_pallet_line_z_min && p[2] < filter_pallet_line_z_max
                                             && ( (
                                                          abs(p[0] - p_pre[0]) < filter_pallet_jx
                                                          &&  abs(p[1] - p_pre[1]) < filter_pallet_jy
                                                          &&  abs(p[2] - p_pre[2]) < filter_pallet_jz

                                                  ) && (abs(p[0] - p_nxt[0]) < filter_pallet_jx
                                                        &&  abs(p[1] - p_nxt[1]) < filter_pallet_jy
                                                        &&  abs(p[2] - p_nxt[2]) < filter_pallet_jz)
                                                  && (abs(p[0] - p_left[0]) < filter_pallet_jx
                                                      &&  abs(p[1] - p_left[1]) < filter_pallet_jy
                                                      &&  abs(p[2] - p_left[2]) < filter_pallet_jz)
                                                  && (abs(p[0] - p_right[0]) < filter_pallet_jx
                                                      &&  abs(p[1] - p_right[1]) < filter_pallet_jy
                                                      &&  abs(p[2] - p_right[2]) < filter_pallet_jz)




                                             )

                                ;

                                pallet_center_line_buffer[valid_num*3] = p[0];
                                pallet_center_line_buffer[valid_num*3 + 1] = p[1];
                                pallet_center_line_buffer[valid_num*3 + 2] = p[2];

                                valid_num += valid;

                            }

                            // map to array
                            // compute new center
                            memset(vertical_line_continuous_count_buffer, 0, 2 * (init_center_line_filter_continuous_buffer_size + 1) * 4 );

                            memset(vertical_line_continuous_mean_buffer, 0, 2 * (init_center_line_filter_continuous_buffer_size + 1) * 4 );

                            for(int j = 0 ; j < valid_num;j++){
                                float *ptr = pallet_center_line_buffer + j*3;


                                int continuous_index = ptr[1]/init_center_line_filter_continuous_valid_resolution + init_center_line_filter_continuous_buffer_size;
                                continuous_index = std::max(0,std::min(continuous_index, init_center_line_filter_continuous_buffer_size+ init_center_line_filter_continuous_buffer_size));

                                vertical_line_continuous_mean_buffer[continuous_index] =
                                        ((float)(vertical_line_continuous_count_buffer[continuous_index])*vertical_line_continuous_mean_buffer[continuous_index] + ptr[1])/(float)(vertical_line_continuous_count_buffer[continuous_index] + 1) ;

                                vertical_line_continuous_count_buffer[continuous_index] ++;

                            }
                            std::cout << "\nvertical_line_continuous_buffer: ";
                            for(int j = 0 ; j < 2*init_center_line_filter_continuous_buffer_size;j++){
                                std::cout <<"[" <<vertical_line_continuous_count_buffer[j] << ", " <<vertical_line_continuous_mean_buffer[j]  << "], " ;
                            }
                            int dist_alone_norm_continuous_max = init_center_line_filter_continuous_buffer_size;
                            int dist_alone_norm_continuous_min = init_center_line_filter_continuous_buffer_size;
                            for(int j = init_center_line_filter_continuous_buffer_size; j >=0 ; j--){
                                if(vertical_line_continuous_count_buffer[j] == 0 ){
                                    break;
                                }
                                dist_alone_norm_continuous_min = j;

                            }
                            for(int j = init_center_line_filter_continuous_buffer_size ;j < init_center_line_filter_continuous_buffer_size+init_center_line_filter_continuous_buffer_size; j++){
                                if(vertical_line_continuous_count_buffer[j] == 0 ){
                                    break;
                                }
                                dist_alone_norm_continuous_max = j;

                            }

                            f32_t dist_alone_norm_continuous_dist = (dist_alone_norm_continuous_max - dist_alone_norm_continuous_min) *init_center_line_filter_continuous_valid_resolution;

                            int dist_alone_norm_continuous_center =  (dist_alone_norm_continuous_max + dist_alone_norm_continuous_min)/2 - init_center_line_filter_continuous_buffer_size;
                            f32_t max_y = vertical_line_continuous_mean_buffer[dist_alone_norm_continuous_max];
                            f32_t min_y = vertical_line_continuous_mean_buffer[dist_alone_norm_continuous_min];

                            f32_t center_y = 0.5*( max_y+ min_y);
                            continuous_len = max_y - min_y;
                            continuous_len_limit = 0.5*(continuous_len) + init_center_line_filter_continuous_valid_resolution;
                            MLOGW("get mean buffer: [%f, %f],continuous_len: %f, center: [%f]",min_y,max_y,continuous_len, center_y);
                            refined_y_offset = center_y;

                            cluster.refined_y = refined_y_offset;
                            cluster.continuous_len = continuous_len;


                            // compare filter_pallet_line_continuous_min

                            // check continuous line len
                            if(continuous_len < filter_pallet_line_continuous_min){
                                cluster.valid_status = -1;
                                continue;
                            }

                            auto pose_refine = transform::createSe3(0.0, double(refined_y_offset),0.0,0.0,0.0,0.0);

                            cluster.est_pose = cluster.est_pose* pose_refine;
                            cluster.est_pose_inv = cluster.est_pose.inverse();




                            float dist_alone_norm_continuous_center_len = dist_alone_norm_continuous_center*init_center_line_filter_continuous_valid_resolution;
                            MLOGI("init_center_line_filter_continuous_buffer_size: %i, dist_alone_norm_continuous: [%i, %i, %i] , dist: %f, center: %f ",init_center_line_filter_continuous_buffer_size,dist_alone_norm_continuous_min, dist_alone_norm_continuous_max, dist_alone_norm_continuous_center,dist_alone_norm_continuous_dist,dist_alone_norm_continuous_center_len);




                        }
                        {
                            int point_num = (row_end - row_start) * cloud_dim_width;
                            float_num  =point_num* 3;

                            int valid_num = 0;
                            int row_offset = cloud_dim_width * 3;

                            for(int j = 0 ; j < point_num ; j++){
                                f32_t *p = pallet_pocket_buffer + j*3;
                                p[1] -= refined_y_offset;

                            }

                            for(int j = cloud_dim_width ; j < point_num - cloud_dim_width; j++){
                                f32_t *p = pallet_pocket_buffer + j*3;
                                const f32_t *p_pre = p - row_offset;
                                const f32_t *p_nxt = p + row_offset;
                                const f32_t *p_left = p - 3;
                                const f32_t *p_right = p + 3;


                                bool valid = p[0] > filter_pallet_x_min && p[0] < filter_pallet_x_max
                                             && p[1] > filter_pallet_y_min && p[1] < filter_pallet_y_max
                                             && p[2] > filter_pallet_z_min && p[2] < filter_pallet_z_max
                                             && ( (
                                                          abs(p[0] - p_pre[0]) < filter_pallet_jx
                                                          &&  abs(p[1] - p_pre[1]) < filter_pallet_jy
                                                          &&  abs(p[2] - p_pre[2]) < filter_pallet_jz

                                                  ) && (abs(p[0] - p_nxt[0]) < filter_pallet_jx
                                                        &&  abs(p[1] - p_nxt[1]) < filter_pallet_jy
                                                        &&  abs(p[2] - p_nxt[2]) < filter_pallet_jz)
                                                       && (abs(p[0] - p_left[0]) < filter_pallet_jx
                                                           &&  abs(p[1] - p_left[1]) < filter_pallet_jy
                                                           &&  abs(p[2] - p_left[2]) < filter_pallet_jz)
                                                          && (abs(p[0] - p_right[0]) < filter_pallet_jx
                                                              &&  abs(p[1] - p_right[1]) < filter_pallet_jy
                                                              &&  abs(p[2] - p_right[2]) < filter_pallet_jz)




                                                                                                                                                                                                                                                                                       )

                                ;
                                pallet_pocket_buffer[valid_num*3] = p[0];
                                pallet_pocket_buffer[valid_num*3 + 1] = p[1];
                                pallet_pocket_buffer[valid_num*3 + 2] = p[2];

                                valid_num += valid;

                            }

                            float_num = valid_num*3;


                            f32_t  pallet_pocket_empty_x_inv = 1.0f/pallet_pocket_empty_x;
                            {


                                MLOGW("check point: %i", valid_num);
                                for(int j = 0 ; j < valid_num;j++){
                                    const f32_t *p = pallet_pocket_buffer + j*3;
                                    f32_t x_dist = (pallet_pocket_empty_x - p[0])*pallet_pocket_empty_x_inv;
                                    x_dist = std::max( std::min(x_dist,1.0f) , 0.0f);
                                    f32_t z_max = fork_shape_height*x_dist;
//                                    f32_t z_min = 0.0;
//                                    std::cout << "p:[" << p[0] <<","<< p[1] <<"," << p[2] <<"], z:[" << filter_pallet_z_pocket_low<<", " <<filter_pallet_z_pocket_low +z_max<<"]\n";


                                    if( p[2] > filter_pallet_z_pocket_low && p[2] < (filter_pallet_z_pocket_low +z_max )){
                                        if( p[1] > filter_pallet_space_left_y_min && p[1] < filter_pallet_space_left_y_max){
                                            pallet_space_left.emplace_back(geometry::float3{p[0],p[1],p[2]});
                                        }
                                        if( p[1] > filter_pallet_space_center_y_min && p[1] < filter_pallet_space_center_y_max){
                                            pallet_space_center.emplace_back(geometry::float3{p[0],p[1],p[2]});
                                        }
                                        if( p[1] > filter_pallet_space_right_y_min && p[1] < filter_pallet_space_right_y_max){
                                            pallet_space_right.emplace_back(geometry::float3{p[0],p[1],p[2]});
                                        }
                                        if( p[1] > filter_pallet_pocket_left_y_min && p[1] < filter_pallet_pocket_left_y_max){
                                            pallet_pocket_left.emplace_back(geometry::float3{p[0],p[1],p[2]});
                                        }

                                        if( p[1] > filter_pallet_pocket_right_y_min && p[1] < filter_pallet_pocket_right_y_max){
                                            pallet_pocket_right.emplace_back(geometry::float3{p[0],p[1],p[2]});
                                        }
                                    }



                                }

                                MLOGI("get pocket space cluster: %zu, %zu, %zu, %zu, %zu", pallet_space_left.size(), pallet_space_center.size(), pallet_space_right.size(),pallet_pocket_left.size(),pallet_pocket_right.size());

                                bool pallet_space_count_valid = pallet_space_left.size() > pallet_space_valid_num
                                             && pallet_space_center.size() > pallet_space_valid_num
                                             && pallet_space_right.size() > pallet_space_valid_num;
                                // check space valid point
                                if(!pallet_space_count_valid){
                                    MLOGW("get pocket space cluster: %zu, %zu, %zu, %zu, %zu", pallet_space_left.size(), pallet_space_center.size(), pallet_space_right.size(),pallet_pocket_left.size(),pallet_pocket_right.size());

                                    cluster.valid_status = -1;
                                    continue;
                                }


                                std::sort(pallet_space_left.begin(), pallet_space_left.end(), [](auto& v1, auto& v2){
                                    return (v1.x + abs(v1.y)) < (v2.x + abs(v2.y));
                                });

                                std::sort(pallet_space_center.begin(), pallet_space_center.end(), [](auto& v1, auto& v2){
                                    return (v1.x + abs(v1.y)) < (v2.x + abs(v2.y));
                                });

                                std::sort(pallet_space_right.begin(), pallet_space_right.end(), [](auto& v1, auto& v2){
                                    return (v1.x + abs(v1.y)) < (v2.x + abs(v2.y));
                                });

                                std::sort(pallet_pocket_left.begin(), pallet_pocket_left.end(), [](auto& v1, auto& v2){
                                    return (v1.x + abs(v1.y)) < (v2.x + abs(v2.y));
                                });

                                std::sort(pallet_pocket_right.begin(), pallet_pocket_right.end(), [](auto& v1, auto& v2){
                                    return (v1.x + abs(v1.y)) < (v2.x + abs(v2.y));
                                });



                                float pallet_space_left_x = 0.0, pallet_space_left_y = 0.0, pallet_space_left_z = 0.0;
                                float pallet_space_center_x = 0.0, pallet_space_center_y = 0.0, pallet_space_center_z = 0.0;
                                float pallet_space_right_x = 0.0, pallet_space_right_y = 0.0, pallet_space_right_z = 0.0;
                                float pallet_pocket_left_x = 0.0, pallet_pocket_left_y = 0.0, pallet_pocket_left_z = 0.0;
                                float pallet_pocket_right_x = 0.0, pallet_pocket_right_y = 0.0, pallet_pocket_right_z = 0.0;

                                // x_min, x_max, y_min, y_max,z_min, z_max
                                float pallet_space_left_min_max[6] = {10.0, -10.0, 10.0, -10.0, 10.0, -10.0};
                                float pallet_space_center_min_max[6] = {10.0, -10.0, 10.0, -10.0, 10.0, -10.0};
                                float pallet_space_right_min_max[6] = {10.0, -10.0, 10.0, -10.0, 10.0, -10.0};

                                int pallet_space_left_valid_count = 0;
                                int pallet_space_center_valid_count = 0;
                                int pallet_space_right_valid_count = 0;
                                int pallet_pocket_left_valid_count = 0;
                                int pallet_pocket_right_valid_count = 0;


                                {
                                    auto& space_cluster = pallet_space_left;

                                    auto& min_max = pallet_space_left_min_max;
                                    float sum_x = 0.0, sum_y = 0.0 , sum_z = 0.0;
                                    int cluster_valid_num = 0;
                                    {
                                        int continuous_count = 0;
                                        int continuous_count_reach = 0;

                                        float start_x = 0.0;


                                        for(int j = 1 ; j < space_cluster.size();j++){
                                            auto& p = space_cluster[j];

                                            bool valid = std::abs(p.x - space_cluster[j-1].x)< filter_space_continuous_dist;


                                            continuous_count = (continuous_count_reach || valid) ? continuous_count+1:0;
                                            if(continuous_count == filter_space_continuous_num){
                                                start_x = p.x;
                                                continuous_count_reach = 1;
                                            }
                                            if(continuous_count_reach){

                                                if( std::abs(p.x - start_x) < filter_space_continuous_thresh
                                                        ){

                                                    cluster_valid_num++;
                                                    sum_x += p.x;
                                                    sum_y += p.y;
                                                    sum_z += p.z;

                                                    min_max[0] = std::min(min_max[0], p.x);
                                                    min_max[1] = std::max(min_max[1], p.x);
                                                    min_max[2] = std::min(min_max[2], p.y);
                                                    min_max[3] = std::max(min_max[3], p.y);
                                                    min_max[4] = std::min(min_max[4], p.z);
                                                    min_max[5] = std::max(min_max[5], p.z);


                                                }else{
                                                    break;
                                                }
                                            }

                                        }

                                        if(cluster_valid_num > 0 ){
                                            sum_x /= float(cluster_valid_num);
                                            sum_y /= float(cluster_valid_num);
                                            sum_z /= float(cluster_valid_num);
                                        }
                                    }
                                    pallet_space_left_valid_count = cluster_valid_num;
                                    pallet_space_left_x = sum_x, pallet_space_left_y = sum_y, pallet_space_left_z = sum_z;
                                }

                                {
                                    auto& space_cluster = pallet_space_center;
                                    auto& min_max = pallet_space_center_min_max;

                                    float sum_x = 0.0, sum_y = 0.0 , sum_z = 0.0;
                                    int cluster_valid_num = 0;
                                    {
                                        int continuous_count = 0;
                                        int continuous_count_reach = 0;

                                        float start_x = 0.0;


                                        for(int j = 1 ; j < space_cluster.size();j++){
                                            auto& p = space_cluster[j];

                                            bool valid = std::abs(p.x - space_cluster[j-1].x)< filter_space_continuous_dist;


                                            continuous_count = (continuous_count_reach || valid) ? continuous_count+1:0;
                                            if(continuous_count == filter_space_continuous_num){
                                                start_x = p.x;
                                                continuous_count_reach = 1;
                                            }
                                            if(continuous_count_reach){

                                                if( std::abs(p.x - start_x) < filter_space_continuous_thresh
                                                        ){

                                                    cluster_valid_num++;
                                                    sum_x += p.x;
                                                    sum_y += p.y;
                                                    sum_z += p.z;

                                                    min_max[0] = std::min(min_max[0], p.x);
                                                    min_max[1] = std::max(min_max[1], p.x);
                                                    min_max[2] = std::min(min_max[2], p.y);
                                                    min_max[3] = std::max(min_max[3], p.y);
                                                    min_max[4] = std::min(min_max[4], p.z);
                                                    min_max[5] = std::max(min_max[5], p.z);

                                                }else{
                                                    break;
                                                }
                                            }

                                        }

                                        if(cluster_valid_num > 0 ){
                                            sum_x /= float(cluster_valid_num);
                                            sum_y /= float(cluster_valid_num);
                                            sum_z /= float(cluster_valid_num);
                                        }
                                    }
                                    pallet_space_center_valid_count = cluster_valid_num;
                                    pallet_space_center_x = sum_x, pallet_space_center_y = sum_y, pallet_space_center_z = sum_z;
                                }
                                {
                                    auto& space_cluster = pallet_space_right;
                                    auto& min_max = pallet_space_right_min_max;

                                    float sum_x = 0.0, sum_y = 0.0 , sum_z = 0.0;
                                    int cluster_valid_num = 0;
                                    {
                                        int continuous_count = 0;
                                        int continuous_count_reach = 0;

                                        float start_x = 0.0;


                                        for(int j = 1 ; j < space_cluster.size();j++){
                                            auto& p = space_cluster[j];

                                            bool valid = std::abs(p.x - space_cluster[j-1].x)< filter_space_continuous_dist;


                                            continuous_count = (continuous_count_reach || valid) ? continuous_count+1:0;
                                            if(continuous_count == filter_space_continuous_num){
                                                start_x = p.x;
                                                continuous_count_reach = 1;
                                            }
                                            if(continuous_count_reach){

                                                if( std::abs(p.x - start_x) < filter_space_continuous_thresh
                                                        ){

                                                    cluster_valid_num++;
                                                    sum_x += p.x;
                                                    sum_y += p.y;
                                                    sum_z += p.z;

                                                    min_max[0] = std::min(min_max[0], p.x);
                                                    min_max[1] = std::max(min_max[1], p.x);
                                                    min_max[2] = std::min(min_max[2], p.y);
                                                    min_max[3] = std::max(min_max[3], p.y);
                                                    min_max[4] = std::min(min_max[4], p.z);
                                                    min_max[5] = std::max(min_max[5], p.z);

                                                }else{
                                                    break;
                                                }
                                            }

                                        }

                                        if(cluster_valid_num > 0 ){
                                            sum_x /= float(cluster_valid_num);
                                            sum_y /= float(cluster_valid_num);
                                            sum_z /= float(cluster_valid_num);
                                        }
                                    }
                                    pallet_space_right_valid_count = cluster_valid_num;
                                    pallet_space_right_x = sum_x, pallet_space_right_y = sum_y, pallet_space_right_z = sum_z;
                                }

                                {
                                    auto& space_cluster = pallet_pocket_left;
                                    float sum_x = 0.0, sum_y = 0.0 , sum_z = 0.0;
                                    int cluster_valid_num = 0;
                                    {
                                        int continuous_count = 0;
                                        int continuous_count_reach = 0;

                                        float start_x = 0.0;


                                        for(int j = 1 ; j < space_cluster.size();j++){
                                            auto& p = space_cluster[j];

                                            bool valid = std::abs(p.x - space_cluster[j-1].x)< filter_space_continuous_dist;


                                            continuous_count = (continuous_count_reach || valid) ? continuous_count+1:0;
                                            if(continuous_count == filter_space_continuous_num){
                                                start_x = p.x;
                                                continuous_count_reach = 1;
                                            }
                                            if(continuous_count_reach){

                                                if( std::abs(p.x - start_x) < filter_space_continuous_thresh
                                                        ){

                                                    cluster_valid_num++;
                                                    sum_x += p.x;
                                                    sum_y += p.y;
                                                    sum_z += p.z;


                                                }else{
                                                    break;
                                                }
                                            }

                                        }

                                        if(cluster_valid_num > pallet_pocket_max_num ){
                                            sum_x /= float(cluster_valid_num);
                                            sum_y /= float(cluster_valid_num);
                                            sum_z /= float(cluster_valid_num);
                                        }else{
                                            sum_x = 1.0 ;
                                            sum_y = 1.0;
                                            sum_z = 1.0;

                                        }
                                    }
                                    pallet_pocket_left_valid_count = cluster_valid_num;
                                    pallet_pocket_left_x = sum_x, pallet_pocket_left_y = sum_y, pallet_pocket_left_z = sum_z;
                                }
                                {
                                    auto& space_cluster = pallet_pocket_right;
                                    float sum_x = 0.0, sum_y = 0.0 , sum_z = 0.0;
                                    int cluster_valid_num = 0;
                                    {
                                        int continuous_count = 0;
                                        int continuous_count_reach = 0;

                                        float start_x = 0.0;


                                        for(int j = 1 ; j < space_cluster.size();j++){
                                            auto& p = space_cluster[j];

                                            bool valid = std::abs(p.x - space_cluster[j-1].x)< filter_space_continuous_dist;


                                            continuous_count = (continuous_count_reach || valid) ? continuous_count+1:0;
                                            if(continuous_count == filter_space_continuous_num){
                                                start_x = p.x;
                                                continuous_count_reach = 1;
                                            }
                                            if(continuous_count_reach){

                                                if( std::abs(p.x - start_x) < filter_space_continuous_thresh
                                                        ){

                                                    cluster_valid_num++;
                                                    sum_x += p.x;
                                                    sum_y += p.y;
                                                    sum_z += p.z;


                                                }else{
                                                    break;
                                                }
                                            }

                                        }

                                        if(cluster_valid_num > pallet_pocket_max_num ){
                                            sum_x /= float(cluster_valid_num);
                                            sum_y /= float(cluster_valid_num);
                                            sum_z /= float(cluster_valid_num);
                                        }else{
                                            sum_x = 1.0 ;
                                            sum_y = 1.0;
                                            sum_z = 1.0;

                                        }
                                    }
                                    pallet_pocket_right_valid_count = cluster_valid_num;
                                    pallet_pocket_right_x = sum_x, pallet_pocket_right_y = sum_y, pallet_pocket_right_z = sum_z;
                                }


                                MLOGI("filter_pallet_z_pocket [%f, %f]", filter_pallet_z_pocket_low,filter_pallet_z_pocket_high);
                                MLOGI("space min_max range, left: [%f, %f, %f, %f, %f, %f], center: [%f, %f, %f, %f, %f, %f], right: [%f, %f, %f, %f, %f, %f]",
                                      pallet_space_left_min_max[0],pallet_space_left_min_max[1],pallet_space_left_min_max[2],pallet_space_left_min_max[3],pallet_space_left_min_max[4],pallet_space_left_min_max[5],
                                      pallet_space_center_min_max[0],pallet_space_center_min_max[1],pallet_space_center_min_max[2],pallet_space_center_min_max[3],pallet_space_center_min_max[4],pallet_space_center_min_max[5],
                                      pallet_space_right_min_max[0],pallet_space_right_min_max[1],pallet_space_right_min_max[2],pallet_space_right_min_max[3],pallet_space_right_min_max[4],pallet_space_right_min_max[5]

                                      );

                                // filter second part of space form [filter_pallet_z_pocket_high, 0.0]
                                float pallet_space_left_move_mean_x = 0.5f*(pallet_space_left_min_max[0] + pallet_space_left_min_max[1]);
                                float pallet_space_center_move_mean_x = 0.5f*(pallet_space_center_min_max[0] + pallet_space_center_min_max[1]);

                                float pallet_space_right_move_mean_x = 0.5f*(pallet_space_right_min_max[0] + pallet_space_right_min_max[1]);
                                ;

                                pallet_space_left.clear();
                                pallet_space_center.clear();
                                pallet_space_right.clear();
                                pallet_top_line.clear();

                                for(int j = valid_num - 1 ; j >= 0  ;j--){
                                    const f32_t *p = pallet_pocket_buffer + j*3;


                                    if( p[2] > filter_pallet_z_pocket_low && p[2] <  filter_space_second_height_high )

                                    {
                                        if( p[1] > filter_pallet_space_left_y_min && p[1] < filter_pallet_space_left_y_max
                                        && std::abs(p[0] - pallet_space_left_move_mean_x) <filter_space_second_move_mean_thresh

                                        ){
                                            float c = static_cast<float>(pallet_space_left.size());
                                            pallet_space_left_move_mean_x = (c*pallet_space_left_move_mean_x + p[0])/(c+ 1.0f);

                                            pallet_space_left.emplace_back(geometry::float3{p[0],p[1],p[2]});
                                        }
                                        if( p[1] > filter_pallet_space_center_y_min && p[1] < filter_pallet_space_center_y_max
                                                                                       && std::abs(p[0] - pallet_space_center_move_mean_x) < filter_space_second_move_mean_thresh

                                        ){
                                            float c = static_cast<float>(pallet_space_center.size());
                                            pallet_space_center_move_mean_x = (c*pallet_space_center_move_mean_x + p[0])/(c+ 1.0f);

                                            pallet_space_center.emplace_back(geometry::float3{p[0],p[1],p[2]});
                                        }
                                        if( p[1] > filter_pallet_space_right_y_min && p[1] < filter_pallet_space_right_y_max
                                                                                      && std::abs(p[0] - pallet_space_right_move_mean_x) <filter_space_second_move_mean_thresh

                                        ){
                                            float c = static_cast<float>(pallet_space_right.size());
                                            pallet_space_right_move_mean_x = (c*pallet_space_right_move_mean_x + p[0])/(c+ 1.0f);

                                            pallet_space_right.emplace_back(geometry::float3{p[0],p[1],p[2]});
                                        }


                                    }else if(
                                            p[0] > filter_pallet_line_x_min && p[0] < filter_pallet_line_x_max
                                            && p[1] > -continuous_len_limit && p[1] < continuous_len_limit
                                            && p[2] > filter_pallet_line_z_min && p[2] < filter_pallet_line_z_max
                                    ){

                                        pallet_top_line.emplace_back(geometry::float3{p[0],p[1],p[2]});

                                    }


                                }
                                MLOGI("pallet_space_move_mean_x, %f, %f, %f",pallet_space_left_move_mean_x,pallet_space_center_move_mean_x,pallet_space_right_move_mean_x);
                                MLOGI("get pocket space cluster: %zu, %zu, %zu, %zu, %zu", pallet_space_left.size(), pallet_space_center.size(), pallet_space_right.size(),pallet_pocket_left.size(),pallet_pocket_right.size());

                                {

                                    float pallet_space_left_sum_x = 0.0, pallet_space_left_sum_y = 0.0, pallet_space_left_sum_z = 0.0;
                                    for( auto&p : pallet_space_left){
                                        pallet_space_left_sum_x += p.x;
                                        pallet_space_left_sum_y += p.y;
                                        pallet_space_left_sum_z += p.z;
                                    }

                                    float pallet_space_center_sum_x = 0.0, pallet_space_center_sum_y = 0.0, pallet_space_center_sum_z = 0.0;
                                    for( auto&p : pallet_space_center){
                                        pallet_space_center_sum_x += p.x;
                                        pallet_space_center_sum_y += p.y;
                                        pallet_space_center_sum_z += p.z;
                                    }

                                    float pallet_space_right_sum_x = 0.0, pallet_space_right_sum_y = 0.0, pallet_space_right_sum_z = 0.0;
                                    for( auto&p : pallet_space_right){
                                        pallet_space_right_sum_x += p.x;
                                        pallet_space_right_sum_y += p.y;
                                        pallet_space_right_sum_z += p.z;
                                    }
                                    float pallet_space_left_mean_x = pallet_space_left_sum_x / static_cast<float >(pallet_space_left.size());
                                    float pallet_space_left_mean_y = pallet_space_left_sum_y / static_cast<float >(pallet_space_left.size());
                                    float pallet_space_left_mean_z = pallet_space_left_sum_z / static_cast<float >(pallet_space_left.size());

                                    float pallet_space_center_mean_x = pallet_space_center_sum_x / static_cast<float >(pallet_space_center.size());
                                    float pallet_space_center_mean_y = pallet_space_center_sum_y / static_cast<float >(pallet_space_center.size());
                                    float pallet_space_center_mean_z = pallet_space_center_sum_z / static_cast<float >(pallet_space_center.size());

                                    float pallet_space_right_mean_x = pallet_space_right_sum_x / static_cast<float >(pallet_space_right.size());
                                    float pallet_space_right_mean_y = pallet_space_right_sum_y / static_cast<float >(pallet_space_right.size());
                                    float pallet_space_right_mean_z = pallet_space_right_sum_z / static_cast<float >(pallet_space_right.size());


                                    geometry::float3 pallet_space_left_point{
                                            pallet_space_left_mean_x,
                                            pallet_space_left_mean_y,
                                            pallet_space_left_mean_z
                                    };

                                    geometry::float3 pallet_space_center_point{
                                            pallet_space_center_mean_x,
                                            pallet_space_center_mean_y,
                                            pallet_space_center_mean_z
                                    };

                                    geometry::float3 pallet_space_right_point{
                                            pallet_space_right_mean_x,
                                            pallet_space_right_mean_y,
                                            pallet_space_right_mean_z
                                    };

                                    float all_sum_x = pallet_space_left_sum_x + pallet_space_center_sum_x + pallet_space_right_sum_x;
                                    float all_sum_y = pallet_space_left_sum_y + pallet_space_center_sum_y + pallet_space_right_sum_y;
                                    float all_sum_z = pallet_space_left_sum_z + pallet_space_center_sum_z + pallet_space_right_sum_z;
                                    float all_mean_x = all_sum_x/static_cast<float>(pallet_space_left.size() + pallet_space_center.size() + pallet_space_right.size());
                                    float all_mean_y = all_sum_y/static_cast<float>(pallet_space_left.size() + pallet_space_center.size() + pallet_space_right.size());
                                    float all_mean_z = all_sum_z/static_cast<float>(pallet_space_left.size() + pallet_space_center.size() + pallet_space_right.size());

                                    bool space_center_to_line_dist_ok = true;


                                    if( pallet_space_width_center > 0.0){

                                        float center_dist = geometry::distanceToLine(pallet_space_left_point, pallet_space_right_point - pallet_space_left_point,pallet_space_center_point);
                                        MLOGI("refined pallet space center_dist: %f ",center_dist);
                                        space_center_to_line_dist_ok = std::abs(center_dist) < pallet_space_center_to_line_dist_max;

                                    }

                                    std::vector<geometry::float3> data_3_space[3] = {pallet_space_left,pallet_space_center, pallet_space_right};
                                    std::vector<geometry::float3> data_3_space_and_top[4] = {pallet_space_left,pallet_space_center, pallet_space_right,pallet_top_line };
                                    std::vector<geometry::float3> data_top[1] = { pallet_top_line };

                                    f32_t tx = all_mean_x, ty = all_mean_y,tz = all_mean_z,nx,ny,nz,nd ;

                                    compute_points_vec_norm2d(data_3_space,3,-1.0,0.0, tx, ty, tz, nx, ny, nz ,nd);
                                    MLOGW("refined norm2d:[%f,%f,%f],[%f,%f,%f,%f] ", tx, ty,tz,nx,ny,nz,nd)

                                    f32_t pallet_dir_3_space = angle_normal( std::atan2(ny, nx) + M_PIf32);
                                    MLOGI("refined pallet_dir_3_space: %f", pallet_dir_3_space);

                                    compute_points_vec_norm2d(data_3_space_and_top,4,-1.0,0.0, tx, ty, tz, nx, ny, nz ,nd);
                                    MLOGW("refined norm2d:[%f,%f,%f],[%f,%f,%f,%f] ", tx, ty,tz,nx,ny,nz,nd)

                                    f32_t pallet_dir_3_space_and_top = angle_normal( std::atan2(ny, nx) + M_PIf32);
                                    MLOGI("refined pallet_dir_3_space_and_top: %f", pallet_dir_3_space_and_top);

                                    compute_points_vec_norm2d(data_top,1,-1.0,0.0, tx, ty, tz, nx, ny, nz ,nd);
                                    MLOGW("refined norm2d:[%f,%f,%f],[%f,%f,%f,%f] ", tx, ty,tz,nx,ny,nz,nd)

                                    f32_t pallet_dir_top = angle_normal( std::atan2(ny, nx) + M_PIf32);
                                    MLOGI("refined pallet_dir_top: %f", pallet_dir_3_space_and_top);


                                    // find weighted mean dir
                                    float weighted_mean_dir = (0.0f + pallet_dir_top*projector_dir_weight_t + pallet_dir_3_space_and_top*projector_dir_weight_p3t
                                                               + pallet_dir_3_space*projector_dir_weight_p3)/(projector_dir_weight_0 + projector_dir_weight_t+projector_dir_weight_p3t+ projector_dir_weight_p3);
                                    MLOGI("refined weighted_mean_dir: %f", weighted_mean_dir);

                                    bool space_dir_valid = std::abs(pallet_dir_top) < pallet_space_direction_diff_max
                                                           && std::abs(pallet_dir_3_space_and_top) < pallet_space_direction_diff_max
                                                           && std::abs(pallet_dir_3_space) < pallet_space_direction_diff_max
                                                           && std::abs(weighted_mean_dir) < pallet_space_direction_diff_max;


                                    __attribute__ ((aligned (32))) auto    relative_pose = transform::createSe3(0.0, 0.0, 0.0 ,0.0,0.0,static_cast<double>(weighted_mean_dir) );


                                    cluster.est_pose_project = relative_pose;
                                    cluster.est_pose_project_inv = relative_pose.inverse();

                                    cluster.est_pose = cluster.est_pose* relative_pose;
                                    cluster.est_pose_inv = cluster.est_pose.inverse();

                                    std::cout << "relative_pose:\n" << relative_pose.matrix() <<"\n";
                                    std::cout << "cluster.est_pose:\n" << cluster.est_pose.matrix() <<"\n";
                                    std::cout << "cluster.est_pose_inv:\n" << cluster.est_pose_inv.matrix() <<"\n";



                                    bool space_count_valid = pallet_space_left_valid_count > pallet_space_valid_num
                                                             && pallet_space_center_valid_count> pallet_space_valid_num
                                                             && pallet_space_right_valid_count > pallet_space_valid_num;


                                    bool empty_pocket_ok = ((pallet_pocket_left_valid_count  < pallet_pocket_max_num)
                                                            || (pallet_pocket_right_valid_count < pallet_pocket_max_num)
                                                           )|| (  pallet_pocket_left_x > pallet_pocket_empty_x
                                                                  && pallet_pocket_right_x > pallet_pocket_empty_x
                                                           )
                                    ;

                                    bool detect_all_cond_valid = space_count_valid &&  space_dir_valid && space_center_to_line_dist_ok && empty_pocket_ok;
                                    MLOGI("space valid condition: detect_all_cond_valid: %i, space_count_valid:%i, space_dir_valid:%i, space_center_to_line_dist_ok:%i, empty_pocket_ok:%i ", detect_all_cond_valid, space_count_valid,space_dir_valid,space_center_to_line_dist_ok,empty_pocket_ok);

                                    if(detect_all_cond_valid){
                                        cluster.valid_status = 1;

                                    }else{
                                        cluster.valid_status = -1;
                                        continue;
                                    }





                                }










                            }

                        }


                        if(output_mode == 1){
                            // start projector


                            int valid_num = pallet_top_line.size() + pallet_space_left.size() +pallet_space_center.size() + pallet_space_right.size();
                            int valid_num_count = 0;

                            for( auto&p : pallet_space_left){
                                pallet_pocket_buffer[valid_num_count*3 ] = p.x;
                                pallet_pocket_buffer[valid_num_count*3 + 1 ] = p.y;
                                pallet_pocket_buffer[valid_num_count*3 + 2] = p.z;


                                valid_num_count++;
                            }
                            for( auto&p : pallet_space_center){
                                pallet_pocket_buffer[valid_num_count*3 ] = p.x;
                                pallet_pocket_buffer[valid_num_count*3 + 1 ] = p.y;
                                pallet_pocket_buffer[valid_num_count*3 + 2] = p.z;

                                valid_num_count++;
                            }

                            for( auto&p : pallet_space_right){
                                pallet_pocket_buffer[valid_num_count*3 ] = p.x;
                                pallet_pocket_buffer[valid_num_count*3 + 1 ] = p.y;
                                pallet_pocket_buffer[valid_num_count*3 + 2] = p.z;
                                valid_num_count++;
                            }
                            for( auto&p : pallet_top_line){
                                pallet_pocket_buffer[valid_num_count*3 ] = p.x;
                                pallet_pocket_buffer[valid_num_count*3 + 1 ] = p.y;
                                pallet_pocket_buffer[valid_num_count*3 + 2] = p.z;
                                valid_num_count++;
                            }




                            f64_t tx, ty, tz, roll, pitch,yaw;

                            transform::extractSe3(cluster.est_pose_project_inv,tx, ty, tz, roll, pitch,yaw );
                            MLOGI("est_pose_project_inv[%i]: tx: %f, ty: %f, tz: %f, roll: %f, pitch: %f,yaw: %f",i, tx, ty, tz, roll, pitch,yaw )

                            pointcloud_transform(pallet_pocket_buffer , valid_num, pallet_project_buffer, tx, ty, tz, roll, pitch, yaw );
                            //
                            pallet_projector_index_buffer_vec.resize(valid_num);

                            for(int ia = 0 ; ia < valid_num; ia++){

                                pallet_projector_index_buffer_vec[ia][0] =static_cast<i32_t>((pallet_project_buffer[ia * 3 + 1] + projector_offset_y) *
                                                                                             projector_resolution_inv);

                                pallet_projector_index_buffer_vec[ia][1] =static_cast<i32_t>((pallet_project_buffer[ia * 3 + 2] + projector_offset_z) *
                                                                                             projector_resolution_inv);
                            }
                            std::sort(pallet_projector_index_buffer_vec.begin(),
                                      pallet_projector_index_buffer_vec.end(),
                                      [](auto& v1, auto& v2){


                                          return (v1[0]*3935559000370003845+ v1[1] ) < (v2[0]*3935559000370003845+ v2[1] );
                                      } );


                            pallet_projector_index_buffer_vec.erase( std::unique( pallet_projector_index_buffer_vec.begin(), pallet_projector_index_buffer_vec.end() ), pallet_projector_index_buffer_vec.end() );


                            valid_num = pallet_projector_index_buffer_vec.size();
                            for(int ia = 0 ; ia < valid_num; ia++){

                                pallet_project_buffer[ia * 3 ] = 0.0;
                                pallet_project_buffer[ia * 3 + 1 ] = static_cast<float>(pallet_projector_index_buffer_vec[ia][0]) * projector_resolution - projector_offset_y;
                                pallet_project_buffer[ia * 3 + 2] = static_cast<float>(pallet_projector_index_buffer_vec[ia][1]) * projector_resolution - projector_offset_z;
                            }


                            // start match
                            int best_core_id_y = 0,best_core_id_z = 0 ;
                            f32_t best_score = -100.0;


                            std::cout << "compute score:\n";
                            MLOGI("get match score [%i, %i], [%i, %i]\n",-projector_search_z_range_len, projector_search_z_range_len,-projector_search_y_range_len,projector_search_y_range_len );

                            for( int iz = - projector_search_z_range_len ; iz <projector_search_z_range_len; iz++ ){


                                int izz = (iz + projector_search_z_range_len)*projector_search_y_range_num;
                                for( int iy = -projector_search_y_range_len; iy < projector_search_y_range_len; iy++ ){


                                    int match_count = 0;
                                    for(int ia = 0 ; ia < valid_num; ia++){


                                        int yy = pallet_projector_index_buffer_vec[ia][0] + iy;
                                        int zz = pallet_projector_index_buffer_vec[ia][1] + iz;


                                        match_count += (yy > project_template_top_index[0]
                                                        && yy < project_template_top_index[1]
                                                        && zz > project_template_top_index[2]
                                                        && zz < project_template_top_index[3])
                                                       || (yy > project_template_left_index[0]
                                                           && yy < project_template_left_index[1]
                                                           && zz > project_template_left_index[2]
                                                           && zz < project_template_left_index[3])
                                                       || (yy > project_template_center_index[0]
                                                           && yy < project_template_center_index[1]
                                                           && zz > project_template_center_index[2]
                                                           && zz < project_template_center_index[3])
                                                       || (yy > project_template_right_index[0]
                                                           && yy < project_template_right_index[1]
                                                           && zz > project_template_right_index[2]
                                                           && zz < project_template_right_index[3])
                                                ;

                                    }

                                    float p = static_cast<float>(match_count)*projector_template_pixel_count_inv ;

                                    float score = (p - projector_template_q )*projector_template_q_inv;


                                    pallet_project_template_score_buffer[izz + iy + projector_search_y_range_len ] = score;



                                    if (score > best_score){
                                        best_score = score;
                                        best_core_id_y = iy;
                                        best_core_id_z = iz;
                                    }
                                }
                            }


                            MLOGI("best index [%i, %i],best score %f, %f",best_core_id_y,best_core_id_z,best_score,projector_similarity_min   );

                            if ( best_score > projector_similarity_min){

                                cluster.valid_status = 2;

                            }else{
                                cluster.valid_status = -1;
                                continue;
                            }

                            f64_t template_match_best_y = static_cast<f64_t>(-best_core_id_y) * projector_resolution;
                            f64_t template_match_best_z = static_cast<f64_t>(-best_core_id_z) * projector_resolution;

                            MLOGI("template_match_best  [%f, %f], %f",template_match_best_y,template_match_best_z,best_score   );

                            __attribute__ ((aligned (32))) auto relative_pose = transform::createSe3(0.0, template_match_best_y, template_match_best_z ,0.0,0.0,0.0 );

                            cluster.est_pose =  cluster.est_pose*relative_pose;

                            {
                                f64_t tx, ty, tz, roll, pitch, yaw;
                                transform::extractSe3(cluster.est_pose , tx, ty, tz, roll, pitch, yaw );
                                cluster.pallet_pose_center.x = tx;
                                cluster.pallet_pose_center.y = ty;
                                cluster.pallet_pose_center.z = tz;
                                cluster.pallet_pose_yaw = yaw;
                            }
                            cluster.est_pose_inv =  cluster.est_pose.inverse();
                            cluster.confidence = best_score ;
                            std::cout << "final pallet pose:\n" << cluster.est_pose.matrix()  << "\n";

                        }
                        else{


                            if ( (output_mode -30) == i){

                                int valid_num = pallet_top_line.size() + pallet_space_left.size() +pallet_space_center.size() + pallet_space_right.size();


                                int valid_num_count = 0;

                                for( auto&p : pallet_space_left){
                                    pallet_pocket_buffer[valid_num_count*3 ] = p.x;
                                    pallet_pocket_buffer[valid_num_count*3 + 1 ] = p.y;
                                    pallet_pocket_buffer[valid_num_count*3 + 2] = p.z;


                                    valid_num_count++;
                                }

                                for( auto&p : pallet_space_center){
                                    pallet_pocket_buffer[valid_num_count*3 ] = p.x;
                                    pallet_pocket_buffer[valid_num_count*3 + 1 ] = p.y;
                                    pallet_pocket_buffer[valid_num_count*3 + 2] = p.z;

                                    valid_num_count++;
                                }

                                for( auto&p : pallet_space_right){
                                    pallet_pocket_buffer[valid_num_count*3 ] = p.x;
                                    pallet_pocket_buffer[valid_num_count*3 + 1 ] = p.y;
                                    pallet_pocket_buffer[valid_num_count*3 + 2] = p.z;
                                    valid_num_count++;
                                }
                                for( auto&p : pallet_top_line){
                                    pallet_pocket_buffer[valid_num_count*3 ] = p.x;
                                    pallet_pocket_buffer[valid_num_count*3 + 1 ] = p.y;
                                    pallet_pocket_buffer[valid_num_count*3 + 2] = p.z;
                                    valid_num_count++;
                                }



                                f64_t tx, ty, tz, roll, pitch,yaw;

                                transform::extractSe3(cluster.est_pose_project_inv,tx, ty, tz, roll, pitch,yaw );
                                MLOGI("est_pose_project_inv[%i]: tx: %f, ty: %f, tz: %f, roll: %f, pitch: %f,yaw: %f",i, tx, ty, tz, roll, pitch,yaw )

                                pointcloud_transform(pallet_pocket_buffer , valid_num, pallet_project_buffer, tx, ty, tz, roll, pitch, yaw );



                                output_cloud_buffer.buffer = pallet_project_buffer;
                                output_cloud_buffer.float_num = valid_num*3;
                                MLOGI("filter_ground use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));
                                return &output_cloud_buffer;
                            }

                            if ( (output_mode -40) == i){

                                int valid_num = pallet_top_line.size() + pallet_space_left.size() +pallet_space_center.size() + pallet_space_right.size();


                                int valid_num_count = 0;

                                for( auto&p : pallet_space_left){
                                    pallet_pocket_buffer[valid_num_count*3 ] = p.x;
                                    pallet_pocket_buffer[valid_num_count*3 + 1 ] = p.y;
                                    pallet_pocket_buffer[valid_num_count*3 + 2] = p.z;


                                    valid_num_count++;
                                }

                                for( auto&p : pallet_space_center){
                                    pallet_pocket_buffer[valid_num_count*3 ] = p.x;
                                    pallet_pocket_buffer[valid_num_count*3 + 1 ] = p.y;
                                    pallet_pocket_buffer[valid_num_count*3 + 2] = p.z;

                                    valid_num_count++;
                                }

                                for( auto&p : pallet_space_right){
                                    pallet_pocket_buffer[valid_num_count*3 ] = p.x;
                                    pallet_pocket_buffer[valid_num_count*3 + 1 ] = p.y;
                                    pallet_pocket_buffer[valid_num_count*3 + 2] = p.z;
                                    valid_num_count++;
                                }
                                for( auto&p : pallet_top_line){
                                    pallet_pocket_buffer[valid_num_count*3 ] = p.x;
                                    pallet_pocket_buffer[valid_num_count*3 + 1 ] = p.y;
                                    pallet_pocket_buffer[valid_num_count*3 + 2] = p.z;
                                    valid_num_count++;
                                }



                                f64_t tx, ty, tz, roll, pitch,yaw;

                                transform::extractSe3(cluster.est_pose_project_inv,tx, ty, tz, roll, pitch,yaw );
                                MLOGI("est_pose_project_inv[%i]: tx: %f, ty: %f, tz: %f, roll: %f, pitch: %f,yaw: %f",i, tx, ty, tz, roll, pitch,yaw )

                                pointcloud_transform(pallet_pocket_buffer , valid_num, pallet_project_buffer, tx, ty, tz, roll, pitch, yaw );
                                //
                                pallet_projector_index_buffer_vec.resize(valid_num);

                                for(int ia = 0 ; ia < valid_num; ia++){

                                    pallet_projector_index_buffer_vec[ia][0] =static_cast<i32_t>((pallet_project_buffer[ia * 3 + 1] + projector_offset_y) *
                                                                                                 projector_resolution_inv);

                                    pallet_projector_index_buffer_vec[ia][1] =static_cast<i32_t>((pallet_project_buffer[ia * 3 + 2] + projector_offset_z) *
                                                                                                 projector_resolution_inv);
                                }
                                std::sort(pallet_projector_index_buffer_vec.begin(),
                                          pallet_projector_index_buffer_vec.end(),
                                          [](auto& v1, auto& v2){


                                              return (v1[0]*3935559000370003845+ v1[1] ) < (v2[0]*3935559000370003845+ v2[1] );
                                          } );


                                pallet_projector_index_buffer_vec.erase( std::unique( pallet_projector_index_buffer_vec.begin(), pallet_projector_index_buffer_vec.end() ), pallet_projector_index_buffer_vec.end() );


                                valid_num = pallet_projector_index_buffer_vec.size();
                                for(int ia = 0 ; ia < valid_num; ia++){

                                    pallet_project_buffer[ia * 3 ] = 0.0;
                                    pallet_project_buffer[ia * 3 + 1 ] = static_cast<float>(pallet_projector_index_buffer_vec[ia][0]) * projector_resolution - projector_offset_y;
                                    pallet_project_buffer[ia * 3 + 2] = static_cast<float>(pallet_projector_index_buffer_vec[ia][1]) * projector_resolution - projector_offset_z;
                                }


                                int best_core_id_y = 0,best_core_id_z = 0 ;
                                f32_t best_score = -100.0;


                                std::cout << "compute score:\n";
                                MLOGI("get match score [%i, %i], [%i, %i]\n",-projector_search_z_range_len, projector_search_z_range_len,-projector_search_y_range_len,projector_search_y_range_len );
                                for( int iz = - projector_search_z_range_len ; iz <projector_search_z_range_len; iz++ ){


                                    int izz = (iz + projector_search_z_range_len)*projector_search_y_range_num;
                                    for( int iy = -projector_search_y_range_len; iy < projector_search_y_range_len; iy++ ){


                                        int match_count = 0;
                                        for(int ia = 0 ; ia < valid_num; ia++){


                                            int yy = pallet_projector_index_buffer_vec[ia][0] + iy;
                                            int zz = pallet_projector_index_buffer_vec[ia][1] + iz;


                                            match_count += (yy > project_template_top_index[0]
                                                            && yy < project_template_top_index[1]
                                                            && zz > project_template_top_index[2]
                                                            && zz < project_template_top_index[3])
                                                           || (yy > project_template_left_index[0]
                                                               && yy < project_template_left_index[1]
                                                               && zz > project_template_left_index[2]
                                                               && zz < project_template_left_index[3])
                                                           || (yy > project_template_center_index[0]
                                                               && yy < project_template_center_index[1]
                                                               && zz > project_template_center_index[2]
                                                               && zz < project_template_center_index[3])
                                                           || (yy > project_template_right_index[0]
                                                               && yy < project_template_right_index[1]
                                                               && zz > project_template_right_index[2]
                                                               && zz < project_template_right_index[3])
                                                    ;

                                        }

                                        float p = static_cast<float>(match_count)*projector_template_pixel_count_inv ;

                                        float score = (p - projector_template_q )*projector_template_q_inv;


                                        pallet_project_template_score_buffer[izz + iy + projector_search_y_range_len ] = score;
                                        projector_output_img_buffer[ izz + iy + projector_search_y_range_len] = static_cast<u8_t>(
                                                p*255 );
                                        if((iz == 0) && (iy + projector_search_y_range_len < 10 )  ){
                                            projector_output_img_buffer[ izz + iy + projector_search_y_range_len] = static_cast<u8_t>(
                                                    255 );
                                        }


                                        printf("[%i, %.3f, %.3f], ", match_count,p, score);
                                        if (score > best_score){
                                            best_score = score;
                                            best_core_id_y = iy;
                                            best_core_id_z = iz;
                                        }
                                    }

                                }


                                //encode and save

                                lodepng::State state;
                                state.info_raw.colortype = LCT_GREY;
                                state.info_raw.bitdepth = 8;

                                unsigned error = lodepng::encode(projector_output_img_encode_buffer, projector_output_img_buffer,projector_search_y_range_num,  projector_search_z_range_num,state);
                                if(error) std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
                                else lodepng::save_file(projector_output_img_encode_buffer, "/tmp/match.png");


                                MLOGI("best core [%i, %i], %f",best_core_id_y,best_core_id_z,best_score   );

                                if ( best_score > projector_similarity_min){

                                    cluster.valid_status = 2;

                                }

                                f64_t template_match_best_y = static_cast<f64_t>(-best_core_id_y) * projector_resolution;
                                f64_t template_match_best_z = static_cast<f64_t>(-best_core_id_z) * projector_resolution;

                                MLOGI("template_match_best  [%f, %f], %f",template_match_best_y,template_match_best_z,best_score   );

                                __attribute__ ((aligned (32))) auto relative_pose = transform::createSe3(0.0, template_match_best_y, template_match_best_z ,0.0,0.0,0.0 );

                                cluster.est_pose =  cluster.est_pose*relative_pose;

                                {
                                    f64_t tx, ty, tz, roll, pitch, yaw;
                                    transform::extractSe3(cluster.est_pose , tx, ty, tz, roll, pitch, yaw );
                                    cluster.pallet_pose_center.x = tx;
                                    cluster.pallet_pose_center.y = ty;
                                    cluster.pallet_pose_center.z = tz;
                                    cluster.pallet_pose_yaw = yaw;
                                }
                                cluster.est_pose_inv =  cluster.est_pose.inverse();
                                cluster.confidence = best_score ;
                                std::cout << "final pallet pose:\n" << cluster.est_pose.matrix()  << "\n";




                                output_cloud_buffer.buffer = pallet_project_buffer;
                                output_cloud_buffer.float_num = valid_num*3;
                                MLOGI("filter_ground use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));
                                return &output_cloud_buffer;
                            }

                            if ( (output_mode -20) == i){
                                output_cloud_buffer.buffer = pallet_pocket_buffer;
                                output_cloud_buffer.float_num = float_num;
                                MLOGI("filter_ground use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));
                                return &output_cloud_buffer;
                            }
                            if ( (output_mode -10) == i){
                                output_cloud_buffer.buffer = pallet_pocket_buffer;
                                output_cloud_buffer.float_num = (cluster.valid_status == 1 )? float_num: 3;
                                MLOGI("filter_ground use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));
                                return &output_cloud_buffer;
                            }
                        }

                    }
                }
            }else{
                filter_pallet_status = -4;

                return 0;
            }


        }

        if(pallet_cluster.empty()){
            filter_pallet_status = -4;

            return 0;
        }



        pallet_cluster.erase(std::remove_if(pallet_cluster.begin(), pallet_cluster.end(),[](auto&x){
            return x.valid_status <= 0;
        }), pallet_cluster.end() );

        if(pallet_cluster.empty()){
            filter_pallet_status = -4;

            return 0;
        }

        std::sort(pallet_cluster.begin(), pallet_cluster.end(),[](auto& v1, auto& v2){
            return v1.pallet_pose_center.z < v2.pallet_pose_center.z;
        });

        if(pallet_cluster.size() > 1){
            int start_id = 0;
            for(int i = 1 ;i < pallet_cluster.size() ;i++){
                if( (pallet_cluster[i].pallet_pose_center.z - pallet_cluster[start_id].pallet_pose_center.z) < pallet_top_tail_height ){
                    pallet_cluster[i].valid_status = -1;
                }else{
                    start_id = i;
                }

            }
        }

        pallet_cluster.erase(std::remove_if(pallet_cluster.begin(), pallet_cluster.end(),[](auto&x){
            return x.valid_status <= 0;
        }), pallet_cluster.end() );

        if(pallet_cluster.empty()){

            filter_pallet_status = -4;
            return 0;
        }
        filter_pallet_status = 1;

        MLOGI("pallet_cluster.size:%zu",pallet_cluster.size());
        for(auto& p: pallet_cluster){
            MLOGI("center: [%f, %f, %f], yaw: %f", p.pallet_pose_center.x, p.pallet_pose_center.y,  p.pallet_pose_center.z,  p.pallet_pose_yaw );
        }




        return 0;

    }

    PointCloudBuffer_ptr PalletDetector::filter_ground(u32_t output_mode) {

        common::Time start_time = common::FromUnixNow();
        // computer center and norm for given initial ground window, within thresh

        int cloud_dim_height = config.cloud.dim.height;
        int cloud_dim_width = config.cloud.dim.width;
        int search_direction = config.filter_ground.search_direction;

        int init_ground_height_min =  config.filter_ground.init_ground_height_min;
        int init_ground_height_max = config.filter_ground.init_ground_height_max > cloud_dim_height ? cloud_dim_height : config.filter_ground.init_ground_height_max;

        int init_ground_width_min =  config.filter_ground.init_ground_width_min;
        int init_ground_width_max = config.filter_ground.init_ground_width_max > cloud_dim_width ? cloud_dim_width : config.filter_ground.init_ground_width_max ;

        float init_ground_nz_min = config.filter_ground.init_ground_nz_min;

        int init_ground_height = init_ground_height_max - init_ground_height_min;
        int init_ground_width = init_ground_width_max - init_ground_width_min;

        float adaptive_z_thresh_min = config.filter_ground.adaptive_z_min;
        float adaptive_z_thresh_max = config.filter_ground.adaptive_z_max;
        float  far_uncertain_z_max = config.filter_ground.far_uncertain_z_max;
        float  far_uncertain_x_change_min = config.filter_ground.far_uncertain_x_change_min;
        float far_uncertain_adaptive_z_max = config.filter_ground.far_uncertain_adaptive_z_max;

        int far_uncertain_row = config.filter_ground.far_uncertain_row > init_ground_height ? init_ground_height:config.filter_ground.far_uncertain_row ;

        int row_scan_fold = 4;



        MLOGI("init_ground_height  : [%i, %i]", init_ground_height_min ,init_ground_height_max);
        MLOGI("init_ground_width  : [%i, %i]", init_ground_width_min,init_ground_width_max );

        MLOGI("init_ground_dim : [%i, %i}]",init_ground_height, init_ground_width );
        MLOGI("cloud_dim : [%i, %i]",cloud_dim_height, cloud_dim_width );
        if( init_ground_height == 0
        || init_ground_width == 0
        ){
            MLOGW("init_ground_dim contains zero value: [%i, %i}]",init_ground_height, init_ground_width );

            filter_ground_status = -1;
            return 0;
        }


        ground_init_buffer = (f32_t*)ta_realloc(&mem_cfg,ground_init_buffer, init_ground_height*init_ground_width*3*4);
        ground_output_buffer = (f32_t*)ta_realloc(&mem_cfg,ground_output_buffer, cloud_dim_height*cloud_dim_width*3*4);

        cloud_label_table = (i8_t*) ta_realloc(&mem_cfg,cloud_label_table, cloud_dim_height*cloud_dim_width);
        ground_pixel_count_buffer = (i32_t*)ta_realloc(&mem_cfg,ground_pixel_count_buffer,cloud_dim_height*row_scan_fold);


        if(
                !ground_init_buffer
                || !ground_output_buffer
                || !cloud_label_table
                || !ground_pixel_count_buffer

        ){
            MLOGW("PalletDetector: create buffer fail: %p, %p, %p, %p", ground_init_buffer, ground_output_buffer,cloud_label_table,ground_pixel_count_buffer);
            filter_ground_status = -2;
            return nullptr;
        }


        if(output_mode == 0){
            memcpy(ground_output_buffer, cloud_buffer, cloud_dim_height*cloud_dim_width*3*4);
            for(int i = 0 ; i < cloud_dim_height;i++){
                for(int j = 0 ;j < cloud_dim_width;j++){
                    int k = (i * cloud_dim_width + j)*3;
//                    const f32_t *p = cloud_buffer + k;
                    f32_t *p_dst = ground_output_buffer + k;

//                Eigen::Matrix <float, 3, 1> vp (p[0] -cx, p[1] - cy, p[2] - cz);
//                float cos_theta = vp.dot (normal);
//                    p_dst[0] = p[0];
//                    p_dst[1] = p[1];
                    // dist alone norm
//                p_dst[2] = cos_theta;//std::abs(cos_theta) < 0.05 ? 0.0 : cos_theta;
                    // z sub
                    p_dst[2] = p_dst[2] < adaptive_z_thresh_max ? p_dst[2] - 0.5 : p_dst[2];

                }
            }



            filter_ground_status = 0;


            output_cloud_buffer.buffer = ground_output_buffer;
            output_cloud_buffer.float_num = cloud_dim_height * cloud_dim_width * 3;
            MLOGI("filter_ground use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));

            return &output_cloud_buffer;

        }


        memset(cloud_label_table,0,cloud_dim_height*cloud_dim_width);
        memset(ground_pixel_count_buffer,0,cloud_dim_height*row_scan_fold);



        f32_t init_ground_x_min = config.filter_ground.init_ground_cx_min;
        f32_t init_ground_x_max = config.filter_ground.init_ground_cx_max;
        f32_t init_ground_y_min = config.filter_ground.init_ground_cy_min;
        f32_t init_ground_y_max = config.filter_ground.init_ground_cy_max;
        f32_t init_ground_z_min = config.filter_ground.init_ground_cz_min;
        f32_t init_ground_z_max = config.filter_ground.init_ground_cz_max;

        MLOGI("init_ground_thresh : [%f, %f],[%f, %f],[%f, %f] ",init_ground_x_min, init_ground_x_max,init_ground_y_min, init_ground_y_max,init_ground_z_min, init_ground_z_max );

        int valid_num = 0;
        for( int i = init_ground_height_min; i < init_ground_height_max;i++){

            for(int j = init_ground_width_min ; j < init_ground_width_max;j++ ){

                int k = (i * cloud_dim_width + j)*3;
                const f32_t *p = cloud_buffer + k;

                bool valid = p[0] > init_ground_x_min && p[0] < init_ground_x_max
                             && p[1] > init_ground_y_min && p[1] < init_ground_y_max
                             && p[2] > init_ground_z_min && p[2] < init_ground_z_max;

                ground_init_buffer[valid_num*3] = p[0];
                ground_init_buffer[valid_num*3 + 1] = p[1];
                ground_init_buffer[valid_num*3 + 2] = p[2];
                valid_num += valid;
            }
        }
        MLOGW("valid_num: %i ", valid_num );

        if(valid_num < config.filter_ground.init_valid_num_min){
            MLOGW("valid_num: %i < %i", valid_num, config.filter_ground.init_valid_num_min);
            memcpy(ground_output_buffer, cloud_buffer, cloud_dim_height*cloud_dim_width*3*4);
            for(int i = 0 ; i < cloud_dim_height;i++){
                for(int j = 0 ;j < cloud_dim_width;j++){
                    int k = (i * cloud_dim_width + j)*3;
//                    const f32_t *p = cloud_buffer + k;
                    f32_t *p_dst = ground_output_buffer + k;

//                Eigen::Matrix <float, 3, 1> vp (p[0] -cx, p[1] - cy, p[2] - cz);
//                float cos_theta = vp.dot (normal);
//                    p_dst[0] = p[0];
//                    p_dst[1] = p[1];
                    // dist alone norm
//                p_dst[2] = cos_theta;//std::abs(cos_theta) < 0.05 ? 0.0 : cos_theta;
                    // z sub
                    p_dst[2] = p_dst[2] < adaptive_z_thresh_max ? p_dst[2] - 0.5 : p_dst[2];

                }
            }

            filter_ground_status = 0;
            return &output_cloud_buffer;

        }

        f32_t init_ground_x_sum = 0.0;
        f32_t init_ground_y_sum = 0.0;
        f32_t init_ground_z_sum = 0.0;

        f32_t valid_num_inv = 1.0 /f32_t(valid_num);
        for(int i = 0 ; i < valid_num;i++){
            init_ground_x_sum += ground_init_buffer[i*3];
            init_ground_y_sum += ground_init_buffer[i*3 + 1];
            init_ground_z_sum += ground_init_buffer[i*3 + 2];
        }
        f32_t  cx = init_ground_x_sum * valid_num_inv;
        f32_t  cy = init_ground_y_sum * valid_num_inv;
        f32_t  cz = init_ground_z_sum * valid_num_inv;


        MLOGI("find center: [%f, %f, %f]", cx, cy, cz);

        perception::NormalEst3d normalEst3D;
        normalEst3D.setViewerPoint(viewpoint_x,viewpoint_y,viewpoint_z);
        normalEst3D.addCenter(cx,cy,cz);
        for(int i = 0 ; i < valid_num;i++){
            normalEst3D.addPoint(ground_init_buffer[i*3],ground_init_buffer[i*3 + 1],ground_init_buffer[i*3 +2]);
        }

        f32_t cnx, cny, cnz, cnd;
        normalEst3D.compute(cnx, cny, cnz, cnd);
        Eigen::Matrix <float, 3, 1> normal (cnx, cny, cnz);

        MLOGI("find norm: [%f, %f, %f, %f]", cnx, cny, cnz,cnd);


        if (cnz < init_ground_nz_min
        ){
            MLOGI("find norm: [%f, %f, %f, %f] error with nz_limit [%f]", cnx, cny, cnz,cnd,init_ground_nz_min );
            memcpy(ground_output_buffer, cloud_buffer, cloud_dim_height*cloud_dim_width*3*4);
            for(int i = 0 ; i < cloud_dim_height;i++){
                for(int j = 0 ;j < cloud_dim_width;j++){
                    int k = (i * cloud_dim_width + j)*3;
//                    const f32_t *p = cloud_buffer + k;
                    f32_t *p_dst = ground_output_buffer + k;

//                Eigen::Matrix <float, 3, 1> vp (p[0] -cx, p[1] - cy, p[2] - cz);
//                float cos_theta = vp.dot (normal);
//                    p_dst[0] = p[0];
//                    p_dst[1] = p[1];
                    // dist alone norm
//                p_dst[2] = cos_theta;//std::abs(cos_theta) < 0.05 ? 0.0 : cos_theta;
                    // z sub
                    p_dst[2] = p_dst[2] < adaptive_z_thresh_max ? p_dst[2] - 0.5 : p_dst[2];

                }
            }

            filter_ground_status = 0;
            return &output_cloud_buffer;



        }


        for(int i = 0 ; i < cloud_dim_height;i++){
            for(int j = 0 ;j < cloud_dim_width;j++){
                int k = (i * cloud_dim_width + j)*3;
                const f32_t *p = cloud_buffer + k;
                f32_t *p_dst = ground_output_buffer + k;

//                Eigen::Matrix <float, 3, 1> vp (p[0] -cx, p[1] - cy, p[2] - cz);
//                float cos_theta = vp.dot (normal);
                p_dst[0] = p[0];
                p_dst[1] = p[1];
                // dist alone norm
//                p_dst[2] = cos_theta;//std::abs(cos_theta) < 0.05 ? 0.0 : cos_theta;
                // z sub
                p_dst[2] = p[2] - cz;//std::abs(cos_theta) < 0.05 ? 0.0 : cos_theta;

            }
        }




        filter_ground_status = 1;

        {
            MLOGI("search_direction: %i  : [%f, %f]", search_direction, adaptive_z_thresh_min ,adaptive_z_thresh_max);
            MLOGI("init_ground_height_min: %i, init_ground_height_max: %i, cloud_dim_height: %i",
                  init_ground_height_min, init_ground_height_max, cloud_dim_height);


            if(search_direction == 1){

                // max to min
                // cloud_dim_height init_ground_height_max  init_ground_height_min 0
                //                                                    |--> search direction
                for(int i = cloud_dim_height-1  ; i  > init_ground_height_min ;i--){
                    int i_w = i * cloud_dim_width;
                    for(int j = 0 ;j < cloud_dim_width;j++){
                        int k = (i_w + j);
                        int l = (i_w + j)*3;
//                        f32_t *p = cloud_buffer + l;
                        f32_t *p_dst = ground_output_buffer + l;
                        i8_t *t = cloud_label_table+k;
                        t[0] = 1;
//                        p_dst[2] = ((p_dst[2]> adaptive_z_thresh_min) && (p_dst[2] < adaptive_z_thresh_max) ) ? p_dst[2] - 0.5: p_dst[2];
//                        p_dst[2] -= 0.5;
                    }
                }

                for(int i =init_ground_height_min; i>=0; i-- ){
                    int i_w = i * cloud_dim_width;
                    int i_w_last = (i+far_uncertain_row) * cloud_dim_width;
                    int i_w_last_1 = (i+1) * cloud_dim_width;
                    int i_w_last_2 = (i+2) * cloud_dim_width;
                    int i_w_last_3 = (i+3) * cloud_dim_width;

                    for(int j = 0 ;j < cloud_dim_width;j++){
                        int k = (i_w + j);
                        int l = (i_w + j)*3;
                        int k_last = (i_w_last + j);
                        int l_last = (i_w_last + j)*3;

                        int k_last_1 = (i_w_last_1 + j);
                        int l_last_1 = (i_w_last_1 + j)*3;
                        int k_last_2 = (i_w_last_2 + j);
                        int l_last_2 = (i_w_last_2 + j)*3;
                        int k_last_3 = (i_w_last_3 + j);
                        int l_last_3 = (i_w_last_3 + j)*3;
//                        f32_t *p = cloud_buffer + l;
                        f32_t *p_dst = ground_output_buffer + l;
                        f32_t *p_dst_last= ground_output_buffer + l_last;
                        f32_t *p_dst_last_1= ground_output_buffer + l_last_1;
                        f32_t *p_dst_last_2= ground_output_buffer + l_last_2;
                        f32_t *p_dst_last_3= ground_output_buffer + l_last_3;


                        i8_t *t = cloud_label_table+k;
                        i8_t *t_last = cloud_label_table+k_last;
                        i8_t *t_last_1 = cloud_label_table+k_last_1;
                        i8_t *t_last_2 = cloud_label_table+k_last_2;
                        i8_t *t_last_3 = cloud_label_table+k_last_3;

                        bool is_ground = ((p_dst[2]> adaptive_z_thresh_min) && (p_dst[2] < adaptive_z_thresh_max) )
                                         || ((p_dst[2] <far_uncertain_z_max)
                                             //                                && ( t_last_1[0] == 1 && t_last_2[0]==1 && t_last_3[0]==1 )
                                             && (!(p_dst[2] > p_dst_last_1[2]
                                                   && p_dst_last_1[2] > p_dst_last_2[2]
                                                   && p_dst_last_2[2] > p_dst_last_3[2])
                                                 || ( (p_dst[0] -p_dst_last[0] ) > far_uncertain_x_change_min)
                                                 || ( (p_dst[0] -p_dst_last_3[0] ) > far_uncertain_x_change_min)
                                             )  )

                        ;

                        t[0] = is_ground;

//                        p_dst[2] = is_ground ? p_dst[2] - 0.5: p_dst[2];
//                        p_dst_last[2] = is_ground ? p_dst_last[2] - 0.5: p_dst_last[2];

//                        p_dst[2] -= 0.5;
                    }
                }

                float rolling_mean_z = cz;


                for(int i =init_ground_height_min; i>=0; i-- ){
                    int i_w = i * cloud_dim_width;
                    int i_w_last = (i+far_uncertain_row) * cloud_dim_width;
                    int i_w_last_1 = (i+1) * cloud_dim_width;
                    int i_w_last_2 = (i+2) * cloud_dim_width;
                    int i_w_last_3 = (i+3) * cloud_dim_width;

                    //rolling mean z
                    //
                    float row_rolling_mean_z_array[4] ={0.0,0.0,0.0,0.0};
//                    int ground_pixel_count[4] = {0,0,0,0};

                    for(int f = 0 ; f < row_scan_fold; f++){

                        int start = cloud_dim_width*(f)/row_scan_fold;
                        int end = cloud_dim_width*(f+1)/row_scan_fold;
                        i32_t ground_pixel_count = 0;

                        {
                            float row_rolling_mean_z = rolling_mean_z;
                            int row_rolling_mean_z_num = 1;

                            for(int j = start ;j < end;j++){
                                int k = (i_w + j);
                                int l = (i_w + j)*3;
                                f32_t *p_dst = ground_output_buffer + l;
                                i8_t *t = cloud_label_table+k;

                                row_rolling_mean_z_num +=  (t[0] == 1);
                                row_rolling_mean_z += p_dst[2]*(t[0] == 1);

                            }
                            row_rolling_mean_z /= float(row_rolling_mean_z_num);
                            row_rolling_mean_z_array[f] = row_rolling_mean_z;

                            for(int j = start ;j < end;j++){
                                int k = (i_w + j);
                                int l = (i_w + j)*3;
                                f32_t *p_dst = ground_output_buffer + l;
                                i8_t *t = cloud_label_table+k;

                                bool is_ground = (t[0] == 1) || ( (p_dst[2] -row_rolling_mean_z ) < far_uncertain_adaptive_z_max);

                                t[0] = is_ground;
                                ground_pixel_count += is_ground;

                            }
                        }
                        ground_pixel_count_buffer[i *4 + f] = ground_pixel_count ;

                    }
                    rolling_mean_z = row_rolling_mean_z_array[0];

                }

                for(int i = 0 ; i < cloud_dim_height;i++){
                    int i_w = i * cloud_dim_width;
                    for(int j = 0 ;j < cloud_dim_width;j++){
                        int k = (i_w + j);
                        int l = (i_w + j)*3;
                        const f32_t *p = cloud_buffer + l;
                        f32_t *p_dst = ground_output_buffer + l;
                        i8_t *t = cloud_label_table+k;


                        p_dst[2] = t[0] == 1 ? p_dst[2] - 0.5: p_dst[2];
//                    p_dst[2] = 0.0;
                    }
                }

            }else{
                filter_ground_status = -10;

                return 0;
            }
        }

        if (output_mode == 2){

            output_cloud_buffer.buffer = ground_init_buffer;
            output_cloud_buffer.float_num = valid_num * 3;
            MLOGI("filter_ground use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));

            return &output_cloud_buffer;

        }else if(output_mode == 1) {

            output_cloud_buffer.buffer = ground_output_buffer;
            output_cloud_buffer.float_num = cloud_dim_height * cloud_dim_width * 3;
            MLOGI("filter_ground use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));

            return &output_cloud_buffer;

        }
        return 0;
    }
}