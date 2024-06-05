//
// Created by waxz on 5/21/24.
//
#include "pointcloud_pallet_detect_impl.h"
#include "math/geometry/normal_estimation_3d.h"
#include "pointcloud_process.h"

#include <iostream>
#include "common/string_logger.h"
#include "common/clock_time.h"
#include "math/math_basic.h"
#include "math/geometry.h"


namespace perception{

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


        if( !ground_init_buffer || !ground_output_buffer){
            MLOGW("PalletDetector: create buffer fail: %p, %p", ground_init_buffer, ground_output_buffer);
            return -1;
        }
        return 0;
    }

    void PalletDetector::stop() {

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

    }

    void PalletDetector::set_vertical_init_dim(u64_t height_min, u64_t height_max, u64_t width_min, u64_t width_max) {
        config.filter_vertical.init_center_height_min = height_min;
        config.filter_vertical.init_center_height_max = height_max;
        config.filter_vertical.init_center_width_min = width_min;
        config.filter_vertical.init_center_width_max = width_max;
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

        if( filter_ground_status <= 0 ){
            MLOGW("filter_ground_status error : [%i}]",filter_ground_status );
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


                ){
            MLOGW("PalletDetector: create buffer fail: %p, %p, %p, %p, %p, %p, %p, %p, %p, %p, %p",
                  ground_init_buffer, ground_output_buffer,cloud_label_table,ground_pixel_count_buffer,vertical_center_buffer,vertical_output_buffer,vertical_center_index_buffer,vertical_center_index_row_valid_num_buffer,vertical_center_vertical_index_buffer,vertical_filter_index_buffer,vertical_output_index_buffer );
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


        if (output_mode == 0){

            output_cloud_buffer.buffer = vertical_center_buffer;
            output_cloud_buffer.float_num = init_center_height * init_center_width * 3;
            MLOGI("filter_vertical use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));
            return &output_cloud_buffer;

        }

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
            filter_vertical_status = -3;
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
                        for(int j1 = init_center_line_search_start_index ; j1 <center_line_search_window_len;j1++){
                            int j_left = find_valid_center_mark_index - j1;
                            int j_right = find_valid_center_mark_index +j1;

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
                                0
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
            vertical_filter_index_list.clear();
            vertical_filter_index_vec.clear();

            int center_line_markers_len = center_line_markers.size();
//            MLOGI("center_line_markers_len: %i",center_line_markers_len);
            for(int i = 0; i < center_line_markers_len;i++){
                auto& l1 = center_line_markers[i];
//                MLOGI("get center_line_markers[%i], point_left:[%f, %f, %f], point_center:[%f, %f, %f], point_right:[%f, %f, %f], dir_left_to_right:[%f, %f, %f], intersection:[%f, %f, %f]",i,l1.point_left.x,l1.point_left.y,l1.point_left.z ,l1.point_center.x,l1.point_center.y,l1.point_center.z ,l1.point_right.x,l1.point_right.y,l1.point_right.z,l1.dir_left_to_right.x,l1.dir_left_to_right.y,l1.dir_left_to_right.z,l1.intersection_to_a_axis.x,l1.intersection_to_a_axis.y,l1.intersection_to_a_axis.z );

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
                for(int j = 0 ; j < vertical_filter_index_buffer_num;j++){
                    float *ptr = src_buffer + vertical_filter_index_buffer[j]*3;

                    geometry::float3 point {
                        ptr[0],
                        ptr[1],
                        ptr[2]
                    };
                    f32_t dist_alone_norm = l1.filtered_line_dir.dot( point  - filter_line_norm_center);
                    dist_alone_norm_min = std::min(dist_alone_norm_min , dist_alone_norm);
                    dist_alone_norm_max = std::max(dist_alone_norm_max , dist_alone_norm);
//                    MLOGI("dist_alone_norm: [%f, %f, %f], dist_alone_norm_intersect: %f", point.x,point.y,point.z, dist_alone_norm);
                }
                float dist_alone_norm_max_len = dist_alone_norm_max - dist_alone_norm_min;
//                MLOGI("dist_alone_norm_max_len: %f, dist_alone_norm_min: %f, dist_alone_norm_max: %f ",dist_alone_norm_max_len, dist_alone_norm_min,dist_alone_norm_max);

                bool valid =  dist_alone_norm_max_len > init_center_line_filter_len_valid_min;
                if(!valid){
                    l1.valid_status = -1;
                }

                if (output_mode == 2){

                    for(int j = 0 ; j < vertical_filter_index_buffer_num;j++){
                        vertical_filter_index_list.push_back(vertical_filter_index_buffer[j] + src_buffer_offset);
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






        if (output_mode == 1){

            output_cloud_buffer.buffer = vertical_center_buffer;
            output_cloud_buffer.float_num = valid_num * 3;
            MLOGI("filter_vertical use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));

            return &output_cloud_buffer;

        }

        if (output_mode == 2){

            vertical_filter_index_vec.clear();
            std::copy(vertical_filter_index_list.begin(), vertical_filter_index_list.end(),std::back_inserter(vertical_filter_index_vec));

            std::sort(vertical_filter_index_vec.begin(), vertical_filter_index_vec.end());

            valid_num = vertical_filter_index_list.size();

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
            for(int i = 0; i < center_line_markers_len;i++) {
                auto &l1 = center_line_markers[i];

                auto& filtered_line_center = l1.filtered_line_center;
                auto & filtered_line_dir = l1.filtered_line_dir;
                for(int j = 0 ; j < step_num;j++){
                    auto p = filtered_line_center + filtered_line_dir*(j*step_resolution - 1.0);

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

    PointCloudBuffer_ptr PalletDetector::filter_pallet(u32_t output_mode) {

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
                f32_t *p = cloud_buffer + k;
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
            filter_ground_status = -3;
            return nullptr;
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
            MLOGI("find norm: [%f, %f, %f, %f] error with nz_limit [%f, %f]", cnx, cny, cnz,cnd,init_ground_nz_min );

        }


        for(int i = 0 ; i < cloud_dim_height;i++){
            for(int j = 0 ;j < cloud_dim_width;j++){
                int k = (i * cloud_dim_width + j)*3;
                f32_t *p = cloud_buffer + k;
                f32_t *p_dst = ground_output_buffer + k;

                Eigen::Matrix <float, 3, 1> vp (p[0] -cx, p[1] - cy, p[2] - cz);
                float cos_theta = vp.dot (normal);
                p_dst[0] = p[0];
                p_dst[1] = p[1];
                p_dst[2] = cos_theta;//std::abs(cos_theta) < 0.05 ? 0.0 : cos_theta;

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
                        f32_t *p = cloud_buffer + l;
                        f32_t *p_dst = ground_output_buffer + l;
                        i8_t *t = cloud_label_table+k;


                        p_dst[2] = t[0] == 1 ? p_dst[2] - 0.5: p_dst[2];
//                    p_dst[2] = 0.0;
                    }
                }

            }else{
                return 0;
            }
        }

        if (output_mode == 0){

            output_cloud_buffer.buffer = ground_init_buffer;
            output_cloud_buffer.float_num = valid_num * 3;
            MLOGI("filter_ground use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));

            return &output_cloud_buffer;

        }else {

            output_cloud_buffer.buffer = ground_output_buffer;
            output_cloud_buffer.float_num = cloud_dim_height * cloud_dim_width * 3;
            MLOGI("filter_ground use time: %ld ms\n", common::ToMillSeconds(common::FromUnixNow() - start_time));

            return &output_cloud_buffer;

        }
        return 0;
    }
}