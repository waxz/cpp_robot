//
// Created by waxz on 9/22/23.
//

#include "ros_helper.h"
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h> // usleep
#include <time.h>
#include <math.h>
#include "common/signals.h"

#include "tinyalloc/tinyalloc.h"

#include "message_center_handler.h"
#include "message_center_types.h"

#include "common/data_holder.h"
#include "common/string_logger.h"


static bool program_run = true;

void signal_handler(int sig) {
    printf("get signal %i\n", sig);
    program_run = false;
}



// 10000000 Byte = 10 MB
// 50000000 Byte = 50 MB

#define STATIC_MEMORY_SIZE 50000000
static unsigned char memory_pool[STATIC_MEMORY_SIZE];
static const ta_cfg_t memory_pool_cfg = {
        .base = memory_pool,
        .limit = &memory_pool[sizeof(memory_pool)],
        .max_blocks = 512,
        .split_thresh = 16,
        .alignment = 8,
};

void test(void ** p){
    *p = NULL;
}

void ta_print(){
    printf("ta_num_used : %lu\n",ta_num_used(&memory_pool_cfg));
    printf("ta_num_free : %lu\n",ta_num_free(&memory_pool_cfg));
    printf("ta_num_fresh : %lu\n",ta_num_fresh(&memory_pool_cfg));
    printf("ta_check : %i\n",ta_check(&memory_pool_cfg));
}

//void test_pool(){
//
//    MemPool pool;
//    pool = MemPool_create();
//
//    MemPool_set_nodes(&pool,10);
//
//    {
//        LaserScanT scan = LaserScanT_create();
//        scan.angle_min = 10;
//        scan.angle_max = 100;
//
//        LaserScanT * scan_ptr = MemPool_get(&pool,&scan);
//        printf("get:%.3f,%.3f\n",scan_ptr->angle_min,scan_ptr->angle_max );
//    }
//    {
//        PoseStampedT tf_data = PoseStampedT_create();
//        tf_data.position.x = 123;
//        PoseStampedT * tf_ptr = MemPool_get(&pool,&tf_data);
//        printf("get:%.3f\n",tf_ptr->position.x);
//    }
//
//}

#include "omp.h"
#define MLOGI(x...)
#define LOG(x...)

int main(int argc, char** argv){
#ifdef _OPENMP

#endif
#ifdef _OPENMP

//    omp_set_dynamic(0);     // Explicitly disable dynamic teams
    omp_set_num_threads(8); // Use 4 threads for all consecutive parallel regions
    printf("use _OPENMP thread num: %i\n",  omp_get_num_threads());
#pragma omp parallel
    {
        int ID = omp_get_thread_num();
        printf("ID = %i\n", ID);

    }
#endif



    if(argc <2){
        printf("usage:\n %s config.toml\n",argv[0]);
        return 0;
    }

    ta_init(&memory_pool_cfg);


    char* toml = argv[1];

    message_handler_t handler = ros_handler_create();

    bool rt = handler.create(&handler, toml , &memory_pool_cfg);
    if(!rt){
        handler.close(&handler);
        return 0;
    }

    // exit

    set_signal_handler(signal_handler);


    printf("dds_create_from_toml ok\n");


    int scan_size = 1200;
    LaserScan_ptr send_scan_ptr = LaserScan_alloc(scan_size,&memory_pool_cfg);

    strcpy(send_scan_ptr->frame_id, "map");
    send_scan_ptr->range_min = 0.0f;
    send_scan_ptr->range_max = 20.0f;
    send_scan_ptr->angle_min = -1.7f;
    send_scan_ptr->angle_max = 1.7f;
    send_scan_ptr->angle_increment = (send_scan_ptr->angle_max - send_scan_ptr->angle_min) / (float)scan_size;

    float* scan_ptr_ranges = send_scan_ptr->buffer;
    float* scan_ptr_intensities = send_scan_ptr->buffer + send_scan_ptr->ranges_size;

    for(int i = 0 ; i < scan_size+1;i++){
        scan_ptr_ranges[i] = 5.0 + 0.01*i;
        scan_ptr_intensities[i] = i+0.5;
    }

    void* send_scan_buffer[] = {send_scan_ptr};


    Twist_ptr send_twist_ptr = Twist_alloc(&memory_pool_cfg);

    send_twist_ptr->linear.x = 1.0;
    send_twist_ptr->linear.y = 2.0;
    send_twist_ptr->linear.z = 3.0;

    send_twist_ptr->angular.x = 4.0;
    send_twist_ptr->angular.y = 5.0;
    send_twist_ptr->angular.z = 6.0;

    void* send_twist_buffer[] = {send_twist_ptr};


    PoseStamped_ptr send_tf_pose = PoseStamped_alloc(&memory_pool_cfg);
    send_tf_pose->position.x = 1.0;
    send_tf_pose->position.y = 2.0;
    send_tf_pose->position.z = 3.0;

    send_tf_pose->quaternion.w = 1.0;
    send_tf_pose->quaternion.x = 0.0;
    send_tf_pose->quaternion.y = 0.0;
    send_tf_pose->quaternion.z = 0.0;

    void* send_tf_pose_buffer[] = {send_tf_pose};


    UInt8MultiArray_ptr send_uint8array = UInt8MultiArray_alloc(10,&memory_pool_cfg);

    UInt16MultiArray_ptr send_uint16array = UInt16MultiArray_alloc(10,&memory_pool_cfg);
    for(int i=0;i<10;i++){
        send_uint8array->buffer[i] = i;
        send_uint16array->buffer[i] = i;
    }
    void* send_u8array_pose_buffer[] = {send_uint8array};
    void* send_u16array_pose_buffer[] = {send_uint16array};


    OccupancyGrid_ptr send_map = OccupancyGrid_alloc(100,100,&memory_pool_cfg);
    send_map->origin.quaternion.w = 1.0;
    send_map->origin.position.x = 1.0;
    send_map->origin.position.y = 2.0;

    send_map->resolution = 0.05;
    strcpy(send_map->frame_id,"map");
    for(int i = 0 ; i < 100; i++){
        for (int j = 0 ; j < 100;j++){
            if( i == j){
                send_map->data[i*100+j] = 100;
            }else{
                send_map->data[i*100+j] = -1;
            }
        }
    }

    void* send_map_buffer[] = {send_map};


    Odometry_ptr send_odom = Odometry_alloc(&memory_pool_cfg);

    strcpy(send_odom->frame_id,"odom");
    strcpy(send_odom->child_frame_id,"base_link");

    send_odom->twist_cov[0] = 0.01;
    send_odom->twist_cov[1*6 + 1] = 0.01;
    send_odom->twist_cov[2*6 + 2] = 0.01;
    send_odom->twist_cov[3*6 + 3] = 0.01;
    send_odom->twist_cov[4*6 + 4] = 0.01;
    send_odom->twist_cov[5*6 + 5] = 0.01;

    send_odom->pose_cov[0] = 0.01;
    send_odom->pose_cov[1*6 + 1] = 0.01;
    send_odom->pose_cov[2*6 + 2] = 0.01;
    send_odom->pose_cov[3*6 + 3] = 0.01;
    send_odom->pose_cov[4*6 + 4] = 0.01;
    send_odom->pose_cov[5*6 + 5] = 0.01;

    send_odom->pose.quaternion.w = 1.0;

    send_odom->pose.position.x = 1.0;
    send_odom->pose.position.y = 2.0;
    send_odom->pose.position.z = 3.0;

    send_odom->twist.linear.x = 0.1;
    send_odom->twist.linear.y = 0.2;
    send_odom->twist.linear.z = 0.3;
    send_odom->twist.angular.x = 0.4;
    send_odom->twist.angular.y = 0.5;
    send_odom->twist.angular.z = 0.6;

    void* send_odom_buffer[] = {send_odom};

    //
    Path_ptr send_path = Path_alloc(50,&memory_pool_cfg);
    strcpy(send_path->frame_id,"map");
    for(int i = 0 ; i < 50;i++){
        PoseStamped_ptr p = &send_path->data[i];
        strcpy(p->frame_id,"map");
        p->position.x = i*0.1;
        p->position.y = i*0.2;
        p->position.z = 0.0;

        p->quaternion.w = 1.0;
        p->quaternion.x = 0.0;
        p->quaternion.y = 0.0;
        p->quaternion.z = 0.0;

    }

    void * send_path_buffer[] ={send_path};


    PoseStamped_ptr send_pose = PoseStamped_alloc(&memory_pool_cfg);
    strcpy(send_pose->frame_id,"map");
    send_pose->position.x =  0.1;
    send_pose->position.y =  0.2;
    send_pose->position.z =  0.3;

    send_pose->quaternion.w =  0.4;
    send_pose->quaternion.x =  0.5;
    send_pose->quaternion.y =  0.6;
    send_pose->quaternion.z =  0.7;

    void* send_pose_buffer[] ={send_pose};


    HeaderString_ptr send_header_string = HeaderString_alloc(100,&memory_pool_cfg);

    strcpy(send_header_string->frame_id,"map");
    strcpy(send_header_string->data,"qwertyuiop   1234567890");
    void* send_header_string_buffer[] = {send_header_string};


    int cnt = 0;

    double elapsed_time_max = 0.0;
    double elapsed_time_max_all = 0.0;

    while (program_run && handler.is_ok(&handler)){
        cnt++;

        clock_t start_time, end_time;
        double elapsed_time;

        // Record the start time
        start_time = clock();

        if(0){
            handler.write_data(&handler,"scan_pub",send_scan_buffer,1);

            //  100hz
//        usleep(10000);
            // 1000hz
//        usleep(1000);
//
//        continue;

            handler.write_data(&handler,"twist_pub",send_twist_buffer,1);
            handler.write_data(&handler,"tf_pub",send_tf_pose_buffer,1);
            handler.write_data(&handler,"uint8array_pub",send_u8array_pose_buffer,1);
            handler.write_data(&handler,"uint16array_pub",send_u16array_pose_buffer,1);
            handler.write_data(&handler,"map_pub",send_map_buffer,1);
            handler.write_data(&handler,"odom_pub",send_odom_buffer,1);
            handler.write_data(&handler,"path_pub",send_path_buffer,1);
            handler.write_data(&handler,"pose_pub",send_pose_buffer,1);
            handler.write_data(&handler,"headerstring_pub",send_header_string_buffer,1);
            // Record the end time
            end_time = clock();

            // Calculate the elapsed time in seconds
            elapsed_time = 1000.0 * ((double) (end_time - start_time)) / CLOCKS_PER_SEC;

            if (elapsed_time > elapsed_time_max){
                elapsed_time_max = elapsed_time;
            }
            if (elapsed_time > elapsed_time_max_all){
                elapsed_time_max_all = elapsed_time;
            }


            if ((cnt % 100) == 0 ){
                printf("Time elapsed_time_max: %.4f ms, elapsed_time_max_all: %.4f ms \n", elapsed_time_max,elapsed_time_max_all );
                elapsed_time_max = 0.0;
            }
        }


        {
            if(1){
                ChannelBuffer_ptr recv_pointcloud2_buffer = handler.read_data(&handler,"pointcloud_sub");
                if(recv_pointcloud2_buffer){
                    for(int i = 0; i <  recv_pointcloud2_buffer->buffer_size;i++){
                        PointCloud2_ptr data =  (PointCloud2_ptr)recv_pointcloud2_buffer->buffer[i];

                        MLOGW("recv PointCloud2: frame: [%s], dim: [%u, %u, %u]\n",
                              data->frame_id, data->height , data->width, data->channel
                        );
                        LOG(
                                printf("\npoints\n");
                                u64_t point_num = data->height * data->width;
                                for(int j = 0 ; j < point_num ;j++){
                                    if (!isnan(data->buffer[j*3 + 0]) &&!isnan(data->buffer[j*3 + 1]) && !isnan(data->buffer[j*3 + 2])  ){
                                        printf("[%.3f, %.3f, %.3f]", data->buffer[j*3 + 0], data->buffer[j*3 + 1], data->buffer[j*3 + 2] );
                                    }

                                }
                                printf("\npoints\n");
                                );


                    }
                }
            }
            usleep(1000000);
            continue;
        }

        {
            ChannelBuffer_ptr recv_scan_buffer = handler.read_data(&handler, "scan_sub");
            if (recv_scan_buffer) {
                for (int i = 0; i < recv_scan_buffer->buffer_size; i++) {
                    LaserScan_ptr data = (LaserScan_ptr) recv_scan_buffer->buffer[i];
                    MLOGI("recv scan [%i], frame: %s, range :%.3f, size: %i ", i, data->frame_id, data->buffer[0],data->ranges_size);

                }
            }
        }
        {
            ChannelBuffer_ptr recv_twist_buffer = handler.read_data(&handler, "twist_sub");
            if (recv_twist_buffer) {
                for (int i = 0; i < recv_twist_buffer->buffer_size; i++) {
                    Twist_ptr data = (Twist_ptr) recv_twist_buffer->buffer[i];
                    MLOGI("recv twist [%i], [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f]", i, data->linear.x, data->linear.y,
                          data->linear.z,
                          data->angular.x, data->angular.y, data->angular.z);

                }
            }
        }
        {

            send_tf_pose->quaternion.w = 1.0 * ((cnt/10)%4 == 0);
            send_tf_pose->quaternion.x = 1.0 * ((cnt/10)%4 == 1);
            send_tf_pose->quaternion.y = 1.0 * ((cnt/10)%4 == 2);
            send_tf_pose->quaternion.z = 1.0 * ((cnt/10)%4 == 3);

            MLOGI("send tf [%i], [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f, %.3f]", (cnt/10)%4, send_tf_pose->position.x,
                  send_tf_pose->position.y, send_tf_pose->position.z,
                  send_tf_pose->quaternion.w, send_tf_pose->quaternion.x, send_tf_pose->quaternion.y, send_tf_pose->quaternion.z);
            ChannelBuffer_ptr recv_tf_buffer = handler.read_data(&handler, "tf_sub");
            if (recv_tf_buffer) {
                for (int i = 0; i < recv_tf_buffer->buffer_size; i++) {
                    PoseStamped_ptr data = (PoseStamped_ptr) recv_tf_buffer->buffer[i];
                    MLOGI("recv tf [%i], [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f, %.3f]", i, data->position.x,
                          data->position.y, data->position.z,
                          data->quaternion.w, data->quaternion.x, data->quaternion.y, data->quaternion.z);

                }
            }
        }
        {
            ChannelBuffer_ptr recv_u8_buffer = handler.read_data(&handler,"uint8array_sub");
            if(recv_u8_buffer){
                for(int i = 0;i < recv_u8_buffer->buffer_size;i++){
                    UInt8MultiArray_ptr data = (UInt8MultiArray_ptr)recv_u8_buffer->buffer[i];
                    MLOGI("recv %s\n[","u8");
//                    for(int j = 0 ; j < data->element_size;j++){
//                        printf("%i, ",data->buffer[j]);
//                    }
//                    printf("]\n");
                }
            }
        }
        {

            ChannelBuffer_ptr recv_u16_buffer = handler.read_data(&handler,"uint16array_sub");
            if(recv_u16_buffer){
                for(int i = 0;i < recv_u16_buffer->buffer_size;i++){
                    UInt16MultiArray_ptr data = (UInt16MultiArray_ptr)recv_u16_buffer->buffer[i];
                    MLOGI("recv %s\n[","u16");
//                    for(int j = 0 ; j < data->element_size;j++){
//                        printf("%i, ",data->buffer[j]);
//                    }
//                    printf("]\n");
                }
            }
        }
        {
            int a = cnt /10;
            if(a >= 100*100){
                cnt = 0;
                a = 0;
            }
            send_map->data[a] = 100;


            ChannelBuffer_ptr recv_map_buffer = handler.read_data(&handler,"map_sub");
            if(recv_map_buffer){
                for(int i =0 ; i < recv_map_buffer->buffer_size;i++){
                    OccupancyGrid_ptr data = (OccupancyGrid_ptr )recv_map_buffer->buffer[i];
                    MLOGI("recv map origin [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f, %.3f], info: [%i, %i, %.3f], data:\n",
                          data->origin.position.x,data->origin.position.y,data->origin.position.z,
                          data->origin.quaternion.w, data->origin.quaternion.x,data->origin.quaternion.y, data->origin.quaternion.z,
                          data->width, data->height, data->resolution);

//                    for(int j = 0; j< data->width;j++){
//                        for(int k = 0 ; k < data->height;k++){
//                            printf("%i ",data->data[j*100 + k]);
//                        }
//                        printf("\n");
//                    }
//                    printf("\n");
                }
            }
        }


        {
            ChannelBuffer_ptr recv_odom_buffer = handler.read_data(&handler,"odom_sub");

            if(recv_odom_buffer){
                for(int i = 0 ;i <recv_odom_buffer->buffer_size;i++ ){
                    Odometry_ptr data = (Odometry_ptr)recv_odom_buffer->buffer[i];

                    MLOGI("recv odom: frame: [%s, %s], pose: [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f, %.3f], twist [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f]\n",
                          data->frame_id, data->child_frame_id,
                          data->pose.position.x, data->pose.position.y,data->pose.position.z,
                          data->pose.quaternion.w, data->pose.quaternion.x,data->pose.quaternion.y,data->pose.quaternion.z,
                          data->twist.linear.x,data->twist.linear.y,data->twist.linear.z,data->twist.angular.x,data->twist.angular.y,data->twist.angular.z
                          );
//                    printf("\npose_cov: ");
//                    for(int j = 0; j < 36;j++){
//                        printf("%.3f, ", data->pose_cov[j]);
//                    }
//                    printf("\ntwist_cov: ");
//                    for(int j = 0; j < 36;j++){
//                        printf("%.3f, ", data->twist_cov[j]);
//                    }
                }
            }
        }
        {
            ChannelBuffer_ptr recv_path_buffer = handler.read_data(&handler,"path_sub");
            if(recv_path_buffer){
                for (int i = 0 ; i < recv_path_buffer->buffer_size;i++){
                    Path_ptr  data = (Path_ptr)recv_path_buffer->buffer[i];

                    MLOGI("recv path, stamp: %llu ,frame_id : %s, element_size %i",data->stamp ,data->frame_id, data->element_size);

                    for(int j = 0 ; j < data->element_size;j++){
                        PoseStamped_ptr p = &data->data[j];
                        MLOGI("recv path pose %i, [%.3f, %.3f, %.3f],[%.3f, %.3f, %.3f, %.3f]",
                              j,
                              p->position.x,p->position.y,p->position.z,
                              p->quaternion.w,p->quaternion.x,p->quaternion.y,p->quaternion.z
                              )

                    }
                }
            }
        }
        {
            ChannelBuffer_ptr recv_pose_buffer =   handler.read_data(&handler,"pose_sub");
            if(recv_pose_buffer){
                for(int i = 0 ; i < recv_pose_buffer->buffer_size;i++){
                    PoseStamped_ptr data = (PoseStamped_ptr)recv_pose_buffer->buffer[i];
                    MLOGI("recv pose, frame_id : %s, [%.3f, %.3f, %.3f],[%.3f, %.3f, %.3f, %.3f]",data->frame_id,data->position.x,data->position.y,data->position.z,data->quaternion.w,data->quaternion.x,data->quaternion.y,data->quaternion.z)

                }
            }

        }
        {
            ChannelBuffer_ptr recv_headerstring_buffer =   handler.read_data(&handler,"headerstring_sub");
            if(recv_headerstring_buffer){
                for(int i = 0 ; i < recv_headerstring_buffer->buffer_size;i++){
                    HeaderString_ptr data = recv_headerstring_buffer->buffer[i];
                    MLOGI("recv headerstring, frame_id : %s, data : %s",data->frame_id, data->data);
                }

            }
        }


        usleep(10000);

    }
    handler.close(&handler);
    ta_print();
    return 0;
}