//
// Created by waxz on 5/14/24.
//
#include <math.h>
#include "pointcloud_process.h"

#define HEIGHT 480
#define WIDTH 640
#define DIM  HEIGHT*WIDTH*3
static float src_buffer[DIM];
static float src_clip_buffer[DIM];

static  float dst_buffer[DIM];

int main(int argc, char** argv){

    f32_t tx = 1.0, ty = 0.5, tz = 2.0;
    f32_t roll = 0.0, pitch = 0.0, yaw = M_PI_2;


    src_buffer[0] = 0.1;
    src_buffer[1] = 0.2;
    src_buffer[2] = 0.3;

    src_buffer[3] = 0.1;
    src_buffer[4] = 0.0;
    src_buffer[5] = 0.0;

    src_buffer[6] = 0.0;
    src_buffer[7] = 0.1;
    src_buffer[8] = 0.0;


    src_buffer[9] = 0.0;
    src_buffer[10] = 0.0;
    src_buffer[11] = 0.1;




    pointcloud_clip(src_buffer,100,100,src_clip_buffer,0,1,1,4);

    pointcloud_transform(src_clip_buffer,
                         3,
//                         HEIGHT*WIDTH ,
                         dst_buffer, tx, ty, tz, roll, pitch,yaw );





}