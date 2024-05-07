//
// Created by waxz on 4/24/24.
//

#ifndef CMAKE_SUPER_BUILD_MESSAGE_CENTER_TYPES_H
#define CMAKE_SUPER_BUILD_MESSAGE_CENTER_TYPES_H


#include "common/c_style.h"
#include "tinyalloc/tinyalloc.h"


#ifdef __cplusplus
extern "C" {
#endif

#define MSG_STRUCT_MAX_FRAME_ID_LEN 50


typedef struct MessageBase{

    const char* type_name ;
    u64_t stamp;
    u32_t base_size ;
    u32_t buffer_size ;
    u32_t full_size ;


    // buffer
    u32_t buffer[1];

}MessageBase,*MessageBasePtr ;

MessageBase MessageBase_create();
MessageBasePtr MessageBase_allocate(u32_t size, ta_cfg_t * cfg);
MessageBasePtr MessageBase_reallocate(u32_t size, MessageBasePtr ptr, ta_cfg_t* cfg);
u32_t* MessageBaseT_getBuffer(MessageBasePtr ptr);


STRUCT_BEGIN(Point)
    f64_t x;
    f64_t y;
    f64_t z;
STRUCT_END(Point)

STRUCT_BEGIN(Quaternion)
    f64_t w;
    f64_t x;
    f64_t y;
    f64_t z;
STRUCT_END(Quaternion)

STRUCT_BEGIN(Pose)
    Point position;
    Quaternion quaternion;
STRUCT_END(Pose)

// all zero in quaternion will make all data to be NAN
STRUCT_BEGIN(PoseStamped)
    char frame_id[MSG_STRUCT_MAX_FRAME_ID_LEN];
    u64_t stamp;
    Point position;
    Quaternion quaternion;
STRUCT_END(PoseStamped)

PoseStamped_ptr PoseStamped_alloc(u32_t size, const ta_cfg_t* cfg);
PoseStamped_ptr PoseStamped_realloc(u32_t size, PoseStamped_ptr ptr, const ta_cfg_t* cfg);



STRUCT_BEGIN(Twist)
    Point angular;
    Point linear;
STRUCT_END(Twist)
Twist_ptr Twist_alloc(u32_t size, const ta_cfg_t* cfg);
Twist_ptr Twist_realloc(u32_t size, Twist_ptr ptr, const ta_cfg_t* cfg);

STRUCT_BEGIN(UInt8MultiArray)
    u32_t element_size;
    u8_t buffer[1];
STRUCT_END(UInt8MultiArray)

UInt8MultiArray_ptr UInt8MultiArray_alloc(u32_t size, const ta_cfg_t* cfg);
UInt8MultiArray_ptr UInt8MultiArray_realloc(u32_t size, UInt8MultiArray_ptr ptr, const ta_cfg_t* cfg);

void UInt8MultiArrayT_set_buffer(UInt8MultiArray_ptr t, u32_t size);
u8_t* UInt8MultiArrayT_get_buffer(UInt8MultiArray_ptr t);
STRUCT_BEGIN(UInt16MultiArray)
    u32_t element_size;
    u16_t buffer[1];
STRUCT_END(UInt16MultiArray)
UInt16MultiArray_ptr UInt16MultiArray_alloc(u32_t size, const ta_cfg_t* cfg);
UInt16MultiArray_ptr UInt16MultiArray_realloc(u32_t size, UInt16MultiArray_ptr ptr, const ta_cfg_t* cfg);

void UInt16MultiArray_set_buffer(UInt16MultiArray_ptr t, u32_t size);
u16_t* UInt16MultiArray_get_buffer(UInt16MultiArray_ptr t);

//Odometry
STRUCT_BEGIN(Odometry)
    char frame_id[MSG_STRUCT_MAX_FRAME_ID_LEN];
    char child_frame_id[MSG_STRUCT_MAX_FRAME_ID_LEN];
    u64_t stamp;
    Pose pose;
    f64_t pose_cov[36];
    Twist twist;
    f64_t twist_cov[36];
STRUCT_END(Odometry)

Odometry_ptr Odometry_alloc(u32_t size, const ta_cfg_t* cfg);
Odometry_ptr Odometry_realloc(u32_t size, Odometry_ptr ptr, const ta_cfg_t* cfg);


STRUCT_BEGIN(Path)
    char frame_id[MSG_STRUCT_MAX_FRAME_ID_LEN];
    u64_t stamp;
    u32_t element_size;
    PoseStamped data[1];
STRUCT_END(Path)

Path_ptr Path_alloc(u32_t size, const ta_cfg_t* cfg);
Path_ptr Path_realloc(u32_t size, Path_ptr ptr, const ta_cfg_t* cfg);

void Path_set_buffer(Path_ptr t, u32_t size);

//HeaderString
STRUCT_BEGIN(HeaderString)
    char frame_id[MSG_STRUCT_MAX_FRAME_ID_LEN];
    u64_t stamp;
    u32_t element_size;
    char data[1];
STRUCT_END(HeaderString)
HeaderString_ptr HeaderString_alloc(u32_t size, const ta_cfg_t* cfg);
HeaderString_ptr HeaderString_realloc(u32_t size, HeaderString_ptr ptr, const ta_cfg_t* cfg);

void HeaderString_set_buffer(HeaderString_ptr t, u32_t size);

STRUCT_BEGIN(OccupancyGrid)
    char frame_id[MSG_STRUCT_MAX_FRAME_ID_LEN];
    u64_t stamp;
    f32_t resolution;
    u32_t width;
    u32_t height;
    Pose origin;
    i8_t data[1];
STRUCT_END(OccupancyGrid)

OccupancyGrid_ptr OccupancyGrid_alloc(u32_t width, u32_t height, const ta_cfg_t* cfg);
OccupancyGrid_ptr OccupancyGrid_realloc(u32_t width, u32_t height, OccupancyGrid_ptr ptr, const ta_cfg_t* cfg);

void OccupancyGrid_set_buffer(OccupancyGrid_ptr t, u32_t width, u32_t height);


STRUCT_BEGIN(LaserScan)


    char frame_id[MSG_STRUCT_MAX_FRAME_ID_LEN];
    u64_t stamp;
    f32_t range_min;
    f32_t range_max;
    f32_t angle_min;
    f32_t angle_max;
    f32_t angle_increment;
    u32_t ranges_size;
    f32_t buffer[1];

STRUCT_END(LaserScan)
void LaserScan_set_buffer(LaserScan* t, u32_t size);
LaserScan_ptr LaserScan_alloc(u32_t size, const ta_cfg_t* cfg);
LaserScan_ptr LaserScan_realloc(u32_t size, LaserScan_ptr ptr, const ta_cfg_t* cfg);
f32_t* LaserScan_get_ranges(LaserScan* t);
f32_t* LaserScan_get_intensities(LaserScan* t);


STRUCT_BEGIN(Pointcloud)
    char frame_id[MSG_STRUCT_MAX_FRAME_ID_LEN];
    u64_t stamp;
    u32_t height;
    u32_t width;
    u32_t channel;
    bool is_dense;
    f32_t buffer[1];
STRUCT_END(Pointcloud)
void Pointcloud_set_buffer(Pointcloud_ptr t, u32_t height, u32_t width, u32_t channel);
Pointcloud_ptr Pointcloud_alloc(u32_t height, u32_t width, u32_t channel, const ta_cfg_t* cfg);
Pointcloud_ptr Pointcloud_realloc(u32_t height, u32_t width, u32_t channel, Pointcloud_ptr ptr, const ta_cfg_t* cfg);




typedef struct Scalar{
    u64_t stamp;
    char name[MSG_STRUCT_MAX_FRAME_ID_LEN];
    f32_t value;
} Scalar,*ScalarPtr;



STRUCT_BEGIN(ScalarList)
    u32_t size;
    Scalar data[1];
STRUCT_END(ScalarList)
void ScalarList_set_buffer(ScalarList_ptr t, u32_t size);
ScalarList_ptr ScalarList_alloc(u32_t size, const ta_cfg_t* cfg);
ScalarList_ptr ScalarList_realloc(u32_t size, ScalarList_ptr ptr, const ta_cfg_t* cfg);

typedef struct Signal{
    char name[MSG_STRUCT_MAX_FRAME_ID_LEN];
    bool value;
} Signal,*Signal_ptr;

STRUCT_BEGIN(SignalList)
    u32_t size;
    Signal data[1];
STRUCT_END(SignalList)

void SignalList_set_buffer(SignalList_ptr t, u32_t size);
SignalList_ptr SignalList_alloc(u32_t size, const ta_cfg_t* cfg);
SignalList_ptr SignalList_realloc(u32_t size, SignalList_ptr ptr, const ta_cfg_t* cfg);






#ifdef __cplusplus
}
#endif



#endif //CMAKE_SUPER_BUILD_MESSAGE_CENTER_TYPES_H
