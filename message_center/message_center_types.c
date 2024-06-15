//
// Created by waxz on 4/25/24.
//

#include "message_center_types.h"
#include "tinyalloc/tinyalloc.h"


//==== MessageBasePtrT
MessageBase MessageBase_create(){
    MessageBase t = {0};
    t.type_name = "MessageBase";
    t.base_size = sizeof(MessageBase);
    return t;
}





//==== MessageBasePtrT
STRUCT_NEW(Twist);


Twist_ptr Twist_alloc(const ta_cfg_t* cfg){
    Twist target = Twist_create();
    Twist_ptr ptr = ta_alloc(cfg, target.base_size + target.buffer_size);
    if(ptr)
        *ptr = target;
    return ptr;
}
Twist_ptr Twist_realloc(Twist_ptr ptr, const ta_cfg_t* cfg){
    Twist target = Twist_create();
    Twist_ptr new_ptr = ta_realloc(cfg, ptr,target.base_size + target.buffer_size);
    if(new_ptr)
        *new_ptr = target;
    return new_ptr;
}

STRUCT_NEW(Pose);
STRUCT_NEW(PoseStamped);
PoseStamped_ptr PoseStamped_alloc(const ta_cfg_t* cfg){
    PoseStamped target = PoseStamped_create();
    target.quaternion.w = 1.0;
    PoseStamped_ptr ptr = ta_alloc(cfg, target.base_size + target.buffer_size);
    if(ptr)
        *ptr = target;
    return ptr;
}
PoseStamped_ptr PoseStamped_realloc(PoseStamped_ptr ptr, const ta_cfg_t* cfg){
    PoseStamped target = PoseStamped_create();
    target.quaternion.w = 1.0;
    PoseStamped_ptr new_ptr = ta_realloc(cfg,ptr, target.base_size + target.buffer_size);
    if(new_ptr)
        *new_ptr = target;
    return new_ptr;
}


STRUCT_NEW(UInt8MultiArray);
void UInt8MultiArrayT_set_buffer(UInt8MultiArray_ptr t, u32_t size){
    t->buffer_size = size * sizeof (u8_t);
    t->element_size = size;
}
u8_t* UInt8MultiArrayT_get_buffer(UInt8MultiArray_ptr t){
    return t->buffer;
}
UInt8MultiArray_ptr UInt8MultiArray_alloc(u32_t size, const ta_cfg_t* cfg){
    UInt8MultiArray target = UInt8MultiArray_create();
    UInt8MultiArrayT_set_buffer(&target,size);
    UInt8MultiArray_ptr ptr = ta_alloc(cfg, target.base_size + target.buffer_size);
    if(ptr)
        *ptr = target;
    return ptr;
}
UInt8MultiArray_ptr UInt8MultiArray_realloc(u32_t size, UInt8MultiArray_ptr ptr, const ta_cfg_t* cfg){
    UInt8MultiArray target = UInt8MultiArray_create();
    UInt8MultiArrayT_set_buffer(&target,size);
    UInt8MultiArray_ptr new_ptr = ta_realloc(cfg, ptr, target.base_size + target.buffer_size);
    if(new_ptr)
        *new_ptr = target;
    return new_ptr;
}




STRUCT_NEW(UInt16MultiArray);
void UInt16MultiArray_set_buffer(UInt16MultiArray_ptr t, u32_t size){
    t->buffer_size = size * sizeof (u16_t);
    t->element_size = size;
}
UInt16MultiArray_ptr UInt16MultiArray_alloc(u32_t size, const ta_cfg_t* cfg){
    UInt16MultiArray target = UInt16MultiArray_create();
    UInt16MultiArray_set_buffer(&target,size);
    UInt16MultiArray_ptr ptr = ta_alloc(cfg, target.base_size + target.buffer_size);
    if(ptr)
        *ptr = target;
    return ptr;
}
UInt16MultiArray_ptr UInt16MultiArray_realloc(u32_t size,UInt16MultiArray_ptr ptr, const ta_cfg_t* cfg){
    UInt16MultiArray target = UInt16MultiArray_create();
    UInt16MultiArray_set_buffer(&target,size);
    UInt16MultiArray_ptr new_ptr = ta_realloc(cfg,ptr, target.base_size + target.buffer_size);
    if(new_ptr)
        *new_ptr = target;
    return new_ptr;
}


u16_t* UInt16MultiArray_get_buffer(UInt16MultiArray_ptr t){
    return t->buffer;
}


STRUCT_NEW(Odometry)
Odometry_ptr Odometry_alloc(const ta_cfg_t* cfg){
    Odometry target = Odometry_create();
    Odometry_ptr ptr = ta_alloc(cfg, target.base_size + target.buffer_size);
    if(ptr)
        *ptr = target;
    return ptr;
}
Odometry_ptr Odometry_realloc(Odometry_ptr ptr, const ta_cfg_t* cfg){
    Odometry target = Odometry_create();
    Odometry_ptr new_ptr = ta_realloc(cfg,ptr, target.base_size + target.buffer_size);
    if(new_ptr)
        *new_ptr = target;
    return new_ptr;
}


STRUCT_NEW(Path)
Path_ptr Path_alloc(u32_t size, const ta_cfg_t* cfg){
    Path target = Path_create();
    Path_set_buffer(&target, size);
    Path_ptr ptr = ta_alloc(cfg, target.base_size + target.buffer_size);
    if(ptr)
        *ptr = target;
    return ptr;
}
Path_ptr Path_realloc(u32_t size, Path_ptr ptr, const ta_cfg_t* cfg){
    Path target = Path_create();
    Path_set_buffer(&target, size);
    Path_ptr new_ptr = ta_realloc(cfg,ptr, target.base_size + target.buffer_size);
    if(new_ptr)
        *new_ptr = target;
    return new_ptr;
}


void Path_set_buffer(Path_ptr t, u32_t size){
    t->element_size = size;
    t->buffer_size = size * sizeof (PoseStamped);
}



STRUCT_NEW(HeaderString);
HeaderString_ptr HeaderString_alloc(u32_t size, const ta_cfg_t* cfg){
    HeaderString target = HeaderString_create();
    HeaderString_set_buffer(&target,size);
    HeaderString_ptr ptr = ta_alloc(cfg, target.base_size + target.buffer_size);
    if(ptr)
        *ptr = target;
    return ptr;
}
HeaderString_ptr HeaderString_realloc(u32_t size, HeaderString_ptr ptr, const ta_cfg_t* cfg){
    HeaderString target = HeaderString_create();
    HeaderString_set_buffer(&target,size);
    HeaderString_ptr new_ptr = ta_realloc(cfg, ptr, target.base_size + target.buffer_size);
    if(new_ptr)
        *new_ptr = target;
    return new_ptr;
}
// string need etra byte
void HeaderString_set_buffer(HeaderString_ptr t, u32_t size){
    t->element_size = size;
    t->buffer_size = (size + 1) * sizeof (char);
}

STRUCT_NEW(OccupancyGrid)
void OccupancyGrid_set_buffer(OccupancyGrid_ptr t, u32_t width, u32_t height){
    t->width = width;
    t->height = height;
    t->buffer_size = width*height*sizeof(i8_t);
}

OccupancyGrid_ptr OccupancyGrid_alloc(u32_t width, u32_t height, const ta_cfg_t* cfg){
    OccupancyGrid target = OccupancyGrid_create();
    OccupancyGrid_set_buffer(&target,width , height);
    OccupancyGrid_ptr ptr = ta_alloc(cfg, target.base_size + target.buffer_size);
    if(ptr)
        *ptr = target;
    return ptr;
}
OccupancyGrid_ptr OccupancyGrid_realloc(u32_t width, u32_t height, OccupancyGrid_ptr ptr, const ta_cfg_t* cfg){
    OccupancyGrid target = OccupancyGrid_create();
    OccupancyGrid_set_buffer(&target,width , height);
    OccupancyGrid_ptr new_ptr = ta_realloc(cfg,ptr, target.base_size + target.buffer_size);
    if(new_ptr)
        *new_ptr = target;
    return new_ptr;
}


STRUCT_NEW(LaserScan);

void LaserScan_set_buffer(LaserScan_ptr t, u32_t size){
    t->ranges_size = size;
    t->buffer_size = (size + size)*sizeof(f32_t) ;
    t->full_size = t->base_size + t->buffer_size;
}
LaserScan_ptr LaserScan_alloc(u32_t size, const ta_cfg_t* cfg){
    LaserScan target = LaserScan_create();
    LaserScan_set_buffer(&target,size);
    LaserScan_ptr ptr = ta_alloc(cfg, target.full_size);
    if(ptr)
        *ptr = target;
    return ptr;
}
LaserScan_ptr LaserScan_realloc(u32_t size, LaserScan_ptr ptr, const ta_cfg_t* cfg){
    LaserScan target = LaserScan_create();
    LaserScan_set_buffer(&target,size);
    LaserScan_ptr new_ptr = ta_realloc(cfg, ptr, target.full_size);
    if(new_ptr)
        *new_ptr = target;
    return new_ptr;
}
f32_t* LaserScan_get_ranges(LaserScan* t){
    return t->buffer;
}
f32_t* LaserScan_get_intensities(LaserScan* t){
    return t->buffer + t->ranges_size;
}
//
STRUCT_NEW(PointCloud2);
void PointCloud2_set_buffer(PointCloud2_ptr t, u32_t height, u32_t width, u32_t channel){
    t->height = height;
    t->width = width;
    t->channel = channel;

    t->buffer_size = channel*height * width * sizeof (f32_t);
    t->full_size = t->base_size + t->buffer_size;
}

PointCloud2_ptr PointCloud2_alloc(u32_t height, u32_t width, u32_t channel, const ta_cfg_t* cfg){
    PointCloud2 target = PointCloud2_create();
    PointCloud2_set_buffer(&target,height, width,channel);
    PointCloud2_ptr ptr = ta_alloc(cfg, target.full_size);
    if(ptr)
        *ptr = target;
    return ptr;
}

PointCloud2_ptr PointCloud2_realloc(u32_t height, u32_t width, u32_t channel, PointCloud2_ptr ptr, const ta_cfg_t* cfg){
    PointCloud2 target = PointCloud2_create();
    PointCloud2_set_buffer(&target,height, width,channel);
    PointCloud2_ptr new_ptr = ta_realloc(cfg, ptr, target.full_size);
    if(new_ptr)
        *new_ptr = target;
    return new_ptr;
}

STRUCT_NEW(ScalarList);

void ScalarList_set_buffer(ScalarList_ptr t, u32_t size){
    t->size = size;
    t->buffer_size = size * sizeof (Scalar);
    t->full_size = t->base_size + t->buffer_size;

}
ScalarList_ptr ScalarList_alloc(u32_t size, const ta_cfg_t* cfg){
    ScalarList target = ScalarList_create();
    ScalarList_set_buffer(&target, size);
    ScalarList_ptr ptr = ta_alloc(cfg, target.full_size);
    if(ptr)
        *ptr = target;
    return ptr;
}
ScalarList_ptr ScalarList_realloc(u32_t size, ScalarList_ptr ptr, const ta_cfg_t* cfg){
    ScalarList target = ScalarList_create();
    ScalarList_set_buffer(&target, size);
    ScalarList_ptr new_ptr = ta_realloc(cfg, ptr, target.full_size);
    if(new_ptr)
        *new_ptr = target;
    return new_ptr;
}

STRUCT_NEW(SignalList);
void SignalList_set_buffer(SignalList_ptr t, u32_t size){
    t->size = size;
    t->buffer_size = size * sizeof (Signal);
    t->full_size = t->base_size + t->buffer_size;
}
SignalList_ptr SignalList_alloc(u32_t size, const ta_cfg_t* cfg){
    SignalList target = SignalList_create();
    SignalList_set_buffer(&target, size);
    SignalList_ptr ptr = ta_alloc(cfg, target.full_size);
    if(ptr)
        *ptr = target;
    return ptr;
}
SignalList_ptr SignalList_realloc(u32_t size, SignalList_ptr ptr, const ta_cfg_t* cfg){
    SignalList target = SignalList_create();
    SignalList_set_buffer(&target, size);
    SignalList_ptr new_ptr = ta_realloc(cfg, ptr, target.full_size);
    if(new_ptr)
        *new_ptr = target;
    return new_ptr;
}



