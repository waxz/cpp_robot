//
// Created by waxz on 5/21/24.
//
#include "pointcloud_pallet_detect.h"

#include "common/string_logger.h"

#include "pointcloud_pallet_detect_impl.h"









void detector_set_input(struct pointcloud_pallet_detector_t* h, f32_t * buffer,u64_t height, u64_t width, f32_t vx, f32_t vy, f32_t vz){
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        handler->set_input(buffer,height,width,vx,vy,vz );
    }
}



bool detector_create(struct pointcloud_pallet_detector_t* h, const char* filename, const ta_cfg_t* cfg){

    perception::PalletDetector* handler = new perception::PalletDetector();
    int rt = handler->create(filename,cfg);
    h->handler = handler;
    MLOGI("%s ret = %i, handler = %p\n","detector_create_from_toml",rt,handler);
    if(rt < 0 ){
        h->close(h);
        return false;
    }
    return true;
}


void detector_close (struct pointcloud_pallet_detector_t* h){
    MLOGI("%s h->handler = %p\n","detector_close",h->handler);
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        handler->stop();
        MLOGI("detector_close at : %p\n",h->handler);
        delete handler;
        h->handler = nullptr;
    }

}

PointCloudBuffer_ptr detector_filter_ground(struct pointcloud_pallet_detector_t* h, u32_t output_mode){
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        return handler->filter_ground(output_mode );
    }
    return 0;
}
PointCloudBuffer_ptr detector_filter_vertical (struct pointcloud_pallet_detector_t* h, u32_t output_mode){
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        return handler->filter_vertical(output_mode );
    }
    return 0;
}

PointCloudBuffer_ptr detector_filter_pallet (struct pointcloud_pallet_detector_t* h, u32_t output_mode){
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        return handler->filter_pallet(output_mode );
    }
    return 0;
}

PalletInfoBuffer_ptr detector_get_pallet (struct pointcloud_pallet_detector_t* h, u32_t output_mode){
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        return handler->get_pallet(output_mode );
    }
    return 0;
}


void detector_set_ground_uncertain_thresh (struct pointcloud_pallet_detector_t* h,  f32_t far_uncertain_z_max, f32_t far_uncertain_x_change_min,f32_t far_uncertain_adaptive_z_max, i32_t far_uncertain_row){
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        return handler->set_ground_uncertain_thresh(far_uncertain_z_max,far_uncertain_x_change_min,far_uncertain_adaptive_z_max, far_uncertain_row  );
    }
}


void detector_set_ground_init_dim(struct pointcloud_pallet_detector_t* h, u64_t height_min , u64_t height_max ,u64_t width_min, u64_t width_max){
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        handler->set_ground_init_dim(height_min,height_max,width_min,width_max);
    }

}
void detector_set_vertical_init_dim(struct pointcloud_pallet_detector_t* h, u64_t height_min , u64_t height_max ,u64_t width_min, u64_t width_max){
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        handler->set_vertical_init_dim(height_min,height_max,width_min,width_max);
    }

}

void detector_set_ground_init_thresh(struct pointcloud_pallet_detector_t* h,  f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max, f32_t nz_min){
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        handler->set_ground_init_thresh(x_min, x_max, y_min, y_max,z_min,z_max,nz_min );
    }
}

void detector_set_pallet_row(struct pointcloud_pallet_detector_t* h, i32_t row_high, i32_t row_low){
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        handler->set_pallet_row(row_high, row_low);
    }
}
void detector_set_pallet_thresh(struct pointcloud_pallet_detector_t* h,  f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max, f32_t jx_max, f32_t jy_max, f32_t jz_max){
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        handler->set_pallet_thresh(x_min, x_max, y_min, y_max,z_min,z_max,jx_max, jy_max, jz_max );
    }
}
void detector_set_vertical_init_thresh(struct pointcloud_pallet_detector_t* h,  f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max, f32_t jx_max, f32_t jy_max, f32_t jz_max){
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        handler->set_vertical_init_thresh(x_min, x_max, y_min, y_max,z_min,z_max,jx_max, jy_max, jz_max );
    }
}
void detector_set_ground_adaptive_thresh(struct pointcloud_pallet_detector_t* h,  f32_t x_min, f32_t x_max, f32_t y_min, f32_t y_max, f32_t z_min, f32_t z_max){
    if(h->handler){
        perception::PalletDetector* handler = (perception::PalletDetector*)h->handler;
        handler->set_ground_adaptive_thresh(x_min, x_max, y_min, y_max,z_min,z_max );
    }
}


pointcloud_pallet_detector_t pointcloud_pallet_detector_create(){
    pointcloud_pallet_detector_t target = {0};


    target.create = detector_create;
    target.close = detector_close;
    target.set_input = detector_set_input;
    target.set_ground_init_dim = detector_set_ground_init_dim;
    target.set_ground_init_thresh = detector_set_ground_init_thresh;
    target.set_ground_adaptive_thresh = detector_set_ground_adaptive_thresh;
    target.set_ground_uncertain_thresh = detector_set_ground_uncertain_thresh;

    target.set_vertical_init_dim = detector_set_vertical_init_dim;
    target.set_vertical_init_thresh = detector_set_vertical_init_thresh;
    target.set_pallet_row = detector_set_pallet_row;
    target.set_pallet_thresh = detector_set_pallet_thresh;


    target.filter_ground = detector_filter_ground;
    target.filter_vertical = detector_filter_vertical;
    target.filter_pallet = detector_filter_pallet;
    target.get_pallet = detector_get_pallet;



//    target.read = ros_read;
//    target.write = ros_write;

//    target.pool = ros_get_pool;
//    target.is_ok = ros_is_ok;
//    target.read_data = ros_read_data;
//    target.write_data = ros_write_data;



    return target;
}


