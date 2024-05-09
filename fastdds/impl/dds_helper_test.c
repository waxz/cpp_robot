//
// Created by waxz on 5/8/24.
//


#include "dds_helper.h"
#include <stdio.h>
#include "common/signals.h"
#include <unistd.h> // usleep

#include "tinyalloc/tinyalloc.h"



#include "common/string_logger.h"

#include "message_center_types.h"


// 10000000 Byte = 10 MB
// 100000000 Byte = 100 MB
#define STATIC_MEMORY_SIZE 100000000
static unsigned char memory_pool[STATIC_MEMORY_SIZE];
static const ta_cfg_t memory_pool_cfg = {
        memory_pool,
        &memory_pool[sizeof(memory_pool)],
        512,
        16,
        8,
};

void ta_print(){
    printf("ta_num_used : %lu\n",ta_num_used(&memory_pool_cfg));
    printf("ta_num_free : %lu\n",ta_num_free(&memory_pool_cfg));
    printf("ta_num_fresh : %lu\n",ta_num_fresh(&memory_pool_cfg));
    printf("ta_check : %i\n",ta_check(&memory_pool_cfg));
}

static bool program_run = true;

void signal_handler(int sig) {
    printf("get signal %i\n", sig);
    program_run = false;
}

int main(int argc, char **argv) {


    if (argc < 2) {

        printf("usage: %s config.toml", argv[0]);
        return 0;
    }
    ta_init(&memory_pool_cfg);

    const char *filename = argv[1];

    message_handler_t handler = dds_handler_create();

    handler.create(&handler, filename, &memory_pool_cfg);

    set_signal_handler(signal_handler);


    while (program_run && handler.is_ok(&handler)) {

       ChannelBuffer_ptr recv_pointcloud2_buffer = handler.read_data(&handler,"cloud_sub");

       if(recv_pointcloud2_buffer && recv_pointcloud2_buffer->buffer_size > 0){

           printf("recv->buffer_size: %u\n",  recv_pointcloud2_buffer->buffer_size  );

           for(int i = 0 ; i < recv_pointcloud2_buffer->buffer_size;i++){
               PointCloud2_ptr data =  (PointCloud2_ptr)recv_pointcloud2_buffer->buffer[i];

               MLOGI("recv PointCloud2: frame: [%s], dim: [%u, %u, %u]\n",
                     data->frame_id, data->height , data->width, data->channel
               );


           }
       }else{
//           printf("recv fail\n");

       }

        usleep(100000);

    }
    handler.close(&handler);

}