//
// Created by waxz on 5/8/24.
//


#include "dds_helper.h"
#include <stdio.h>
#include "common/signals.h"
#include <unistd.h> // usleep

#include "tinyalloc/tinyalloc.h"

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

    const char *filename = argv[1];

    message_handler_t handler = dds_handler_create();

    handler.create(&handler, filename, &memory_pool_cfg);

    set_signal_handler(signal_handler);


    while (program_run && handler.is_ok(&handler)) {

        usleep(100000);

    }
    handler.close(&handler);

}