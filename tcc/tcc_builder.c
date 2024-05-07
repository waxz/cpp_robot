//
// Created by waxz on 3/30/24.
//

#include "tcc_builder.h"

#include "common/string_func.h"
#include "common/string_logger.h"

#include "libtcc.h"

static void
tcc_error(void* opaque, const char* msg)
{
    printf("[TCC:ERR] %s\n", msg);
}


void* tcc_build(){
    const char* TCC_HOME = getenv("TCC_HOME");
    if(TCC_HOME == NULL){
        MLOGW("[TCC:ERR] %s is NOT DEFINED\n","TCC_HOME");
//        return NULL;
    }

    TCCState* tcc = tcc_new();
    if (tcc){
        //-nostdlib
        tcc_set_options(tcc, "-g -std=c11 -Wall -m64 -bench -I/usr/include");
        tcc_set_error_func(tcc, 0x0, tcc_error);
        tcc_set_output_type(tcc, TCC_OUTPUT_MEMORY);

        printf("GCC version %i.%i.%i\n", __GNUC__, __GNUC_MINOR__, __GNUC_PATCHLEVEL__);
        char gcc_include_path[100];
        //                         /usr/lib/gcc/x86_64-linux-gnu/9/include/stdint-gcc.h
        sprintf(gcc_include_path, "/usr/lib/gcc/x86_64-linux-gnu/%i/include", __GNUC__);
        printf("gcc_include_path: %s\n", gcc_include_path);

        tcc_add_include_path(tcc, gcc_include_path);

        {
            if(TCC_HOME != NULL){
                char lib_path[500];
                sprintf(lib_path, "%s/lib/tcc", TCC_HOME);
                char include_path[500];
                sprintf(include_path, "%s/lib/tcc/include", TCC_HOME);

                MLOGI("tcc_set_lib_path %s",lib_path );
                MLOGI("tcc_set_lib_path %s",include_path );
//                tcc_set_lib_path(tcc, lib_path);
                tcc_add_library_path(tcc, lib_path);
                tcc_add_include_path(tcc, include_path);
            }

        }

        {
            const char *TCC_HEADER = getenv("TCC_HEADER");
            if (TCC_HEADER) {
                printf("[TCC:INFO] TCC_HEADER:%s\n", TCC_HEADER);
                char *x;
                int xi = 0;
                SPLIT_STRING(TCC_HEADER, ":", xi, x, tcc_add_include_path(tcc, x));
            }
        }
        {
            const char *TCC_LINK_PATH = getenv("TCC_LINK_PATH");
            if (TCC_LINK_PATH) {
                printf("[TCC:INFO] TCC_LINK_PATH:%s\n", TCC_LINK_PATH);
                char *x;
                int xi = 0;
                SPLIT_STRING(TCC_LINK_PATH, ":", xi, x, tcc_add_library_path(tcc, x));
            }
        }
        {
            const char *TCC_LINK_LIB = getenv("TCC_LINK_LIB");
            if (TCC_LINK_LIB) {
                printf("[TCC:INFO] TCC_LINK_LIB:%s\n", TCC_LINK_LIB);
                char *x;
                int xi = 0;

                SPLIT_STRING(TCC_LINK_LIB, ":", xi, x, { tcc_add_library(tcc, x); });
            }
        }
        {
            const char *TCC_SOURCE = getenv("TCC_SOURCE");
            if (TCC_SOURCE) {
                printf("[TCC:INFO] TCC_ADD_SOURCE:%s\n", TCC_SOURCE);
                char *x;
                int xi = 0;

                SPLIT_STRING(TCC_SOURCE, ":", xi, x, tcc_add_file(tcc, x));
            }

        }
        {
            const char *TCC_OPTION = getenv("TCC_OPTION");
            if (TCC_OPTION) {
                printf("[TCC:INFO] TCC_OPTION:%s\n", TCC_OPTION);
                char *x;
                int xi = 0;

                SPLIT_STRING(TCC_OPTION, ":", xi, x, tcc_set_options(tcc, x));
            }

        }

//        tcc_set_output_type(tcc, TCC_OUTPUT_MEMORY);

//        printf("tcc_set_lib_path: %s\n", lib_path);
//        printf("tcc_add_include_path: %s\n", include_path);
//        tcc_set_lib_path(tcc, lib_path);
//        tcc_add_include_path(tcc, include_path);

    }else
    {
        printf("[TCC:ERR] Failed to create tcc context!\n");
    }
    return tcc;
}
int tcc_drop(void* ptcc){
    if (!ptcc){
        return -1;
    }

    TCCState* tcc = ptcc;
    tcc_delete(tcc);
    return 0;
}
int tcc_output(void* ptcc){
    if (!ptcc){
        return -1;
    }
    TCCState* tcc = ptcc;
    int ret = tcc_relocate(tcc);
    if (ret < 0)
    {
        MLOGW("[TCC:ERR] %s", "tcc_relocate failed ");
        return -1;
    }
}
int tcc_compile(void* ptcc, const char* code){
    if (!ptcc){
        return -1;
    }

    TCCState* tcc = ptcc;
    MLOGI("tcc_compile, code: %s",code);

    int ret = tcc_compile_string(tcc, code);
    MLOGW("run %s, ret: %i ", "tcc_compile_string",ret);

    if (ret < 0)
    {
        MLOGW("[TCC:ERR] %s ", "tcc_compile_string failed ");
        return -1;
    }

    return ret;
}

void* tcc_get(void* ptcc, const char* name){
    MLOGI("tcc_get, name: %s", name);
    if (!ptcc){
        return NULL;
    }
    TCCState* tcc = ptcc;
    void* ptr = tcc_get_symbol(tcc, name);
    MLOGI("tcc_get, name: %s, ptr: %p", name, ptr);

    return ptr;
}
int tcc_add(void* ptcc, const char* name, void * value){
    MLOGI("tcc_add, name: %s, value: %p", name, value);

    if (!ptcc){
        return -1;
    }
    TCCState* tcc = ptcc;
    return tcc_add_symbol(tcc, name,value);
}
