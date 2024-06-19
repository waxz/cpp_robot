//
// Created by waxz on 10/17/23.
//

#ifndef CMAKE_SUPER_BUILD_C_STYLE_H
#define CMAKE_SUPER_BUILD_C_STYLE_H

//#include <stdint.h>
//#include <uchar.h>
#include "common/types.h"
#ifdef __cplusplus
extern "C" {
#endif


// basic type







//https://stackoverflow.com/questions/11152160/initializing-a-struct-to-0
#define STRING(a)   #a
#define XSTRING(a)  STRING(a)



#define STRUCT_BEGIN(struct_name) \
typedef struct struct_name {\
    char* type_name;                   \
    u32_t base_size;\
    u32_t buffer_size;            \
    u32_t full_size;            \

#define STRUCT_END(struct_name) \
} struct_name,*struct_name##_ptr;\
struct_name struct_name##_create();


#define STRUCT_NEW(struct_name) \
struct_name struct_name##_create(){ \
      struct_name t = {0};              \
      t.type_name = (char*)XSTRING(struct_name)  ;     \
      t.base_size = sizeof(struct_name);\
      t.full_size = t.base_size;                        \
      return t;       \
}



#ifdef __cplusplus
}
#endif

#endif //CMAKE_SUPER_BUILD_C_STYLE_H
