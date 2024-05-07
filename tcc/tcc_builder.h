//
// Created by waxz on 3/30/24.
//

#ifndef CMAKE_SUPER_BUILD_TCC_BUILDER_H
#define CMAKE_SUPER_BUILD_TCC_BUILDER_H

//https://stackoverflow.com/questions/21835664/why-declare-a-c-function-as-static-inline

#ifdef __cplusplus
extern "C" {
#endif

void* tcc_build();
int tcc_drop(void* tcc);
int tcc_compile(void* tcc, const char* code);
int tcc_output(void* tcc);

void* tcc_get(void* tcc, const char* name);
int tcc_add(void* tcc, const char* name, void * value);


#ifdef __cplusplus
}
#endif

#endif //CMAKE_SUPER_BUILD_TCC_BUILDER_H
