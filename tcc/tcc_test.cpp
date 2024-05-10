//
// Created by waxz on 12/20/23.
//
#include "tcc_builder.h"

#include "common/string_func.h"

typedef bool (*hello_t)(int);
int main() {

    void* tcc = tcc_build();
    R"(hello\nworld\n)";
    const char* code =R"(
#include <tcclib.h>
#include <stdarg.h>
#include <float.h>
#include <stdbool.h>
extern int b;
bool hello_tcc(int a){

    printf("hello_tcc, get a = %i , b = %i",a,b);
    return a > b ;
}
)";

    tcc_compile(tcc,code);

    int b = 88;
    tcc_add(tcc,"b",&b);
    tcc_output(tcc);

    hello_t f = (hello_t)tcc_get(tcc, "hello_tcc");

    if (f){
        {
            bool rt = f(100);
            printf("\nrt: %i\n", rt);
        }
        {
            bool rt = f(10);
            printf("\nrt: %i\n", rt);
        }

    }
    tcc_drop(tcc);
}