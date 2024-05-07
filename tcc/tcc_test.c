//
// Created by waxz on 12/20/23.
//
#include "tcc_builder.h"

#include "common/string_func.h"

typedef void (*hello_t)(int);
int main() {

    void* tcc = tcc_build();

    const char* code = "#include <stdio.h>\n"
                       "extern int b;\n"
                       "void hello_tcc(int a){\n"
                       "printf(\"hello_tcc, get a = %i , b = %i\",a,b);\n"
                       "}";
    tcc_compile(tcc,code);

    int b = 88;
    tcc_add(tcc,"b",&b);
    tcc_output(tcc);

    hello_t f = (hello_t)tcc_get(tcc, "hello_tcc");
    if (f){
        f(100);
    }


    tcc_drop(tcc);

}