cmake_policy(SET CMP0074 NEW)
find_package(Threads REQUIRED)
# target_link_libraries( target PUBLIC Threads::Threads)

# ---------------------------------------------------------------------------------------------------------
# Handle C++ standard version.
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # use -std=c++... instead of -std=gnu++...

# ---------------------------------------------------------------------------------------------------------

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

# ---------------------------------------------------------------------------------------------------------

# flags

# fix ld: unrecognized option '--push-state--no-as-needed'
# https://stackoverflow.com/questions/50024731/ld-unrecognized-option-push-state-no-as-needed
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold")



# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Debug' as none was specified.")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
            "MinSizeRel" "RelWithDebInfo")
endif()

# https://stackoverflow.com/questions/17707044/getting-cmake-to-give-an-error-warning-about-unreferenced-symbols

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Werror=return-type")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
set(CMAKE_EXE_LINKER_FLAGS    "-Wl,--as-needed ${CMAKE_EXE_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--as-needed ${CMAKE_SHARED_LINKER_FLAGS}")

#https://stackoverflow.com/questions/48754619/what-are-cmake-build-type-debug-release-relwithdebinfo-and-minsizerel

function(print_all_env)
    message(STATUS "CMake Environment Variables:")
    get_cmake_property(_variableNames VARIABLES)
    foreach(_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
    message(STATUS "End of CMake Environment Variables")

endfunction()

# Define a function to copy files matching a pattern from source to destination directory
function(copy_files_with_pattern SOURCE_DIR PATTERN DEST_DIR OUTPUT_VARIABLE)
    file(GLOB FILES "${SOURCE_DIR}/${PATTERN}")
    message("copy_files_with_pattern FILES: ${FILES}")
    message("copy_files_with_pattern PATTERN: ${PATTERN}")
    message("copy_files_with_pattern DEST_DIR: ${DEST_DIR}")

    message("copy_files_with_pattern OUTPUT_VARIABLE: ${OUTPUT_VARIABLE} : ${${OUTPUT_VARIABLE}}")

    foreach(FILE ${FILES})
        get_filename_component(FILENAME ${FILE} NAME)
        set(DEST_PATH "${DEST_DIR}/${FILENAME}")
        add_custom_command(
                OUTPUT "${DEST_PATH}"
                COMMAND ${CMAKE_COMMAND} -E copy "${FILE}" "${DEST_PATH}"
                DEPENDS "${FILE}"
                COMMENT "Copying ${FILENAME} to ${DEST_DIR}"
        )
        list(APPEND ${OUTPUT_VARIABLE} "${DEST_PATH}")
    endforeach()
    set(${OUTPUT_VARIABLE} "${${OUTPUT_VARIABLE}}" PARENT_SCOPE)
    message("copy_files_with_pattern output: ${OUTPUT_VARIABLE} : ${${OUTPUT_VARIABLE}}")
endfunction()


#PIC

function(set_pic target)
    set_target_properties(${target} PROPERTIES
            POSITION_INDEPENDENT_CODE 1
    )
endfunction()

list(APPEND ABSL_GCC_FLAGS
        "-Wall"
        "-Wextra"
        "-Wcast-qual"
        "-Wconversion-null"
        "-Wformat-security"
        "-Wmissing-declarations"
        "-Woverlength-strings"
        "-Wpointer-arith"
        "-Wundef"
        "-Wunused-local-typedefs"
        "-Wunused-result"
        "-Wvarargs"
        "-Wvla"
        "-Wwrite-strings"
        "-DNOMINMAX"
        )

list(APPEND ABSL_GCC_TEST_FLAGS
        "-Wall"
        "-Wextra"
        "-Wcast-qual"
        "-Wconversion-null"
        "-Wformat-security"
        "-Woverlength-strings"
        "-Wpointer-arith"
        "-Wundef"
        "-Wunused-local-typedefs"
        "-Wunused-result"
        "-Wvarargs"
        "-Wvla"
        "-Wwrite-strings"
        "-DNOMINMAX"
        "-Wno-deprecated-declarations"
        "-Wno-missing-declarations"
        "-Wno-self-move"
        "-Wno-sign-compare"
        "-Wno-unused-function"
        "-Wno-unused-parameter"
        "-Wno-unused-private-field"
        )

function (SET_GCC_FLAGS target)
    target_compile_options(
            ${target}
            PRIVATE
            ${ABSL_GCC_FLAGS}
    )

endfunction()

#https://stackoverflow.com/questions/48754619/what-are-cmake-build-type-debug-release-relwithdebinfo-and-minsizerel
#https://stackoverflow.com/questions/78322480/improve-g-compiler-flags-for-debug-and-release
#https://developers.redhat.com/articles/2022/06/02/use-compiler-flags-stack-protection-gcc-and-clang#control_flow_integrity
# -fsplit-stack may cause crash

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Wformat=2 -Wconversion -Wtrampolines -Wshadow ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer  -fstack-protector -fstack-protector-strong  -fstack-protector-all -fstack-protector-explicit  -fsplit-stack")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer  -fstack-protector -fstack-protector-strong  -fstack-protector-all -fstack-protector-explicit")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mshstk -Walloca ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wstack-usage=1000000 -fstack-usage")

if (CMAKE_BUILD_TYPE MATCHES Release)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -pedantic")
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-generate -fprofile-use")


    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--whole-archive  -Wl,--no-whole-archive")
    #  -ffp-contract=fast -flto
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -O3 -march=native -ftree-vectorize -fopt-info-vec-optimized -ffp-contract=fast -flto")

# release
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -O3 -march=native -ftree-vectorize -fopt-info-vec-optimized")

    # debug
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g  -ffast-math -march=native")


endif ()

#if (CMAKE_BUILD_TYPE MATCHES Debug)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -pedantic")
#    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-generate -fprofile-use")
#
#
#    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wl,--whole-archive  -Wl,--no-whole-archive")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -O3 -march=native -ftree-vectorize -fopt-info-vec-optimized ")
#
#endif ()
if (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    #-Wall -Wextra -pedantic
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -Ofast  -ffast-math -ftree-vectorize   -march=native -funsafe-loop-optimizations -mavx -mfma")
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -Wall -Wextra   -Ofast   ")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -Wall -Wextra   -march=native   -O3 -g -DNDEBUG  -Ofast -funsafe-loop-optimizations ")
#
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -ftree-vectorize  -ffast-math -fopt-info-vec-optimized  -opt-report=5 ")
#
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -pthread  -mstackrealign  -march=native")
#
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -pedantic")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -pedantic  -Wno-unknown-pragmas -Wno-sign-compare -Woverloaded-virtual -Wwrite-strings -Wno-unused")

    #origin
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -O3 -g -march=native -ftree-vectorize -fopt-info-vec-optimized -ffp-contract=fast -flto")

    #==============new
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Og -D_GLIBCXX_DEBUG -D_GLIBCXX_ASSERTIONS ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g  -ffast-math -march=native")

    #https://developers.redhat.com/articles/2022/06/02/use-compiler-flags-stack-protection-gcc-and-clang#



endif ()
if (CMAKE_BUILD_TYPE MATCHES Debug)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -Wall -Wextra -pedantic   -Wno-unknown-pragmas -Wno-sign-compare -Woverloaded-virtual -Wwrite-strings -Wno-unused")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g -O0 -o -ggdb  -ggdb3 -fprofile-arcs -mstackrealign  -march=native  ")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 ")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -mstackrealign  -march=native  -fstack-protector")

#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -fno-optimize-sibling-calls -fno-omit-frame-pointer -fno-builtin-malloc -fno-builtin-calloc -fno-builtin-realloc -fno-builtin-free")

    #set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=undefined   -fsanitize=address -fsanitize=leak -fsanitize=leak ")
    #set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address  -fsanitize=leak")
    #set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=address  -fsanitize=leak")
endif ()

# ASAN_OPTIONS=abort_on_error=1:debug=1:strict_string_checks=1:detect_stack_use_after_return=1:check_initialization_order=1:strict_init_order=1:symbolize=1:detect_leaks=0
function(set_asan target)
#    set(ENV{ASAN_OPTIONS} "strict_string_checks=1:detect_stack_use_after_return=1:check_initialization_order=1:strict_init_order=1:symbolize=1")

    # Treat all warnings as errors
    #    target_compile_options(${target} PRIVATE "-Werror")

    target_compile_options(${target} PUBLIC "-fsanitize=undefined")
    target_compile_options(${target} PUBLIC "-fsanitize=address")
    target_compile_options(${target} PUBLIC "-fsanitize=leak")



#    target_compile_options(${target} PUBLIC "-fno-builtin-malloc")
#    target_compile_options(${target} PUBLIC "-fno-builtin-calloc")
#    target_compile_options(${target} PUBLIC "-fno-builtin-realloc")
#    target_compile_options(${target} PUBLIC "-fno-builtin-free")


#    target_compile_options(${target} PUBLIC "-fsanitize-address-use-after-scope")

#
#    target_compile_options(${target} PUBLIC "-fsanitize-recover=address")

#    target_compile_options(${target} PUBLIC "-fno-optimize-sibling-calls")


#    target_compile_options(${target} PUBLIC "-fno-omit-frame-pointer")

#    target_compile_options(${target} PUBLIC "-fstack-protector")
#    target_compile_options(${target} PUBLIC  "-fuse-ld=gold")

#    target_compile_options(${target} PUBLIC -fsanitize-coverage=trace-pc-guard -fsanitize=address,undefined,leak -fuse-ld=gold)


    target_link_libraries(${target} PUBLIC "-fsanitize=address  -fsanitize=leak -fsanitize=undefined")
    #    target_compile_options(${target} PUBLIC "-fsanitize=memory")
    #    target_link_libraries(${target} PUBLIC "-fsanitize=memory")
    #    target_compile_options(${target} PUBLIC "-fsanitize=thread")
    #    target_link_libraries(${target} PUBLIC "-fsanitize=thread")

endfunction(set_asan)
# ---------------------------------------------------------------------------------------------------------


function(print_include_dir target)
    get_target_property(LIBA_INCLUDES ${target} INCLUDE_DIRECTORIES)
    message(${target} INCLUDES : ${LIBA_INCLUDES})
endfunction()

# ---------------------------------------------------------------------------------------------------------


function(set_omp target)
    find_package(OpenMP)
    if(OpenMP_CXX_FOUND)
        target_link_libraries(${target} PUBLIC OpenMP::OpenMP_CXX)
        target_include_directories(${target} PUBLIC ${OpenMP_CXX_INCLUDE_DIRS} )
        target_link_libraries(${target} PUBLIC
                ${OpenMP_CXX_LIBRARIES}
                )
        target_compile_options(${target} PUBLIC ${OpenMP_CXX_FLAGS})
    endif()

endfunction()










# ---------------------------------------------------------------------------------------------------------

macro(install_target)
    message(STATUS "Configuring installation for target(s) ${ARGV0}")
    install(TARGETS ${ARGV0}
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
            )
endmacro()

# ---------------------------------------------------------------------------------------------------------

# https://stackoverflow.com/a/51987470
#https://stackoverflow.com/questions/32183975/how-to-print-all-the-properties-of-a-target-in-cmake

# NOTE: Only used in multi-configuration environments
#set(CMAKE_CONFIGURATION_TYPES "Debug;RelWithDebInfo" CACHE STRING "My multi config types" FORCE)
# Get all propreties that cmake supports
execute_process(COMMAND cmake --help-property-list OUTPUT_VARIABLE CMAKE_PROPERTY_LIST)

# Convert command output into a CMake list
STRING(REGEX REPLACE ";" "\\\\;" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")
STRING(REGEX REPLACE "\n" ";" CMAKE_PROPERTY_LIST "${CMAKE_PROPERTY_LIST}")
# Fix https://stackoverflow.com/questions/32197663/how-can-i-remove-the-the-location-property-may-not-be-read-from-target-error-i
list(FILTER CMAKE_PROPERTY_LIST EXCLUDE REGEX "^LOCATION$|^LOCATION_|_LOCATION$")
# For some reason, "TYPE" shows up twice - others might too?
list(REMOVE_DUPLICATES CMAKE_PROPERTY_LIST)

# build whitelist by filtering down from CMAKE_PROPERTY_LIST in case cmake is
# a different version, and one of our hardcoded whitelisted properties
# doesn't exist!
unset(CMAKE_WHITELISTED_PROPERTY_LIST)
foreach(prop ${CMAKE_PROPERTY_LIST})
    if(prop MATCHES "^(INTERFACE|[_a-z]|IMPORTED_LIBNAME_|MAP_IMPORTED_CONFIG_)|^(COMPATIBLE_INTERFACE_(BOOL|NUMBER_MAX|NUMBER_MIN|STRING)|EXPORT_NAME|IMPORTED(_GLOBAL|_CONFIGURATIONS|_LIBNAME)?|NAME|TYPE|NO_SYSTEM_FROM_IMPORTED)$")
        list(APPEND CMAKE_WHITELISTED_PROPERTY_LIST ${prop})
    endif()
endforeach(prop)

function(print_properties)
    message ("CMAKE_PROPERTY_LIST = ${CMAKE_PROPERTY_LIST}")
endfunction(print_properties)

function(print_whitelisted_properties)
    message ("CMAKE_WHITELISTED_PROPERTY_LIST = ${CMAKE_WHITELISTED_PROPERTY_LIST}")
endfunction(print_whitelisted_properties)

function(set_test tgt)
    target_include_directories( ${tgt} PUBLIC
            ${CMAKE_SOURCE_DIR}/include
            ${CMAKE_SOURCE_DIR}/include/fakeit/single_header
            ${FRUIT_INCLUDE_DIR}
    )

    target_link_libraries(${tgt} PUBLIC
            libcatch2
            doctest::doctest
            nanobench::nanobench
            ${FRUIT_LIBRARY}
    )
    #https://abseil.io/docs/cpp/platforms/compilerflags#gcc-flags
    SET_GCC_FLAGS(${tgt})
    set_asan(${tgt})
endfunction()

function(print_target_properties tgt)
    if(NOT TARGET ${tgt})
        message("There is no target named '${tgt}'")
        return()
    endif()

    get_target_property(target_type ${tgt} TYPE)
    if(target_type STREQUAL "INTERFACE_LIBRARY")
        set(PROP_LIST ${CMAKE_WHITELISTED_PROPERTY_LIST})
    else()
        set(PROP_LIST ${CMAKE_PROPERTY_LIST})
    endif()

    foreach (prop ${PROP_LIST})
        string(REPLACE "<CONFIG>" "${CMAKE_BUILD_TYPE}" prop ${prop})
        # message ("Checking ${prop}")
        get_property(propval TARGET ${tgt} PROPERTY ${prop} SET)
        if (propval)
            get_target_property(propval ${tgt} ${prop})
            message ("${tgt} ${prop} = ${propval}")
        endif()
    endforeach(prop)
endfunction(print_target_properties)

