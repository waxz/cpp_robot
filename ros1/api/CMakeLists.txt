
#option(ENABLE_LTO "Enable cross language linking time optimization" OFF)
#if(ENABLE_LTO)
#    include(CheckIPOSupported)
#    check_ipo_supported(RESULT supported OUTPUT error)
#    if(supported)
#        message(STATUS "IPO / LTO enabled")
#        set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
#        add_link_options(-fuse-ld=lld)
#    else()
#        message(STATUS "IPO / LTO not supported: <${error}>")
#    endif()
#endif()

#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=gold")

#add_link_options("-fuse-ld=lld")
#set(CMAKE_CXX_LINKER_)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")


#add_library(ros_helper_message SHARED src/ros_helper_message.c)

#add_library(tinyalloc SHARED ${CMAKE_CURRENT_SOURCE_DIR}/include/tinyalloc/tinyalloc.c)


#target_link_libraries(ros_helper_message PUBLIC tinyalloc)
add_library(ros_helper_impl SHARED  src/ros_helper_impl.cpp src/ros_helper_topic.cpp)

#target_include_directories( ros_helper_message PUBLIC
#        ${CMAKE_CURRENT_SOURCE_DIR}/include
#)
target_include_directories( ros_helper_impl PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
target_link_libraries(ros_helper_impl PRIVATE libros)
target_link_libraries(ros_helper_impl PUBLIC
        toml11::toml11
        tinyalloc
#        ros_helper_message
        message_center_types_tinyalloc
        absl::strings  absl::str_format
        absl::log absl::log_internal_check_impl absl::log_internal_check_op
#        absl::statusor
#        absl::debugging
#        absl::random_random
        absl::any
#        absl::inlined_vector
#        absl::flat_hash_map
)

add_library(ros_helper SHARED src/ros_helper.cpp)
target_link_libraries(ros_helper PRIVATE libros)

target_link_libraries(ros_helper PUBLIC ros_helper_impl)
add_executable(ros_helper_test src/ros_helper_test.c)
target_link_libraries( ros_helper_test PUBLIC ros_helper)
set_asan(ros_helper_test)


#===copy

copy_files_with_pattern("${CMAKE_CURRENT_SOURCE_DIR}/src" "*toml" "${EXECUTABLE_OUTPUT_PATH}" TOML_COPIED_FILES)
copy_files_with_pattern("${CMAKE_CURRENT_SOURCE_DIR}/src" "*xml" "${EXECUTABLE_OUTPUT_PATH}" XML_COPIED_FILES)
list(APPEND COMBINED_LIST ${TOML_COPIED_FILES} ${XML_COPIED_FILES})

message("TOML_COPIED_FILES ${TOML_COPIED_FILES}")
message("XML_COPIED_FILES ${XML_COPIED_FILES}")
message("COMBINED_LIST ${COMBINED_LIST}")

add_custom_target(ros_test_file_copy DEPENDS ${COMBINED_LIST})
add_dependencies(ros_helper_test ros_test_file_copy)
#==

set_target_properties(ros_helper_impl PROPERTIES
        BUILD_WITH_INSTALL_RPATH FALSE
        LINK_FLAGS "-Wl,-rpath,$ORIGIN/../lib")
set_target_properties(ros_helper PROPERTIES
        BUILD_WITH_INSTALL_RPATH FALSE
        LINK_FLAGS "-Wl,-rpath,$ORIGIN/../lib")

# state that SqrtLibrary need PIC when the default is shared libraries
#set_target_properties(ros_helper_impl PROPERTIES  POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS} )
#set_target_properties(ros_helper_impl PROPERTIES  POSITION_INDEPENDENT_CODE TRUE )
#set_target_properties(ros_helper PROPERTIES  POSITION_INDEPENDENT_CODE TRUE )

INSTALL(TARGETS tinyalloc DESTINATION lib)
INSTALL(TARGETS ros_helper_impl DESTINATION lib)
INSTALL(TARGETS ros_helper DESTINATION lib)
#INSTALL(TARGETS ros_helper_message DESTINATION lib)
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/tinyalloc DESTINATION ${CMAKE_INSTALL_PREFIX}/include/)

