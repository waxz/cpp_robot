add_library(dds_handler_static STATIC dds_handler.cpp dds_handler_variant.cpp )
set_pic(dds_handler_static )
add_library(dds_handler_shared SHARED dds_handler.cpp dds_handler_variant.cpp )
INSTALL(TARGETS dds_handler_static DESTINATION lib)
INSTALL(TARGETS dds_handler_shared DESTINATION lib)

target_link_libraries(dds_handler_static PUBLIC
        dds_message_type_lib
        toml11::toml11
        absl::strings  absl::str_format
        absl::log absl::log_internal_check_impl absl::log_internal_check_op
        #        absl::statusor
        #        absl::debugging
        #        absl::random_random
        absl::any
        #        absl::inlined_vector
        #        absl::flat_hash_map
)

target_include_directories(dds_handler_static PUBLIC ${CMAKE_SOURCE_DIR}/include
        ${toml11_ROOT}/include

)
set_omp(dds_handler_static)

target_link_libraries(dds_handler_shared PUBLIC
        dds_message_type_lib
        toml11::toml11
        absl::strings  absl::str_format
        absl::log absl::log_internal_check_impl absl::log_internal_check_op
        #        absl::statusor
        #        absl::debugging
        #        absl::random_random
        absl::any
        #        absl::inlined_vector
        #        absl::flat_hash_map
)

target_include_directories(dds_handler_shared PUBLIC ${CMAKE_SOURCE_DIR}/include
        ${toml11_ROOT}/include

)
set_omp(dds_handler_shared)



#set_asan(dds_handler_static )

add_executable(dds_builder_pub_test dds_builder_pub_test.cpp)
target_link_libraries(dds_builder_pub_test PUBLIC dds_handler_shared )

set_asan(dds_builder_pub_test)

add_executable(dds_builder_sub_test dds_builder_sub_test.cpp)
target_link_libraries(dds_builder_sub_test PUBLIC dds_handler_shared )
set_asan(dds_builder_sub_test)


add_dependencies(dds_builder_pub_test dds_test_file_copy)
add_dependencies(dds_builder_sub_test dds_test_file_copy)

add_library(dds_helper_shared SHARED dds_helper.cpp)
add_library(dds_helper_static SHARED dds_helper.cpp)
set_pic(dds_helper_static )
target_link_libraries(dds_helper_shared PUBLIC
        dds_handler_static
)
target_link_libraries(dds_helper_static PUBLIC
        dds_handler_static
)
target_include_directories(dds_helper_static PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
target_include_directories(dds_helper_shared PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)
INSTALL(TARGETS dds_helper_shared DESTINATION lib)




add_executable(dds_helper_test dds_helper_test.c)
target_link_libraries(dds_helper_test PUBLIC dds_helper_shared )

copy_files_with_pattern("${CMAKE_CURRENT_SOURCE_DIR}/" "*toml" "${EXECUTABLE_OUTPUT_PATH}" TOML_COPIED_FILES)
copy_files_with_pattern("${CMAKE_CURRENT_SOURCE_DIR}/" "*xml" "${EXECUTABLE_OUTPUT_PATH}" XML_COPIED_FILES)
list(APPEND COMBINED_LIST ${TOML_COPIED_FILES} ${XML_COPIED_FILES})
message("TOML_COPIED_FILES ${TOML_COPIED_FILES}")
message("XML_COPIED_FILES ${XML_COPIED_FILES}")
message("COMBINED_LIST ${COMBINED_LIST}")
add_custom_target(dds_test_file_copy DEPENDS ${COMBINED_LIST})


add_dependencies(dds_helper_test dds_test_file_copy)
set_asan(dds_helper_test)



add_executable(dds_simple_participant dds_simple_participant.cpp)
target_link_libraries(dds_simple_participant PUBLIC
#        fastcdr
#        fastrtps
#        dds_message_type_lib
#        absl::strings  absl::str_format absl::log absl::log_internal_check_impl absl::log_internal_check_op absl::any
        dds_handler_static
)
target_include_directories( dds_simple_participant PUBLIC ${CMAKE_SOURCE_DIR}/include
        ${toml11_ROOT}/include
)
add_dependencies(dds_simple_participant dds_test_file_copy)
set_asan(dds_simple_participant )
install(TARGETS dds_simple_participant DESTINATION bin)
