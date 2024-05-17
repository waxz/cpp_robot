copy_files_with_pattern("${CMAKE_CURRENT_SOURCE_DIR}/" "*toml" "${EXECUTABLE_OUTPUT_PATH}" TOML_COPIED_FILES)
copy_files_with_pattern("${CMAKE_CURRENT_SOURCE_DIR}/" "*xml" "${EXECUTABLE_OUTPUT_PATH}" XML_COPIED_FILES)
list(APPEND COMBINED_LIST ${TOML_COPIED_FILES} ${XML_COPIED_FILES})
message("TOML_COPIED_FILES ${TOML_COPIED_FILES}")
message("XML_COPIED_FILES ${XML_COPIED_FILES}")
message("COMBINED_LIST ${COMBINED_LIST}")
add_custom_target(test_file_copy DEPENDS ${COMBINED_LIST})

if (BUILD_DDS AND BUILD_ROS1)

    message(STATUS "build demo with dds+ros1")
    add_executable(dds_ros_bridge dds_ros_bridge.cpp)
    target_link_libraries( dds_ros_bridge PUBLIC ros_helper dds_helper_shared)
#    set_asan(dds_ros_bridge)
    add_dependencies(dds_ros_bridge test_file_copy)

#    set_rpath(dds_ros_bridge)
#    set_target_properties(dds_ros_bridge PROPERTIES
#            BUILD_WITH_INSTALL_RPATH FALSE
#            BUILD_RPATH_USE_ORIGIN TRUE
#            LINK_FLAGS "-Wl,-rpath,$ORIGIN/../lib")
    set_target_properties(dds_ros_bridge PROPERTIES INSTALL_RPATH "$ORIGIN/../lib")
    INSTALL(TARGETS dds_ros_bridge DESTINATION bin )
    INSTALL(FILES  ${COMBINED_LIST} DESTINATION bin )


endif ()