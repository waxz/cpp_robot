#configure_file(package.xml.in ${CMAKE_CURRENT_SOURCE_DIR}/package.xml)



set(ROS_DISTRO $ENV{ROS_DISTRO})
set(LD_LIBRARY_PATH $ENV{LD_LIBRARY_PATH})
message( LD_LIBRARY_PATH = ${LD_LIBRARY_PATH})
message( ROS_DISTRO = ${ROS_DISTRO})
set(ROS_INSTALL_PATH /opt/ros/${ROS_DISTRO})


#https://stackoverflow.com/questions/36648375/what-are-the-differences-between-imported-target-and-interface-libraries


#add_ros(ros_lib rosconsole roscpp  roscpp_serialization xmlrpcpp rostime cpp_common)
function(add_ros target)

    set(ROS_LIB_LIST ${ARGN})
    message(add_ros ROS_LIB_LIST "\n ${ROS_LIB_LIST}\n")

    target_link_directories(${target} PUBLIC ${ROS_INSTALL_PATH}/lib)
    foreach(e IN LISTS ARGN)
#        target_link_libraries(${target} PUBLIC ${e})
        set(${e}_ROOT ${ROS_INSTALL_PATH}/share)
        message(${e}_ROOT : ${${e}_ROOT})
        find_package(${e} REQUIRED )
        message( "find ${e}_INCLUDE_DIRS: " ${${e}_INCLUDE_DIRS} )
        message("find ${e}_LIBRARIES: " ${${e}_LIBRARIES} )
        target_link_libraries(${target} PUBLIC ${${e}_LIBRARIES})
        target_include_directories(${target} PUBLIC ${${e}_INCLUDE_DIRS})
    endforeach()
    target_include_directories(${target} PUBLIC ${ROS_INSTALL_PATH}/include)
    target_link_libraries(${target} PUBLIC tf )

endfunction()

function(add_ros_interface target)

    set(ROS_LIB_LIST ${ARGN})
    message(add_ros ROS_LIB_LIST "\n ${ROS_LIB_LIST}\n")

    target_link_directories(${target} INTERFACE ${ROS_INSTALL_PATH}/lib)
    foreach(e IN LISTS ARGN)
        #        target_link_libraries(${target} PUBLIC ${e})
        set(${e}_ROOT ${ROS_INSTALL_PATH}/share)
        message(${e}_ROOT : ${${e}_ROOT})
        find_package(${e} REQUIRED )
        message( "find ${e}_INCLUDE_DIRS: " ${${e}_INCLUDE_DIRS} )
        message("find ${e}_LIBRARIES: " ${${e}_LIBRARIES} )
        target_link_libraries(${target} INTERFACE ${${e}_LIBRARIES})
        target_include_directories(${target} INTERFACE ${${e}_INCLUDE_DIRS})
    endforeach()
    target_include_directories(${target} INTERFACE ${ROS_INSTALL_PATH}/include)
    target_link_libraries(${target} INTERFACE tf )

endfunction()


#
#if(  DEFINED catkin_package)
#    #file(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/package.xml  ${PACKAGE_XML_CONTENT})
#    catkin_package(
#            INCLUDE_DIRS include
#            LIBRARIES ${PROJECT_NAME})
#
#    set(shared_dirs "launch" "param" )
#    foreach(dir ${shared_dirs})
#        if(EXISTS "${dir}" AND IS_DIRECTORY "${dir}")
#            message("EXISTS：${dir}")
#            #Installing roslaunch Files or Other Resources
#            install(DIRECTORY ${dir}/
#                    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#                    PATTERN ".svn" EXCLUDE)
#        endif()
#    endforeach()
#
#
#
#endif()
#
#
