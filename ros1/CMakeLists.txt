
if (ROS_DISTRO)
    message(STATUS "Found ros ${ROS_DISTRO}, so building ROS-specific functionality.")

    add_library(libros INTERFACE)

    include(cmake/set_ros.cmake)
    #    set_target_properties(liblua PROPERTIES  IMPORTED_LOCATION "${LUA_LIBRARIES}")
    #    target_include_directories(liblua INTERFACE ${LUA_INCLUDE_DIR} ${CMAKE_SOURCE_DIR}/third_party/sol)
    #    target_link_libraries(liblua INTERFACE  dl )

    add_ros_interface(libros rosconsole roscpp  roscpp_serialization xmlrpcpp rostime cpp_common)



else()
    message(STATUS "Couldn't find ros, so not building ROS-specific functionality. Did you source the ROS setup.sh file?")

    message(STATUS "Build roscpp from source")

    ###############
    ## roscpp_VERSION_MAJOR
    ###########
    list(GET roscpp_VERSION_LIST 0 roscpp_VERSION_MAJOR)
    list(GET roscpp_VERSION_LIST 1 roscpp_VERSION_MINOR)
    list(GET roscpp_VERSION_LIST 2 roscpp_VERSION_PATCH)

    #configure_file(./libros/include/ros/common.h.in ./libros/include/ros/common.h)

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/include/ros/common.h.in ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/include/ros/common.h)

    # Output test results to config.h
    #configure_file(./libros/libros/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/config.h)

    configure_file(${CMAKE_BINARY_DIR}/ros1/console_bridge/console_bridge_export.h ${CMAKE_CURRENT_SOURCE_DIR}/libros/console_bridge-master/include/console_bridge/console_bridge_export.h COPYONLY)



    ### boost
    #https://stackoverflow.com/questions/33242956/cmake-passing-lists-on-command-line
    #    build_external_project(Boost lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/boost-1.82.0.tar.xz -DCMAKE_BUILD_TYPE=Release '-DBOOST_INCLUDE_LIBRARIES=thread;chrono;filesystem;system;date_time;regex' -DBUILD_SHARED_LIBS=ON )


#    set(BOOST_REQUIRED_MODULES thread chrono filesystem system date_time regex)
    set(BOOST_REQUIRED_MODULES thread chrono filesystem system date_time regex)


    find_package(Boost 1.71.0 REQUIRED COMPONENTS ${BOOST_REQUIRED_MODULES})
    message(STATUS "Boost_ROOT: ${Boost_ROOT}")



    set(BUILD_SHARED_LIBS ON)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libros/console_bridge-master console_bridge)

    set(LIBROS_SOURCE_FILES
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/actionlib-noetic-devel/actionlib/src/connection_monitor.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/actionlib-noetic-devel/actionlib/src/goal_id_generator.cpp


            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcClient.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcDispatch.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcServer.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcServerConnection.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcServerMethod.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcSocket.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcSource.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcUtil.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/src/XmlRpcValue.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/libb64/src/cdecode.c
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/libb64/src/cencode.c

            ${CMAKE_CURRENT_SOURCE_DIR}/libros/rosconsole-noetic-devel/src/rosconsole/rosconsole_backend.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/rosconsole-noetic-devel/src/rosconsole/rosconsole.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/rosconsole-noetic-devel/src/rosconsole/impl/rosconsole_print.cpp

            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2/src/cache.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2/src/buffer_core.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2/src/static_cache.cpp

            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/src/buffer.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/src/transform_listener.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/src/buffer_client.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/src/buffer_server.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/src/transform_broadcaster.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/src/static_transform_broadcaster.cpp

            ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/cpp_common/src/debug.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/cpp_common/src/header.cpp

            ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/rostime/src/rate.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/rostime/src/duration.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/rostime/src/time.cpp

            ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/roscpp_serialization/src/serialization.cpp


            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/master.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/network.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/subscriber.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/common.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/publisher_link.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_publication.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/connection.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/single_subscriber_publisher.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/param.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_server.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/wall_timer.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/xmlrpc_manager.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/publisher.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/timer.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/io.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/names.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/topic.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/topic_manager.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/poll_manager.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/publication.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/statistics.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/intraprocess_subscriber_link.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/intraprocess_publisher_link.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/callback_queue.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_server_link.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_client.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/node_handle.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/connection_manager.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/file_log.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport/transport.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport/transport_udp.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport/transport_tcp.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/subscriber_link.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_client_link.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport_publisher_link.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/transport_subscriber_link.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service_manager.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/rosout_appender.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/init.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/subscription.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/subscription_queue.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/spinner.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/internal_timer_manager.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/message_deserializer.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/poll_set.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/service.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/this_node.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/src/libros/steady_timer.cpp


            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry-noetic-devel/tf/src/cache.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry-noetic-devel/tf/src/tf.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry-noetic-devel/tf/src/transform_broadcaster.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry-noetic-devel/tf/src/transform_listener.cpp

    )



    set(LIBROS_INCLUDE_DIR
            ${Boost_INCLUDE_DIRS}
            #        ${CMAKE_CURRENT_BINARY_DIR}
            #        ${CMAKE_CURRENT_BINARY_DIR}/console_bridge

            ${CMAKE_CURRENT_SOURCE_DIR}/libros/include

            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/clients/roscpp/include

            ${CMAKE_CURRENT_SOURCE_DIR}/libros/actionlib-noetic-devel/actionlib/include
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/libb64/include
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/ros_comm-noetic-devel/utilities/xmlrpcpp/include
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/rosconsole-noetic-devel/include
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2/include
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry2-noetic-devel/tf2_ros/include

            ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/roscpp_traits/include
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/cpp_common/include
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/rostime/include
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/roscpp_core-noetic-devel/roscpp_serialization/include
            ${CMAKE_CURRENT_SOURCE_DIR}/libros/geometry-noetic-devel/tf/include

    )

    add_library(libros SHARED ${LIBROS_SOURCE_FILES})
    target_include_directories(libros PUBLIC ${LIBROS_INCLUDE_DIR} )
    target_link_libraries(libros PUBLIC ${Boost_LIBRARIES} console_bridge)
    set_rpath_origin(libros)

    INSTALL(TARGETS libros DESTINATION lib)
#    INSTALL(DIRECTORY ${LIBROS_INCLUDE_DIR} DESTINATION ${CMAKE_INSTALL_PREFIX})

    #    install_external_project(Boost)
    #    set(shared_dirs ${Boost_ROOT}/lib )
    #    foreach(dir ${shared_dirs})
    #        if(EXISTS "${dir}" AND IS_DIRECTORY "${dir}")
    #            message("EXISTS：${dir}")
    #            #Installing roslaunch Files or Other Resources
    #            install(DIRECTORY ${dir}/
    #                    DESTINATION lib
    #                    PATTERN ".svn" EXCLUDE)
    #        endif()
    #    endforeach()
endif()


add_subdirectory(api)
