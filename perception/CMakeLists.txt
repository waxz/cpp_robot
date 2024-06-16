
set(THIRD_PARTY_DIR ${CMAKE_SOURCE_DIR}/third_party)



#build_external_project(Ceres lib ${THIRD_PARTY_DIR}/ceres-solver-2.2.0.tar.gz  -DEigen3_ROOT=${Eigen3_ROOT} -DCMAKE_BUILD_TYPE=Release -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DOCUMENTATION=OFF)
#find_package(Ceres  REQUIRED)
#message(Ceres_ROOT : ${Ceres_ROOT} ,CERES_LIBRARIES : ${CERES_LIBRARIES} )

#build_external_project(autodiff lib ${THIRD_PARTY_DIR}/autodiff-1.1.2.tar.gz  -DCMAKE_BUILD_TYPE=Release  -DAUTODIFF_BUILD_TESTS=OFF -DAUTODIFF_BUILD_PYTHON=OFF -DAUTODIFF_BUILD_EXAMPLES=OFF -DAUTODIFF_BUILD_DOCS=OFF)
#find_package(autodiff REQUIRED)

#message(STATUS "optim use  EIGEN3_INCLUDE_DIRS : ${EIGEN3_INCLUDE_DIRS}")

#build_external_project_command(optim lib ${THIRD_PARTY_DIR}/optim-master.zip "EIGEN_INCLUDE_PATH=${EIGEN3_INCLUDE_DIRS} ./configure -i ./install -l eigen -p" "make && mkdir -p ./install/lib" "make install"  -DCMAKE_BUILD_TYPE=Release  -DAUTODIFF_BUILD_TESTS=OFF -DAUTODIFF_BUILD_PYTHON=OFF -DAUTODIFF_BUILD_EXAMPLES=OFF -DAUTODIFF_BUILD_DOCS=OFF)

#--header-only-version
#build_external_project_command(optim lib ${THIRD_PARTY_DIR}/optim-master.zip "EIGEN_INCLUDE_PATH=${EIGEN3_INCLUDE_DIRS} ./configure --header-only-version -i ./install -l eigen -p" "make && mkdir -p ./install/lib" "make install"  -DCMAKE_BUILD_TYPE=Release  -DAUTODIFF_BUILD_TESTS=OFF -DAUTODIFF_BUILD_PYTHON=OFF -DAUTODIFF_BUILD_EXAMPLES=OFF -DAUTODIFF_BUILD_DOCS=OFF)


#find_package(optim REQUIRED)
#/home/waxz/CLionProjects/libroscpp/cmake-build-release-host/force_optim/build/lib/optim/src/optim/install/lib/liboptim.so
#message("optim_BUILD_DIR "${optim_BUILD_DIR})
#set(optim_ROOT ${optim_BUILD_DIR}/optim)
#message("optim_ROOT "${optim_ROOT})

#set(optim_INCLUDE_DIR ${optim_ROOT}/header_only_version)
#message("optim_INCLUDE_DIR "${optim_INCLUDE_DIR})

#set(optim_INCLUDE_DIR ${optim_ROOT}/include/optim)
#set(optim_LIBRARY ${optim_ROOT}/lib/liboptim.so)
#message("optim_INCLUDE_DIR "${optim_INCLUDE_DIR})
#message("optim_LIBRARY "${optim_LIBRARY})


#add_library(liboptim SHARED IMPORTED)
#set_target_properties(liboptim PROPERTIES
#        IMPORTED_LOCATION ${optim_LIBRARY}
#)
#
#set_property(TARGET liboptim PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${optim_INCLUDE_DIR}  ${EIGEN3_INCLUDE_DIRS})
#set_property(TARGET liboptim PROPERTY INTERFACE_LINK_LIBRARIES autodiff::autodiff )
##target_include_directories(liboptim INTERFACE ${optim_INCLUDE_DIR} )
##target_link_libraries(liboptim INTERFACE ${optim_LIBRARY} autodiff::autodiff  )
#print_include_dir(liboptim)

#====



add_executable(point_cloud_test  point_cloud_test.cpp)
target_include_directories(point_cloud_test PUBLIC ${CMAKE_SOURCE_DIR}/include  )

target_link_libraries (point_cloud_test Eigen3::Eigen)


#add_executable(ceres_simple ceres_simple.cpp)
#target_link_libraries(ceres_simple PUBLIC Ceres::ceres)
#target_link_libraries (ceres_simple PUBLIC Eigen3::Eigen)
#target_include_directories(ceres_simple PUBLIC ${CMAKE_SOURCE_DIR}/include  )

#add_executable(ceres_numeric_wrap_test ceres_numeric_wrap_test.cpp)
#target_link_libraries (ceres_numeric_wrap_test PUBLIC Eigen3::Eigen)
#target_link_libraries(ceres_numeric_wrap_test PUBLIC Ceres::ceres)
#target_include_directories(point_cloud_test PUBLIC ${CMAKE_SOURCE_DIR}/include  )

#add_executable(optimlib_test optimlib_test.cpp)

#target_include_directories(optimlib_test PUBLIC ${CMAKE_SOURCE_DIR}/include  ${optim_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} )
#target_link_libraries(optimlib_test PUBLIC autodiff::autodiff )


add_library(pointcloud_process SHARED pointcloud_process.cpp #pointcloud_calib_optim.cpp
        pointcloud_pallet_detect_impl.cpp  pointcloud_pallet_detect.cpp )
#set_asan(pointcloud_process)
#set_stackalign(pointcloud_process)
#set_debug(pointcloud_process)
set_arch_native(pointcloud_process)

target_include_directories(pointcloud_process PUBLIC ${CMAKE_SOURCE_DIR}/include  ${optim_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS}
        ${toml11_ROOT}/include

)

#target_link_libraries(pointcloud_process PUBLIC Ceres::ceres)
target_link_libraries (pointcloud_process PUBLIC Eigen3::Eigen)
target_link_libraries (pointcloud_process PUBLIC tinyalloc)

#target_link_libraries(pointcloud_process PUBLIC autodiff::autodiff )
add_executable(pointcloud_process_test pointcloud_process_test.c)
target_link_libraries(pointcloud_process_test PUBLIC pointcloud_process )

add_executable(pointcloud_process_cpp_test pointcloud_process_test.cpp)
target_link_libraries(pointcloud_process_cpp_test PUBLIC pointcloud_process )

add_executable(pallet_detector pallet_detector.cpp)
target_link_libraries(pallet_detector pointcloud_process dds_helper_shared)


INSTALL(TARGETS pointcloud_process DESTINATION lib)
INSTALL(FILES "pointcloud_process.h" DESTINATION include)
INSTALL(FILES "pointcloud_calib_optim.h" DESTINATION include)
INSTALL(FILES "pointcloud_pallet_detect.h" DESTINATION include)
