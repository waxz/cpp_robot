set(THIRD_PARTY_DIR ${CMAKE_SOURCE_DIR}/third_party)

#https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html#asio-and-tinyxml2-libraries
# sudo apt install libasio-dev


#https://fast-dds.docs.eprosima.com/en/latest/notes/versions.html#dependencies-compatibilities-library-dependencies
# version
#Fast DDS  v2.14.0
#Fast CDR  v2.2.0
#Foonathan Memory Vendor v1.3.1
#Asio v1.18.1
#TinyXML2 v6.0.0
#OpenSSL v3.1.1
#    set(BOOST_REQUIRED_MODULES  system)
#    find_package(Boost 1.82.0 REQUIRED COMPONENTS ${BOOST_REQUIRED_MODULES})

#https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html#cmake-installation

# C++ API
# https://github.com/eProsima/foonathan_memory_vendor/releases
# https://github.com/eProsima/Fast-CDR/releases
# https://github.com/eProsima/Fast-DDS/releases


# fast-dds-gen
# https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html#fast-dds-gen-installation
#Fast DDS-Gen supports Java versions from 11 to 19.
#The Fast-DDS-Gen folder contains the following packages:
#
#    share/fastddsgen, where the generated Java application is.
#
#    scripts, containing some user friendly scripts.
# To make these scripts accessible from any shell session and directory, add the scripts folder path to the PATH environment variable.

# DDS on microcontrollers
# https://github.com/eProsima/Micro-XRCE-DDS

#    build_external_project(asio lib ${CMAKE_CURRENT_SOURCE_DIR}/third_party/asio-1.30.2.tar.bz2  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON)
#    find_package(asio REQUIRED)

build_external_project(tinyxml2 lib ${THIRD_PARTY_DIR}/tinyxml2-9.0.0.tar.gz  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON)
find_package(tinyxml2 REQUIRED)

#foonathan_memory_vendor-1.3.1.tar.gz
build_external_project(foonathan_memory lib ${THIRD_PARTY_DIR}/foonathan_memory_vendor-1.3.1.tar.gz  -DCMAKE_BUILD_TYPE=Release  -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON)
find_package(foonathan_memory REQUIRED)

#Fast-CDR-2.2.0.tar.gz
#cmake_minimum_required(VERSION 3.16)
build_external_project(fastcdr lib ${THIRD_PARTY_DIR}/Fast-CDR-2.2.0.tar.gz  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON)
set(fastcdr_LIB ${fastcdr_ROOT}/lib/libfastcdr.so)
#message(1fastcdf_ROOT ${fastcdf_ROOT})

# fastcdr-config.cmake
#set(fastcdr_ROOT ${fastcdr_ROOT}/lib/cmake/fastcdr)
#message(1fastcdf_ROOT ${fastcdf_ROOT})
find_package(fastcdr REQUIRED)


#fastrtps-config.cmake
#cmake_minimum_required(VERSION 3.16)
# Fast-DDS-2.11.2.tar.gz

# -DTHIRDPARTY_Asio=FORCE
# -DTHIRDPARTY_TinyXML2=FORCE
#-DAsio_INCLUDE_DIR=${Boost_ROOT}/include/boost -DTHIRDPARTY_BOOST_INCLUDE_DIR=${Boost_ROOT}/include
build_external_project(fastrtps lib ${THIRD_PARTY_DIR}/Fast-DDS-2.14.0.tar.gz  -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -Dfoonathan_memory_ROOT=${foonathan_memory_ROOT} -Dfastcdr_ROOT=${fastcdr_ROOT}  -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DFASTDDS_STATISTICS=OFF -DTinyXML2_ROOT=${tinyxml2_ROOT} )

find_package(fastrtps REQUIRED)
install_external_project(fastrtps)


# global static viriable double free
# https://stackoverflow.com/questions/13234200/double-free-or-corruption-prev-while-calling-do-global-dtors-aux


# create message types library, link fastdds and fastcdr

# Glob all .cxx files in the source directory
file(GLOB DDS_MESSAGE_SOURCES "./message/*.cxx")
MESSAGE(STATUS "DDS_MESSAGE_SOURCES=${DDS_MESSAGE_SOURCES}")

# Add the globbed sources to your library
add_library(dds_message_type_lib STATIC ${DDS_MESSAGE_SOURCES})
target_include_directories(dds_message_type_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/message )
set_pic(dds_message_type_lib)
target_link_libraries(dds_message_type_lib PUBLIC fastcdr fastrtps message_center_types_tinyalloc)
INSTALL(TARGETS dds_message_type_lib DESTINATION lib)


add_subdirectory(impl)
#add_subdirectory(interface)



