#add_library( message_center_types_tinyalloc STATIC message_center_types.c)
#set_pic( message_center_types_tinyalloc )
#target_include_directories(message_center_types_tinyalloc PUBLIC ${CMAKE_SOURCE_DIR}/include  ${CMAKE_CURRENT_SOURCE_DIR})
#target_link_libraries(message_center_types_tinyalloc PUBLIC tinyalloc )


#==https://stackoverflow.com/questions/10487256/cmake-how-to-properly-copy-static-librarys-header-file-into-usr-include
#https://stackoverflow.com/questions/55451084/why-does-installdirectory-files-matching-pattern-copy-empty-director


add_library(tinyalloc SHARED  ${CMAKE_SOURCE_DIR}/include/tinyalloc/tinyalloc.c)
#set_pic(tinyalloc)
INSTALL(TARGETS tinyalloc DESTINATION lib)
INSTALL(DIRECTORY ${CMAKE_SOURCE_DIR}/include/tinyalloc DESTINATION  include FILES_MATCHING PATTERN "*.h*")



add_library( message_center_types_tinyalloc STATIC)
set_pic( message_center_types_tinyalloc )
target_sources(message_center_types_tinyalloc
        PRIVATE message_center_types.c
        PUBLIC FILE_SET HEADERS
        BASE_DIRS ${CMAKE_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}
        FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/message_center_handler.h
        ${CMAKE_CURRENT_SOURCE_DIR}/message_center_types.h
        ${CMAKE_SOURCE_DIR}/include/common/c_style.h
        ${CMAKE_SOURCE_DIR}/include/common/types.h
)

target_include_directories(message_center_types_tinyalloc PUBLIC ${CMAKE_SOURCE_DIR}/include  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(message_center_types_tinyalloc PUBLIC tinyalloc )
install(TARGETS message_center_types_tinyalloc FILE_SET HEADERS)

#INSTALL (
#        DIRECTORY ${CMAKE_SOURCE_DIR}/include/
#        DESTINATION include
#        FILES_MATCHING PATTERN "*.h*")