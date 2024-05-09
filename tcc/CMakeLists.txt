


add_library(tcc_builder SHARED tcc_builder.c)
target_include_directories(tcc_builder PUBLIC ${tinycc_LIBTCC_INCLUDE_DIR}  )
target_link_libraries(tcc_builder PUBLIC ${tinycc_LIB} dl pthread m rt ${tinycc_LIB})
target_include_directories(tcc_builder PUBLIC   ${CMAKE_SOURCE_DIR}/include )
set_rpath(tcc_builder)


add_executable(tcc_test_c tcc_test.c)
target_link_libraries(tcc_test_c PUBLIC tcc_builder)

add_executable(tcc_test_cpp tcc_test.cpp)
target_link_libraries(tcc_test_cpp PUBLIC tcc_builder)

INSTALL(TARGETS tcc_builder DESTINATION lib)

