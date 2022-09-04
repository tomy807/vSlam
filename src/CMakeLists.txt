find_package(OpenCV 4 REQUIRED)
add_library(config STATIC config.cpp)
target_include_directories(config PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(config ${OpenCV_LIBS})