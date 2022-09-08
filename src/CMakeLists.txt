find_package(OpenCV 4 REQUIRED)

find_package( Sophus REQUIRED )
include_directories("/usr/local/include/sophus")
add_library(config STATIC config.cpp camera.cpp frame.cpp mappoint.cpp g2o_types.cpp map.cpp visual_odometry.cpp)
target_include_directories(config PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(config ${OpenCV_LIBS})
target_link_libraries(config Sophus::Sophus)
target_link_libraries(config ${PROJECT_SOURCE_DIR}/3rdparty/g2o/lib/libg2o.so)