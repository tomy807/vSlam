find_package(OpenCV 4 REQUIRED)

find_package( Sophus REQUIRED )
include_directories("/usr/local/include/sophus")
add_library(myslam SHARED config.cpp camera.cpp frame.cpp mappoint.cpp g2o_types.cpp map.cpp visual_odometry.cpp)
target_include_directories(myslam PUBLIC ${CMAKE_SOURCE_DIR}/include)
# target_link_libraries(config ${OpenCV_LIBS})
target_link_libraries(myslam Sophus::Sophus)
# target_link_libraries(config ${PROJECT_SOURCE_DIR}/3rdparty/g2o/lib/libg2o.so)

target_link_libraries( myslam
    ${THIRD_PARTY_LIBS}
)