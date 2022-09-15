find_package(GTest REQUIRED)

include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV 4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(TEST_BINARY ${PROJECT_NAME}_test)
add_executable(${TEST_BINARY}
                test.cpp)
target_link_libraries(${TEST_BINARY} myslam GTest::Main)
gtest_discover_tests(${TEST_BINARY} WORKING_DIRECTORY
                     ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(yolotest test_yolo.cpp)
target_link_libraries(yolotest ${OpenCV_LIBS})
