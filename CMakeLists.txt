project(vSLAM)
cmake_minimum_required(VERSION 3.0.0)

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )


# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV 4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus 

find_package( Sophus REQUIRED )
include_directories("/usr/local/include/sophus")

# find_package( G2O REQUIRED )
include_directories( "/home/tomy807/BAtest/visual-slam/g2o-20170730_git_git/g2o" )

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )


set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension ${G2O_LIBS}
)

include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory(src)
add_subdirectory(test)