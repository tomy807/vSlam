#include <gtest/gtest.h>
#include "../include/config.h"
#include "../include/camera.h"
#include <fstream>
#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz/vizcore.hpp>


using namespace myslam;
// using namespace std;
std::string filedir;
std::string dataset_dir;
std::vector<std::string> rgb_files, depth_files;
std::vector<double> rgb_times, depth_times;

TEST(CONFIG,CONFIGTEST){
    
    filedir="/home/tomy807/vslam/config/default.yaml";
    Config::setParameterFile ( filedir);
    dataset_dir = Config::get<std::string> ( "dataset_dir" );
    EXPECT_TRUE(dataset_dir=="/home/tomy807/rgbd_dataset_freiburg1_xyz");
    std::ifstream fin ( dataset_dir+"/associations.txt" );
    if ( !fin )
    {
        std::cout<<"please generate the associate file called associate.txt!"<<std::endl;
    }

    while ( !fin.eof() )
    {
        std::string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }

}
TEST(CAMERA,CAMERATEST){
    Camera::Ptr camera ( new myslam::Camera );

    cv::viz::Viz3d vis ( "Visual Odometry" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose ( cam_pose );

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );
    std::cout<<"read total "<<rgb_files.size() <<" entries"<<std::endl;
    for ( int i=0; i<rgb_files.size(); i++ )
    {
        cv::Mat color = cv::imread ( rgb_files[i] );
        cv::Mat depth = cv::imread ( depth_files[i], -1 );

        cv::Mat img_show = color.clone();
        cv::imshow ( "image", img_show );
        cv::waitKey ( 1 );
        vis.spinOnce ( 1, false );
    }
}