#include <gtest/gtest.h>
#include "../include/config.h"
#include "../include/camera.h"
#include "../include/frame.h"
#include "../include/mappoint.h"
#include "../include/map.h"
#include "../include/visual_odometry.h"
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz/vizcore.hpp>
#include <boost/timer.hpp>


using namespace myslam;
using namespace std;
// std::string filedir;
// std::string dataset_dir;
// std::vector<std::string> rgb_files, depth_files;
// std::vector<double> rgb_times, depth_times;
// Map::Ptr    map_;
// cv::Ptr<cv::ORB> orb_ = cv::ORB::create ( 500, 1.2, 4 );

// TEST(CONFIG,CONFIGTEST){
    
//     filedir="/home/tomy807/vslam/config/default.yaml";
//     Config::setParameterFile ( filedir);
//     dataset_dir = Config::get<std::string> ( "dataset_dir" );
//     EXPECT_TRUE(dataset_dir=="/home/tomy807/rgbd_dataset_freiburg1_xyz");
//     std::ifstream fin ( dataset_dir+"/associations.txt" );
//     if ( !fin )
//     {
//         std::cout<<"please generate the associate file called associate.txt!"<<std::endl;
//     }

//     while ( !fin.eof() )
//     {
//         std::string rgb_time, rgb_file, depth_time, depth_file;
//         fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
//         rgb_times.push_back ( atof ( rgb_time.c_str() ) );
//         depth_times.push_back ( atof ( depth_time.c_str() ) );
//         rgb_files.push_back ( dataset_dir+"/"+rgb_file );
//         depth_files.push_back ( dataset_dir+"/"+depth_file );

//         if ( fin.good() == false )
//             break;
//     }

// }

// TEST(CAMERA,CAMERATEST){
//     Camera::Ptr camera ( new myslam::Camera );
//     cv::viz::Viz3d vis ( "Visual Odometry" );
//     cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
//     cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
//     cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
//     vis.setViewerPose ( cam_pose );

//     world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
//     camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
//     vis.showWidget ( "World", world_coor );
//     vis.showWidget ( "Camera", camera_coor );
//     std::cout<<"read total "<<rgb_files.size() <<" entries"<<std::endl;
//     for ( int i=0; i<rgb_files.size(); i++ )
//     {
//         cv::Mat color = cv::imread ( rgb_files[i] );
//         cv::Mat depth = cv::imread ( depth_files[i], -1 );
//         if ( color.data==nullptr || depth.data==nullptr )
//             break;
//         myslam::Frame::Ptr pFrame = Frame::createFrame();
//         pFrame->camera_ = camera;
//         pFrame->color_ = color;
//         pFrame->depth_ = depth;
//         pFrame->time_stamp_ = rgb_times[i];

//         cv::Mat img_show = color.clone();
//         cv::imshow ( "image", img_show );
//         cv::waitKey ( 1 );
//         vis.spinOnce ( 1, false );
//     }
// }

// vector<cv::KeyPoint> extractKeyPoints(cv::Mat color)
// {
//     vector<cv::KeyPoint>    keypoints_curr_;
//     orb_->detect ( color, keypoints_curr_ );
//     return keypoints_curr_;
// }
// cv::Mat computeDescriptors(cv::Mat color,vector<cv::KeyPoint> keypoints_curr_)
// {   cv::Mat descriptors_curr_;
//     orb_->compute ( color, keypoints_curr_, descriptors_curr_ );
//     return descriptors_curr_;
// }

// TEST(FRAME,EXTRACTKEYPOINTandDEPTH){
//     cv::Mat color = cv::imread ( rgb_files[3] );
//     cv::Mat depth = cv::imread ( depth_files[3], -1 );
//     Camera::Ptr camera ( new myslam::Camera );
//     Frame::Ptr pFrame = Frame::createFrame();

//     pFrame->camera_ = camera;
//     pFrame->color_ = color;
//     pFrame->depth_ = depth;
//     pFrame->time_stamp_ = rgb_times[3];
//     vector<cv::KeyPoint> result=extractKeyPoints(color);
//     for(auto kp: result){
//         cout << cvRound(kp.pt.x) <<"  "<< cvRound(kp.pt.y)<< endl;
//     }
//     std::cout << pFrame->findDepth(result[0]) << std::endl;

// }

// TEST(FRAME,DESCRIPTOR){
//     cv::Mat color = cv::imread ( rgb_files[3] );
//     cv::Mat depth = cv::imread ( depth_files[3], -1 );
//     Camera::Ptr camera ( new myslam::Camera );
//     Frame::Ptr pFrame = Frame::createFrame();

//     pFrame->camera_ = camera;
//     pFrame->color_ = color;
//     pFrame->depth_ = depth;
//     pFrame->time_stamp_ = rgb_times[3];
//     vector<cv::KeyPoint> kps=extractKeyPoints(color);
//     auto descriptors=computeDescriptors(color,kps);
//     EXPECT_EQ(descriptors.rows,kps.size());

// }


// void featureMatching(vector<cv::KeyPoint> kps1,cv::Mat descpritors1,vector<cv::KeyPoint> kps2,cv::Mat descpritors2)
// {
//     cv::FlannBasedMatcher   matcher_flann_(new cv::flann::LshIndexParams ( 5,10,2 ));
//     vector<cv::DMatch> matches;
//     // select the candidates in map 
//     vector<cv::KeyPoint> candidate={};
//     for ( auto& allpoints: kps2 )
//     {
//             // add to candidate 
//             candidate.push_back( allpoints );
//     }
    
    
//     matcher_flann_.match ( descpritors1, descpritors2, matches );
//     // select the best matches
//     float min_dis = std::min_element (
//                         matches.begin(), matches.end(),
//                         [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
//     {
//         return m1.distance < m2.distance;
//     } )->distance;

//     vector<int> match_2dkp_index_={};
//     for ( cv::DMatch& m : matches )
//     {
//         if ( m.distance < max<float> ( min_dis*2.0, 30.0 ) )
//         {
//             match_2dkp_index_.push_back( m.trainIdx );
//         }
//     }
//     cout<<"good matches: "<<match_2dkp_index_.size() <<endl;
// }

// TEST(MAPPOINT,FEATUREMATCHING){
//     cv::Mat color = cv::imread ( rgb_files[3] );
//     cv::Mat depth = cv::imread ( depth_files[3], -1 );
//     Camera::Ptr camera ( new myslam::Camera );
//     Frame::Ptr pFrame = Frame::createFrame();

//     pFrame->camera_ = camera;
//     pFrame->color_ = color;
//     pFrame->depth_ = depth;
//     pFrame->time_stamp_ = rgb_times[3];
//     vector<cv::KeyPoint> kps1=extractKeyPoints(color);
//     auto descriptors1=computeDescriptors(color,kps1);

//     cv::Mat color1 = cv::imread ( rgb_files[5] );
//     cv::Mat depth1 = cv::imread ( depth_files[5], -1 );
//     Frame::Ptr pFrame1 = Frame::createFrame();

//     pFrame1->camera_ = camera;
//     pFrame1->color_ = color1;
//     pFrame1->depth_ = depth1;
//     pFrame1->time_stamp_ = rgb_times[3];
//     vector<cv::KeyPoint> kps2=extractKeyPoints(color1);
//     auto descriptors2=computeDescriptors(color1,kps2);
//     cout << kps1.size() << endl;
//     cout << kps2.size() << endl;

//     featureMatching(kps1,descriptors1,kps2,descriptors2);
// }


TEST(TEST,TEST){
    string filedir="/home/tomy807/vslam/config/default.yaml";
    Config::setParameterFile(filedir);
    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );

    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    cout<<"dataset: "<<dataset_dir<<endl;
    ifstream fin ( dataset_dir+"/associations.txt" );
    // if ( !fin )
    // {
    //     cout<<"please generate the associate file called associate.txt!"<<endl;
    //     return 1;
    // }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while ( !fin.eof() )
    {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin>>rgb_time>>rgb_file>>depth_time>>depth_file;
        rgb_times.push_back ( atof ( rgb_time.c_str() ) );
        depth_times.push_back ( atof ( depth_time.c_str() ) );
        rgb_files.push_back ( dataset_dir+"/"+rgb_file );
        depth_files.push_back ( dataset_dir+"/"+depth_file );

        if ( fin.good() == false )
            break;
    }

    myslam::Camera::Ptr camera ( new myslam::Camera );

    // visualization
    cv::viz::Viz3d vis ( "Visual Odometry" );
    cv::viz::WCoordinateSystem world_coor ( 1.0 ), camera_coor ( 0.5 );
    cv::Point3d cam_pos ( 0, -1.0, -1.0 ), cam_focal_point ( 0,0,0 ), cam_y_dir ( 0,1,0 );
    cv::Affine3d cam_pose = cv::viz::makeCameraPose ( cam_pos, cam_focal_point, cam_y_dir );
    vis.setViewerPose ( cam_pose );

    world_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 2.0 );
    camera_coor.setRenderingProperty ( cv::viz::LINE_WIDTH, 1.0 );
    vis.showWidget ( "World", world_coor );
    vis.showWidget ( "Camera", camera_coor );

    cout<<"read total "<<rgb_files.size() <<" entries"<<endl;
    for ( int i=0; i<rgb_files.size(); i++ )
    {
        cout<<"****** loop "<<i<<" ******"<<endl;
        cv::Mat color = cv::imread ( rgb_files[i] );
        cv::Mat depth = cv::imread ( depth_files[i], -1 );
        if ( color.data==nullptr || depth.data==nullptr )
            break;
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->color_ = color;
        pFrame->depth_ = depth;
        pFrame->time_stamp_ = rgb_times[i];

        boost::timer timer;
        vo->addFrame ( pFrame );
        cout<<"VO costs time: "<<timer.elapsed() <<endl;

        if ( vo->state_ == myslam::VisualOdometry::LOST )
            break;
        SE3 Twc = pFrame->T_c_w_.inverse();

        // show the map and the camera pose
        cv::Affine3d M (
            cv::Affine3d::Mat3 (
                Twc.rotationMatrix() ( 0,0 ), Twc.rotationMatrix() ( 0,1 ), Twc.rotationMatrix() ( 0,2 ),
                Twc.rotationMatrix() ( 1,0 ), Twc.rotationMatrix() ( 1,1 ), Twc.rotationMatrix() ( 1,2 ),
                Twc.rotationMatrix() ( 2,0 ), Twc.rotationMatrix() ( 2,1 ), Twc.rotationMatrix() ( 2,2 )
            ),
            cv::Affine3d::Vec3 (
                Twc.translation() ( 0,0 ), Twc.translation() ( 1,0 ), Twc.translation() ( 2,0 )
            )
        );

        cv::Mat img_show = color.clone();
        for ( auto& pt:vo->map_->map_points_ )
        {
            myslam::MapPoint::Ptr p = pt.second;
            Vector2d pixel = pFrame->camera_->world2pixel ( p->pos_, pFrame->T_c_w_ );
            cv::circle ( img_show, cv::Point2d ( pixel ( 0,0 ),pixel ( 1,0 ) ), 5, cv::Scalar ( 0,255,0 ), 2 );
        }

        cv::imshow ( "image", img_show );
        cv::waitKey ( 1 );
        vis.setWidgetPose ( "Camera", M );
        vis.spinOnce ( 1, false );
        cout<<endl;
    }

}
