#include <gtest/gtest.h>
#include "../include/config.hpp"
#include <iostream>

TEST(CONFIG,CONFIGTEST){
    string filedir="/home/tomy807/vslam/config/default.yaml";
    myslam::Config::setParameterFile ( filedir);
    string dataset_dir = myslam::Config::get<string> ( "dataset_dir" );
    EXPECT_TRUE(dataset_dir=="/home/tomy807/rgbd_dataset_freiburg1_xyz");
}