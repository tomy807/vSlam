#include <gtest/gtest.h>
#include "../include/config.hpp"
#include <fstream>
#include <iostream>

using namespace myslam;

TEST(CONFIG,CONFIGTEST){
    string filedir="/home/tomy807/vslam/config/default.yaml";
    Config::setParameterFile ( filedir);
    string dataset_dir = Config::get<string> ( "dataset_dir" );
    EXPECT_TRUE(dataset_dir=="/home/tomy807/rgbd_dataset_freiburg1_xyz");
    ifstream fin ( dataset_dir+"/associations.txt" );
    if ( !fin )
    {
        cout<<"please generate the associate file called associate.txt!"<<endl;
    }
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

}