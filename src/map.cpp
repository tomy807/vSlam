#include "../include/map.h"

using namespace myslam;

void Map::insertKeyFrame ( Frame::Ptr frame ){

    // std::cout<<"Key frame size = "<<keyframes_.size()<<std::endl;
    if ( keyframes_.find(frame->id_) == keyframes_.end() ){

        keyframes_.insert( make_pair(frame->id_, frame) );
    }
    else{
        keyframes_[ frame->id_ ] = frame;
    }
}

void Map::insertMapPoint ( MapPoint::Ptr map_point ){

    // std::cout<<"Map Points size = "<<map_points_.size()<<std::endl;
    if ( map_points_.find(map_point->id_) == map_points_.end() ){

        map_points_.insert( make_pair(map_point->id_, map_point) );
    }
    else{
        map_points_[map_point->id_] = map_point;
    }
}