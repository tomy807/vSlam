#include "../include/mappoint.h"

using namespace myslam;

MapPoint::MapPoint()
: id_(-1), pos_(Vector3f(0,0,0)), norm_(Vector3f(0,0,0)), good_(true),
  visible_times_(0), matched_times_(0){}

MapPoint::MapPoint ( long unsigned int id, const Vector3f& position, const Vector3f& norm, Frame* frame, const cv::Mat& descriptor )
: id_(id), pos_(position), norm_(norm), good_(true),
  visible_times_(1), matched_times_(1), descriptor_(descriptor){
    observed_frames_.push_back(frame);
}

MapPoint::Ptr MapPoint::createMapPoint(){
    return MapPoint::Ptr( 
        new MapPoint( factory_id_++, Vector3f(0,0,0), Vector3f(0,0,0) )
    );
}

MapPoint::Ptr MapPoint::createMapPoint ( 
    const Vector3f& pos_world, 
    const Vector3f& norm, 
    const cv::Mat& descriptor, 
    Frame* frame ){
    return MapPoint::Ptr( 
        new MapPoint( factory_id_++, pos_world, norm, frame, descriptor )
    );
}

unsigned long MapPoint::factory_id_ = 0;
