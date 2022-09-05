#ifndef MAP_HPP
#define MAP_HPP

#include "common_include.h"
#include "frame.h"
#include "mappoint.h"

namespace myslam{
    class Map{
        public:
            typedef std::shared_ptr<Map> Ptr;
            std::unordered_map<unsigned long, MapPoint::Ptr >  map_points_;        // all landmarks
            std::unordered_map<unsigned long, Frame::Ptr >     keyframes_;         // all key-frames

            Map() {}
            
            void insertKeyFrame( Frame::Ptr frame );
            void insertMapPoint( MapPoint::Ptr map_point );
    };
}

#endif // MAP_H