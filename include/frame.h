#ifndef FRAME_HPP
#define FRAME_HPP

#include "common_include.h"
#include "camera.h"

namespace myslam {

    class MapPoint;
    
    // template <class Scalar_=float>
    class Frame{
        public:
            typedef std::shared_ptr<Frame> Ptr;
            unsigned long                  id_;         // id of this frame
            double                         time_stamp_; // when it is recorded
            SE3                            T_c_w_;      // transform from world to camera
            Camera::Ptr                    camera_;     // Pinhole RGBD Camera model 
            cv::Mat                        color_, depth_; // color and depth image 
            std::vector<cv::KeyPoint>      keypoints_;  // key points in image
            std::vector<MapPoint*>         map_points_; // associated map points
            bool                           is_key_frame_;  // whether a key-frame
            
        public: 
            Frame();
            Frame( long id, double time_stamp=0, SE3 T_c_w=SE3(), Camera::Ptr camera=nullptr, cv::Mat color=cv::Mat(),cv::Mat depth=cv::Mat() );
            ~Frame();
            
            static Frame::Ptr createFrame(); 
            
            double findDepth( const cv::KeyPoint& kp );
            
            Vector3f getCamCenter() const;
            
            void setPose( const SE3& T_c_w );
            
            bool isInFrame( const Vector3f& pt_world );
    };

}

#endif // FRAME_H