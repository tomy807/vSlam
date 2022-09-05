#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "common_include.h"
namespace myslam{
    // template <class Scalar_=float>
    class Camera{
        public:
            typedef std::shared_ptr<Camera> Ptr;
            float   fx_, fy_, cx_, cy_, depth_scale_;
            Camera();
            Camera(float fx, float fy, float cx, float cy, float depth_scale=0):
                fx_ ( fx ), fy_ ( fy ), cx_ ( cx ), cy_ ( cy ), depth_scale_ ( depth_scale ){}

            Vector3f world2camera( const Vector3f& p_w, const SE3& T_c_w );
            Vector3f camera2world( const Vector3f& p_c, const SE3& T_c_w );
            Vector2f camera2pixel( const Vector3f& p_c );
            Vector3f pixel2camera( const Vector2f& p_p, double depth=1 ); 
            Vector3f pixel2world ( const Vector2f& p_p, const SE3& T_c_w, double depth=1 );
            Vector2f world2pixel ( const Vector3f& p_w, const SE3& T_c_w );
    };
}

#endif 