#include "../include/camera.h"
#include "../include/config.h"

using namespace myslam;

Camera::Camera(){
    fx_ = Config::get<float>("camera.fx");
    fy_ = Config::get<float>("camera.fy");
    cx_ = Config::get<float>("camera.cx");
    cy_ = Config::get<float>("camera.cy");
    depth_scale_ = Config::get<float>("camera.depth_scale");
}
Vector3f Camera::world2camera ( const Vector3f& p_w, const SE3& T_c_w ){
    return T_c_w*p_w;
}

Vector3f Camera::camera2world ( const Vector3f& p_c, const SE3& T_c_w ){
    return T_c_w.inverse() *p_c;
}

Vector2f Camera::camera2pixel ( const Vector3f& p_c ){
    return Vector2f (
               fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
               fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
           );
}

Vector3f Camera::pixel2camera ( const Vector2f& p_p, double depth ){
    return Vector3f (
               ( p_p ( 0,0 )-cx_ ) *depth/fx_,
               ( p_p ( 1,0 )-cy_ ) *depth/fy_,
               depth
           );
}

Vector2f Camera::world2pixel ( const Vector3f& p_w, const SE3& T_c_w ){
    return camera2pixel ( world2camera(p_w, T_c_w) );
}

Vector3f Camera::pixel2world ( const Vector2f& p_p, const SE3& T_c_w, double depth ){
    return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
}

