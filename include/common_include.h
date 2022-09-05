#ifndef COMMON_INCLUDE_HPP
#define COMMON_INCLUDE_HPP
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <typeinfo>
#include <condition_variable>
#include <set>
#include <unordered_map>
#include <map>
#include <fmt/core.h>
// using namespace std;
// using Eigen::Vector2d;
// using Eigen::Vector3d;
using Eigen::Vector3f;
using Eigen::Vector2f;
typedef Sophus::SE3f SE3;
#endif