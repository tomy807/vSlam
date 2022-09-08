#ifndef MYSLAM_G2O_TYPES_HPP
#define MYSLAM_G2O_TYPES_HPP

#include "common_include.h"
#include "camera.h"
#include "../3rdparty/g2o/g2o/core/base_binary_edge.h"
#include "../3rdparty/g2o/g2o/types/types_sba.h"
#include "../3rdparty/g2o/g2o/core/base_vertex.h"
#include "../3rdparty/g2o/g2o/core/base_unary_edge.h"
#include "../3rdparty/g2o/g2o/core/block_solver.h"
#include "../3rdparty/g2o/g2o/core/solver.h"
#include "../3rdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "../3rdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "../3rdparty/g2o/g2o/core/robust_kernel.h"
#include "../3rdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "../3rdparty/g2o/g2o/types/types_six_dof_expmap.h"


namespace myslam {
/// vertex and edges used in g2o ba
/// 位姿顶点
class EdgeProjectXYZRGBD : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void computeError();
    virtual void linearizeOplus();
    virtual bool read( std::istream& in ){}
    virtual bool write( std::ostream& out) const {}
    
};

// only to optimize the pose, no point
class EdgeProjectXYZRGBDPoseOnly: public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Error: measure = R*point+t
    virtual void computeError();
    virtual void linearizeOplus();
    
    virtual bool read( std::istream& in ){}
    virtual bool write( std::ostream& out) const {}
    
    Vector3d point_;
};

class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap >
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    virtual void computeError();
    virtual void linearizeOplus();
    
    virtual bool read( std::istream& in ){}
    virtual bool write(std::ostream& os) const {};
    
    Vector3d point_;
    Camera* camera_;
};

}  // namespace myslam

#endif // MYSLAM_G2O_TYPES_H
