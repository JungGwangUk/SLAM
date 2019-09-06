#include <ros/ros.h>


#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

class EKFSystem
{
public:
    typedef Eigen::Matrix<double,4,1> Vector4d;

    typedef Eigen::Matrix<double,3,2> Matrix3x2d;
    typedef Eigen::Matrix<double,2,3> Matrix2x3d;


    Eigen::Vector3d X_;

    Eigen::Matrix3d Q_;
    Eigen::Matrix2d R_;
    Eigen::Matrix3d P_;

    Eigen::Matrix3d F_;
    Matrix2x3d H_;
    Matrix3x2d K_;

    void Init(Eigen::Vector3d X, Eigen::Matrix3d P, Eigen::Matrix2d R, Eigen::Matrix3d Q, Matrix2x3d H);
    void Init();

    void EKF(Eigen::Vector3d &X, const Eigen::Vector3d u, const Eigen::Vector2d z, const float dt);

    void PredictFromModel(Eigen::Vector4d &X, Vector4d u, float dt); // Quaternion
    void PredictFromModel(Eigen::Vector3d &X, Eigen::Vector3d u, float dt); // Euler

};