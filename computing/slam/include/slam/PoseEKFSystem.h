#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

class PoseEKFSystem
{
public:
    Eigen::Vector3d X_;

    Eigen::Matrix3d Q_;
    Eigen::Matrix3d R_;
    Eigen::Matrix3d P_;

    Eigen::Matrix3d F_;
    Eigen::Matrix3d H_;
    Eigen::Matrix3d K_;

    void Init();

    void EKF(Eigen::Vector3d &X, const Eigen::Vector3d u, const Eigen::Vector3d z, const float dt);

    void PredictFromModel(Eigen::Vector3d &X, Eigen::Vector3d u, double dt);
};