#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

class PoseEKFSystem2
{
public:
  typedef Eigen::Matrix<double,6,6> Matrix6d;
  typedef Eigen::Matrix<double,6,1> Vector6d;
  typedef Eigen::Matrix<double,5,1> Vector5d;

  Vector6d X_;

  Matrix6d Q_;
  Matrix6d R_;
  Matrix6d P_;

  Matrix6d F_;
  Matrix6d H_;
  Matrix6d K_;

  void Init();

  void EKF(Vector6d &X, const Vector5d u, const Vector6d z, const float dt);

  void PredictFromModel(Vector6d &X, Vector5d u, double dt);
};