#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>

#include <pharos_msgs/StateStamped2016.h>

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

ros::Publisher ekf_imu_pub_;

bool vehicle_init = false;
bool imu_init = false;
bool convergent = false;

double _v_dot;

pharos_msgs::StateStamped2016 _prev_vehicle;
pharos_msgs::StateStamped2016 _curr_vehicle;

sensor_msgs::Imu _prev_imu;

double NormalizeAngle(double angle);

Vector4d ToQuaternion(double yaw, double pitch, double roll);

void EKF(Eigen::Vector3d &X, const Eigen::Vector3d u, const Eigen::Vector2d z, const double dt);

void Init(Eigen::Vector3d X, Eigen::Matrix3d P, Eigen::Matrix2d R, Eigen::Matrix3d Q, Matrix2x3d H);

void PredictFromModel(Eigen::Vector3d &X, Eigen::Vector3d u, double dt);

