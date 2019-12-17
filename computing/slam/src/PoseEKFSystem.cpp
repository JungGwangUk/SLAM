#include <slam/PoseEKFSystem.h>

void PoseEKFSystem::EKF(Eigen::Vector3d &X, const Eigen::Vector3d u, const Eigen::Vector3d z, const float dt)
{
//  PredictFromModel(X, u, dt);

    F_ << 0,   0,  0,
          0,   0,  0,
          0,   0,  0;
//    std::cout << F_ <<std::endl;

    F_ = Eigen::Matrix3d::Identity() + F_*dt;

    P_ = F_*P_*F_.transpose() + Q_;

    K_ = P_*H_.transpose()*(H_*P_*H_.transpose() + R_).inverse();

    Eigen::Vector3d S = z - H_*X;
//    std::cout << "-------------z------------" <<std::endl;
//    std::cout << z.transpose() <<std::endl;
//    std::cout << "-------------X_pred------------" <<std::endl;
//    std::cout << X_.transpose() <<std::endl;

    X_ = X + K_*S;


    P_ = P_ - K_*H_*P_;

    X = X_;
//    std::cout << "---------X-----------" << std::endl;
//    std::cout << X_.transpose() << std::endl;
//    std::cout << "---------F-----------" << std::endl;
//    std::cout << F_ << std::endl;
//    std::cout << "---------P-----------" << std::endl;
//    std::cout << P_ << std::endl;
//    std::cout << "---------K-----------" << std::endl;
//    std::cout << K_ << std::endl;

}

void PoseEKFSystem::PredictFromModel(Eigen::Vector3d &X, Eigen::Vector3d u, double dt)
{
    double v,theta,psi;
    v = u[0];
    theta = u[1];
    psi = u[2];


    Eigen::Vector3d X_pred;

    double x_dot = v*cos(theta)*cos(psi);
    double y_dot = v*cos(theta)*sin(psi);
    double z_dot = -v*sin(theta);

    X_pred[0] = X[0] + x_dot*dt;
    X_pred[1] = X[1] + y_dot*dt;
    X_pred[2] = X[2] + z_dot*dt;

    X = X_pred;
}

void PoseEKFSystem::Init()
{
    X_.setZero();
    P_.setIdentity();
    H_.setIdentity();
    R_.setIdentity();
    Q_.setIdentity();

    P_ = P_*0.0001;
    Q_ = Q_*0.001;
    R_ = R_*0.1;
//    Q_(2,2) = 0.1;
}