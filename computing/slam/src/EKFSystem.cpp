#include <slam/EKFSystem.h>

double NormalizeAngle(double angle)
{
    if(angle > M_PI)
        angle -= 2*M_PI;
    else if(angle < -M_PI)
        angle += 2*M_PI;

    return angle;
}

void EKFSystem::EKF(Eigen::Vector3d &X, const Eigen::Vector3d u, const Eigen::Vector2d z, const float dt)
{
    double p,q,r;
    p = u[0];
    q = u[1];
    r = u[2];

    double f11 = q*cos(X_[0])*tan(X_[1]) - r*sin(X_[0])*tan(X_[1]);
    double f12 = q*sin(X_[0])/pow(cos(X_[1]),2) + r*cos(X_[0])/pow(cos(X_[1]),2);
    double f13 = 0;
    double f21 = -q*sin(X_[0]) - r*cos(X_[0]);
    double f22 = 0;
    double f23 = 0;
    double f31 = q*cos(X_[0])/cos(X_[1]) - r*sin(X_[0])/cos(X_[1]);
    double f32 = q*sin(X_[0])/cos(X_[1])*tan(X_[1]) + r*cos(X_[0])/cos(X_[1])*tan(X_[1]);
    double f33 = 0;

    F_ << f11,   f12,  f13,
          f21,   f22,  f23,
          f31,   f32,  f33;
//    std::cout << F_ <<std::endl;

    F_ = Eigen::Matrix3d::Identity() + F_*dt;

    PredictFromModel(X_, u, dt);

    P_ = F_*P_*F_.transpose() + Q_;

    K_ = P_*H_.transpose()*(H_*P_*H_.transpose() + R_).inverse();

    Eigen::Vector2d S = z - H_*X_;
//    std::cout << "-------------z------------" <<std::endl;
//    std::cout << z.transpose() <<std::endl;
//    std::cout << "-------------X_pred------------" <<std::endl;
//    std::cout << X_.transpose() <<std::endl;
    S[0] = NormalizeAngle(S[0]);
    S[1] = NormalizeAngle(S[1]);

    X_ = X_ + K_*S;


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

void EKFSystem::Init(Eigen::Vector3d X, Eigen::Matrix3d P, Eigen::Matrix2d R, Eigen::Matrix3d Q, Matrix2x3d H)
{
    X_ = X;
    P_ = P;
    R_ = R;
    Q_ = Q;
    H_ = H;
}

void EKFSystem::Init()
{
    X_.setZero();
    P_.setIdentity();
    H_.setIdentity();
    R_.setIdentity();
    Q_.setIdentity();

    P_ = P_*5;
    Q_ = Q_*0.0001;
    R_ = R_*5;
    Q_(2,2) = 0.1;
}

void EKFSystem::PredictFromModel(Eigen::Vector4d &X, Vector4d u, float dt)
{
    double p,q,r;
    p = u[1];
    q = u[2];
    r = u[3];
//    r = 0.0;

    Eigen::Vector4d X_pred;
    Eigen::Matrix4d F;
    F << 0,   r,  -q,   p,
        -r,   0,   p,   q,
         q,  -p,   0,   r,
        -p,  -q,  -r,   0;
    F = Eigen::Matrix4d::Identity() + F*(dt/2.0);

    X_pred = F*X;

    X = X_pred;
}

void EKFSystem::PredictFromModel(Eigen::Vector3d &X, Eigen::Vector3d u, float dt)
{
    double p,q,r;
    p = u[0];
    q = u[1];
    r = u[2];

    Eigen::Vector3d X_pred;

    double roll_rate = p + q*sin(X[0])*tan(X[1]) + r*cos(X[0])*tan(X[1]);
    double pitch_rate = q*cos(X[0]) - r*sin(X[0]);
    double yaw_rate = q*sin(X[0])/cos(X[1]) + r*cos(X[0])/cos(X[1]);

    X_pred[0] = X[0] + roll_rate*dt;
    X_pred[1] = X[1] + pitch_rate*dt;
    X_pred[2] = X[2] + yaw_rate*dt;

    X = X_pred;
}