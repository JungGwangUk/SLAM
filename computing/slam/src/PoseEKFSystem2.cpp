#include <slam/PoseEKFSystem2.h>

double NormalizingAngle(double angle)
{
  if(angle > M_PI)
    angle -= 2*M_PI;
  else if(angle < -M_PI)
    angle += 2*M_PI;

  return angle;
}

void PoseEKFSystem2::EKF(Vector6d &X, const Vector5d u, const Vector6d z, const float dt)
{
  double b1, s1, b2, s2, b3, s3;
  if(z[3]>X[3]){
    b1=z[3];
    s1=X[3];
  }
  else{
    b1=X[3];
    s1=z[3];
  }
  if(z[4]>X[4]){
    b2=z[4];
    s2=X[4];
  }
  else{
    b2=X[4];
    s2=z[4];
  }
  if(z[5]>X[5]){
    b3=z[5];
    s3=X[5];
  }
  else{
    b3=X[5];
    s3=z[5];
  }


  double v, q, r;
  v = u[0];
  q = u[3];
  r = u[4];

  F_ = Matrix6d::Identity();
  F_(0,4) = -v*sin(X[4])*cos(X[5])*dt;
  F_(0,5) = -v*cos(X[4])*sin(X[5])*dt;
  F_(1,4) = -v*sin(X[4])*sin(X[5])*dt;
  F_(1,5) = v*cos(X[4])*cos(X[5])*dt;
  F_(2,4) = -v*cos(X[4])*dt;
  F_(5,3) = (q*cos(X_[0])/cos(X_[1]) - r*sin(X_[0])/cos(X_[1]))*dt;
  F_(5,4) = (q*sin(X_[0])/cos(X_[1])*tan(X_[1]) + r*cos(X_[0])/cos(X_[1])*tan(X_[1]))*dt;

  P_ = F_*P_*F_.transpose() + Q_;

  K_ = P_*H_.transpose()*(H_*P_*H_.transpose() + R_).inverse();

  Vector6d S = z - H_*X;

  S[3] = NormalizingAngle(S[3]);
  S[4] = NormalizingAngle(S[4]);
  S[5] = NormalizingAngle(S[5]);

  X_ = X + K_*S;

  if(X_[3] < s1 || X_[3] > b1  )
    printf("roll : %f, %f, %f\n", s1, b1, X_[3]);
  if(X_[4] < s2 || X_[4] > b2  )
    printf("pitch : %f, %f, %f\n", s2, b2, X_[4]);
  if(X_[5] < s3 || X_[5] > b3  )
    printf("yaw : %f, %f, %f\n", s3, b3, X_[5]);

  P_ = P_ - K_*H_*P_;

  X = X_;
}

void PoseEKFSystem2::PredictFromModel(Vector6d &X, Vector5d u, double dt)
{
  double v,roll_rate,pitch_rate, q, r;
  v = u[0];
  roll_rate = u[1];
  pitch_rate = u[2];
  q = u[3];
  r = u[4];

  Vector6d X_pred;

  double x_dot = v*cos(X[4])*cos(X[5]);
  double y_dot = v*cos(X[4])*sin(X[5]);
  double z_dot = -v*sin(X[4]);
  double yaw_rate = q*sin(X[3])/cos(X[4]) + r*cos(X[3])/cos(X[4]);

  X_pred[0] = X[0] + x_dot*dt;
  X_pred[1] = X[1] + y_dot*dt;
  X_pred[2] = X[2] + z_dot*dt;
  X_pred[3] = X[3] + roll_rate*dt;
  X_pred[4] = X[4] + pitch_rate*dt;
  X_pred[5] = X[5] + yaw_rate*dt;

  X = X_pred;
}

void PoseEKFSystem2::Init()
{
  X_.setZero();
  P_.setIdentity();
  H_.setIdentity();
  R_.setIdentity();
  Q_.setIdentity();

  P_ = P_*0.0001;
  Q_ = Q_*0.01;
  R_ = R_*10;
  Q_(3,3) = 0.0001;
  Q_(4,4) = 0.0001;
  Q_(5,5) = 0.0001;
  R_(3,3) = 0.1;
  R_(4,4) = 0.1;
  R_(5,5) = 0.1;
}