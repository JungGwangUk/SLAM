#include <imu_ekf/imu_ekf_node.h>

double NormalizeAngle(double angle)
{
    if(angle > M_PI)
        angle -= 2*M_PI;
    else if(angle < -M_PI)
        angle += 2*M_PI;

    return angle;
}

Vector4d ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Vector4d q; // w, x, y, z
    q[0] = cy * cp * cr + sy * sp * sr;
    q[1] = cy * cp * sr - sy * sp * cr;
    q[2] = sy * cp * sr + cy * sp * cr;
    q[3] = sy * cp * cr - cy * sp * sr;

    return q;
}

void EKF(Eigen::Vector3d &X, const Eigen::Vector3d u, const Eigen::Vector2d z, const double dt)
{
    static double prev_p1, prev_p2;

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

    F_ = Eigen::Matrix3d::Identity() + F_*dt;

    PredictFromModel(X_, u, dt);

    P_ = F_*P_*F_.transpose() + Q_;

    K_ = P_*H_.transpose()*(H_*P_*H_.transpose() + R_).inverse();

    Eigen::Vector2d S = z - H_*X_;

    S[0] = NormalizeAngle(S[0]);
    S[1] = NormalizeAngle(S[1]);

    X_ = X_ + K_*S;


    P_ = P_ - K_*H_*P_;

    if(fabs(P_(0,0) - prev_p1) < 0.0001 && fabs(P_(1,1) - prev_p2) < 0.0001 && !convergent)
    {
        convergent = true;
        ROS_INFO("EKF was convergent!!");
    }

    X = X_;

    prev_p1 = P_(0,0);
    prev_p2 = P_(1,1);
}

void PredictFromModel(Eigen::Vector3d &X, Eigen::Vector3d u, double dt)
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

void ImuCB(const sensor_msgs::ImuConstPtr &msg)
{
    if(!vehicle_init)
        return;

    if(!imu_init)
    {
        _prev_imu = *msg;
        imu_init = true;
        return;
    }

    static double g = -9.81;
    double dt = (msg->header.stamp-_prev_imu.header.stamp).toSec();
    if(dt == 0)
    {
        dt = 0.01;
        ROS_WARN("imu data dt is ZERO!! set to dt = 0.01");
    }

    Eigen::Vector3d u(_prev_imu.angular_velocity.x, _prev_imu.angular_velocity.y, _prev_imu.angular_velocity.z);
    Eigen::Vector2d z;

    z[1] = asin((msg->linear_acceleration.x - _v_dot)/g); // calculate pitch
    if(std::isnan(z[1]))
    {
        z[1] = 0.0;
        ROS_WARN("calculated pitch was NAN!!");
    }
    z[0] = asin((-msg->linear_acceleration.y + _curr_vehicle.state.velocity*msg->angular_velocity.z)/(g*cos(z[1]))); // calculate roll

    EKF(X_, u, z, dt);

    Vector4d quat = ToQuaternion(X_[2], X_[1], X_[0]);

    sensor_msgs::Imu ekf_imu;
    ekf_imu = *msg;
    ekf_imu.orientation.x = quat[1];
    ekf_imu.orientation.y = quat[2];
    ekf_imu.orientation.z = quat[3];
    ekf_imu.orientation.w = quat[0];

    if(convergent)
        ekf_imu_pub_.publish(ekf_imu);

    _prev_imu = *msg;
}

void VehicleCB(const pharos_msgs::StateStamped2016ConstPtr &msg)
{
    if(!vehicle_init){
        _prev_vehicle = *msg;
        vehicle_init = true;
        return;
    }

    double dt, curr_v, prev_v;

    _curr_vehicle = *msg;
    dt = (_curr_vehicle.header.stamp - _prev_vehicle.header.stamp).toSec();
    if(dt == 0)
    {
        dt = 0.01;
        ROS_WARN("vehicle data dt is ZERO!! set to dt = 0.01");
    }
    curr_v = _curr_vehicle.state.velocity;
    prev_v = _prev_vehicle.state.velocity;

    _v_dot = (curr_v - prev_v)/dt;

    _prev_vehicle = *msg;
}

void Init()
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_ekf_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber imu_sub = nh.subscribe("/ebimu/RvizData", 1000, ImuCB);
    ros::Subscriber vehicle_sub = nh.subscribe("/vehicle/state2016", 1000, VehicleCB);

    ekf_imu_pub_ = nh.advertise<sensor_msgs::Imu>("ekf_imu", 100);

    Init(); // Initializing EKF parameters

    ROS_INFO("imu EKF is start!!");

    ros::spin();
}