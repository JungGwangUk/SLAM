#include <ros/ros.h>

#include <iostream>
#include <fstream>

#include <cstdio>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <tf/tf.h>

#include <sensor_msgs/Imu.h>

using namespace std;

FILE *imu_data;
FILE *quat_data;

bool is_first = false;

sensor_msgs::Imu prev_imu;

Eigen::Vector4d prev_quat(0.0, 0.0, 0.0, 1.0);

void Quat2Euler(double qx, double qy, double qz, double qw, Eigen::Vector3d& eul);

void ImuCB(const sensor_msgs::ImuConstPtr &input)
{
    if(!is_first)
    {
        is_first = true;
        prev_imu = *input;
        return;
    }

    double dt = (input->header.stamp-prev_imu.header.stamp).toSec();

    double p,q,r;
    p = prev_imu.angular_velocity.x*M_PI/180.0;
    q = prev_imu.angular_velocity.y*M_PI/180.0;
    r = prev_imu.angular_velocity.z*M_PI/180.0;

    Eigen::Vector4d est_quat(0.0, 0.0, 0.0, 1.0);

    Eigen::Matrix4d F;
    F << 0,   r,  -q,   p,
        -r,   0,   p,   q,
         q,  -p,   0,   r,
        -p,  -q,  -r,   0;

    F = Eigen::Matrix4d::Identity() + F*(dt/2.0);

    est_quat = F*prev_quat;

    tf::Quaternion quat;
    double roll, pitch, yaw;
    quat.setValue(est_quat[0], est_quat[1], est_quat[2], est_quat[3]);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    cout << est_quat.transpose() << endl;
//    Eigen::Quaterniond qq(est_quat[3],est_quat[0],est_quat[1],est_quat[2]);
//
//    auto euler = qq.toRotationMatrix().eulerAngles(0,1,2);
    Eigen::Vector3d euler;
    Quat2Euler(est_quat[0], est_quat[1], est_quat[2], est_quat[3], euler);

    fprintf(imu_data, "%lf %lf %lf %lf %lf %lf %lf\n",input->header.stamp.toSec(), roll, pitch, yaw, euler[0], euler[1], euler[2]);
    fprintf(quat_data, "%lf %lf %lf %lf\n", est_quat[3], est_quat[0], est_quat[1], est_quat[2]);

    prev_quat = est_quat;
    prev_imu = *input;
}

void Quat2Euler(double qx, double qy, double qz, double qw, Eigen::Vector3d& eul)
{
    double aSinInput = -2.0*(qx*qz-qw*qy);
    if(aSinInput > 1)
        aSinInput = 1.0;
    else if(aSinInput < -1)
        aSinInput = -1.0;

    eul[0] = atan2( 2.0*(qx*qy+qw*qz), qw*qw + qx*qx - qy*qw - qz*qz );
    eul[1] = asin( aSinInput );
    eul[2] = atan2( 2.0*(qy*qz+qw*qx), qw*qw - qx*qx - qy*qy + qz*qz );
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber sub_imu = nh.subscribe("ebimu/RvizData", 1000, ImuCB);

    imu_data = fopen("imu_data.txt", "w");
    quat_data = fopen("quat.txt", "w");

    ros::spin();

    fclose(imu_data);
    fclose(quat_data);
}