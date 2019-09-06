#include <ros/ros.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>

using namespace std;

FILE *output;

Eigen::Vector3f getRPYfromRM(Eigen::Matrix3f rotation_matrix){
    Eigen::Vector3f rpy;
    rpy[0] = atan2f(rotation_matrix(2,1),rotation_matrix(2,2));
    rpy[1] = atan2f(-rotation_matrix(2,0),sqrt(pow(rotation_matrix(2,1),2)+pow(rotation_matrix(2,2),2)));
    rpy[2] = atan2f(rotation_matrix(1,0),rotation_matrix(0,0));

    return rpy;
}

void OdomCB(const nav_msgs::OdometryConstPtr &input)
{
    static tf::TransformBroadcaster br;

    //Novatel Odmo tf
    tf::Transform novatel_transform;
    novatel_transform.setOrigin( tf::Vector3(input->pose.pose.position.x, input->pose.pose.position.y, input->pose.pose.position.z) );

    std::string novatel_odom_frame_id = input->header.frame_id;
    std::string novatel_odom_child_frame_id = input->child_frame_id;

    tf::Quaternion novatel_quat;
    novatel_quat.setX(input->pose.pose.orientation.x);
    novatel_quat.setY(input->pose.pose.orientation.y);
    novatel_quat.setZ(input->pose.pose.orientation.z);
    novatel_quat.setW(input->pose.pose.orientation.w);

    novatel_transform.setRotation(novatel_quat);
    br.sendTransform(tf::StampedTransform(novatel_transform, input->header.stamp, novatel_odom_frame_id ,novatel_odom_child_frame_id));

    Eigen::AngleAxisf a_rotation_x (0.0,Eigen::Vector3f::UnitX ());
    Eigen::AngleAxisf a_rotation_y (0.0,Eigen::Vector3f::UnitY ());
    Eigen::AngleAxisf a_rotation_z (M_PI/2,Eigen::Vector3f::UnitZ ());
    Eigen::Matrix3f a_rm;
    a_rm = a_rotation_z*a_rotation_y*a_rotation_x;
    Eigen::Vector3f rear_axle_rpy = getRPYfromRM(a_rm);
    tf::Transform rear_axle_Transform;
    rear_axle_Transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
    tf::Quaternion rear_axle_qt;
    rear_axle_qt.setRPY(rear_axle_rpy[0],rear_axle_rpy[1],rear_axle_rpy[2]);
    rear_axle_Transform.setRotation(rear_axle_qt);
    br.sendTransform(tf::StampedTransform(rear_axle_Transform, input->header.stamp, novatel_odom_child_frame_id, "vehicle"));


    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(novatel_quat);
    mat.getEulerYPR(yaw, pitch, roll);

    cout << roll << " " << pitch << " " << yaw << endl;
    Eigen::AngleAxisf rotation_x (roll,Eigen::Vector3f::UnitX ());
    Eigen::AngleAxisf rotation_y (pitch,Eigen::Vector3f::UnitY ());
    Eigen::AngleAxisf rotation_z (yaw,Eigen::Vector3f::UnitZ ());
    Eigen::Matrix3f rm;
    rm = rotation_z*rotation_y*rotation_x;

    Eigen::Matrix3f matrix = rm*a_rm;
    Eigen::Vector3f rpy = getRPYfromRM(matrix);

    fprintf(output, "%lf %lf %lf %lf\n",input->header.stamp.toSec(), rpy[0], rpy[1], rpy[2]);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test2_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber odom_sub = nh.subscribe("odom/novatel", 1000, OdomCB);

    output = fopen("output.txt", "w");

    ros::spin();

    fclose(output);

}