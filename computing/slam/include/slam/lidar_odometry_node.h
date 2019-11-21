#include <ros/ros.h>

#include <velodyne_msgs/IMURPYpose.h>
#include <velodyne_pointcloud/rawdata.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

ros::Publisher L_points_pub_;
ros::Publisher R_points_pub_;
ros::Publisher T_points_pub_;

velodyne_msgs::IMURPYpose curr_pose_;

velodyne_rawdata::RawData rawData_;

Eigen::Matrix4d Tml_;       ///> Transform matrix imu to left lidar
Eigen::Matrix4d Tmr_;       ///> Transform matrix imu to right lidar
Eigen::Matrix4d Tmt_;       ///> Transform matrix imu to top lidar

void Init(ros::NodeHandle pnh);

void ImuPoseCB(const velodyne_msgs::IMURPYposeConstPtr pose);

void LPointsCB(const sensor_msgs::PointCloud2ConstPtr points);
void RPointsCB(const sensor_msgs::PointCloud2ConstPtr points);
void TPointsCB(const sensor_msgs::PointCloud2ConstPtr points);



