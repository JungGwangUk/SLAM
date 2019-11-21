#include <slam/lidar_odometry_node.h>

void LPointsCB(const sensor_msgs::PointCloud2ConstPtr points)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2 points_msg;

  pcl::fromROSMsg(*points, *cloud_ptr);

  Eigen::Matrix4d T;
  T = rawData_.PoseToMatrix(curr_pose_)*Tml_;

  pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, T);

  pcl::toROSMsg(*transformed_cloud_ptr, points_msg);

  points_msg.header.frame_id = "odom";

  L_points_pub_.publish(points_msg);
}

void RPointsCB(const sensor_msgs::PointCloud2ConstPtr points)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2 points_msg;

  pcl::fromROSMsg(*points, *cloud_ptr);

  Eigen::Matrix4d T;
  T = rawData_.PoseToMatrix(curr_pose_)*Tmr_;

  pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, T);

  pcl::toROSMsg(*transformed_cloud_ptr, points_msg);

  points_msg.header.frame_id = "odom";

  R_points_pub_.publish(points_msg);
}

void TPointsCB(const sensor_msgs::PointCloud2ConstPtr points)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2 points_msg;

  pcl::fromROSMsg(*points, *cloud_ptr);

  Eigen::Matrix4d T;
  T = rawData_.PoseToMatrix(curr_pose_)*Tmt_;

  pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, T);

  pcl::toROSMsg(*transformed_cloud_ptr, points_msg);

  points_msg.header.frame_id = "odom";

  T_points_pub_.publish(points_msg);
}

void ImuPoseCB(const velodyne_msgs::IMURPYposeConstPtr pose)
{
  curr_pose_ = *pose;
}

void Init(ros::NodeHandle pnh)
{
  std::vector<double> l_tf_vector, r_tf_vector, t_tf_vector;
  Eigen::Matrix4d Tml, Tmr, Tmt;

  if(!pnh.getParam("left/tf", l_tf_vector))
  {
    ROS_ERROR_STREAM("No left lidar tf information.  Using zero values!");
    l_tf_vector.resize(6,0.0);
  }
  Tml_ = rawData_.PoseToMatrix(l_tf_vector);

  if(!pnh.getParam("right/tf", r_tf_vector))
  {
    ROS_ERROR_STREAM("No left lidar tf information.  Using zero values!");
    r_tf_vector.resize(6,0.0);
  }
  Tmr_ = rawData_.PoseToMatrix(r_tf_vector);

  if(!pnh.getParam("top/tf", t_tf_vector))
  {
    ROS_ERROR_STREAM("No left lidar tf information.  Using zero values!");
    t_tf_vector.resize(6,0.0);
  }
  Tmt_ = rawData_.PoseToMatrix(t_tf_vector);


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_odometry_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Subscriber imu_pose_sub = nh.subscribe("imu_pose", 1000, ImuPoseCB);

  ros::Subscriber left_velodyne_sub = nh.subscribe("vlp_l/velodyne_points", 1000, LPointsCB);
  ros::Subscriber right_velodyne_sub = nh.subscribe("vlp_r/velodyne_points", 1000, RPointsCB);
  ros::Subscriber top_velodyne_sub = nh.subscribe("vlp_t/velodyne_points", 1000, TPointsCB);

  L_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("transformed_LPoints", 10);
  R_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("transformed_RPoints", 10);
  T_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("transformed_TPoints", 10);

  Init(pnh);

  ros::spin();
}