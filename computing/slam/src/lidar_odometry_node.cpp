#include <slam/lidar_odometry_node.h>

void LPointsCB(const sensor_msgs::PointCloud2ConstPtr points)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2 points_msg;

  pcl::fromROSMsg(*points, *cloud_ptr);

  Eigen::Matrix4d T;

  if(!(points->header.frame_id == "imu"))
    T = rawData_.PoseToMatrix(curr_pose_)*Tml_;
  else
    T = rawData_.PoseToMatrix(curr_pose_);

  pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, T);

  *assemble_cloud_ptr_ += *transformed_cloud_ptr;

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

  if(!(points->header.frame_id == "imu"))
    T = rawData_.PoseToMatrix(curr_pose_)*Tmr_;
  else
    T = rawData_.PoseToMatrix(curr_pose_);

  pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, T);

  *assemble_cloud_ptr_ += *transformed_cloud_ptr;

  pcl::toROSMsg(*transformed_cloud_ptr, points_msg);

  points_msg.header.frame_id = "odom";

  R_points_pub_.publish(points_msg);
}

void TPointsCB(const sensor_msgs::PointCloud2ConstPtr points)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_assemble_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  sensor_msgs::PointCloud2 points_msg;
  sensor_msgs::PointCloud2 assemble_points_msg;

  pcl::fromROSMsg(*points, *cloud_ptr);

  Eigen::Matrix4d T;

  if(!(points->header.frame_id == "imu"))
    T = rawData_.PoseToMatrix(curr_pose_)*Tmt_;
  else
    T = rawData_.PoseToMatrix(curr_pose_);

  pcl::transformPointCloud(*cloud_ptr, *transformed_cloud_ptr, T);

  *assemble_cloud_ptr_ += *transformed_cloud_ptr;

  Eigen::Matrix4d T_inv = T.inverse();
  pcl::transformPointCloud(*assemble_cloud_ptr_, *transformed_assemble_cloud_ptr, T_inv);

  pcl::toROSMsg(*transformed_cloud_ptr, points_msg);
  pcl::toROSMsg(*transformed_assemble_cloud_ptr, assemble_points_msg);

  points_msg.header.frame_id = "odom";
  assemble_points_msg.header.frame_id = "imu";

  T_points_pub_.publish(points_msg);

  if(dist_>accumulation_dist_)
  {
    A_points_pub_.publish(assemble_points_msg);
    assemble_cloud_ptr_->clear();
    dist_ = 0.0;
  }

}

void ImuPoseCB(const velodyne_msgs::IMURPYposeConstPtr pose)
{
  curr_pose_ = *pose;
  double dx = curr_pose_.x - prev_pose_.x;
  double dy = curr_pose_.y - prev_pose_.y;
  double dz = curr_pose_.z - prev_pose_.z;

  dist_ += sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2));

  prev_pose_ = curr_pose_;
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
  A_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("transformed_APoints", 10);

  Init(pnh);

  ros::spin();
}