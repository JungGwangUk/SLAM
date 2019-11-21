#include <preprocessing/imu_pose_node.h>

velodyne_msgs::IMURPYpose QuatToEuler(geometry_msgs::Quaternion q)
{
  velodyne_msgs::IMURPYpose rpy;
  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  rpy.roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1)
      rpy.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
      rpy.pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  rpy.yaw = std::atan2(siny_cosp, cosy_cosp);

  return rpy;
}

void ImuCB(const sensor_msgs::ImuConstPtr &msg)
{
  velodyne_msgs::IMURPYpose pose;
  nav_msgs::Odometry odom;

  double curr_t, dt;

  curr_t = msg->header.stamp.toSec();
  dt = curr_t-_prev_t;


  pose = QuatToEuler(msg->orientation);

  pose.x = _prev_pose.x + cos(_prev_pose.yaw)*cos(_prev_pose.pitch)*_prev_vel*dt;
  pose.y = _prev_pose.y + sin(_prev_pose.yaw)*cos(_prev_pose.pitch)*_prev_vel*dt;
  pose.z = _prev_pose.z + sin(_prev_pose.pitch)*_prev_vel*dt;

  pose.stamp = msg->header.stamp;

  odom.header.frame_id = "odom";
  odom.header.stamp = msg->header.stamp;
  odom.pose.pose.position.x = pose.x;
  odom.pose.pose.position.y = pose.y;
  odom.pose.pose.position.z = pose.z;
  odom.pose.pose.orientation = msg->orientation;

  _imu_pose_pub.publish(pose);

  _prev_pose = pose;

  _prev_t = curr_t;
  _prev_vel = _curr_vel;
}

void VehicleStateCB(const pharos_msgs::StateStamped2016ConstPtr &state)
{
  _curr_vel = state->state.velocity;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_pose_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Subscriber imu_sub = nh.subscribe("ekf_imu", 1000, ImuCB);
  ros::Subscriber vehicle_state_sub = nh.subscribe("vehicle/state2016", 1000, VehicleStateCB);

  _imu_pose_pub = nh.advertise<velodyne_msgs::IMURPYpose>("imu_pose", 1000);

  _imu_odom_pub = nh.advertise<nav_msgs::Odometry>("odom/imu", 1000);

  ros::spin();
}
