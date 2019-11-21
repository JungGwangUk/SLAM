#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pharos_msgs/StateStamped2016.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <velodyne_msgs/IMURPYpose.h>

#include <nav_msgs/Odometry.h>

using namespace std;

struct RPY
{
    double roll;
    double pitch;
    double yaw;
};

struct RPYpose
{
    struct
    {
        double x;
        double y;
        double z;
    }Pose;
    RPY Rotation;

};

bool init_lidar = false;
bool init_imu = false;

double _curr_vel;
double _prev_vel;

double _prev_t;

ros::Publisher _imu_pose_pub;
ros::Publisher _imu_odom_pub;

velodyne_msgs::IMURPYpose _prev_pose;

vector<RPYpose> _pose_array;

