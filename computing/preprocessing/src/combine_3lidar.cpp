#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <velodyne_pointcloud/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

static pcl::PointCloud<pcl::PointXYZI>::Ptr left_cloud(new pcl::PointCloud<pcl::PointXYZI>);
static pcl::PointCloud<pcl::PointXYZI>::Ptr right_cloud(new pcl::PointCloud<pcl::PointXYZI>);
static pcl::PointCloud<pcl::PointXYZI>::Ptr top_cloud(new pcl::PointCloud<pcl::PointXYZI>);

static Eigen::Matrix4d _tf_ttol;
static Eigen::Matrix4d _tf_ttor;

static double _ttol_x, _ttol_y, _ttol_z, _ttol_roll, _ttol_pitch, _ttol_yaw;
static double _ttor_x, _ttor_y, _ttor_z, _ttor_roll, _ttor_pitch, _ttor_yaw;

static std::string _top_points_topic_name, _left_points_topic_name, _right_points_topic_name;

static ros::Publisher points_pub;

static bool top_time, left_time, right_time;
void TPointCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::fromROSMsg(*input, *top_cloud);

    if(!left_time || !right_time)
        return;

    left_time = false;
    right_time = false;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_left_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_right_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::transformPointCloud(*left_cloud, *transformed_left_ptr, _tf_ttol);
    pcl::transformPointCloud(*right_cloud, *transformed_right_ptr, _tf_ttor);

    *top_cloud += *transformed_left_ptr;
    *top_cloud += *transformed_right_ptr;

    sensor_msgs::PointCloud2 top_msg;
    pcl::toROSMsg(*top_cloud, top_msg);
    points_pub.publish(top_msg);
}

void LPointCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
    left_time = true;

    pcl::fromROSMsg(*input, *left_cloud);

}

void RPointCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
    right_time = true;

    pcl::fromROSMsg(*input, *right_cloud);

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "combine_lidar_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<double>("ttor_x", _ttor_x, 0.0);
    pnh.param<double>("ttor_y", _ttor_y, 0.0);
    pnh.param<double>("ttor_z", _ttor_z, 0.0);
    pnh.param<double>("ttor_roll", _ttor_roll, 0.0);
    pnh.param<double>("ttor_pitch", _ttor_pitch, 0.0);
    pnh.param<double>("ttor_yaw", _ttor_yaw, 0.0);

    pnh.param<double>("ttol_x", _ttol_x, 0.0);
    pnh.param<double>("ttol_y", _ttol_y, 0.0);
    pnh.param<double>("ttol_z", _ttol_z, 0.0);
    pnh.param<double>("ttol_roll", _ttol_roll, 0.0);
    pnh.param<double>("ttol_pitch", _ttol_pitch, 0.0);
    pnh.param<double>("ttol_yaw", _ttol_yaw, 0.0);

    pnh.param<std::string>("top_points_topic_name", _top_points_topic_name, "vlp_t/velodyne_points");
    pnh.param<std::string>("left_points_topic_name", _left_points_topic_name, "vlp_l/velodyne_points");
    pnh.param<std::string>("right_points_topic_name", _right_points_topic_name, "vlp_r/velodyne_points");

    ros::Subscriber top_points_sub = nh.subscribe(_top_points_topic_name, 1000, TPointCB);
    ros::Subscriber left_points_sub = nh.subscribe(_left_points_topic_name, 1000, LPointCB);
    ros::Subscriber right_points_sub = nh.subscribe(_right_points_topic_name, 1000, RPointCB);

    points_pub = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1000);

    Eigen::Translation3d tl_ttol(_ttol_x, _ttol_y, _ttol_z);
    Eigen::AngleAxisd rot_x_ttol(_ttol_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y_ttol(_ttol_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z_ttol(_ttol_yaw, Eigen::Vector3d::UnitZ());
    _tf_ttol = (tl_ttol * rot_z_ttol * rot_y_ttol * rot_x_ttol).matrix(); // top to left

    Eigen::Translation3d tl_ttor(_ttor_x, _ttor_y, _ttor_z);
    Eigen::AngleAxisd rot_x_ttor(_ttor_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y_ttor(_ttor_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z_ttor(_ttor_yaw, Eigen::Vector3d::UnitZ());
    _tf_ttor = (tl_ttor * rot_z_ttor * rot_y_ttor * rot_x_ttor).matrix(); // top to right

    ros::spin();

    return 0;
}