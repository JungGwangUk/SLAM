#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <velodyne_pointcloud/point_types.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

bool input_point = false;

static pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);

static Eigen::Matrix4d _tf_btoi;

static double _btoi_x, _btoi_y, _btoi_z, _btoi_roll, _btoi_pitch, _btoi_yaw;

static std::string _base_points_topic_name, _input_points_topic_name;

static ros::Publisher points_pub;

void BPointCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
    if(input_point)
    {
        input_point = false;
    }
    else
        return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr base_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_input_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*input, *base_cloud);

    pcl::transformPointCloud(*input_cloud, *transformed_input_ptr, _tf_btoi);

    *base_cloud += *transformed_input_ptr;

    sensor_msgs::PointCloud2 base_msg;
    pcl::toROSMsg(*base_cloud, base_msg);
    points_pub.publish(base_msg);
}

void IPointCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
    input_point = true;
    pcl::fromROSMsg(*input, *input_cloud);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "combine_2lidar_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<double>("btoi_x", _btoi_x, 0.0);
    pnh.param<double>("btoi_y", _btoi_y, 0.0);
    pnh.param<double>("btoi_z", _btoi_z, 0.0);
    pnh.param<double>("btoi_roll", _btoi_roll, 0.0);
    pnh.param<double>("btoi_pitch", _btoi_pitch, 0.0);
    pnh.param<double>("btoi_yaw", _btoi_yaw, 0.0);

    pnh.param<std::string>("base_points_topic_name", _base_points_topic_name, "ns2/velodyne_points");
    pnh.param<std::string>("input_points_topic_name", _input_points_topic_name, "ns1/velodyne_points");


    ros::Subscriber base_points_sub = nh.subscribe(_base_points_topic_name, 1000, BPointCB);
    ros::Subscriber input_points_sub = nh.subscribe(_input_points_topic_name, 1000, IPointCB);

    points_pub = nh.advertise<sensor_msgs::PointCloud2>("velodyne_points", 1000);

    Eigen::Translation3d tl_btoi(_btoi_x, _btoi_y, _btoi_z);
    Eigen::AngleAxisd rot_x_btoi(_btoi_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rot_y_btoi(_btoi_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd rot_z_btoi(_btoi_yaw, Eigen::Vector3d::UnitZ());
    _tf_btoi = (tl_btoi * rot_z_btoi * rot_y_btoi * rot_x_btoi).matrix(); // base to input

    ros::spin();

    return 0;
}