//
// Created by pharos on 19. 3. 27.
//
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>
#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif

struct pose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

static bool initial_scan_loaded = false;
static bool has_converged;

static pose previous_pose, guess_pose, current_pose,ndt_pose, localizer_pose, added_pose;

static gpu::GNormalDistributionsTransform gpu_ndt;

static ros::Time current_scan_time;

static int final_num_iteration, max_iter;

static float ndt_res;

static double min_scan_range;
static double max_scan_range;
static double voxel_leaf_size;
static double trans_eps, step_size;
static double fitness_score;
static double min_add_scan_shift;
static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;

static Eigen::Matrix4f tf_btol, tf_ltob;

static pcl::PointCloud<pcl::PointXYZI> map;

static ros::Publisher ndt_map_pub;
static ros::Publisher current_pose_pub;

static geometry_msgs::PoseStamped current_pose_msg;


void points_callback(const sensor_msgs::PointCloud2ConstPtr& input)
{
    double r;
    pcl::PointXYZI p;
    pcl::PointCloud<pcl::PointXYZI> tmp, scan;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    tf::Quaternion q;

    Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f t_lidar_base(Eigen::Matrix4f::Identity());
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    current_scan_time = input->header.stamp;

    pcl::fromROSMsg(*input, tmp);

    //Set point scan range and filtering
    min_scan_range = 5.0;
    max_scan_range = 200.0;
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
    {
        p.x = (double)item->x;
        p.y = (double)item->y;
        p.z = (double)item->z;
        p.intensity = (double)item->intensity;

        r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        if (min_scan_range < r && r < max_scan_range)
        {
            scan.push_back(p);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));

    // Add initial point cloud to velodyne_map
    if (!initial_scan_loaded)
    {
        pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, tf_btol);
        map += *transformed_scan_ptr;
        initial_scan_loaded=true;
    }

    // Apply voxelgrid filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(scan_ptr);
    voxel_grid_filter.filter(*filtered_scan_ptr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(map));

    gpu_ndt.setTransformationEpsilon(trans_eps);
    gpu_ndt.setStepSize(step_size);
    gpu_ndt.setResolution(ndt_res);
    gpu_ndt.setMaximumIterations(max_iter);
    gpu_ndt.setInputSource(filtered_scan_ptr);

    static bool is_first_map = true;
    if(is_first_map)
    {
        gpu_ndt.setInputTarget(map_ptr);
        is_first_map=false;
    }

    guess_pose.x = previous_pose.x;
    guess_pose.y = previous_pose.y;
    guess_pose.z = previous_pose.z;
    guess_pose.roll = previous_pose.roll;
    guess_pose.pitch = previous_pose.pitch;
    guess_pose.yaw = previous_pose.yaw;

    pose guess_pose_for_ndt;
    guess_pose_for_ndt = guess_pose;

    Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());

    Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);

    Eigen::Matrix4f init_guess =
            (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;

    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    gpu_ndt.align(init_guess);
    fitness_score = gpu_ndt.getFitnessScore();
    t_localizer = gpu_ndt.getFinalTransformation();
    has_converged = gpu_ndt.hasConverged();
    final_num_iteration = gpu_ndt.getFinalNumIteration();

    t_lidar_base= t_localizer * tf_ltob;

    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);

    tf::Matrix3x3 mat_l, mat_b;

    mat_l.setValue(static_cast<double>(t_localizer(0, 0)), static_cast<double>(t_localizer(0, 1)),
                   static_cast<double>(t_localizer(0, 2)), static_cast<double>(t_localizer(1, 0)),
                   static_cast<double>(t_localizer(1, 1)), static_cast<double>(t_localizer(1, 2)),
                   static_cast<double>(t_localizer(2, 0)), static_cast<double>(t_localizer(2, 1)),
                   static_cast<double>(t_localizer(2, 2)));

    mat_b.setValue(static_cast<double>(t_lidar_base(0, 0)), static_cast<double>(t_lidar_base(0, 1)),
                   static_cast<double>(t_lidar_base(0, 2)), static_cast<double>(t_lidar_base(1, 0)),
                   static_cast<double>(t_lidar_base(1, 1)), static_cast<double>(t_lidar_base(1, 2)),
                   static_cast<double>(t_lidar_base(2, 0)), static_cast<double>(t_lidar_base(2, 1)),
                   static_cast<double>(t_lidar_base(2, 2)));

    // Update localizer_pose.
    localizer_pose.x = t_localizer(0, 3);
    localizer_pose.y = t_localizer(1, 3);
    localizer_pose.z = t_localizer(2, 3);
    mat_l.getRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw, 1);

    // Update ndt_pose.
    ndt_pose.x = t_lidar_base(0, 3);
    ndt_pose.y = t_lidar_base(1, 3);
    ndt_pose.z = t_lidar_base(2, 3);
    mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

    current_pose.x = ndt_pose.x;
    current_pose.y = ndt_pose.y;
    current_pose.z = ndt_pose.z;
    current_pose.roll = ndt_pose.roll;
    current_pose.pitch = ndt_pose.pitch;
    current_pose.yaw = ndt_pose.yaw;

    transform.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
    q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, current_scan_time, "map", "lidar_base"));

    previous_pose.x = current_pose.x;
    previous_pose.y = current_pose.y;
    previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;
    previous_pose.pitch = current_pose.pitch;
    previous_pose.yaw = current_pose.yaw;

    double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
    if (shift >= min_add_scan_shift)
    {
        map += *transformed_scan_ptr;
        added_pose.x = current_pose.x;
        added_pose.y = current_pose.y;
        added_pose.z = current_pose.z;
        added_pose.roll = current_pose.roll;
        added_pose.pitch = current_pose.pitch;
        added_pose.yaw = current_pose.yaw;

        gpu_ndt.setInputTarget(map_ptr);
    }


    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_grid_filter.setInputCloud(map_ptr);
    voxel_grid_filter.filter(*filtered_map_ptr);

    sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*filtered_map_ptr, *map_msg_ptr);
    ndt_map_pub.publish(*map_msg_ptr);

    q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    current_pose_msg.header.frame_id = "map";
    current_pose_msg.header.stamp = current_scan_time;
    current_pose_msg.pose.position.x = current_pose.x;
    current_pose_msg.pose.position.y = current_pose.y;
    current_pose_msg.pose.position.z = current_pose.z;
    current_pose_msg.pose.orientation.x = q.x();
    current_pose_msg.pose.orientation.y = q.y();
    current_pose_msg.pose.orientation.z = q.z();
    current_pose_msg.pose.orientation.w = q.w();

    current_pose_pub.publish(current_pose_msg);

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Fitness score: " << fitness_score << std::endl;
    std::cout << "map: " << map.points.size() << " points." << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
    std::cout << "(" << current_pose.x << ", " << current_pose.y << ", " << current_pose.z << ", " << current_pose.roll
              << ", " << current_pose.pitch << ", " << current_pose.yaw << ")" << std::endl;
    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << t_localizer << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

}

int main(int argc, char** argv)
{
    //Setting parameters
    max_iter = 30;
    ndt_res = 3.0;
    step_size = 0.1;
    trans_eps = 0.01;

    voxel_leaf_size = 1.0;

    min_scan_range = 5.0;
    max_scan_range = 200.0;
    min_add_scan_shift = 2.0;

    guess_pose.x = 0.0;
    guess_pose.y = 0.0;
    guess_pose.z = 0.0;
    guess_pose.roll = 0.0;
    guess_pose.pitch = 0.0;
    guess_pose.yaw = 0.0;

    added_pose.x = 0.0;
    added_pose.y = 0.0;
    added_pose.z = 0.0;
    added_pose.roll = 0.0;
    added_pose.pitch = 0.0;
    added_pose.yaw = 0.0;

    _tf_x = 0.0;
    _tf_y = 0.0;
    _tf_z = 0.0;
    _tf_roll = 0.0;
    _tf_pitch = 0.0;
    _tf_yaw = 0.0;


    ros::init(argc, argv, "gw_ndt_mapping");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
    Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
    Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
    tf_btol = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
    tf_ltob = tf_btol.inverse();

    map.header.frame_id = "map";

    ndt_map_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
    current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/current_pose", 1000);

    ros::Subscriber points_sub = nh.subscribe("ns2/velodyne_points", 100000, points_callback);

    ros::spin();

    return 0;
}