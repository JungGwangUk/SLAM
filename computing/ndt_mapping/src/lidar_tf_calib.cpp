#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/rawdata.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <pcl/registration/ndt.h>
#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif

#include <tf/transform_broadcaster.h>


static gpu::GNormalDistributionsTransform gpu_ndt;
static int is_SetTarget = 2;
static ros::Publisher input_point_pub;
static ros::Publisher base_point_pub;

float min_scan_range = 0.0;
float max_scan_range = 200.0;

static int _max_iter;
static float _ndt_res;
static double _step_size, _trans_eps;
static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;

static std::string _points_topic_name;
static std::string _base_points_topic_name;

sensor_msgs::PointCloud2ConstPtr base_points_msg(new sensor_msgs::PointCloud2());

struct RPYpose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

std::vector<RPYpose> RPYposes;

void BasePointCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());

    base_points_msg = input;

    pcl::fromROSMsg(*input,*target_cloud);

    pcl::PointXYZI p;
    double r;
    target_cloud_filtered->clear();
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = target_cloud->begin(); item != target_cloud->end(); item++)
    {
        p.x = (double)item->x;
        p.y = (double)item->y;
        p.z = (double)item->z;
        p.intensity = 0;

        r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        if (min_scan_range < r && r < max_scan_range)
        {
            target_cloud_filtered->push_back(p);
        }
    }

    gpu_ndt.setInputTarget(target_cloud_filtered);

    if(is_SetTarget>=1)
        is_SetTarget = 0;

}

void PointCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
    static bool has_converged;
    static double fitness_score;
    static int final_num_iteration;
    Eigen::Matrix4f FinalTM(Eigen::Matrix4f::Identity());


    static double trans_eps = 0.01;
    static double step_size = 0.1;
    static float ndt_res = 3;
    static int max_iter = 30;

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_input_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    Eigen::AngleAxisf init_rotation_x(_tf_roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(_tf_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(_tf_yaw, Eigen::Vector3f::UnitZ());

    Eigen::Translation3f init_translation(_tf_x, _tf_y, _tf_z);

    Eigen::Matrix4f init_guess =
            (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();


    pcl::fromROSMsg(*input,*input_cloud);

    // Setting NDT parameters
    gpu_ndt.setTransformationEpsilon(trans_eps);
    gpu_ndt.setStepSize(step_size);
    gpu_ndt.setResolution(ndt_res);
    gpu_ndt.setMaximumIterations(max_iter);

    pcl::PointXYZI p;
    double r;

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = input_cloud->begin(); item != input_cloud->end(); item++)
    {
        p.x = (double)item->x;
        p.y = (double)item->y;
        p.z = (double)item->z;
        p.intensity = 255;

        r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        if (min_scan_range < r && r < max_scan_range)
        {
            input_filtered_cloud->push_back(p);
        }
    }

    gpu_ndt.setInputSource(input_filtered_cloud);

    if(is_SetTarget>=1)
    {
        ROS_WARN("SetTarget not yet!!!!");
        return;
    }

    gpu_ndt.align(init_guess);

    // Result parameters of NDT
    fitness_score = gpu_ndt.getFitnessScore();
    FinalTM = gpu_ndt.getFinalTransformation();
    has_converged = gpu_ndt.hasConverged();
    final_num_iteration = gpu_ndt.getFinalNumIteration();

    tf::Matrix3x3 mat_R;
    double roll, pitch, yaw;

    mat_R.setValue(static_cast<double>(FinalTM(0, 0)), static_cast<double>(FinalTM(0, 1)),
                   static_cast<double>(FinalTM(0, 2)), static_cast<double>(FinalTM(1, 0)),
                   static_cast<double>(FinalTM(1, 1)), static_cast<double>(FinalTM(1, 2)),
                   static_cast<double>(FinalTM(2, 0)), static_cast<double>(FinalTM(2, 1)),
                   static_cast<double>(FinalTM(2, 2)));

    mat_R.getRPY(roll, pitch, yaw);

    pcl::transformPointCloud(*input_filtered_cloud, *transformed_input_cloud, FinalTM);

    sensor_msgs::PointCloud2Ptr transformed_input_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*transformed_input_cloud,*transformed_input_msg);
    transformed_input_msg->header.frame_id = base_points_msg->header.frame_id;
    input_point_pub.publish(transformed_input_msg);

    is_SetTarget++;

    base_point_pub.publish(base_points_msg);

    std::cout << "R:-----------------------------------------------------------------" << std::endl;
    std::cout << "Fitness score: " << fitness_score << std::endl;
    std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
    std::cout << "(" << FinalTM(0, 3) << ", " << FinalTM(1, 3) << ", " << FinalTM(2, 3) << ", " << roll
              << ", " << pitch << ", " << yaw << ")" << std::endl;
    std::cout << "Transformation Matrix:" << std::endl;
    std::cout << FinalTM << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    RPYpose estimate;
    estimate.x = FinalTM(0, 3);
    estimate.y = FinalTM(1, 3);
    estimate.z = FinalTM(2, 3);
    estimate.roll = roll;
    estimate.pitch = pitch;
    estimate.yaw = yaw;

    RPYposes.push_back(estimate);
}

void FinalTF()
{
    double sum_x=0.0, sum_y=0.0, sum_z=0.0, sum_roll=0.0, sum_pitch=0.0, sum_yaw=0.0;
    double final_x, final_y, final_z, final_roll, final_pitch, final_yaw;

    for(int i = 0; i<RPYposes.size(); i++)
    {
        sum_x += RPYposes[i].x;
        sum_y += RPYposes[i].y;
        sum_z += RPYposes[i].z;
        sum_roll += RPYposes[i].roll;
        sum_pitch += RPYposes[i].pitch;
        sum_yaw += RPYposes[i].yaw;
    }

    final_x = sum_x/RPYposes.size();
    final_y = sum_y/RPYposes.size();
    final_z = sum_z/RPYposes.size();
    final_roll = sum_roll/RPYposes.size();
    final_pitch = sum_pitch/RPYposes.size();
    final_yaw = sum_yaw/RPYposes.size();

    std::cout << "FinalTF\n" << "x : " << final_x << std::endl;
    std::cout << "y : " << final_y << std::endl;
    std::cout << "z : " << final_z << std::endl;
    std::cout << "roll : " << final_roll << std::endl;
    std::cout << "pitch : " << final_pitch << std::endl;
    std::cout << "yaw : " << final_yaw << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"lidar_tf_calib");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("points_topic_name", _points_topic_name, "vlp_r/velodyne_points");
    pnh.param<std::string>("base_points_topic_name", _base_points_topic_name, "velodyne_points");

    pnh.param<double>("tf_x", _tf_x, 0.0);
    pnh.param<double>("tf_y", _tf_y, 0.0);
    pnh.param<double>("tf_z", _tf_z, 0.0);
    pnh.param<double>("tf_roll", _tf_roll, 0.0);
    pnh.param<double>("tf_pitch", _tf_pitch, 0.0);
    pnh.param<double>("tf_yaw", _tf_yaw, 0.0);

    pnh.param<int>("max_iter", _max_iter, 20);
    pnh.param<float>("ndt_res", _ndt_res, 2.0);
    pnh.param<double>("step_size", _step_size, 0.1);
    pnh.param<double>("trans_eps", _trans_eps, 0.01);

    ros::Subscriber PointSub = nh.subscribe(_points_topic_name, 1000, PointCB);
    ros::Subscriber BasePointSub = nh.subscribe(_base_points_topic_name, 1000, BasePointCB);

    base_point_pub = nh.advertise<sensor_msgs::PointCloud2>("base_points", 1000);
    input_point_pub = nh.advertise<sensor_msgs::PointCloud2>("input_points", 1000);

    ros::spin();

    FinalTF();
}