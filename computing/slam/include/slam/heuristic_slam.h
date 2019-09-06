#pragma once

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

#include <slam/InteractiveMarkersAPI.h>
#include <slam/SLAM3DSystem.h>

#include <ndt_gpu/NormalDistributionsTransform.h>
#include <pcl/registration/icp.h>

#define GET_ROS_REQUIRE_PARAM(nh, param_name, var) \
        if(!nh.getParam(param_name, var)){\
            std::string str("Required parameter missing - "#param_name);\
            str += std::string(" in namespace ");\
            str += nh.getNamespace();\
            throw std::runtime_error(str.c_str());\
        }

#define SETTING_LC_CANDIDATE 0;
#define ADDING_LC_EDGE 1;

using namespace visualization_msgs;
using namespace std;

bool _set_input = false;
bool _set_target = false;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;
interactive_markers::MenuHandler _box_menu_handler;
interactive_markers::MenuHandler _gt_box_menu_handler;
interactive_markers::MenuHandler _vertex_menu_handler;

SLAM3DSystem slam3d;
IntMarkerAPI intMarker;
gpu::GNormalDistributionsTransform gpu_ndt;

ros::Publisher _graph_markers_publish;
ros::Publisher _input_scan_pub;
ros::Publisher _target_map_pub;
ros::Publisher _slam_map_pub;
ros::Publisher _check_scan_pub;

pcl::PointCloud<pcl::PointXYZI>::Ptr _input_scan(new pcl::PointCloud<pcl::PointXYZI>);
sensor_msgs::PointCloud2 _input_scan_msg;

pcl::PointCloud<pcl::PointXYZI>::Ptr _target_map(new pcl::PointCloud<pcl::PointXYZI>);
sensor_msgs::PointCloud2 _target_msg;

sensor_msgs::PointCloud2 _check_msg;

double _input_position[7] = {0,0,0,0,0,0,1};
double _final_position[7] = {0};
double _LC_measurement[7];

double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
Eigen::Matrix4f _tf_vtol, _tf_ltov;
string _data_path;

float _lookup_LCdist = 0.0;

int _input_id = 0;
int _target_id = 0;
int _check_candidate = 0;

std::vector<int> _LCids;

std::vector<SLAM3DSystem::ids> _LC_id_map;

void find_LCcandidate();
void LC_scan_visualize(const int id_i, const int id_j);
void estTopose(const double *est, geometry_msgs::Pose &pose);
void ndt_scanmatching(const int input_id, const int LC_id);
void add_LCedge(const int input_id, const int LC_id, double *d);

void boxFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );
void make_vertex_int_marker();
