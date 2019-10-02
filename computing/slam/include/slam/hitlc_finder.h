#pragma once

#include <ros/ros.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>

#include <slam/InteractiveMarkersAPI.h>
#include <slam/SLAM3DSystem.h>

#include <ndt_gpu/NormalDistributionsTransform.h>

#include <hitlc_msgs/InputTargetArray.h>
#include <hitlc_msgs/InputTarget.h>

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> _server;

double _input_position[7] = {0,0,0,0,0,0,1};
double _target_position[7] = {0,0,0,0,0,0,1};

pcl::PointCloud<pcl::PointXYZI>::Ptr _input_map_pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr _target_map_pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>);;

sensor_msgs::PointCloud2 _input_map_msg;
sensor_msgs::PointCloud2 _target_map_msg;

ros::Publisher _input_map_pub;
ros::Publisher _target_map_pub;
ros::Publisher _graph_markers_pub;
ros::Publisher _LC_info_pub;

IntMarkerAPI _intMarker;
SLAM3DSystem _slam3d;
gpu::GNormalDistributionsTransform _gpu_ndt;

interactive_markers::MenuHandler _input_vertex_menu_handler;

hitlc_msgs::InputTargetArray _ITarray;

void ndt_scanmatching();
