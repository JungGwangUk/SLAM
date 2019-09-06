#include <ros/ros.h>
#include <iostream>
#include <fstream>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <velodyne_pointcloud/point_types.h>


#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>

#include <slam/SLAM3DSystem.h>
#include <slam/EKFSystem.h>
#include <slam/PoseEKFSystem.h>

#include <pharos_msgs/StateStamped2016.h>

#ifdef CUDA_FOUND
#include <ndt_gpu/NormalDistributionsTransform.h>
#endif

using namespace std;
using namespace Eigen;
using namespace g2o;

static SLAM3DSystem::RPYpose _previous_pose, _current_pose, _added_pose;

static int _final_num_iteration, _max_iter;
static int _LC_index = 0;
static int _map_pub_id = 0;
static int _min_edges_for_LC;

static float _ndt_res;
static float _min_scan_range, _max_scan_range; //param
static float _max_lookup_LCdist, _target_map_lengh;
static float _lookup_LCdist = 0.0;
static float _voxel_leaf_size;
static float _min_add_scan_shift;
static float _max_score_for_LC;

static double _trans_eps, _step_size;
static double _fitness_score;
static double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
static double _v_dot;

static bool _initial_scan_loaded = false;
static bool _has_converged;
static bool _is_optimized = false;
static bool _is_first_vertex = true;
static bool _use_vehicle_statas = true;
static bool _use_imu = true;
static bool _use_ekf = false;
static bool _use_gps_init = false;
static bool _save_data = false;

static std::string _points_topic_name;
static std::string _vehicle_state_topic_name;
static std::string _gps_state_topic_name;
static std::string _save_path;
static std::string _g2o_solver;
static std::string _imu_topic_name;
static pharos_msgs::StateStamped2016 _curr_states;
static pharos_msgs::StateStamped2016 _prev_states;

//static sensor_msgs::Imu _prev_imu;

static ros::Time _prev_scan_time;

static ros::Publisher _input_cloud_publish;
static ros::Publisher _target_map_publish;
static ros::Publisher _ndt_LC_map_publish;

static ros::Publisher _LC_target_map_publish;
static ros::Publisher _LC_input_cloud_publish;
static ros::Publisher _LC_matched_cloud_publish;

static ros::Publisher _graph_markers_publish;

static ros::Publisher _ekf_odom_publish;
static ros::Publisher _predict_odom_publish;
static ros::Publisher _ndt_odom_publish;

static Eigen::Matrix4f _tf_vtol, _tf_ltov;

static Vector3d _curr_rpy(0.0, 0.0, 0.0);
static Vector3d _pred_pose(0.0, 0.0, 0.0);

VertexSE3WithData* _vertex_old_ptr(new VertexSE3WithData);

pcl::PointCloud<pcl::PointXYZI> _target_points;

vector<SLAM3DSystem::ids> _LC_id_map;


void SLAM(number_t *estimate, pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud);
void SaveParam();



