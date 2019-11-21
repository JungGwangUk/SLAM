//
// Created by pharos on 19. 4. 4.
//
#include <slam/hitlc_slam.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr final_map_ptr(new pcl::PointCloud<pcl::PointXYZI>);

SLAM3DSystem slam3d;
PoseEKFSystem pose_ekf;
gpu::GNormalDistributionsTransform gpu_ndt;

nav_msgs::Odometry origin;
nav_msgs::Odometry gt;

sensor_msgs::Imu imu;

hitlc_msgs::InputTargetArray set_array;
int hitlc_count = 0;


ros::Publisher gt_pub;
ros::Publisher imu_odom_pub;

bool init_gt = false;
bool first_novatel =true;
FILE *fp;
FILE *vehicle_data;
FILE *param_yaml;
FILE *vertex_time;
FILE *imu_data;

void LidarCB(const sensor_msgs::PointCloud2ConstPtr &input)
{
    if(_use_gps_init)
    {
        if(first_novatel)
        {
            ROS_WARN("Waiting for receiving GPS data");
            return;
        }
        else
        {
            if(!init_gt)
            {
                _previous_pose.x = origin.pose.pose.position.x;
                _previous_pose.y = origin.pose.pose.position.y;
                _previous_pose.z = origin.pose.pose.position.z;
                tf::Quaternion q;
                tf::quaternionMsgToTF(origin.pose.pose.orientation, q);
                tf::Matrix3x3(q).getRPY(_previous_pose.roll, _previous_pose.pitch, _previous_pose.yaw);
                _pred_pose << _previous_pose.x, _previous_pose.y, _previous_pose.z;
                pose_ekf.X_ = _pred_pose;
                _curr_rpy << _previous_pose.roll, _previous_pose.pitch, _previous_pose.yaw;
                init_gt = true;
            }
        }
    }
    if(!_use_imu)
    {
        ROS_WARN("Waiting for receiving IMU data");
        return;
    }

    static double num_target_map_idx =  _target_map_lengh/_min_add_scan_shift;
    static double r;

    pcl::PointXYZI p;
    pcl::PointCloud<pcl::PointXYZI> tmp, scan;
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());

    Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
    Eigen::Matrix4f t_lidar_base(Eigen::Matrix4f::Identity());

    static tf::TransformBroadcaster br;
    static tf::TransformBroadcaster br2;

    tf::Transform transform_m2v;
    tf::Transform transform_v2l;

    SLAM3DSystem::RPYpose current_pose, localizer_pose, guess_pose;

    ros::Time curr_scan_time;
    curr_scan_time = input->header.stamp;

    pcl::fromROSMsg(*input, tmp);

    //Set point scan range and filtering
    for (pcl::PointCloud<pcl::PointXYZI>::const_iterator item = tmp.begin(); item != tmp.end(); item++)
    {
        p.x = (double)item->x;
        p.y = (double)item->y;
        p.z = (double)item->z;
        p.intensity = (double)item->intensity;

        r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
        if (_min_scan_range < r && r < _max_scan_range)
        {
            scan.push_back(p);
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr before_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>(scan));
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    // Voxel Grid Filter
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(_voxel_leaf_size, _voxel_leaf_size, _voxel_leaf_size);
    voxel_grid_filter.setInputCloud(before_scan_ptr);
    voxel_grid_filter.filter(*scan_ptr);

    // Add initial point cloud to velodyne_map
    if (!_initial_scan_loaded)
    {
        Matrix4f init_pose;
        slam3d.RPYposeToMatrix(_previous_pose, init_pose);
        pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, init_pose*_tf_vtol);
        _target_points += *transformed_scan_ptr;
        _prev_states = _curr_states;
        _prev_scan_time = curr_scan_time;
        _initial_scan_loaded = true;

        return;
    }

    if(_is_optimized)
    {
        double p[7];
        _vertex_old_ptr->getEstimateData(p);

        slam3d.EstimateToRPYpose(p, _previous_pose);

        _pred_pose << _previous_pose.x, _previous_pose.y, _previous_pose.z;
        pose_ekf.X_= _pred_pose;

        // reset target and final map point cloud
        _target_points.clear();
        final_map_ptr->clear();

        _lookup_LCdist = 0.0;
        _map_pub_id += 50;

        // rebuild final map
        slam3d.buildCloudMap(*final_map_ptr);
        cout << "rebuild final map" << endl;

        sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
        pcl::toROSMsg(*final_map_ptr, *map_msg_ptr);
        map_msg_ptr->header.frame_id = "map";
        _ndt_LC_map_publish.publish(*map_msg_ptr);

        // rebuild target map
        pcl::PointCloud<pcl::PointXYZI> transformed_cloud;
        for(int i=slam3d.vertexMap.size()-num_target_map_idx; i<slam3d.vertexMap.size(); i++)
        {
            Matrix4d T = slam3d.vertexMap[i]->estimate().matrix()*_tf_vtol.matrix().cast<double>();
            pcl::transformPointCloud(slam3d.vertexMap[i]->scan_data, transformed_cloud, T);

            _target_points += transformed_cloud;
        }
        _is_optimized = false;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>(_target_points));

    gpu_ndt.setInputTarget(map_ptr);
    gpu_ndt.setTransformationEpsilon(_trans_eps);
    gpu_ndt.setStepSize(_step_size);
    gpu_ndt.setResolution(_ndt_res);
    gpu_ndt.setMaximumIterations(30);
    gpu_ndt.setInputSource(scan_ptr);

    if(_use_vehicle_statas)
    {
        if(_use_imu)
        {
            guess_pose.x = _pred_pose[0];
            guess_pose.y = _pred_pose[1];
            guess_pose.z = _pred_pose[2];
            guess_pose.roll = _curr_rpy[0];
            guess_pose.pitch = _curr_rpy[1];
            guess_pose.yaw = _curr_rpy[2];
        }
        else
        {
            double dt = (curr_scan_time-_prev_scan_time).toSec();
            guess_pose = _previous_pose;
            guess_pose.yaw = _previous_pose.yaw+_prev_states.state.velocity*tan(_prev_states.state.wheel_angle)*dt/2.0;
            guess_pose.x = _previous_pose.x+_prev_states.state.velocity*cos(_previous_pose.yaw)*dt;
            guess_pose.y = _previous_pose.y+_prev_states.state.velocity*sin(_previous_pose.yaw)*dt;
        }
    }
    else
        guess_pose = _previous_pose;

    Eigen::Matrix4f init_guess;
    slam3d.RPYposeToMatrix(guess_pose, init_guess);
    init_guess = init_guess*_tf_vtol;

    SLAM3DSystem::RPYpose test_pose;
    slam3d.MatrixToRPYpose(init_guess, test_pose);

    // NDT matching
    gpu_ndt.align(init_guess);
    cout << gpu_ndt.getFinalNumIteration() << "  " << gpu_ndt.getFitnessScore() << " : " << gpu_ndt.hasConverged() << endl;

    // Getting NDT result
    t_lidar_base = gpu_ndt.getFinalTransformation();

    t_localizer = t_lidar_base*_tf_ltov;

    // Update localizer_pose from NDT.
    slam3d.MatrixToRPYpose(t_localizer, localizer_pose);

    if(_use_ekf)
    {
        Vector3d z(localizer_pose.x, localizer_pose.y, localizer_pose.z);
        Vector3d u(_curr_states.state.velocity, _curr_rpy[1], _curr_rpy[2]);

        double dist = sqrt(pow(localizer_pose.x-_pred_pose[0],2)+pow(localizer_pose.y-_pred_pose[1],2)+pow(localizer_pose.z-_pred_pose[2],2));

        pose_ekf.EKF(_pred_pose, u, z, 0.01);

        current_pose.x = _pred_pose[0];
        current_pose.y = _pred_pose[1];
        current_pose.z = _pred_pose[2];

        current_pose.roll = _curr_rpy[0];
        current_pose.pitch = _curr_rpy[1];
        current_pose.yaw = localizer_pose.yaw;
    }
    else
        current_pose = localizer_pose;

    Matrix4f final_matrix;
    slam3d.RPYposeToMatrix(current_pose, final_matrix);
    pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, final_matrix*_tf_vtol);

    ////////////////////////////////////////////////////////////
    tf::Quaternion q_t;
    nav_msgs::Odometry predict_odom;
    predict_odom.header.frame_id = "map";
    predict_odom.header.stamp = input->header.stamp;
    predict_odom.pose.pose.position.x = guess_pose.x;
    predict_odom.pose.pose.position.y = guess_pose.y;
    predict_odom.pose.pose.position.z = guess_pose.z;
    q_t.setRPY(guess_pose.roll, guess_pose.pitch, guess_pose.yaw);
    predict_odom.pose.pose.orientation.x = q_t.x();
    predict_odom.pose.pose.orientation.y = q_t.y();
    predict_odom.pose.pose.orientation.z = q_t.z();
    predict_odom.pose.pose.orientation.w = q_t.w();

    _predict_odom_publish.publish(predict_odom);

    nav_msgs::Odometry ndt_odom;
    ndt_odom.header.frame_id = "map";
    ndt_odom.header.stamp = input->header.stamp;
    ndt_odom.pose.pose.position.x = localizer_pose.x;
    ndt_odom.pose.pose.position.y = localizer_pose.y;
    ndt_odom.pose.pose.position.z = localizer_pose.z;
    q_t.setRPY(localizer_pose.roll, localizer_pose.pitch, localizer_pose.yaw);
    ndt_odom.pose.pose.orientation.x = q_t.x();
    ndt_odom.pose.pose.orientation.y = q_t.y();
    ndt_odom.pose.pose.orientation.z = q_t.z();
    ndt_odom.pose.pose.orientation.w = q_t.w();

    _ndt_odom_publish.publish(ndt_odom);

    nav_msgs::Odometry ekf_odom;
    ekf_odom.header.frame_id = "map";
    ekf_odom.header.stamp = input->header.stamp;
    ekf_odom.pose.pose.position.x = current_pose.x;
    ekf_odom.pose.pose.position.y = current_pose.y;
    ekf_odom.pose.pose.position.z = current_pose.z;
    q_t.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    ekf_odom.pose.pose.orientation.x = q_t.x();
    ekf_odom.pose.pose.orientation.y = q_t.y();
    ekf_odom.pose.pose.orientation.z = q_t.z();
    ekf_odom.pose.pose.orientation.w = q_t.w();

    _ekf_odom_publish.publish(ekf_odom);
    //////////////////////////////////////////////////////////

    tf::Quaternion l_q;
    transform_v2l.setOrigin(tf::Vector3(_tf_x, _tf_y, _tf_z));
    l_q.setRPY(_tf_roll, _tf_pitch, _tf_yaw);
    transform_v2l.setRotation(l_q);

    tf::Quaternion q;
    transform_m2v.setOrigin(tf::Vector3(current_pose.x, current_pose.y, current_pose.z));
    q.setRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);
    transform_m2v.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform_m2v, curr_scan_time, "map", "vehicle"));
    br2.sendTransform(tf::StampedTransform(transform_v2l, curr_scan_time, "vehicle", input->header.frame_id));

    /////////////////////////////// write ground truth
    double gt_x, gt_y, gt_z, gt_roll, gt_pitch, gt_yaw;
    tf::Quaternion quat;
    quat.setValue(gt.pose.pose.orientation.x, gt.pose.pose.orientation.y, gt.pose.pose.orientation.z, gt.pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(gt_roll, gt_pitch, gt_yaw);

    gt_x = gt.pose.pose.position.x;
    gt_y = gt.pose.pose.position.y;
    gt_z = gt.pose.pose.position.z;

    fprintf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",curr_scan_time.toSec(), gt_x, gt_y, gt_z, gt_roll, gt_pitch, gt_yaw, current_pose.x, current_pose.y, current_pose.z, current_pose.roll, current_pose.pitch, current_pose.yaw, gt.pose.covariance[0]);
    //////////////////////////////

    double shift = sqrt(pow(current_pose.x - _added_pose.x, 2.0) + pow(current_pose.y - _added_pose.y, 2.0));
    if (shift >= _min_add_scan_shift)
    {
        if (1)
        {
            fprintf(vertex_time, "%lf %lf %lf %lf %lf %lf %lf %lf\n",curr_scan_time.toSec(), gt_x, gt_y, gt_z, gt_roll, gt_pitch, gt_yaw, gt.pose.covariance[0]);

            _lookup_LCdist += shift/2;
            if(_lookup_LCdist > _max_lookup_LCdist)
                _lookup_LCdist = _max_lookup_LCdist;

            _target_points += *transformed_scan_ptr;
            *final_map_ptr += *transformed_scan_ptr;

            if(slam3d.getID() >= num_target_map_idx)
            {
                pcl::PointCloud<pcl::PointXYZI>::iterator iter;
                iter = _target_points.begin();
                int erase_size = slam3d.vertexMap[slam3d.getID()-num_target_map_idx]->scan_data.size();
                _target_points.erase(iter, iter+erase_size);
            }

            _added_pose = current_pose;

            number_t curr_estimate[7] = {current_pose.x, current_pose.y, current_pose.z, q.x(), q.y(), q.z(), q.w()};

            SLAM(curr_estimate, scan_ptr);

            visualization_msgs::MarkerArray markerArray;
            slam3d.makeGraphMarkArray(markerArray, _LC_id_map);
            _graph_markers_publish.publish(markerArray);

            if(slam3d.getID() > _map_pub_id)
            {
                sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
                pcl::toROSMsg(*final_map_ptr, *map_msg_ptr);
                map_msg_ptr->header.frame_id = "map";
                _ndt_LC_map_publish.publish(*map_msg_ptr);
                _map_pub_id += 50;
            }
        }
    }

    _previous_pose = current_pose;
    _prev_states = _curr_states;
    _prev_scan_time = curr_scan_time;

    sensor_msgs::PointCloud2::Ptr target_msg_ptr(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(_target_points, *target_msg_ptr);
    target_msg_ptr->header.frame_id = "map";
    _target_map_publish.publish(*target_msg_ptr);

    _input_cloud_publish.publish(input);
}

void SLAM(number_t *estimate, pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud)
{
    VertexSE3WithData* vertex_new_ptr(new VertexSE3WithData);

    slam3d.createVertexSE3(estimate, *point_cloud, *vertex_new_ptr);

    if(_is_first_vertex)
    {
        vertex_new_ptr->setFixed(true);

        slam3d.addVertex(vertex_new_ptr);

        _is_first_vertex = false;
        _vertex_old_ptr = vertex_new_ptr;

        cout << "Set First Vertex!!" << endl;
        return;
    }

    slam3d.addVertex(vertex_new_ptr);
    EdgeSE3 *e(new EdgeSE3);
    slam3d.createEdgeSE3(_vertex_old_ptr, vertex_new_ptr, *e);
    slam3d.addEdge(e);

    vector<int> LCids;
    if(slam3d.searchLoopClosing(vertex_new_ptr, _lookup_LCdist, 5, LCids) && _hitlc_status == FindingCandidate)
    {
        cout << "find loop closing vertexes!! [" << LCids.size() << " vertex(s)]"<< endl;
        hitlc_msgs::InputTargetArray array;
        for(int i=0; i<LCids.size(); i++)
        {
            hitlc_msgs::InputTarget set;
            VertexsetToHitLCmsg(*vertex_new_ptr, *slam3d.vertexMap[LCids[i]], set);
            set_array.sets.push_back(set);
            hitlc_count++;
        }

        if(hitlc_count>1)
        {
            _hitlc_msg_pub.publish(set_array);
            hitlc_count = 0;
            set_array.sets.clear();

            _hitlc_status = FindingLCEdge;
        }
    }

    _vertex_old_ptr = vertex_new_ptr;
}

void VertexsetToHitLCmsg(VertexSE3WithData input, VertexSE3WithData target, hitlc_msgs::InputTarget &set)
{
    double input_est[7], target_est[7];
    sensor_msgs::PointCloud2 input_cloud, target_cloud;

    input.getEstimateData(input_est);
    target.getEstimateData(target_est);

    pcl::toROSMsg(input.scan_data, input_cloud);
    pcl::toROSMsg(target.scan_data, target_cloud);

    set.input_id = input.id();
    set.input_cloud = input_cloud;
    set.input_est[0] = input_est[0];
    set.input_est[1] = input_est[1];
    set.input_est[2] = input_est[2];
    set.input_est[3] = input_est[3];
    set.input_est[4] = input_est[4];
    set.input_est[5] = input_est[5];
    set.input_est[6] = input_est[6];

    set.target_id = target.id();
    set.target_cloud = target_cloud;
    set.target_est[0] = target_est[0];
    set.target_est[1] = target_est[1];
    set.target_est[2] = target_est[2];
    set.target_est[3] = target_est[3];
    set.target_est[4] = target_est[4];
    set.target_est[5] = target_est[5];
    set.target_est[6] = target_est[6];
}

void VehicleStatesCB(const pharos_msgs::StateStamped2016ConstPtr &states)
{
    static bool is_first = true;
    static pharos_msgs::StateStamped2016 prev_state = *states;

    _curr_states = *states;

    if(!is_first)
        _v_dot = (_curr_states.state.velocity - prev_state.state.velocity)/(_curr_states.header.stamp-prev_state.header.stamp).toSec();

    if(isnan(_v_dot))
    {
        _v_dot = 0.0;
        ROS_WARN("Calculation v_dot is non!!! %f", _v_dot);
    }

    prev_state = _curr_states;
    is_first = false;
}

void OdomCB(const nav_msgs::OdometryConstPtr &input)
{
    nav_msgs::Odometry odom;
    gt = *input;
    if(first_novatel)
    {
        origin = *input;
        first_novatel = false;
    }
    odom.pose = input->pose;
    odom.header.frame_id = "map";
    gt_pub.publish(odom);
}

void ImuCB(const sensor_msgs::ImuConstPtr &input)
{
    static bool is_first = true;
    static sensor_msgs::Imu prev_imu = *input;

    imu = *input;

    // save data
    fprintf(imu_data, "%lf %lf %lf %lf %lf %lf %lf\n",input->header.stamp.toSec(), imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z, imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
    fprintf(vehicle_data, "%lf %lf %lf %lf %lf %lf\n",_curr_states.header.stamp.toSec(), _curr_states.state.velocity,  _curr_states.state.lon_acc, _curr_states.state.lat_acc, _curr_states.state.yaw_rate, _curr_states.state.wheel_angle);
    // ------------

    // Quat to Euler
    double a;
    tf::Quaternion q;
    tf::quaternionMsgToTF(imu.orientation, q);
    tf::Matrix3x3(q).getRPY(_curr_rpy[0], _curr_rpy[1], a);

    if(is_first)
    {
        is_first = false;
        return;
    }

    double d_yaw, prev_roll, prev_pitch, prev_yaw, dt;
    dt = (imu.header.stamp - prev_imu.header.stamp).toSec();

    tf::quaternionMsgToTF(prev_imu.orientation, q);
    tf::Matrix3x3(q).getRPY(prev_roll, prev_pitch, prev_yaw);

    // calculate global yaw
    d_yaw = prev_imu.angular_velocity.y*sin(prev_roll)/cos(prev_pitch) + prev_imu.angular_velocity.z*cos(prev_roll)/cos(prev_pitch);
    _curr_rpy[2] += d_yaw*dt;

    // predict pose using imu data
    Vector3d u(_curr_states.state.velocity, _curr_rpy[1], _curr_rpy[2]);
    pose_ekf.PredictFromModel(_pred_pose, u, dt);

    prev_imu = imu;

    // predict imu pose
    static Vector3d imu_pose(_previous_pose.x, _previous_pose.y, _previous_pose.z);
    Vector3d u2(_curr_states.state.velocity, _curr_rpy[1], _curr_rpy[2]);
    pose_ekf.PredictFromModel(imu_pose, u2, dt);
    nav_msgs::Odometry imu_odom;
    imu_odom.header.stamp = input->header.stamp;
    imu_odom.header.frame_id = "map";
    imu_odom.pose.pose.position.x = imu_pose[0];
    imu_odom.pose.pose.position.y = imu_pose[1];
    imu_odom.pose.pose.position.z = imu_pose[2];

    imu_odom_pub.publish(imu_odom);

    if(!_init_imu)
        _init_imu = true;
}

void LCInfoCB(const hitlc_msgs::LCInfoConstPtr &msg)
{
    _hitlc_status = Optimizing;

    double meas[7];
    for(int i=0; i<7; i++)
        meas[i] = msg->meas[i];

    EdgeSE3 *edge(new EdgeSE3);
    slam3d.createEdgeSE3(slam3d.vertexMap[msg->input_id], slam3d.vertexMap[msg->target_id], meas, *edge);
    slam3d.addEdge(edge);

    slam3d.optimizeGraph(100, true);
    _is_optimized = true;

    SLAM3DSystem::ids ids;
    ids.i = msg->input_id;
    ids.j = msg->target_id;
    _LC_id_map.push_back(ids);
}

void initParam(ros::NodeHandle pnh)
{
    pnh.param<std::string>("points_topic_name", _points_topic_name, "vlp_t/velodyne_points");
    pnh.param<std::string>("vehicle_state_topic_name", _vehicle_state_topic_name, "/vehicle/state2016");
    pnh.param<std::string>("gps_state_topic_name", _gps_state_topic_name, "/odom");
    pnh.param<std::string>("imu_topic_name", _imu_topic_name, "/ebimu/RvizData");

    pnh.param<std::string>("save_path", _save_path, "");

    pnh.param<std::string>("g2o_solver", _g2o_solver, "gn_var");

    pnh.param("use_vehicle_statas", _use_vehicle_statas, true);
    pnh.param("use_imu", _use_imu, true);
    pnh.param("use_ekf", _use_ekf, true);
    pnh.param("use_gps_init", _use_gps_init, false);
    pnh.param("save_data", _save_data, true);

    pnh.param<float>("min_scan_range", _min_scan_range, 5.0);
    pnh.param<float>("max_scan_range", _max_scan_range, 200.0);

    pnh.param<int>("max_iter", _max_iter, 20);
    pnh.param<float>("ndt_res", _ndt_res, 2.0);
    pnh.param<double>("step_size", _step_size, 0.1);
    pnh.param<double>("trans_eps", _trans_eps, 0.01);

    pnh.param<float>("voxel_leaf_size", _voxel_leaf_size, 1.0);

    pnh.param<float>("min_add_scan_shift", _min_add_scan_shift, 1.0);
    pnh.param<float>("target_map_lengh", _target_map_lengh, 5.0);
    pnh.param<float>("max_lookup_LCdist", _max_lookup_LCdist, 30.0);
    pnh.param<float>("max_score_for_LC", _max_score_for_LC, 2.0);
    pnh.param<int>("min_edges_for_LC", _min_edges_for_LC, 5);

    pnh.param<double>("tf_x", _tf_x, 0.0);
    pnh.param<double>("tf_y", _tf_y, 0.0);
    pnh.param<double>("tf_z", _tf_z, 0.0);
    pnh.param<double>("tf_roll", _tf_roll, 0.0);
    pnh.param<double>("tf_pitch", _tf_pitch, 0.0);
    pnh.param<double>("tf_yaw", _tf_yaw, 0.0);

    _previous_pose.x = 0.0;
    _previous_pose.y = 0.0;
    _previous_pose.z = 0.0;
    _previous_pose.roll = 0.0;
    _previous_pose.pitch = 0.0;
    _previous_pose.yaw = 0.0;

    Translation3f tl_vtol(_tf_x, _tf_y, _tf_z);                 // tl: translation
    AngleAxisf rot_x_vtol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
    AngleAxisf rot_y_vtol(_tf_pitch, Eigen::Vector3f::UnitY());
    AngleAxisf rot_z_vtol(_tf_yaw, Eigen::Vector3f::UnitZ());
    _tf_vtol = (tl_vtol * rot_x_vtol * rot_y_vtol * rot_z_vtol).matrix(); // base to lidar
    _tf_ltov = _tf_vtol.inverse();

    slam3d.setTF(_tf_vtol);
    slam3d.setMinAddShift(_min_add_scan_shift);

    slam3d.save_path = _save_path;

    SaveParam();
}

void SaveParam()
{
    param_yaml = fopen((_save_path+"param.yaml").c_str(), "w");
    fprintf(param_yaml, "# Additional features \n");
    fprintf(param_yaml, "use_vehicle_statas: %s\n", _use_vehicle_statas ? "true" : "false");
    fprintf(param_yaml, "use_imu: %s\n", _use_imu ? "true" : "false");
    fprintf(param_yaml, "use_ekf: %s\n", _use_ekf ? "true" : "false");
    fprintf(param_yaml, "use_gps_init: %s\n", _use_gps_init ? "true" : "false");

    fprintf(param_yaml, "# ndt param\n");
    fprintf(param_yaml, "ndt_res: %f\n",_ndt_res);
    fprintf(param_yaml, "max_iter: %d\n",_max_iter);
    fprintf(param_yaml, "step_size: %f\n",_step_size);
    fprintf(param_yaml, "trans_eps: %f\n",_trans_eps);
    fprintf(param_yaml, "voxel_leaf_size: %f\n",_voxel_leaf_size);

    fprintf(param_yaml, "# add graph vertex param\n");
    fprintf(param_yaml, "min_add_scan_shift: %f\n",_min_add_scan_shift);

    fprintf(param_yaml, "# vehicle to lidar tf\n");
    fprintf(param_yaml, "tf_x: %f\n",_tf_x);
    fprintf(param_yaml, "tf_y: %f\n",_tf_y);
    fprintf(param_yaml, "tf_z: %f\n",_tf_z);
    fprintf(param_yaml, "tf_roll: %f\n",_tf_roll);
    fprintf(param_yaml, "tf_pitch: %f\n",_tf_pitch);
    fprintf(param_yaml, "tf_yaw: %f\n",_tf_yaw);

    fclose(param_yaml);

    cout << "Save parameters ("+ _save_path+"param.yaml)" << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hitlc_slam_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    initParam(pnh); // Initializing Parameters

    
    slam3d.initG2O(_g2o_solver); // Initializing G2O

    pose_ekf.Init();

    ros::Subscriber sub_points = nh.subscribe(_points_topic_name, 1000, LidarCB);
    ros::Subscriber sub_vehicle_states = nh.subscribe(_vehicle_state_topic_name, 1000, VehicleStatesCB);
    ros::Subscriber sub_odom = nh.subscribe(_gps_state_topic_name, 1000, OdomCB);
    ros::Subscriber sub_imu = nh.subscribe(_imu_topic_name, 1000, ImuCB);
    ros::Subscriber sub_lc_info = nh.subscribe("hitlc/lc_info", 1000, LCInfoCB);

    _input_cloud_publish = nh.advertise<sensor_msgs::PointCloud2>("/input_cloud", 1000);
    _target_map_publish = nh.advertise<sensor_msgs::PointCloud2>("/target_map", 1000);
    _ndt_LC_map_publish = nh.advertise<sensor_msgs::PointCloud2>("/ndt_LC_map", 1000);
    _LC_matched_cloud_publish = nh.advertise<sensor_msgs::PointCloud2>("/LC_matched_cloud", 1000);
    _LC_target_map_publish = nh.advertise<sensor_msgs::PointCloud2>("/LC_target_map", 1000);
    _LC_input_cloud_publish = nh.advertise<sensor_msgs::PointCloud2>("/LC_input_cloud", 1000);

    _graph_markers_publish = nh.advertise<visualization_msgs::MarkerArray>("/graph_markers", 1000);

    _ekf_odom_publish = nh.advertise<nav_msgs::Odometry>("/ekf_odom", 1000);
    _predict_odom_publish = nh.advertise<nav_msgs::Odometry>("/predict_odom", 1000);
    _ndt_odom_publish = nh.advertise<nav_msgs::Odometry>("/ndt_odom", 1000);
    _hitlc_msg_pub = nh.advertise<hitlc_msgs::InputTargetArray>("hitlc/hitlc_array", 1000);
    gt_pub = nh.advertise<nav_msgs::Odometry>("/ground_truth", 1000);
    imu_odom_pub = nh.advertise<nav_msgs::Odometry>("/imu_odom", 1000);

    fp = fopen((_save_path+"ground_truth.txt").c_str(), "w");
    vehicle_data = fopen((_save_path+"vehicle_data.txt").c_str(), "w");
    vertex_time = fopen((_save_path+"vertex_time.txt").c_str(), "w");
    imu_data = fopen((_save_path+"imu_data.txt").c_str(), "w");

    while(ros::ok())
    {
        ros::spinOnce();
    }

    if(_save_data)
        slam3d.saveData();

    fclose(fp);
    fclose(vehicle_data);
    fclose(imu_data);


    return 0;
}