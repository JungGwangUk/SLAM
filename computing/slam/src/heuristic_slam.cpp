#include <slam/heuristic_slam.h>

sensor_msgs::PointCloud2 gt_msg;
sensor_msgs::PointCloud2 empty_msg;

pcl::PointCloud<pcl::PointXYZ>::Ptr gt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr vertex_cloud(new pcl::PointCloud<pcl::PointXYZ>);

ros::Publisher gt_pub;
double gt_position[7] = {0,0,0,0,0,0,1};
void gtFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback );

// %Tag(frameCallback)%
void frameCallback(const ros::TimerEvent&)
{
    static uint32_t counter = 0;

    static tf::TransformBroadcaster br;

    tf::Transform t;

    ros::Time time = ros::Time::now();

    tf::TransformBroadcaster br_i;
    t.setOrigin(tf::Vector3(_input_position[0], _input_position[1], _input_position[2]));
    t.setRotation(tf::Quaternion(_input_position[3], _input_position[4], _input_position[5], _input_position[6]));

    br_i.sendTransform(tf::StampedTransform(t, time, "map", "lidar"));

    tf::TransformBroadcaster br_g;
    t.setOrigin(tf::Vector3(gt_position[0], gt_position[1], gt_position[2]));
    t.setRotation(tf::Quaternion(gt_position[3], gt_position[4], gt_position[5], gt_position[6]));

    br_g.sendTransform(tf::StampedTransform(t, time, "map", "odom"));

    counter++;

    _input_scan_pub.publish(_input_scan_msg);
    _target_map_pub.publish(_target_msg);
    _check_scan_pub.publish(_check_msg);

    gt_pub.publish(gt_msg);
}
// %EndTag(frameCallback)%

// %Tag(processFeedback)%
void vertexFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' ";

    std::string name = feedback->marker_name;
    name.erase(0,7);
    int vertex_id = stoi(name);

    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            if(feedback->menu_entry_id == 1)
            {
                ROS_INFO_STREAM( s.str() << ": menu item 'set Input vertex'" );

                pcl::PointCloud<pcl::PointXYZI> local_map;
                slam3d.buildLocalCloudMap(vertex_id, 5, 5, local_map);
                pcl::transformPointCloud(local_map, *_input_scan, slam3d.vertexMap[vertex_id]->estimate().inverse().matrix());

                pcl::toROSMsg(*_input_scan, _input_scan_msg);
                _input_scan_msg.header.frame_id = "lidar";

                slam3d.vertexMap[vertex_id]->getEstimateData(_input_position);

                _input_id = vertex_id;
                _check_msg = empty_msg;
                _set_input = true;
            }
            else if(feedback->menu_entry_id == 2)
            {
                ROS_INFO_STREAM( s.str() << ": menu item 'set Target vertex'" );
                slam3d.buildLocalCloudMap(vertex_id, 5, 5, *_target_map);
                pcl::toROSMsg(*_target_map, _target_msg);
                _target_msg.header.frame_id = "map";

                _target_id = vertex_id;
                _check_msg = empty_msg;
                _set_target = true;
            }
            break;

        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        {
            ROS_INFO_STREAM( s.str() << ": clicked" );
            pcl::PointCloud<pcl::PointXYZI> cloud;
            slam3d.buildLocalCloudMap(vertex_id, 3, 3, cloud);
            pcl::toROSMsg(cloud, _check_msg);
            _check_msg.header.frame_id = "map";
        }
            break;
    }

    if(_set_input && _set_target)
    {
        _server->clear();
        InteractiveMarker int_marker;
        intMarker.make6DofMarker("current_vertex" ,true, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, _input_position, true, int_marker);

        _server->insert(int_marker);
        _server->setCallback(int_marker.name, &boxFeedback);
        _box_menu_handler.apply( *_server, int_marker.name );
    }

    _server->applyChanges();
}
void boxFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' ";

    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id );
            if(feedback->menu_entry_id == 1)
            {
                ndt_scanmatching(_input_id, _target_id);
            }
            else if(feedback->menu_entry_id == 2)
            {
                add_LCedge(_input_id, _target_id, _LC_measurement);

                visualization_msgs::MarkerArray markerArray;
                slam3d.makeGraphMarkArray(markerArray, _LC_id_map);
                _graph_markers_publish.publish(markerArray);

                pcl::PointCloud<pcl::PointXYZI>::Ptr slam_map(new pcl::PointCloud<pcl::PointXYZI>);
                slam3d.buildCloudMap(*slam_map);

                sensor_msgs::PointCloud2 slam_map_msg;
                pcl::toROSMsg(*slam_map, slam_map_msg);
                slam_map_msg.header.frame_id = "map";
                _slam_map_pub.publish(slam_map_msg);

                _server->clear();
                make_vertex_int_marker();
                _set_target = false;
                _set_input = false;
                _input_scan->clear();
                _target_map->clear();
                _input_scan_msg = empty_msg;
                _target_msg = empty_msg;

//                _LCids.clear();
//                _lookup_LCdist = 0.0;
//                find_LCcandidate();
            }
            else if(feedback->menu_entry_id == 3)
            {
                add_LCedge(_input_id, _target_id, _LC_measurement);

                slam3d.optimizeGraph(100, true);

                visualization_msgs::MarkerArray markerArray;
                slam3d.makeGraphMarkArray(markerArray, _LC_id_map);
                _graph_markers_publish.publish(markerArray);

                pcl::PointCloud<pcl::PointXYZI>::Ptr slam_map(new pcl::PointCloud<pcl::PointXYZI>);
                slam3d.buildCloudMap(*slam_map);

                sensor_msgs::PointCloud2 slam_map_msg;
                pcl::toROSMsg(*slam_map, slam_map_msg);
                slam_map_msg.header.frame_id = "map";
                _slam_map_pub.publish(slam_map_msg);

                _server->clear();

                make_vertex_int_marker();

                _set_target = false;
                _set_input = false;
                _input_scan->clear();
                _target_map->clear();
                _input_scan_msg = empty_msg;
                _target_msg = empty_msg;

                //////////////////////////////////////////////////////////////////
                InteractiveMarker int_marker;
                intMarker.make6DofMarker("ground_truth" ,true, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, gt_position, true, int_marker);

                _server->insert(int_marker);
                _server->setCallback(int_marker.name, &gtFeedback);
                _gt_box_menu_handler.apply( *_server, int_marker.name );
                ////////////////////////////////////////////////////////////////
            }

            break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            intMarker.poseToest(feedback->pose, _input_position);
            ROS_INFO_STREAM( s.str() << ": pose {" << _input_position[0] << ", " << _input_position[1] << "}");
            _input_scan_pub.publish(_input_scan_msg);
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            _input_scan_pub.publish(_input_scan_msg);
            break;

        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        {
            ROS_INFO_STREAM( s.str() << ": clicked" );
        }
            break;
    }

    _server->applyChanges();
}

void gtFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
    switch ( feedback->event_type )
    {
        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            intMarker.poseToest(feedback->pose, gt_position);
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
            if(feedback->menu_entry_id == 1)
            {
                pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

                std::cout << gt_cloud->size() << "  :  " << vertex_cloud->size() << endl;
                icp.setInputSource(gt_cloud);
                icp.setInputTarget(vertex_cloud);
                pcl::PointCloud<pcl::PointXYZ> Final;
                icp.align(Final);
                std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                          icp.getFitnessScore() << std::endl;
                std::cout << icp.getFinalTransformation() << std::endl;

                Eigen::Matrix4f final_trans;
                final_trans = icp.getFinalTransformation();
                slam3d.MatrixToMeasurement(final_trans, gt_position);

                _server->erase("ground_truth");

                InteractiveMarker int_marker;
                intMarker.make6DofMarker("ground_truth" ,true, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, gt_position, true, int_marker);

                _server->insert(int_marker);
                _server->setCallback(int_marker.name, &gtFeedback);
                _gt_box_menu_handler.apply( *_server, int_marker.name );

//                Eigen::Matrix4f init_guess;
//                slam3d.EstimateToMatrix(gt_position, init_guess);
//
//                cout << init_guess << endl;
//                init_guess = init_guess.inverse().eval();
//
//                gpu_ndt.setInputTarget(gt_cloud);
//                gpu_ndt.setTransformationEpsilon(0.01);
//                gpu_ndt.setStepSize(0.1);
//                gpu_ndt.setResolution(4);
//                gpu_ndt.setMaximumIterations(200);
//                gpu_ndt.setInputSource(vertex_cloud);
//
//                gpu_ndt.align(init_guess);
//
//                Eigen::Matrix4f final_trans;
//                final_trans = gpu_ndt.getFinalTransformation();
//                final_trans = final_trans.inverse().eval();
//                slam3d.MatrixToMeasurement(final_trans, gt_position);
            }
            else if(feedback->menu_entry_id == 2)
            {
                Eigen::Matrix4f matrix;
                slam3d.EstimateToMatrix(gt_position, matrix);
                cout << matrix << endl;
            }


                break;
    }
    _server->applyChanges();
}
// %EndTag(processFeedback)%
void make_vertex_int_marker()
{
    vertex_cloud->clear();
    for(int i=0; i<slam3d.vertexMap.size(); i++)
    {
        double est[7];
        string name = "vertex_"+to_string(i);
        slam3d.vertexMap[i]->getEstimateData(est);
        tf::Vector3 position = {est[0], est[1], est[2]};
        InteractiveMarker int_marker;
        intMarker.makeButtonBoxMarker(name , position, int_marker);

        pcl::PointXYZ p;
        p.x = est[0];
        p.y = est[1];
        p.z = est[2];
        vertex_cloud->push_back(p);
        _server->insert(int_marker);
        _server->setCallback(int_marker.name, &vertexFeedback);
        _vertex_menu_handler.apply( *_server, int_marker.name );
    }
}

void ndt_scanmatching(const int input_id, const int target_id)
{
    std::cout << "start matching" << std::endl;
    auto Ti = slam3d.vertexMap[target_id]->estimate();
    auto Tj = slam3d.vertexMap[input_id]->estimate();

    Eigen::Matrix4f tf_vtol = slam3d.getTF();
    Eigen::Matrix4f tf_ltov = tf_vtol.inverse();

    gpu_ndt.setInputTarget(_target_map);
    gpu_ndt.setTransformationEpsilon(0.01);
    gpu_ndt.setStepSize(0.1);
    gpu_ndt.setResolution(2);
    gpu_ndt.setMaximumIterations(200);
    gpu_ndt.setInputSource(_input_scan);

    Eigen::Matrix4f init_guess;
    slam3d.EstimateToMatrix(_input_position, init_guess);
//    init_guess = init_guess*tf_vtol;

    std::cout << tf_vtol << std::endl;

    init_guess(2, 3) = Ti.translation()[2];

    gpu_ndt.align(init_guess);

    std::cout << "finish matching(score : "<< gpu_ndt.getFitnessScore() <<" )" << std::endl;

    Eigen::Matrix4f final_trans;
    final_trans = gpu_ndt.getFinalTransformation();
    slam3d.MatrixToMeasurement(final_trans, _input_position);
    slam3d.MatrixToMeasurement(final_trans, _final_position);

    Eigen::Matrix4f Tij = Ti.inverse().matrix().cast<float>()*(gpu_ndt.getFinalTransformation());
    slam3d.MatrixToMeasurement(Tij, _LC_measurement);

    InteractiveMarker int_marker;
    intMarker.make6DofMarker("current_vertex" ,true, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, _input_position, true, int_marker);

    _server->insert(int_marker);
    _server->setCallback(int_marker.name, &boxFeedback);
    _box_menu_handler.apply( *_server, int_marker.name );
}

void add_LCedge(const int input_id, const int target_id, double *d)
{
    std::cout << "add edge " << input_id << " " << target_id << std::endl;
    g2o::EdgeSE3 *LCedge(new g2o::EdgeSE3);
    slam3d.createEdgeSE3(slam3d.vertexMap[target_id], slam3d.vertexMap[input_id], d, *LCedge);
    slam3d.addEdge(LCedge);
    SLAM3DSystem::ids ids;
    ids.i = target_id;
    ids.j = input_id;
    _LC_id_map.push_back(ids);

    visualization_msgs::MarkerArray markerArray;
    slam3d.makeGraphMarkArray(markerArray, _LC_id_map);
    _graph_markers_publish.publish(markerArray);
}
// %Tag(main)%

void get_param(ros::NodeHandle pn)
{
    GET_ROS_REQUIRE_PARAM(pn, "tf_x", _tf_x);
    GET_ROS_REQUIRE_PARAM(pn, "tf_y", _tf_y);
    GET_ROS_REQUIRE_PARAM(pn, "tf_z", _tf_z);
    GET_ROS_REQUIRE_PARAM(pn, "tf_roll", _tf_roll);
    GET_ROS_REQUIRE_PARAM(pn, "tf_pitch", _tf_pitch);
    GET_ROS_REQUIRE_PARAM(pn, "tf_yaw", _tf_yaw);

    Eigen::Translation3f tl_vtol(_tf_x, _tf_y, _tf_z);                 // tl: translation
    Eigen::AngleAxisf rot_x_vtol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
    Eigen::AngleAxisf rot_y_vtol(_tf_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_vtol(_tf_yaw, Eigen::Vector3f::UnitZ());
    _tf_vtol = (tl_vtol * rot_z_vtol * rot_y_vtol * rot_x_vtol).matrix(); // base to lidar
    _tf_ltov = _tf_vtol.inverse();

    slam3d.setTF(_tf_vtol);
}

void gt_read()
{
    fstream read;
    double t, x, y, z, roll, pitch, yaw, state;
    read.open(_data_path+"vertex_time.txt");
    if(read.good())
    {
        int i=0;
        while(!read.eof())
        {
            istringstream ss;
            string str;
            getline(read, str);
            ss.str(str);
            ss >> t >> x  >> y >> z >> roll >> pitch >> yaw >> state;

//            if(i==0)
//            {
//                tf::Quaternion quat;
//                quat.setRPY(roll, pitch, yaw);
//                gt_position[0] = x;
//                gt_position[1] = y;
//                gt_position[2] = z;
//                gt_position[3] = quat.x();
//                gt_position[4] = quat.y();
//                gt_position[5] = quat.z();
//                gt_position[6] = quat.w();
//            }
            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = z;
            if(state==1)
                gt_cloud->push_back(point);

            i++;
        }
    }
    read.close();

    pcl::toROSMsg(*gt_cloud, gt_msg);
    gt_msg.header.frame_id = "odom";
    gt_pub.publish(gt_msg);

    InteractiveMarker int_marker;
    intMarker.make6DofMarker("ground_truth" ,true, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, gt_position, true, int_marker);

    _server->insert(int_marker);
    _server->setCallback(int_marker.name, &gtFeedback);
    _gt_box_menu_handler.apply( *_server, int_marker.name );
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "heuristic_slam_node");

    ros::NodeHandle n;
    ros::NodeHandle pn("~");
    // create a timer to update the published transforms
    ros::Timer frame_timer = n.createTimer(ros::Duration(0.1), frameCallback);

    _server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

    ros::Duration(0.1).sleep();

    get_param(pn); // get param
    pn.param<std::string>("data_path", _data_path, "");

    _box_menu_handler.insert( "aline", &boxFeedback );
    _box_menu_handler.insert( "Add edge", &boxFeedback );
    _box_menu_handler.insert( "Add edge & Optimize", &boxFeedback );

    _gt_box_menu_handler.insert( "aline", &gtFeedback );
    _gt_box_menu_handler.insert( "get matrix", &gtFeedback );

    _vertex_menu_handler.insert("set Input vertex", &vertexFeedback);
    _vertex_menu_handler.insert("set Target vertex", &vertexFeedback);

    _graph_markers_publish = n.advertise<visualization_msgs::MarkerArray>("/graph_markers", 1000);
    _input_scan_pub = n.advertise<sensor_msgs::PointCloud2>("/cloud", 1000);
    _target_map_pub = n.advertise<sensor_msgs::PointCloud2>("/target_cloud", 1000);
    _check_scan_pub = n.advertise<sensor_msgs::PointCloud2>("/check_cloud", 1000);
    _slam_map_pub = n.advertise<sensor_msgs::PointCloud2>("/map", 1000);

    gt_pub = n.advertise<sensor_msgs::PointCloud2>("/gt", 1000);

    pcl::PointCloud<pcl::PointXYZI> p;
    pcl::PointXYZI point;
    p.push_back(point);
    pcl::toROSMsg(p, empty_msg);
    empty_msg.header.frame_id = "map";

    gt_read();

    slam3d.initG2O("gn_var");
    slam3d.ReadData(_data_path);
    slam3d.optimizeGraph(100, true);

//    slam3d.initG2O("gn_var");

    visualization_msgs::MarkerArray markerArray;
    slam3d.makeGraphMarkArray(markerArray, _LC_id_map);
    _graph_markers_publish.publish(markerArray);

    pcl::PointCloud<pcl::PointXYZI>::Ptr slam_map(new pcl::PointCloud<pcl::PointXYZI>);
    slam3d.buildCloudMap(*slam_map);

    sensor_msgs::PointCloud2 slam_map_msg;
    pcl::toROSMsg(*slam_map, slam_map_msg);
    slam_map_msg.header.frame_id = "map";
    _slam_map_pub.publish(slam_map_msg);
    make_vertex_int_marker();

    _server->applyChanges();

    ros::spin();

    _server.reset();

    ////////////////////////////////////////////////////////////
    FILE *fp;

    fp = fopen((_data_path+"slam.txt").c_str(), "w");

    double x, y, z, roll, pitch, yaw;
    for(int i=0; i<slam3d.vertexMap.size(); i++)
    {
        double pose[7];
        slam3d.vertexMap[i]->getEstimateData(pose);
        tf::Quaternion quat;
        quat.setValue(pose[3], pose[4], pose[5], pose[6]);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        x = pose[0];
        y = pose[1];
        z = pose[2];

        fprintf(fp, "%lf %lf %lf %lf %lf %lf\n",x, y, z, roll, pitch, yaw);
    }

    fclose(fp);
    ////////////////////////////////////////////////////////////
}