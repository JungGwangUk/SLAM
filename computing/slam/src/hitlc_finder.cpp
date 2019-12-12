#include <slam/hitlc_finder.h>

void MsgToArray(const boost::array<double, 7> msg, double *array)
{
  for(int i=0; i<7; i++)
    array[i] = msg[i];
}

void MakeGraphMarkArray(hitlc_msgs::InputTargetArray hitlc_array, visualization_msgs::MarkerArray &markers)
{
    for(int i=0; i<hitlc_array.sets.size(); i++)
    {
        visualization_msgs::Marker input_vertex_marker;
        input_vertex_marker.action = visualization_msgs::Marker::ADD;
        input_vertex_marker.header.frame_id = "hitlc_map";
        input_vertex_marker.ns = "vertex_se3";
        input_vertex_marker.type = visualization_msgs::Marker::SPHERE;
        input_vertex_marker.scale.x = 0.5;
        input_vertex_marker.scale.y = 0.5;
        input_vertex_marker.scale.z = 0.5;
        input_vertex_marker.color.r = 1;
        input_vertex_marker.color.a = 1;
        input_vertex_marker.id = i;

        input_vertex_marker.pose.position.x = hitlc_array.sets[i].input_est[0];
        input_vertex_marker.pose.position.y = hitlc_array.sets[i].input_est[1];
        input_vertex_marker.pose.position.z = hitlc_array.sets[i].input_est[2];

        markers.markers.push_back(input_vertex_marker);

        visualization_msgs::Marker target_vertex_marker;
        target_vertex_marker.action = visualization_msgs::Marker::ADD;
        target_vertex_marker.header.frame_id = "hitlc_map";
        target_vertex_marker.ns = "vertex_se3";
        target_vertex_marker.type = visualization_msgs::Marker::SPHERE;
        target_vertex_marker.scale.x = 0.5;
        target_vertex_marker.scale.y = 0.5;
        target_vertex_marker.scale.z = 0.5;
        target_vertex_marker.color.r = 1;
        target_vertex_marker.color.a = 1;
        target_vertex_marker.id = i*100;

        target_vertex_marker.pose.position.x = hitlc_array.sets[i].target_est[0];
        target_vertex_marker.pose.position.y = hitlc_array.sets[i].target_est[1];
        target_vertex_marker.pose.position.z = hitlc_array.sets[i].target_est[2];

        markers.markers.push_back(target_vertex_marker);

        if(i==hitlc_array.sets.size()-1)
            continue;

        visualization_msgs::Marker input_line_marker;
        input_line_marker.action = visualization_msgs::Marker::ADD;
        input_line_marker.header.frame_id  = "hitlc_map";
        input_line_marker.ns = "edges";
        input_line_marker.type = visualization_msgs::Marker::LINE_LIST;
        input_line_marker.scale.x = 0.2;
        input_line_marker.color.a = 1;
        input_line_marker.color.g = 1;
        input_line_marker.id = i;

        input_line_marker.points.resize(2);
        input_line_marker.points[0].x = hitlc_array.sets[i].input_est[0];
        input_line_marker.points[0].y = hitlc_array.sets[i].input_est[1];
        input_line_marker.points[0].z = hitlc_array.sets[i].input_est[2];

        input_line_marker.points[1].x = hitlc_array.sets[i+1].input_est[0];
        input_line_marker.points[1].y = hitlc_array.sets[i+1].input_est[1];
        input_line_marker.points[1].z = hitlc_array.sets[i+1].input_est[2];

        markers.markers.push_back(input_line_marker);

        visualization_msgs::Marker target_line_marker;
        target_line_marker.action = visualization_msgs::Marker::ADD;
        target_line_marker.header.frame_id  = "hitlc_map";
        target_line_marker.ns = "edges";
        target_line_marker.type = visualization_msgs::Marker::LINE_LIST;
        target_line_marker.scale.x = 0.2;
        target_line_marker.color.a = 1;
        target_line_marker.color.g = 1;
        target_line_marker.id = i*100;

        target_line_marker.points.resize(2);
        target_line_marker.points[0].x = hitlc_array.sets[i].target_est[0];
        target_line_marker.points[0].y = hitlc_array.sets[i].target_est[1];
        target_line_marker.points[0].z = hitlc_array.sets[i].target_est[2];

        target_line_marker.points[1].x = hitlc_array.sets[i+1].target_est[0];
        target_line_marker.points[1].y = hitlc_array.sets[i+1].target_est[1];
        target_line_marker.points[1].z = hitlc_array.sets[i+1].target_est[2];

        markers.markers.push_back(target_line_marker);
    }
}

void InputVertexFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        _intMarker.poseToest(feedback->pose, _input_position);
        ROS_INFO_STREAM(": pose {" << _input_position[0] << ", " << _input_position[1] << "}");
        _input_map_pub.publish(_input_map_msg);
        break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
      if(feedback->menu_entry_id == 1) // Aline
      {
        Eigen::Matrix4f init_guess, final_trans;
        _slam3d.EstimateToMatrix(_input_position, init_guess);
        std::cout << "before : " << std::endl << init_guess << std::endl;
        init_guess(2, 3) = _target_position[2]; // aline Z error
        final_trans = ndt_scanmatching(_target_map_pcl_ptr, _input_map_pcl_ptr, init_guess);

        _slam3d.MatrixToMeasurement(final_trans, _input_position);

        InteractiveMarker int_marker;
        _intMarker.make6DofMarker("input" ,true, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, _input_position, true, int_marker);

        _server->insert(int_marker);
        _server->setCallback(int_marker.name, &InputVertexFeedback);
        _input_vertex_menu_handler.apply( *_server, int_marker.name );
      }
      else if(feedback->menu_entry_id == 2) // Find LC Edge
      {
        for(int i=0; i<_ITarray.sets.size(); i++)
        {
          Eigen::Matrix4f init_guess, final_trans;

          _ITarray.sets[i].target_id;

        }
      }
      else if(feedback->menu_entry_id == 3) // Add LC Edge
      {
        hitlc_msgs::LCinfoArray LC_info_array;

        Eigen::Matrix4f T_input;
        _slam3d.EstimateToMatrix(_input_position, T_input);

        for(int i=1; i<_ITarray.sets.size(); i++)
        {
          Eigen::Matrix4f T0, Ti, T0i, Tt, Tit;;
          double input_est_0[7], input_est_i[7], target_est_i[7];
          MsgToArray(_ITarray.sets[0].input_est, input_est_0);
          MsgToArray(_ITarray.sets[i].input_est, input_est_i);
          MsgToArray(_ITarray.sets[i].target_est, target_est_i);

          _slam3d.EstimateToMatrix(input_est_0, T0);
          _slam3d.EstimateToMatrix(input_est_i, Ti);
          _slam3d.EstimateToMatrix(target_est_i, Tt);

          T0i = T0.inverse()*Ti;
          Tit = T0i.inverse()*T_input.inverse()*Tt;

          double meas[7];

          _slam3d.MatrixToMeasurement(Tit,meas);

          hitlc_msgs::LCInfo LC_info;
          LC_info.input_id = _ITarray.sets[i].input_id;
          LC_info.target_id = _ITarray.sets[i].target_id;

          for(int j=0; j<7; j++)
            LC_info.meas[j] = meas[j];

          LC_info_array.infomations.push_back(LC_info);

        }
        _LC_info_pub.publish(LC_info_array);

      }

      break;
  }
  _server->applyChanges();
}

Eigen::Matrix4f ndt_scanmatching(pcl::PointCloud<pcl::PointXYZI>::Ptr target_map_pcl_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr input_map_pcl_ptr, Eigen::Matrix4f init_guess)
{
  pcl::PointCloud<pcl::PointXYZI> output_points;
  std::cout << "start matching" << std::endl;

  _gpu_ndt.setInputTarget(target_map_pcl_ptr);
  _gpu_ndt.setTransformationEpsilon(0.01);
  _gpu_ndt.setStepSize(0.1);
  _gpu_ndt.setResolution(3.0);
  _gpu_ndt.setMaximumIterations(20);
  _gpu_ndt.setInputSource(input_map_pcl_ptr);

  _gpu_ndt.align(init_guess);

  std::cout << "after : " << std::endl << init_guess << std::endl;
  std::cout << "finish matching(score : "<< _gpu_ndt.getFitnessScore() <<" )" << std::endl;
  std::cout << "iteration num : "<< _gpu_ndt.getFinalNumIteration() << std::endl;
  std::cout << "<final_trans> \n "<< _gpu_ndt.getFinalTransformation() << std::endl;

  Eigen::Matrix4f final_trans;
  final_trans = _gpu_ndt.getFinalTransformation();

  return final_trans;
}

void frameCallback(const ros::TimerEvent&)
{
  static tf::TransformBroadcaster br;

  tf::Transform t;

  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(_input_position[0], _input_position[1], _input_position[2]));
  t.setRotation(tf::Quaternion(_input_position[3], _input_position[4], _input_position[5], _input_position[6]));

  br.sendTransform(tf::StampedTransform(t, time, "hitlc_map", "robot"));

  _input_map_pub.publish(_input_map_msg);
}

void HitLCArrayCB(const hitlc_msgs::InputTargetArrayConstPtr &msg)
{
  _ITarray = *msg;

  visualization_msgs::MarkerArray markerArray;
  MakeGraphMarkArray(*msg, markerArray);
  _graph_markers_pub.publish(markerArray);

  _input_map_pcl_ptr->clear();
  _target_map_pcl_ptr->clear();
  for(int i=0; i<msg->sets.size(); i++)
  {
    double input_est[7], target_est[7];
    Eigen::Matrix4f input_T, target_T;
    pcl::PointCloud<pcl::PointXYZI> pcl, transformed_pcl;

    // convert msg to array
    MsgToArray(msg->sets[i].input_est, input_est);
    MsgToArray(msg->sets[i].target_est, target_est);

    if(i==0)
    {
      MsgToArray(msg->sets[i].input_est, _input_position);
      MsgToArray(msg->sets[i].target_est, _target_position);
    }

    // build input map
    Eigen::Matrix4f T1, Tn;
    _slam3d.EstimateToMatrix(input_est, T1);
    _slam3d.EstimateToMatrix(_input_position, Tn);
    input_T = Tn.inverse()*T1;
    pcl::fromROSMsg(msg->sets[i].input_cloud, pcl);
    pcl::transformPointCloud(pcl, transformed_pcl, input_T);
    *_input_map_pcl_ptr += transformed_pcl;

    // build target map
    _slam3d.EstimateToMatrix(target_est, target_T);
    pcl::fromROSMsg(msg->sets[i].target_cloud, pcl);
    pcl::transformPointCloud(pcl, transformed_pcl, target_T);
    *_target_map_pcl_ptr += transformed_pcl;
  }
  pcl::toROSMsg(*_input_map_pcl_ptr, _input_map_msg);
  pcl::toROSMsg(*_target_map_pcl_ptr, _target_map_msg);

  _input_map_msg.header.frame_id = "robot";
  _input_map_pub.publish(_input_map_msg);

  _target_map_msg.header.frame_id = "hitlc_map";
  _target_map_pub.publish(_target_map_msg);

  // Interactive Marer 생성
  InteractiveMarker int_marker;
  _intMarker.make6DofMarker("input" ,true, visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D, _input_position, true, int_marker);

  _server->clear(); // Interactive Marker 서버 초기화.

  _server->insert(int_marker);
  _server->setCallback(int_marker.name, &InputVertexFeedback);
  _input_vertex_menu_handler.apply( *_server, int_marker.name );

  _server->applyChanges();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hitlc_finder_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    ros::Subscriber hitlc_array_sub = nh.subscribe("hitlc/hitlc_array", 1000, HitLCArrayCB);

    ros::Timer frame_timer = nh.createTimer(ros::Duration(0.1), frameCallback);

    //InteractiveMarker Server Initializing.
    _server.reset( new interactive_markers::InteractiveMarkerServer("hitlc/basic_controls","",false) );

    //Menu handlers setting
    _input_vertex_menu_handler.insert( "aline", &InputVertexFeedback );
    _input_vertex_menu_handler.insert( "Find LC edge", &InputVertexFeedback );
    _input_vertex_menu_handler.insert( "Add LC edge", &InputVertexFeedback );

    //Publisher
    _input_map_pub = nh.advertise<sensor_msgs::PointCloud2>("hitlc/input_map", 100);
    _target_map_pub = nh.advertise<sensor_msgs::PointCloud2>("hitlc/target_map", 100);
    _LC_info_pub = nh.advertise<hitlc_msgs::LCinfoArray>("hitlc/lc_info", 100);

    _graph_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("hitlc/graph_markers", 100);

    ros::Duration(0.1).sleep();

    //Interactive Marker의 변화 적용.
    _server->applyChanges();

    ros::spin();

}