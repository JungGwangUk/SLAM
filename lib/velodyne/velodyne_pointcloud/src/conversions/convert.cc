/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include <velodyne_pointcloud/rawdata.h>
#include "velodyne_pointcloud/convert.h"

double NormalizeAngle(double angle)
{
  if(angle > M_PI)
    angle -= 2*M_PI;
  else if(angle < -M_PI)
    angle += 2*M_PI;

  return angle;
}

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh);

    if (!private_nh.getParam("lidar_compensate", lidar_compensate_))
    {
      ROS_ERROR_STREAM("No lidar compensator usage status.  Using false values!");
      lidar_compensate_ = false;
    }

    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);
      
    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne_pointcloud::
      CloudNodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::CloudNodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to VelodyneScan packets
    velodyne_scan_ =
      node.subscribe("velodyne_packets", 100,
                     &Convert::processScan, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));

    imu_data_ =
      node.subscribe("/imu_pose", 100,
                     &Convert::poseCB, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }
  
  void Convert::callback(velodyne_pointcloud::CloudNodeConfig &config,
                uint32_t level)
  {
  ROS_INFO("Reconfigure Request");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw scan messages. */
  void Convert::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    if(!init_imu_)
    {
      init_imu_ = true;
      return;
    }

    // allocate a point cloud with same time and frame ID as raw data
    PointcloudXYZIR outMsg;
    // outMsg's header is a pcl::PCLHeader, convert it before stamp assignment
    outMsg.pc->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
    outMsg.pc->height = 1;

    outMsg.pc->points.reserve(scanMsg->packets.size() * data_->scansPerPacket());

    // process each packet provided by the driver
    if(lidar_compensate_)
    {
      outMsg.pc->header.frame_id = "imu";   ///> imu coordinates
      Eigen::Matrix4d T,T1,T2,Tml;

      Tml = data_->Tml;
      T2 = data_->PoseToMatrix(pose_array_[pose_array_.size()-1]);

      velodyne_msgs::IMURPYpose d_pose;

      for (size_t i = 0; i < scanMsg->packets.size(); ++i)
      {
        int c = float(scanMsg->packets.size()/(pose_array_.size()-1));
        if(i%c == 0)
        {
          int m = float(i/c);

          T1 = data_->PoseToMatrix(pose_array_[m]);
          T = T2.inverse()*T1;
          if((m+1)<pose_array_.size())
          {
            d_pose.x = pose_array_[m+1].x-pose_array_[m].x;
            d_pose.y = pose_array_[m+1].y-pose_array_[m].y;
            d_pose.z = pose_array_[m+1].z-pose_array_[m].z;
            d_pose.roll = pose_array_[m+1].roll-pose_array_[m].roll;
            d_pose.pitch = pose_array_[m+1].pitch-pose_array_[m].pitch;
            d_pose.yaw = pose_array_[m+1].yaw-pose_array_[m].yaw;

            d_pose.roll = NormalizeAngle(d_pose.roll);
            d_pose.pitch = NormalizeAngle(d_pose.pitch);
            d_pose.yaw = NormalizeAngle(d_pose.yaw);
          }
          else
          {
            d_pose.x = 0.0;
            d_pose.y = 0.0;
            d_pose.z = 0.0;
            d_pose.roll = 0.0;
            d_pose.pitch = 0.0;
            d_pose.yaw = 0.0;
          }

        }

        data_->unpack(scanMsg->packets[i], outMsg, T, d_pose, c, i%c);
      }
    }
    else
    {
      outMsg.pc->header.frame_id = scanMsg->header.frame_id;
      for(size_t i = 0; i < scanMsg->packets.size(); ++i)
      {
        data_->unpack(scanMsg->packets[i], outMsg);
      }
    }


    // publish the accumulated cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg.pc->height * outMsg.pc->width
                     << " Velodyne points, time: " << outMsg.pc->header.stamp);
    output_.publish(outMsg.pc);
    pose_array_.clear();
  }

  void Convert::poseCB(const velodyne_msgs::IMURPYposeConstPtr &pose)
  {
    pose_array_.push_back(*pose);
  }
} // namespace velodyne_pointcloud
