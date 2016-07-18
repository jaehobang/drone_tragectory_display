/*

Handles all communication including publishing and receiving messages. A diferent node will deal with network communication

1. Receives scans and calls the matcher and if we have a match and that passes that along to the robots manager witch will add it to the list of stored correspondences

2. Sends scans when they are given to it

3.

*/

#pragma once

#include <joint_mapping/Agent_Definitions.h>
#include <joint_mapping/AgentPackageMsg.h>
#include <joint_mapping/PointCloudWithPose.h>

#include <asio_udp_device/ASIOUDPDevice.h>

#include <joint_mapping/asio_udp_utils.h>
#include <joint_mapping/Utils.h>
#include <joint_mapping/RelativePose.h>
#include <joint_mapping/AgentPackageMsg.h>

#include <parameter_utils/ParameterUtils.h>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <boost/lexical_cast.hpp>

#include <dense_mapping/DenseVoxelMap.h>

namespace comap {
  class LocalAgent;
}

class Network {
public:
  ~Network() {}

  bool initialize(const ros::NodeHandle& n, boost::shared_ptr<comap::LocalAgent> ptr);
  bool loadParameters(const ros::NodeHandle& n);

  bool preparePackage(std::string name, size_t pose_count, comap::PoseD cur_pose, comap::PoseD delta_odom);
  bool addScan(std::string name, size_t pose_count, comap::PoseD cur_pose, comap::Scan& scan);
  bool addMatches(std::string foreign_robot, std::string local_robot, size_t foreign_idx, size_t local_idx, comap::PoseD rel_pose, comap::PoseD foreign_robot_pose, comap::PoseD local_robot_pose);

  bool sendLatestPackage();

  void udpCallback(const unsigned char* buf, size_t buf_size);
  bool registerCallbacks(const ros::NodeHandle& n);
  void packageCallback(const joint_mapping::AgentPackageMsg::ConstPtr& msg);
  void scanOdomCallback(const joint_mapping::PointCloudWithPose::Ptr& msg);

  void sendScantoMap(comap::PoseD pose, sensor_msgs::PointCloud2 scan);

  // UDP agents
  ASIOUDPDevice udp_device;

  // Agent information
  boost::shared_ptr<comap::LocalAgent> agentPtr;

  // ROS topic publisher & subscriber
  ros::Subscriber agentPackSub; // multi agent package subscriber
  ros::Subscriber scanOdomSub;

  ros::Publisher agentPackPub;  // multi agent package publisher
  ros::Publisher mapPub;
  ros::Publisher posePub;

  size_t update_counter;
  unsigned int update_interval;

  // udp communication
  std::string local_ip;
  unsigned int local_port;
  std::vector<std::string> agent_ip_list;
  std::vector<unsigned int> agent_port_list;
  bool use_udp;

  unsigned int n_robots; // number of robot agents

private:
  comap::AgentPackage output_package_;

  std::string robot_frame_;
  tf2_ros::Buffer tfBuffer;
  boost::shared_ptr<tf2_ros::TransformListener> tfListener;
  boost::shared_ptr<ros::Rate> rate;

  DenseVoxelMap dvm_;
  geometry_msgs::Pose local_pose_;
  float range_max_;
};
