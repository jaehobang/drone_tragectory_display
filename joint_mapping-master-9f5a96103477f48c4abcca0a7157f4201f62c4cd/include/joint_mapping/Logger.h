/*
 * @file Logger.h
   @brief Singleton class that sends data about system performance
   @author Vibhav Ganesh
   @date April 4, 2016
*/

#ifndef LOGGER_H
#define LOGGER_H

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose2D.h>

#include <joint_mapping/Agent_Definitions.h>
#include <joint_mapping/AgentPackageMsg.h>
#include <joint_mapping/MatchPointClouds.h>
#include <joint_mapping/TimedTask.h>

#include <joint_mapping/Timer.h>

#include <std_msgs/UInt32.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

class Logger
{

private:
  ros::Publisher scanPointsNumPub;

  ros::Publisher originalPointsNumPub;
  ros::Publisher downsampleNumPub;
  ros::Publisher keyPointsNumPub;

  ros::Publisher localKeyPointsPub;
  ros::Publisher foreignKeyPointsPub;

  ros::Publisher knnHistPub;
  ros::Publisher relPoseEstimatePub;

  ros::Publisher packetSizePub;
  ros::Publisher matchScanPub;

  ros::Publisher timerPub;

  Timer timer;

  std::string current_task;

  static Logger *s_instance;

  Logger()
  {
    ros::NodeHandle n("~");
    if (!initialize(n)){
      ROS_ERROR("%s: failed to initialize logger",
                ros::this_node::getName().c_str());
    }
    current_task = "none";
  }

  bool initialize(const ros::NodeHandle& n) {
    ros::NodeHandle nl(n);

    scanPointsNumPub =
      nl.advertise<std_msgs::UInt32MultiArray>("scan_points_count", 10);

    originalPointsNumPub =
      nl.advertise<std_msgs::UInt32>("original_points_count", 10);
    downsampleNumPub =
      nl.advertise<std_msgs::UInt32>("down_points_count", 10);
    keyPointsNumPub =
      nl.advertise<std_msgs::UInt32>("key_points_count", 10);

    knnHistPub =
      nl.advertise<std_msgs::UInt32MultiArray>("knn_hist", 10);

    relPoseEstimatePub =
      nl.advertise<geometry_msgs::Pose2D>("relpose_estimate", 10);

    packetSizePub =
      nl.advertise<std_msgs::UInt32>("packet_size", 10);

    matchScanPub =
      nl.advertise<joint_mapping::MatchPointClouds>("match_scan_info", 10);

    timerPub =
      nl.advertise<joint_mapping::TimedTask>("timer", 10);

    localKeyPointsPub=
      nl.advertise<sensor_msgs::PointCloud2>("local_keypoints", 10);
    foreignKeyPointsPub=
      nl.advertise<sensor_msgs::PointCloud2>("foreign_keypoints", 10);

    return true;
  }

public:
  static Logger *instance()
  {
    if (!s_instance)
      s_instance = new Logger();
    return s_instance;
  }

  void sendOriginalPoints(unsigned int num_points) {
    std_msgs::UInt32 msg;
    msg.data = num_points;
    originalPointsNumPub.publish(msg);
  }

  void sendDownsamplePoints(unsigned int num_points) {
    std_msgs::UInt32 msg;
    msg.data = num_points;
    downsampleNumPub.publish(msg);
  }

  void sendScanPoints(unsigned int original, unsigned int downsample, unsigned int keypoints) {

    std::vector<unsigned int> points(3);
    points[0] = original;
    points[1] = downsample;
    points[2] = keypoints;

    std_msgs::UInt32MultiArray msg;
    msg.data = points;
    std_msgs::MultiArrayDimension dim;
    dim.label = "row";
    dim.size = 3;
    dim.size = 3;
    msg.layout.dim.push_back(dim);

    scanPointsNumPub.publish(msg);
  }

  void sendKeyPoints(unsigned int num_points) {
    std_msgs::UInt32 msg;
    msg.data = num_points;
    keyPointsNumPub.publish(msg);
  }

  void sendKnnHist(const std::vector<unsigned int> result_hist) {
    std_msgs::UInt32MultiArray msg;
    msg.data = result_hist;
    std_msgs::MultiArrayDimension dim;
    dim.label = "row";
    dim.size = result_hist.size();
    dim.size = result_hist.size();
    msg.layout.dim.push_back(dim);

    knnHistPub.publish(msg);
  }

  void sendRelPoseEstimate(float x, float y, float theta) {
    geometry_msgs::Pose2D msg;
    msg.x = x;
    msg.y = y;
    msg.theta = theta;
    relPoseEstimatePub.publish(msg);
  }

  void sendPacketSize(unsigned int size) {
    std_msgs::UInt32 msg;
    msg.data = size;
    packetSizePub.publish(msg);
  }

  void sendTimedTask(std::string task_name, signed long usecs) {
    joint_mapping::TimedTask msg;
    msg.task_name = task_name;
    msg.usecs = usecs;

    timerPub.publish(msg);
  }

  void startTimer(std::string task) {
    if (current_task != "none")
      ROS_ERROR("Things are happening in parallel");

    current_task = task;
    timer.tic();
  }

  void endTimer(){
    signed long time = timer.toc();
    sendTimedTask(current_task, time);
    current_task = "none";
  }

  void sendMatchedPointClouds(std::string local_robot,
                              std::string foreign_robot,
                              size_t local_pose_count,
                              size_t foreign_pose_count,
                              sensor_msgs::PointCloud2 local_pc,
                              sensor_msgs::PointCloud2 foreign_pc,
                              float x, float y, float theta)
  {

    joint_mapping::MatchPointClouds msg;
    msg.local_robot = local_robot;
    msg.foreign_robot = foreign_robot;
    msg.local_pose_count = (unsigned int)local_pose_count;
    msg.foreign_pose_count = (unsigned int)foreign_pose_count;
    msg.local_pc = local_pc;
    msg.foreign_pc = foreign_pc;

    geometry_msgs::Pose2D pose;
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    msg.rel_pose = pose;

    matchScanPub.publish(msg);
  }

  void sendLocalKeyPoints(sensor_msgs::PointCloud2 pc)
  {
    localKeyPointsPub.publish(pc);
  }
  void sendForeignKeyPoints(sensor_msgs::PointCloud2 pc)
  {
    foreignKeyPointsPub.publish(pc);
  }
};

#endif
