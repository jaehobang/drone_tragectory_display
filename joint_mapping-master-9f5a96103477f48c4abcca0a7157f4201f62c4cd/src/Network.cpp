
/*
   Implements the network communication as well as sending and receiving messages
   */

#include <joint_mapping/Network.h>
#include <joint_mapping/LocalAgent.h>

#include <joint_mapping/Logger.h>

namespace pu = parameter_utils;

bool Network::initialize(const ros::NodeHandle& n, comap::LocalAgent::Ptr ptr) {

  agentPtr = ptr;

  if (!this->loadParameters(n))
  {
    ROS_ERROR("Network of %s failed to load parameters", agentPtr->name_.c_str());
    return false;
  }

  if (!this->registerCallbacks(n))
  {
    ROS_ERROR("Callbacks not registered");
    return false;
  }

  ros::NodeHandle nl(n);

  tfListener = boost::shared_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tfBuffer));
  rate = boost::shared_ptr<ros::Rate>(new ros::Rate(5.0));

  if (!dvm_.initialize(n))
  {
    ROS_ERROR("%s: failed to initialize dense voxel map", agentPtr->name_.c_str());
    return false;
  }

  if (use_udp) {
    // register udp device
    ROS_WARN("%s starting udp socket at IP %s on port %d", agentPtr->name_.c_str(), local_ip.c_str(), local_port);
    udp_device.openLocalSocket(local_ip, local_port);
    udp_device.openRemoteSocket();
    udp_device.setReadCallback(boost::bind(&Network::udpCallback, this, _1, _2));
    udp_device.start();
  }
}

bool Network::loadParameters(const ros::NodeHandle& n)
{

  // Load the number of robots
  n_robots = agentPtr->num_robots_;

  int index = 0;
  std::vector<std::string> names;
  if (!pu::get("robot_list", names)) return false;
  if (!pu::get("index", index)) return false;
  if (!pu::get("use_udp", use_udp)) return false;

  // load udp communication param

  if (use_udp) {
    for (size_t i = 0; i < n_robots; i++) {
      std::string ip;
      std::string robot_name = names[i];
      if (!pu::get("robot/"+robot_name+"/ip", ip)) return false;
      unsigned int port;
      if (!pu::get("robot/"+robot_name+"/port", port)) return false;
      if (i == index) {
        local_ip = ip;
        local_port = port;
      } else {
        agent_ip_list.push_back(ip);
        agent_port_list.push_back(port);
      }
    }
  }

  if (!pu::get("robot_frame", robot_frame_)) return false;

  // For building the map
  if (!pu::get("range_max", range_max_)) return false;

  return true;
}

bool Network::registerCallbacks(const ros::NodeHandle& n)
{
  ros::NodeHandle nl(n);

  agentPackPub =
    nl.advertise<joint_mapping::AgentPackageMsg>("agentPackage", 10);

  mapPub =
    nl.advertise<sensor_msgs::PointCloud2>("map_points", 10);

  posePub =
    nl.advertise<geometry_msgs::PoseStamped>("scan_pose", 10);

  scanOdomSub =
    nl.subscribe("/"+agentPtr->name_+"/point_cloud_with_pose", 100, &Network::scanOdomCallback, this);

  agentPackSub =
    nl.subscribe("agentPackage", 100, &Network::packageCallback, this);

  return true;
}

bool Network::preparePackage(std::string name, size_t pose_count, comap::PoseD cur_pose, comap::PoseD delta_odom) {
  comap::AgentPackage output_package;
  output_package.robot_ID = agentPtr->ID_;
  output_package.robot_name = name;
  output_package.pose_count = pose_count;
  output_package.curr_pose = cur_pose;
  output_package.delta_odometry = delta_odom;
  output_package.informative_scan = false;
  output_package.flag_matches = false;

  output_package_ = output_package;
}

bool Network::addScan(std::string name, size_t pose_count, comap::PoseD cur_pose, comap::Scan& scan) {
  output_package_.scan= *scan;
  output_package_.informative_scan = true;
}

bool Network::addMatches(std::string foreign_robot, std::string local_robot, size_t foreign_idx, size_t local_idx, comap::PoseD rel_pose, comap::PoseD foreign_robot_pose, comap::PoseD local_robot_pose) {
  output_package_.match_robot_names.push_back(std::make_pair(local_robot, foreign_robot));
  output_package_.match_robot_id.push_back(std::make_pair(agentPtr->ID_, agentPtr->get_foreign_ID(foreign_robot)));
  output_package_.match_meas_index.push_back(std::make_pair(local_idx, foreign_idx));
  output_package_.match_measurement.push_back(rel_pose);
  output_package_.flag_matches = true;
}

void Network::scanOdomCallback(const joint_mapping::PointCloudWithPose::Ptr& msg) {

  comap::Scan scanMsg = comap::Scan(new sensor_msgs::PointCloud2(msg->scan));
  geometry_msgs::TransformStamped transform = msg->transform;

  gtsam::Pose3 pose = gtsam::Pose3(
    gtsam::Rot3::quaternion(transform.transform.rotation.w,
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z),
    gtsam::Point3(transform.transform.translation.x,
                  transform.transform.translation.y,
                  transform.transform.translation.z));

  if (agentPtr->update(pose, scanMsg)) {
    this->sendLatestPackage();
    update_counter++;
    if (update_counter > 5) {
      dvm_.processQueue();
      dvm_.visualize();
      update_counter = 0;
    }
  }
}

void Network::sendScantoMap(gtsam::Pose3 pose, sensor_msgs::PointCloud2 scan) {

  mapPub.publish(scan);

  sensor_msgs::PointCloud::Ptr scan_body(new sensor_msgs::PointCloud);
  sensor_msgs::convertPointCloud2ToPointCloud(scan, *scan_body);


  //mapPub.publish(scan_body);

  gtsam::Vector3 rpy = pose.rotation().rpy();
  tf::Quaternion q;
  q.setRPY(rpy(0), rpy(1), rpy(2));
  geometry_msgs::Quaternion qmsg;
  tf::quaternionTFToMsg(q,qmsg);

  geometry_msgs::Pose pose_msg;
  pose_msg.orientation = qmsg;
  pose_msg.position.x = pose.x();
  pose_msg.position.y = pose.y();
  pose_msg.position.z = pose.z();

  geometry_msgs::PoseStamped stamped_pose;
  stamped_pose.pose = pose_msg;
  stamped_pose.header.frame_id = "world";
  stamped_pose.header.stamp = ros::Time::now();

  posePub.publish(stamped_pose);
  dvm_.addRegisteredPointCloud(pose_msg,scan_body,range_max_);
}

// callback when receive new multi-agent message
void Network::packageCallback(const joint_mapping::AgentPackageMsg::ConstPtr& msg) {

  comap::AgentPackage pack_received = msgToPack(*msg);

  if (agentPtr->name_ != pack_received.robot_name) {
    agentPtr->insert_foreign_package(pack_received, pack_received.robot_name);
  }

}

// udp callback
void Network::udpCallback(const unsigned char* buf, size_t buf_size) {

  std::cout << "[UDP] '" << agentPtr->name_ << "' recevied size: " << buf_size << std::endl;

  //comap::AgentPackage::ConstPtr packptr = deserializePackageMAST(buf, buf_size);
  joint_mapping::AgentPackageMsg::ConstPtr msg = deserializePackageROS(buf, buf_size);
  if (msg) {
    comap::AgentPackage package =  msgToPack(*msg);
    std::string robot_name = package.robot_name;

    if (agentPtr->name_ != robot_name) {
      agentPtr->insert_foreign_package(package, robot_name);
    }
  }
}

bool Network::sendLatestPackage() {
  std::vector<uint8_t> buff = serializePackageROS(                                                  packToMsg(output_package_));
  Logger::instance()->sendPacketSize(buff.size());
  if (!use_udp) {
    agentPackPub.publish(packToMsg(output_package_));
  } else if(buff.size() < 64000) {
    for (unsigned int ii = 0; ii < agent_ip_list.size(); ii++) {
      Logger::instance()->sendPacketSize(buff.size());
      udp_device.send(agent_ip_list.at(ii), agent_port_list.at(ii), buff);
      if (buff.size() > 200) {
        std::cout << "robot : " << agentPtr->name_ <<" [UDP] sent size: " << buff.size() << " to " << agent_ip_list.at(ii) << ":" << agent_port_list.at(ii) << std::endl;
      }
    }
  } else {
    std::cout << "Message of length : " << buff.size() << " is too long" << std::endl;
  }

  /*
     std::vector<uint8_t> buff = serializePackageMAST(output_package_);

     for (unsigned int ii = 0; ii < agent_ip_list.size(); ii++) {
       std::cout << "robot : " << agentPtr->name_ <<" [UDP] sent size: " << buff.size() << std::endl;
     }
  */
}
