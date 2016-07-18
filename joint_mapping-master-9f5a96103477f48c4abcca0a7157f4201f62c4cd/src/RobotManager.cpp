#define TODO(x) ROS_WARN_THROTTLE(1.0, "\t TODO: \"%s\"\n\tFrom line: %d, file: %s\n", x, __LINE__, __FILE__);

#include <joint_mapping/RobotManager.h>
#include <joint_mapping/LocalAgent.h>
#include <joint_mapping/Network.h>

#include <joint_mapping/Utils.h>

#include <joint_mapping/asio_udp_utils.h>
#include <parameter_utils/ParameterUtils.h>

#include <boost/lexical_cast.hpp>

namespace pu = parameter_utils;

/* ************************************************************************** */
// initialization functions
bool RobotManager::initialize(const ros::NodeHandle& n)
{
  name = ros::names::append(n.getNamespace(), "joint_mapping");

  network_ = boost::shared_ptr<Network>(new Network());

  // load param from yaml and launch file
  // This will initialize the agentPtr
  if (!this->loadParameters(n))
  {
    ROS_ERROR("%s: failed to load parameters", name.c_str());
    return false;
  }

  // Initialize the Network and pass the local agent pointer to it
  network_->initialize(n, agentPtr);

  return true;
}


/* ************************************************************************** */
// utils
bool RobotManager::loadParameters(const ros::NodeHandle& n)
{

  // Load the number of robots
  std::vector<std::string> names;
  if (!pu::get("robot_list", names)) return false;
  std::vector<std::string> nameIDs;
  if (!pu::get("robot_id_list", nameIDs)) return false;
  if (names.size() != nameIDs.size()) return false;
  n_robots = names.size();

  int index;
  if (!pu::get("index", index)) return false;

  robot_name = names[index];
  robot_ID = nameIDs[index][0];

//  ROS_INFO("We are creating a pointer for %s, ID %c", robot_name.c_str(), robot_ID);
  // init the agent
  agentPtr = comap::LocalAgent::Ptr(new comap::LocalAgent(robot_ID, robot_name, n_robots));

  // load default setting
  comap::AgentSetting default_setting = loadAgentSetting(n);

  // init use loaded setting
  agentPtr->init(default_setting);
  agentPtr->addNetwork(network_);

  for (int i=0; i < names.size(); i++) {
    if (i != index) {
      agentPtr->addForeignRobot(names[i], nameIDs[i][0]);
    }
  }

  return true;
}
