/*

Local Agent is in charge of making the local updates of the map and doing local things

*/

#pragma once

#include <joint_mapping/Agent.h>
#include <joint_mapping/Network.h>
#include <joint_mapping/ForeignAgent.h>
#include <joint_mapping/Logger.h>

#include <joint_mapping/Clustering.h>
#include <joint_mapping/RelativePoseEM.h>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/inference/Symbol.h>

#include <boost/lexical_cast.hpp>
#include <boost/range/adaptor/map.hpp>

#include <cmath>
#include <algorithm>

namespace comap {

class LocalAgent: public Agent {

private:
  // Network manages all communication
  boost::shared_ptr<Network> network_;

  // Takes in the scans and finds relative odom and loop closure
  boost::shared_ptr<Matcher3D> matcher_;

  // Maintains status about the other agents, map their name to object
  std::vector<std::string> foreign_agent_names_;

  std::map<std::string, ForeignAgent::Ptr> foreign_agents_;

  // Settings for the local agent. Instead of separate class, settings
  // will be contained in their individual classes

  // iSAM2 setting
  PoseD last_odompose_;

  // use for loop training interval control
  size_t pose_count_;
  size_t trained_scan_count_;       // trained scan counter
  size_t last_trained_count_;        // use for loop ANN training
  gtsam::Point3 last_trained_point_;    // use for loop ANN training

  SmallEM local_smallEM_;

  // History of the saliency of key scans.
  std::vector<double> saliency_;

public:
  typedef boost::shared_ptr<LocalAgent> Ptr;
  typedef boost::shared_ptr<const LocalAgent> ConstPtr;

  // Initialize the agent
  LocalAgent(unsigned char ID='a', std::string name="robot1", size_t num_robots=1) : Agent(ID, name, num_robots) {}

  // deconstructor
  virtual ~LocalAgent() {}

  void init(AgentSetting& setting);

  void addForeignRobot(std::string fname, unsigned char fID);

  bool update(const PoseD& pose, Scan& scan);
  const std::string getName() const { return name_; }

  unsigned char getID() const { return ID_; }

  void updateGraph( gtsam::NonlinearFactorGraph& update_graph, gtsam::Values& init_values, size_t pose_count);


  bool is_rel_pose_built(std::string foreign_robot_name) {
    return foreign_agents_[foreign_robot_name]->is_rel_pose_built();
  }

  PoseD get_rel_pose(std::string foreign_robot_name) {
    return foreign_agents_[foreign_robot_name]->get_rel_pose();
  }

  bool insert_foreign_package(const AgentPackage& received_package, std::string robot_name) {
    foreign_agents_[robot_name]->insert_package(received_package);
  }

  void addNetwork(boost::shared_ptr<Network> net) { network_ = net; }

  unsigned char get_foreign_ID(std::string name) {
    return foreign_agents_[name]->ID_;
  }

  size_t pose_count() const { return pose_count_; }        // current cycle count
  gtsam::NonlinearFactorGraph curr_graph() const { return isam_.getFactorsUnsafe(); }

  HypothesisSelectResult relativePoseInference(std::string name);

private:

  void readConfigFile();

};

}
