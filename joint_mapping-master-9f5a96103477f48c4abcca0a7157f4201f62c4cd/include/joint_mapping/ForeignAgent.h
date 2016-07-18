/*

Generic class for foreign agent. Robot manager has a list of these and uses them to update.
*/

#pragma once

#include <joint_mapping/Agent.h>
#include <joint_mapping/Network.h>
#include <joint_mapping/Timer.h>
#include <joint_mapping/SmallEM.h>

#include <joint_mapping/Matcher3D.h>

#include <boost/thread/mutex.hpp>

namespace comap {

class LocalAgent;

class ForeignAgent: public Agent {

private:
 PoseD relative_pose_;
 bool rel_pose_built_;

 boost::mutex package_mutex;
 std::vector<AgentPackage> received_packages_;

 SmallEM small_EM_;

 PoseD last_recorded_pose;
 size_t last_recorded_index;

 std::vector<std::pair<size_t, size_t> > inlier_pose_indices;
 std::vector<PoseD> inlier_relposes;

 // TODO make this a rolling  buffer
 std::vector<sensor_msgs::PointCloud2> foreign_scans_;
 std::vector<PoseD> foreign_scans_pose_;

 boost::shared_ptr<Matcher3D> matcher_;
 boost::shared_ptr<Network> network_;

 Timer loop_closure_timer;
 std::vector<unsigned long> loop_closure_times;
 boost::shared_ptr<LocalAgent> local_agent_;

public:
  typedef boost::shared_ptr<ForeignAgent> Ptr;
  typedef boost::shared_ptr<const ForeignAgent> ConstPtr;

  ForeignAgent(std::string name, unsigned char ID, size_t num_robots) :
   rel_pose_built_(false), Agent(ID, name, num_robots) {}

  void sendScanstoMap();

  virtual ~ForeignAgent() {}

  void initialize(AgentSetting setting, unsigned char local_id, boost::shared_ptr<Matcher> matcher, boost::shared_ptr<Network> network);

  bool is_rel_pose_built() const { return rel_pose_built_; }        // current cycle count
  void set_rel_pose( PoseD pose) { relative_pose_ = pose; rel_pose_built_ = true;}
  PoseD get_rel_pose() {return relative_pose_; } // assumes external code does is_built

  std::vector<std::pair<size_t, size_t> > get_inlier_indices() {
    return inlier_pose_indices; }
  std::vector<PoseD> get_inlier_poses() {
    return inlier_relposes; }

  void insert_package(const AgentPackage& received_package) {
    boost::unique_lock<boost::mutex> scoped_lock(package_mutex);
    received_packages_.push_back(received_package);
  }

  void setMatcher(boost::shared_ptr<Matcher3D> matcher) {
    matcher_ = matcher;
  }

  void setNetwork(boost::shared_ptr<Network> network) {
    network_ = network;
  }

  void setLocalAgent(boost::shared_ptr<LocalAgent> local_agent) {
    local_agent_ = local_agent;
  }

  void init_smallEM() {
    small_EM_.init(inlier_relposes, inlier_pose_indices, relative_pose_);
  }

  bool process_received_packages(gtsam::NonlinearFactorGraph& update_graph, gtsam::Values& init_values);

  void remove_outliers(const Hypothesis& hypothesis);
  GraphValues generateAgentGraph(const Hypothesis& hypothesis);
  gtsam::NonlinearFactorGraph receiveAgentLoopClosure(const AgentPackage& package);

  GraphValues updateAgentGraph(size_t pose_idx, const PoseD& slam_pose);
  gtsam::NonlinearFactorGraph findAgentLoopClosure(size_t pose_idx,
    std::string robot_name,
    const PoseD& slam_pose, Scan& agent_scan,
    const gtsam::NonlinearFactorGraph& update_graph);

};

}
