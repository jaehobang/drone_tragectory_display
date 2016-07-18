/*

Things common between local and foreign agent

*/

#pragma once

#include <vector>

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <joint_mapping/Settings.h>

#include <joint_mapping/Agent_Definitions.h>
#include <joint_mapping/Matcher3D.h>

namespace comap {

struct AgentSetting {

  // ====================================================================
  // algorithm settings
  // icp setting filenames, one for consective frame+saliency, one for loop closure detection
  PointMatcherSetting icp_config_consec;

  // iSAM2 setting
  gtsam::ISAM2Params isam_param;

  // default noise model for consecutive frame icp matching
  gtsam::noiseModel::Diagonal::shared_ptr icp_default_model;

  // between-robot loop closure detection setting (before loop closure is built)
  // use FLIRT feature + ANN + RANSAC based method
  LoopClosureDetectorSetting loop_detect_setting;
  LoopClosureMatcherSetting loop_match_setting;

  Matcher3DSetting matcher_setting;

  // min number of trained local scan to start loop querying
  unsigned int min_cache_size;
  size_t train_min_iter;
  double train_min_distance;
  size_t min_pose_count;

  // build loop closure setting (after loop closure is built)
  // Small EM methoed to identify inlier loop closure
  SmallEMSetting smallEM_setting;
  // default noise model for loop closure icp matching
  gtsam::noiseModel::Diagonal::shared_ptr loop_default_model;
  // perfrom local small EM after a certain count of loop closures
  unsigned int local_loop_count_smallEM;
  // min index interval for accept loop closure: avoid dense pose graph
  unsigned int local_loop_interval;

  // EM-hypo_selection sub-algorithms settings
  ClusteringSetting cluster_setting;
  RelativePoseEMSetting relativeEM_setting;
  HypothesisMergeSetting hypo_merge_setting;
  HypothesisSelectSetting hypo_select_setting;

  // default settings
  AgentSetting() :
    icp_config_consec(),

    isam_param(),

    //loop_detect_setting(),
    //loop_match_setting(),
    min_cache_size(5),
    train_min_iter(5),
    min_pose_count(10),
    train_min_distance(0.2),
    smallEM_setting(),
    local_loop_count_smallEM(10),
    local_loop_interval(500),

    cluster_setting(),
    relativeEM_setting(),
    hypo_merge_setting(),
    hypo_select_setting()

    {
      Eigen::VectorXd temp(6);
      temp << 10e-3, 10e-3, 10e-3, 10e-4, 10e-4, 10e-4;
      icp_default_model = gtsam::noiseModel::Diagonal::Sigmas(temp);

      Eigen::VectorXd temp2(6);
      temp2 << 100e-3, 100e-3, 100e-3, 100e-4, 100e-4, 100e-4;
      loop_default_model = gtsam::noiseModel::Diagonal::Sigmas(temp2);

      //icp_default_model = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 10e-3, 10e-3, 10e-4));
      //loop_default_model = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(3) << 100e-3, 100e-3, 100e-4));
    }

  virtual ~AgentSetting() {}

};

class Agent {

public:
  typedef boost::shared_ptr<Agent> Ptr;
  typedef boost::shared_ptr<const Agent> ConstPtr;

  size_t num_robots_;
  const unsigned char ID_;
  const std::string name_;

  AgentSetting setting_;

protected:
  PoseD curr_pose_;      // current robot pose
  gtsam::Values curr_values_;   // current optimized value
  gtsam::ISAM2Params isam_params_;
  gtsam::ISAM2 isam_;           // agent main optimizer

  std::string output_path;
  bool output_to_file;

public:

  Agent(unsigned char ID, std::string name, size_t num_robots) : ID_(ID), name_(name), num_robots_(num_robots) {}

  gtsam::Values curr_values() { return curr_values_; }
  void insert_value(gtsam::Values& initvalue) { curr_values_.insert(initvalue); }
  PoseD curr_pose() const { return curr_pose_; }    // current system pose

  gtsam::ISAM2 getisam() { return isam_; }

  void init( AgentSetting& setting) { setting_ = setting; }

};

}
