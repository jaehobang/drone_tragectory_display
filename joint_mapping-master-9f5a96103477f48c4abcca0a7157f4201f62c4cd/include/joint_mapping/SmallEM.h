/*
 * @file SmallEM.h
 * @brief Small EM: use EM to identify inlier/outlier loop closure
 * @author Jing Dong, 9/9/14
 * @revised by Vibhav Ganesh, 1/31/16
 */

#pragma once

#include <joint_mapping/Hypothesis.h>

#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose2.h>

#include <gtsam_unstable/slam/TransformBtwRobotsUnaryFactorEM.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

#include <vector>

// flag turn off anything about update inlier model
#define SMALLEM_UPDATE_INLIER_MODEL 1

namespace comap {

/* ************************************************************************* */
// a small EM class, init it after relative pose is built
// use it: give a loop closure indentify whether it's inlier or outlier
class SmallEM {

public:
  // statues
  bool flag_init;
#if SMALLEM_TIMING
  Timer timer_update, timer_opt;
#endif

private:
  // setting
  unsigned char local_ID_, agent_ID_;
  SmallEMSetting setting_;
  // relative measurement cache
  std::vector<PoseD> measure_poses_;
  std::vector<std::pair<size_t, size_t> > measure_index_;
  PoseD relpose_;
  // for visualization, on robot these will be turned off
  std::vector<bool> inlier_vec_;
  std::vector<PoseD> measure_relposes_;
  std::vector<gtsam::Matrix> inlier_model_cov_;

public:
  // constructors
  SmallEM() : flag_init(false) {}

  SmallEM(unsigned char local_ID, unsigned char agent_ID,
      const SmallEMSetting& setting) :
        flag_init(false), local_ID_(local_ID), agent_ID_(agent_ID), setting_(setting) {}

  // decontructor
  virtual ~SmallEM() {}

  // init the small EM when relative pose is built
  void init(
      const std::vector<PoseD>& measure_poses,
      const std::vector<std::pair<size_t, size_t> >& measure_index,
      const PoseD& relpose);

  // perfrom small EM, give whether it's inlier/outlier
  bool perform(
      const PoseD& measure_pose,
      const std::pair<size_t, size_t>& measure_index,
      const gtsam::Values& values,
      const gtsam::NonlinearFactorGraph& graph = gtsam::NonlinearFactorGraph());

  // get methods
  std::vector<PoseD> measure_poses() const { return measure_poses_; }
  std::vector<std::pair<size_t, size_t> > measure_index() const { return measure_index_; }
  PoseD relpose() const { return relpose_; }

  std::vector<PoseD> measure_relposes() const { return measure_relposes_; }
  std::vector<bool> inlier_vec() const { return inlier_vec_; }
  std::vector<gtsam::Matrix> inlier_model_cov() const { return inlier_model_cov_; }
};

/* ************************************************************************* */
// perform Small EM optimization to indicate inlier/outlier loop closure
// can be used for local loop closure if agent_ID == local_ID
// graph used for marginalize, if update model is chosen
// use hypothesis data struct to return
Hypothesis performSmallEM(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<PoseD>& measure_poses,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const PoseD& initial,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value,
    const gtsam::NonlinearFactorGraph& graph,
    const SmallEMSetting& setting);

}   // namespace mast
