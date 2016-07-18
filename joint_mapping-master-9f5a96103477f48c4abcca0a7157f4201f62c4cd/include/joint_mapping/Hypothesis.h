/*
 * @file Hypothesis.h
 * @brief hypothesis indicate relative pose and inlier/outlier information
 * @author Jing Dong
 * @date July 7, 2014
 */

#pragma once

#include <joint_mapping/Settings.h>
#include <joint_mapping/CRProcess.h>
#include <joint_mapping/Agent_Definitions.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Vector.h>

#include <boost/optional.hpp>

#include <map>
#include <vector>

namespace comap {

/* ************************************************************************* */
// data class
struct Hypothesis {

  // relative pose (at origin)
  PoseD relative_pose;
  // inlier/outlier bool vector
  std::vector<bool> inlier_vec;
  // inlier number for convenience
  size_t nr_inlier;

  // utils
  void print() const;
};

struct HypothesisSelectResult {

  // result hypothesis in boost::optional
  boost::optional<Hypothesis> hypothesis;
  // valid hypothesis count (valid : prior > 0)
  size_t valid_count;
  // valid hypothesis index
  std::vector<size_t> valid_idx;
  // hypothesis prior
  std::vector<double> hypothesis_prior;
  // posterior log prob
  std::vector<double> post_log_prob;

  // utils
  void print() const;
};

/* ************************************************************************* */
// methods

// merge Hypothesis given resolution
std::vector<Hypothesis> mergeHypothesis(const std::vector<Hypothesis>& rawhypothesis,
    const HypothesisMergeSetting& setting);

// Hypothesis selection given setting
// output will be a boost::optional pointer. If not find a valid hyopthesis will be empty
HypothesisSelectResult selectHypothesis(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<PoseD>& measure_poses,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value,
    const std::vector<Hypothesis>& hypothesis,
    const HypothesisSelectSetting& setting);


/* ************************************************************************* */
// internal methods

// get transformation prior as <pose, covmatrix>
// only call it when nr_inlier > 0
std::pair<PoseD, gtsam::Matrix> getTransformationPrior(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value,
    const Hypothesis& hypothesis,
    const HypothesisSelectSetting& setting);

// get hypothesis prior for a single hypothesis
double getHypothesisPrior(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value,
    const Hypothesis& hypothesis,
    const HypothesisSelectSetting& setting);

// get 1-D trajectory, with key.index to trajectory vector correspondence
std::vector<double> generateTrajectory(unsigned char ID, const gtsam::Values& value,
    std::map<size_t, size_t>& value_to_traj_idx);


}   // namespace comap
