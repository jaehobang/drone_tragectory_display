/*
 * @file Hypothesis.cpp
 * @brief hypothesis related functions
 * @author Jing Dong
 * @date July 9, 2014
 *
 * modified by Vibhav Ganesh
 */

#include <joint_mapping/Hypothesis.h>
#include <joint_mapping/Clustering.h>

#include <gtsam_unstable/slam/TransformBtwRobotsUnaryFactor.h>

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/Key.h>

#include <boost/pointer_cast.hpp>

#include <cmath>
#include <list>
#include <algorithm>
#include <iostream>

#define _USE_MATH_DEFINES

using namespace std;
using namespace gtsam;

namespace comap{

// short for type between robot factor
typedef TransformBtwRobotsUnaryFactor<PoseD> UnaryFactor;

/* ************************************************************************* */
// merge Hypothesis given resolution
std::vector<Hypothesis> mergeHypothesis(const std::vector<Hypothesis>& rawhypothesis,
    const HypothesisMergeSetting& setting) {

  vector<Hypothesis> candidates;

  // for each raw hypothesis
  for (size_t i = 0; i < rawhypothesis.size(); i++) {

    // first one doesn't need check
    if (i == 0) {
      candidates.push_back(rawhypothesis.at(0));
      continue;
    }
    // check with candidate pool
    bool not_similar_to_pool = true;
    for (size_t j = 0; j < candidates.size(); j++) {
      PoseD pose1 = rawhypothesis.at(i).relative_pose;
      PoseD pose2 = candidates.at(j).relative_pose;
      if (abs(pose1.x() - pose2.x()) < setting.position_res &&
          abs(pose1.y() - pose2.y()) < setting.position_res &&
          abs(pose1.z() - pose2.z()) < setting.position_res &&
          abs(pose1.rotation().yaw() - pose2.rotation().yaw()) < setting.theta_res) {
        not_similar_to_pool = false;
        break;
      }
    }
    if (not_similar_to_pool)
      candidates.push_back(rawhypothesis.at(i));
  }
  return candidates;
}

/* ************************************************************************* */
// Hypothesis selection given setting
// output will be a boost::optional pointer. If not find a valid hyopthesis will be empty
HypothesisSelectResult selectHypothesis(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<PoseD>& measure_poses,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value,
    const std::vector<Hypothesis>& hypothesis,
    const HypothesisSelectSetting& setting) {


  const Vector3 sigma_inlier = setting.measlike_inlier_model->sigmas();
  const Matrix3 cov_inlier = diag(emul(sigma_inlier, sigma_inlier));
  const double log_k_inlier_single = -log(sqrt(2.0 * M_PI * cov_inlier.determinant()));
  const Vector3 sigma_outlier = setting.measlike_outlier_model->sigmas();
  const Matrix3 cov_outlier = diag(emul(sigma_outlier, sigma_outlier));
  const double log_k_outlier_single = -log(sqrt(2.0 * M_PI * cov_outlier.determinant()));

  HypothesisSelectResult result;

  vector<size_t> valid_hypothesis;
  vector<double> hypothesis_prior;

  // ===============================================================================
  // only select max_hypothesis_checked with highest inlier number to check
  if (hypothesis.size() > setting.max_hypothesis_checked) {
    vector<size_t> max_inlier_idx;
    vector<pair<size_t, size_t> > hypothesis_inlier_count;
    for (size_t i = 0; i < hypothesis.size(); i++)
      hypothesis_inlier_count.push_back(make_pair(hypothesis.at(i).nr_inlier, i));
    sort(hypothesis_inlier_count.begin(), hypothesis_inlier_count.end(), comparator_uint);
    for (size_t i = 1; i <= setting.max_hypothesis_checked; i++)
      max_inlier_idx.push_back(hypothesis_inlier_count.at(hypothesis_inlier_count.size()-i).second);

    // calculate hypothesis prior probability (only for those who have max inlier numbers)
    for (size_t i = 0; i < hypothesis.size(); i++) {
      vector<size_t>::iterator iter;
      iter = find(max_inlier_idx.begin(), max_inlier_idx.end(), i);
      if (iter != max_inlier_idx.end()) {

        // hypothesis prior
        double hprior;
        if (setting.hprior_use_CRP)
          hprior = getHypothesisPriorCRP(local_ID, agent_ID, measure_index,
              local_value, agent_value, hypothesis.at(i), setting.hprior_CRP_setting);
        else
          hprior = getHypothesisPrior(local_ID, agent_ID, measure_index,
              local_value, agent_value, hypothesis.at(i), setting);

        if (hprior > 0.0) {
          valid_hypothesis.push_back(i);
          hypothesis_prior.push_back(hprior);
        }
      }
    }

  } else {
    // not exceed max number, calculate hypothesis prior probability
    for (size_t i = 0; i < hypothesis.size(); i++) {

      // hypothesis prior
      double hprior;
      if (setting.hprior_use_CRP)
        hprior = getHypothesisPriorCRP(local_ID, agent_ID, measure_index,
            local_value, agent_value, hypothesis.at(i), setting.hprior_CRP_setting);
      else
        hprior = getHypothesisPrior(local_ID, agent_ID, measure_index,
            local_value, agent_value, hypothesis.at(i), setting);

      if (hprior > 0.0) {
        valid_hypothesis.push_back(i);
        hypothesis_prior.push_back(hprior);
      }
    }
  }
  result.valid_count = valid_hypothesis.size();
  result.valid_idx = valid_hypothesis;

  // if no one is valid, break out
  if (result.valid_count == 0) {
    result.hypothesis = boost::optional<Hypothesis>();
    return result;
  }

  // if CRP prior, need to normalize
  if (setting.hprior_use_CRP)
    normalizePrior(hypothesis_prior);

  // ===============================================================================
  // calculate transformation prior and posterior probability
  vector<double> hypothesis_log_posterior;
  for (size_t i = 0; i < valid_hypothesis.size(); i++) {
    size_t idx = valid_hypothesis.at(i);

    // transformation prior
    pair<PoseD, Matrix> tprior = getTransformationPrior(local_ID, agent_ID,
        measure_index, local_value, agent_value, hypothesis.at(idx), setting);

    // build graph to get posterior
    Key nodekey = Symbol('t', 0);
    NonlinearFactorGraph graph;

    PriorFactor<PoseD> tprior_factor(nodekey, tprior.first, noiseModel::Gaussian::Covariance(tprior.second));
    graph.push_back(tprior_factor);

    // inlier & outlier
    for (size_t j = 0; j < measure_poses.size(); j++) {

      Symbol local_symb(local_ID, measure_index.at(j).first);
      Symbol agent_symb(agent_ID, measure_index.at(j).second);
      if (agent_value.exists(agent_symb)) {
	// prepare tmp values
	Values tmp_local_value, tmp_agent_value;
	tmp_local_value.insert(local_symb, local_value.at<PoseD>(local_symb));
	tmp_agent_value.insert(agent_symb, agent_value.at<PoseD>(agent_symb));

	if (hypothesis.at(idx).inlier_vec.at(j)) {
	  graph.push_back(UnaryFactor(nodekey, measure_poses.at(j),
	      local_symb, agent_symb, tmp_local_value, tmp_agent_value,
	      setting.measlike_inlier_model));
	} else {
	  graph.push_back(UnaryFactor(nodekey, measure_poses.at(j),
	      local_symb, agent_symb, tmp_local_value, tmp_agent_value,
	      setting.measlike_outlier_model));
	}
      }
    }

    // optimize
    GaussNewtonParams opt_param;
    LevenbergMarquardtParams opt_paramL;
    Values init_value;
    init_value.insert(nodekey, hypothesis.at(idx).relative_pose);
    Values value = GaussNewtonOptimizer(graph, init_value, opt_param).optimize();
    //Values value = LevenbergMarquardtOptimizer(graph, init_value, opt_paramL).optimize();
    PoseD map_mean = value.at<PoseD>(nodekey);

    Matrix map_cov = Marginals(graph, value).marginalCovariance(nodekey);

    // generate log posterior probability
    double log_k_tprior  = - log(sqrt(2.0 * M_PI * tprior.second.determinant()));
    double log_k_inlier  = static_cast<double>(hypothesis.at(idx).nr_inlier) * log_k_inlier_single;
    double log_k_outlier = static_cast<double>(measure_poses.size() - hypothesis.at(idx).nr_inlier)
        * log_k_outlier_single;
    double log_k_mapcov  = - log(sqrt(2.0 * M_PI * map_cov.determinant()));

    double log_k = log_k_tprior + log_k_inlier + log_k_outlier + log_k_mapcov;
    double log_hprior = - log(hypothesis_prior.at(i));
    hypothesis_log_posterior.push_back(log_k + log_hprior);
    //cout << "log_k + log_hprior = " << log_k + log_hprior << endl;
  }
  result.hypothesis_prior = hypothesis_prior;
  result.post_log_prob = hypothesis_log_posterior;

  // ===============================================================================
  // check validation and make selection
  int select_hypothesis = -1;   // for none selected, put -1
  std::vector<double>::iterator maxiter = max_element(hypothesis_log_posterior.begin(), hypothesis_log_posterior.end());
  size_t maxidx = maxiter - hypothesis_log_posterior.begin();
  if (hypothesis_prior.at(maxidx) > setting.min_prior_prob)
    select_hypothesis = valid_hypothesis.at(maxidx);

  // return boost optional
  if (select_hypothesis >= 0)
    result.hypothesis = boost::optional<Hypothesis>(hypothesis.at(select_hypothesis));
  else
    result.hypothesis = boost::optional<Hypothesis>();

  return result;
}


/* ************************************************************************* */
// internal methods

// get transformation prior as PriorFactorPose2
std::pair<PoseD, gtsam::Matrix> getTransformationPrior(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value,
    const Hypothesis& hypothesis,
    const HypothesisSelectSetting& setting) {

  // check inlier > 0
  if (hypothesis.nr_inlier == 0)
    throw std::runtime_error("[ERROR:Hypothesis] hypothesis.inlier_number = 0, cannot generate trans prior");

  // add the graph
  Key nodekey = Symbol('t', 0);
  NonlinearFactorGraph graph;
  for (size_t i = 0; i < measure_index.size(); i++) {
    if (hypothesis.inlier_vec.at(i)) {

      Symbol local_symb(local_ID, measure_index.at(i).first);
      Symbol agent_symb(agent_ID, measure_index.at(i).second);
      if (agent_value.exists(agent_symb)) {
	// prepare tmp values
	Values tmp_local_value, tmp_agent_value;
	tmp_local_value.insert(local_symb, local_value.at<PoseD>(local_symb));
	tmp_agent_value.insert(agent_symb, agent_value.at<PoseD>(agent_symb));

	// add 'zero' measurement to the graph
	graph.push_back(UnaryFactor(nodekey, PoseD(),
	    local_symb, agent_symb, tmp_local_value, tmp_agent_value,
	    setting.tprior_model));
      }
    }
  }

  // MAP optimization
  GaussNewtonParams opt_param;
  //LevenbergMarquardtParams opt_paramL;
  Values init_value;
  init_value.insert(nodekey, hypothesis.relative_pose);
  Values value = GaussNewtonOptimizer(graph, init_value, opt_param).optimize();
  //Values value = LevenbergMarquardtOptimizer(graph, init_value, opt_paramL).optimize();
  PoseD pose_mean = value.at<PoseD>(nodekey);
  Matrix pose_cov = Marginals(graph, value).marginalCovariance(nodekey);

  return make_pair(pose_mean, pose_cov);
}


/* ************************************************************************* */
// get hypothesis prior for a single hypothesis
double getHypothesisPrior(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value,
    const Hypothesis& hypothesis,
    const HypothesisSelectSetting& setting) {

  // for null hypothesis just return 0
  if (hypothesis.nr_inlier == 0)
    return 0.0;

  // get trajectories from both agents
  vector<double> traj_local, traj_agent;
  map<size_t, size_t> trajidx_local, trajidx_agent;

  traj_local = generateTrajectory(local_ID, local_value, trajidx_local);
  traj_agent = generateTrajectory(agent_ID, agent_value, trajidx_agent);

  size_t nr_cell_local = static_cast<size_t>(ceil(*(traj_local.end()-1) / setting.hprior_cell_size));
  size_t nr_cell_agent = static_cast<size_t>(ceil(*(traj_agent.end()-1) / setting.hprior_cell_size));

  // get inlier's trajectories for both local and agent
  vector<double> inlier_traj_local, inlier_traj_agent;
  for (size_t i = 0; i < measure_index.size(); i++) {
    if (hypothesis.inlier_vec.at(i)) {
      inlier_traj_local.push_back(traj_local.at(trajidx_local[measure_index.at(i).first]));
      inlier_traj_agent.push_back(traj_agent.at(trajidx_agent[measure_index.at(i).second]));
    }
  }

  // use histogram to indiate non-zero cell numbers
  Histogram hist_local = histogram(inlier_traj_local, setting.hprior_cell_size);
  Histogram hist_agent = histogram(inlier_traj_agent, setting.hprior_cell_size);

  size_t nr_cell_nonzero_local = 0, nr_cell_nonzero_agent = 0;
  for (size_t i = 0; i < hist_local.count.size(); i++)
    if (hist_local.count.at(i) > 0)
      nr_cell_nonzero_local++;
  for (size_t i = 0; i < hist_agent.count.size(); i++)
    if (hist_agent.count.at(i) > 0)
      nr_cell_nonzero_agent++;

  return min(static_cast<double>(nr_cell_nonzero_local)/static_cast<double>(nr_cell_local),
      static_cast<double>(nr_cell_nonzero_agent)/static_cast<double>(nr_cell_agent));
}


/* ************************************************************************* */
// get 1-D trajectory
std::vector<double> generateTrajectory(unsigned char ID, const gtsam::Values& values,
    std::map<size_t, size_t>& value_to_traj_idx) {

  //values.print();

  value_to_traj_idx.clear();
  vector<double> trajvec;

  // get the key list
  // TODO: maybe there's method that don't need key list and directly use Values::const_iterator
  const vector<Key> keylist = std::vector<Key>(values.keys());

  // use a iterator to scan the value
  PoseD pose, last_pose;
  size_t count = 0;
  double curr_mileage = 0.0;
  for (vector<Key>::const_iterator iter = keylist.begin(); iter != keylist.end(); iter++) {

    // get pose value
    Key key = *iter;
    pose = values.at<PoseD>(key);
    Symbol symbol(*iter);
    //symbol.print();

    // chech the first pose
    if (iter == keylist.begin()) {
      if (symbol.index() != 0) {
        // pose 1 missing
        cerr << "[ERROR:Hypothesis] first pose of robot '" << ID << "' is missing, ";
        cerr << "may have packages missing at begining. ";
        cerr << "Trajectory will start from pose ";
        symbol.print();
      }
      // perpare tmp vat for first
      last_pose = pose;
    }

    // trajectory
    //curr_mileage += (pose.t() - last_pose.t()).norm();
    curr_mileage += (pose.translation() - last_pose.translation()).norm();
    trajvec.push_back(curr_mileage);
    // index
    value_to_traj_idx[symbol.index()] = count;
    // prepare for next
    last_pose = pose;
    count++;
  }

  return trajvec;
}

/* ************************************************************************* */
// utils
void Hypothesis::print() const {
  cout << "Relative pose: ";
  relative_pose.print();
  cout << "Inlier count: " << nr_inlier << "/" << inlier_vec.size() << endl;
}

void HypothesisSelectResult::print() const {
  cout << "Select hypothesis: ";
  if (hypothesis) {
    cout << endl << "=========================================" << endl;
    hypothesis->print();
    cout << "=========================================" << endl;
  } else {
    cout << "NULL" << endl;
  }

  cout << "Valid Hypothesis count = " << valid_count << " : ";
  if (valid_count > 0) {
    cout << valid_idx.at(0);
    for (size_t i = 1; i < valid_count; i++)
      cout << "/" << valid_idx.at(i);
    cout << endl;
  }

  cout << "Hypothesis prior:" << endl;
  for (size_t i = 0; i < valid_count; i++)
    cout << "  " << hypothesis_prior.at(i) << endl;

  cout << "Hypothesis log posterior:" << endl;
  for (size_t i = 0; i < valid_count; i++)
    cout << "  " << post_log_prob.at(i) << endl;
}

}   // namespace mast
