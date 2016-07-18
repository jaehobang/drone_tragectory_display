/*
 * @file SmallEM.cpp
 * @brief Small EM: use EM to identify inlier/outlier loop closure
 * @author Jing Dong
 * @date Sep 9, 2014
 */

#include <joint_mapping/SmallEM.h>

#include <boost/pointer_cast.hpp>

#include <iostream>

using namespace std;
using namespace gtsam;

namespace comap {

// short for type between robot factor
typedef TransformBtwRobotsUnaryFactorEM<PoseD> UnaryFactorEM;

/* ************************************************************************* */
// perform Small EM optimization to indicate inlier/outlier loop closure
// TODO: add model update
Hypothesis performSmallEM(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<PoseD>& measure_poses,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const PoseD& initial,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value,
    const gtsam::NonlinearFactorGraph& graph,
    const SmallEMSetting& setting) {

  size_t nr_measure = measure_poses.size();
  Key nodekey = Symbol('t', 0);
  NonlinearFactorGraph graph_EM;

  for (size_t i = 0; i < nr_measure; i++) {

    Symbol local_symb(local_ID, measure_index.at(i).first);
    Symbol agent_symb(agent_ID, measure_index.at(i).second);
    // prepare tmp values: speed up EM factor
    Values tmp_local_value, tmp_agent_value;
    tmp_local_value.insert(local_symb, local_value.at<PoseD>(local_symb));
    tmp_agent_value.insert(agent_symb, agent_value.at<PoseD>(agent_symb));

    // insert EM graph
    graph_EM.push_back(UnaryFactorEM(nodekey, measure_poses.at(i),
        local_symb, agent_symb, tmp_local_value, tmp_agent_value,
        setting.inlier_model, setting.outlier_model,
        setting.inlier_prior, 1.0 - setting.inlier_prior, false));
  }

  // 1. perform EM to initial value
  //LevenbergMarquardtParams opt_paramL;
  GaussNewtonParams opt_param;
  Values init_value;
  init_value.insert(nodekey, initial);
  Values value = GaussNewtonOptimizer(graph_EM, init_value, opt_param).optimize();
  //Values value = LevenbergMarquardtOptimizer(graph_EM, init_value, opt_paramL).optimize();

  // 2. identify inlier and outlier
  vector<bool> hypothesis_inlier_vec;
  size_t inlier_count = 0;

  for (size_t j = 0; j < nr_measure; j++) {
    UnaryFactorEM::shared_ptr factor = boost::dynamic_pointer_cast<UnaryFactorEM>(graph_EM.at(j));
    if (factor && factor->calcIndicatorProb(value)(0) > setting.min_inlier_prob) {
      hypothesis_inlier_vec.push_back(true);
      inlier_count++;
    } else {
      hypothesis_inlier_vec.push_back(false);
    }
  }

  // 3. generate the hypothesis to return
  Hypothesis hypothesis;
  hypothesis.relative_pose = value.at<PoseD>(nodekey);
  hypothesis.inlier_vec = hypothesis_inlier_vec;
  hypothesis.nr_inlier = inlier_count;

  return hypothesis;
}

/* ************************************************************************* */
// class based small EM
// init the small EM when relative pose is built
void SmallEM::init(
    const std::vector<PoseD>& measure_poses,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const PoseD& relpose) {

  // just cache the values
  measure_poses_ = measure_poses;
  measure_index_ = measure_index;
  relpose_ = relpose;
  flag_init = true;
}

/* ************************************************************************* */
// perfrom small EM, give whether it's inlier/outlier
bool SmallEM::perform(
    const PoseD& measure_pose,
    const std::pair<size_t, size_t>& measure_index,
    const gtsam::Values& values,
    const gtsam::NonlinearFactorGraph& graph) {

  // only run after init
  if (!flag_init)
    throw std::runtime_error("SmallEM started without initialization");

  // insert measurement
  measure_poses_.push_back(measure_pose);
  measure_index_.push_back(measure_index);

#if SMALLEM_UPDATE_INLIER_MODEL

  // prepare marginals if update noise model
  Marginals marginals(graph, values);

#endif

  // graph need to be re-initialize since the values are changed
  // measuremens insert in the graph
  size_t nr_measure = measure_poses_.size();
  Key nodekey = Symbol('t', 0);
  NonlinearFactorGraph graph_EM;

  for (size_t i = 0; i < nr_measure; i++) {

    Symbol local_symb(local_ID_, measure_index_.at(i).first);
    Symbol agent_symb(agent_ID_, measure_index_.at(i).second);
    // prepare tmp values: speed up EM factor
    Values tmp_local_value, tmp_agent_value;
    tmp_local_value.insert(local_symb, values.at<PoseD>(local_symb));
    tmp_agent_value.insert(agent_symb, values.at<PoseD>(agent_symb));

    // have a factor
    UnaryFactorEM::shared_ptr factor(new UnaryFactorEM(nodekey, measure_poses_.at(i),
        local_symb, agent_symb, tmp_local_value, tmp_agent_value,
        setting_.inlier_model, setting_.outlier_model,
        setting_.inlier_prior, 1.0 - setting_.inlier_prior, false));

#if SMALLEM_UPDATE_INLIER_MODEL

    // update noise model
    if (setting_.flag_update_model) {
      //cout << "updating small EM inlier model ..." << endl;
      factor->updateNoiseModels(values, marginals);
    }

#endif

    // insert EM graph
    graph_EM.push_back(factor);
  }



  // perform EM to initial value
  GaussNewtonParams opt_param;
  //LevenbergMarquardtParams opt_paramL;
  Values init_value;
  init_value.insert(nodekey, PoseD());
  Values value = GaussNewtonOptimizer(graph_EM, init_value, opt_param).optimize();
  //Values value = LevenbergMarquardtOptimizer(graph_EM, init_value, opt_paramL).optimize();
  relpose_ = value.at<PoseD>(nodekey);

  // these could be turn off for max speed
#if SMALLEM_OUTPUT_INLIER_MODEL
  // cache inlier vec if used
  inlier_vec_.clear();
  for (size_t j = 0; j < nr_measure; j++) {
    UnaryFactorEM::shared_ptr factor = boost::dynamic_pointer_cast<UnaryFactorEM>(graph_EM.at(j));
    if (factor && factor->calcIndicatorProb(value)(0) > setting_.min_inlier_prob)
      inlier_vec_.push_back(true);
    else
      inlier_vec_.push_back(false);
  }

  // compute relposes from measurement
  measure_relposes_.clear();
  for (size_t i = 0; i < nr_measure; i++) {
    Symbol local_symb(local_ID_, measure_index_.at(i).first);
    Symbol agent_symb(agent_ID_, measure_index_.at(i).second);
    PoseD orgA_T_currA = values.at<PoseD>(local_symb);
    PoseD orgB_T_currB = values.at<PoseD>(agent_symb);
    PoseD orgA_T_orgB = orgA_T_currA * measure_poses_.at(i) * (orgB_T_currB.inverse());
    measure_relposes_.push_back(orgA_T_orgB);
  }

  // output inlier model for vis
  inlier_model_cov_.clear();
  for (size_t j = 0; j < nr_measure; j++) {
    UnaryFactorEM::shared_ptr factor = boost::dynamic_pointer_cast<UnaryFactorEM>(graph_EM.at(j));
    inlier_model_cov_.push_back(factor->get_model_inlier_cov());
  }
#endif

  // not need inlier vector: just need the last one
  UnaryFactorEM::shared_ptr factor = boost::dynamic_pointer_cast<UnaryFactorEM>(graph_EM.at(nr_measure-1));
  if (factor && factor->calcIndicatorProb(value)(0) > setting_.min_inlier_prob)
    return true;
  else
    return false;
}

}   // namespace mast
