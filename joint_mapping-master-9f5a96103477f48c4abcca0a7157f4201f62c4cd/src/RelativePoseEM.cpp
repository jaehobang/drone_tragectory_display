/*
 * @file RelativePoseEM.cpp
 * @brief use EM to optimize relative pose hypothesis
 * @author Jing Dong
 * @date July 8, 2014
 */

#include <joint_mapping/RelativePoseEM.h>


#include <iostream>

using namespace std;
using namespace gtsam;

namespace comap {

// short for type between robot factor
typedef TransformBtwRobotsUnaryFactor<PoseD> UnaryFactor;
typedef TransformBtwRobotsUnaryFactorEM<PoseD> UnaryFactorEM;

/* ************************************************************************* */
// perform EM optimization to given clustered initial poses, output hypothesis (not merge hypothesis)
std::vector<Hypothesis> performRelativePoseEM(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<PoseD>& measure_poses,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const std::vector<PoseD>& initial,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value,
    const RelativePoseEMSetting& setting) {


  size_t nr_measure = measure_poses.size();
  size_t nr_initial = initial.size();

  // prepare EM and normal factor graph: from all data
  Key nodekey = Symbol('t', 0);
  NonlinearFactorGraph graph_EM, graph_normal;
  for (size_t i = 0; i < nr_measure; i++) {

    Symbol local_symb(local_ID, measure_index.at(i).first);
    Symbol agent_symb(agent_ID, measure_index.at(i).second);

    if (agent_value.exists(agent_symb)) {
      // prepare tmp values
      Values tmp_local_value, tmp_agent_value;
      tmp_local_value.insert(local_symb, local_value.at<PoseD>(local_symb));
      tmp_agent_value.insert(agent_symb, agent_value.at<PoseD>(agent_symb));

      // insert normal graph
      graph_normal.push_back(UnaryFactor(nodekey, measure_poses.at(i),
	  local_symb, agent_symb, tmp_local_value, tmp_agent_value,
	  setting.inlier_model));

      // insert EM graph
      graph_EM.push_back(UnaryFactorEM(nodekey, measure_poses.at(i),
	  local_symb, agent_symb, tmp_local_value, tmp_agent_value,
	  setting.inlier_model, setting.outlier_model,
	  setting.inlier_prior, 1.0 - setting.inlier_prior,
	  setting.flag_bump_at_zero));
    }
  }

  // for each initial guess, generate the hypothesis
  vector<Hypothesis> hypothesis_vec;

  for (size_t i = 0; i < nr_initial; i++) {

    initial.at(i).print("The clustered rel pose considered is =");

    // 1. perform EM to initial clustered result
    GaussNewtonParams opt_param;
    LevenbergMarquardtParams opt_paramL;
    Values init_value;
    init_value.insert(nodekey, initial.at(i));
    Values value = GaussNewtonOptimizer(graph_EM, init_value, opt_param).optimize();
    //Values value = LevenbergMarquardtOptimizer(graph_EM, init_value, opt_paramL).optimize();
    PoseD initial_EM = value.at<PoseD>(nodekey);

    // 2. identify inlier and outlier
    vector<bool> hypothesis_inlier_vec;
    size_t inlier_count = 0;

    for (size_t j = 0; j < nr_measure; j++) {
      UnaryFactorEM::shared_ptr factor = boost::dynamic_pointer_cast<UnaryFactorEM>(graph_EM.at(j));
      if (factor && factor->calcIndicatorProb(value)(0) > setting.min_inlier_prob) {
        hypothesis_inlier_vec.push_back(true);
        inlier_count++;
      } else {
        if (factor) {
          std::cout << "Not inlier cause factor prob was " << factor->calcIndicatorProb(value)(0) << std::endl;
        }
        hypothesis_inlier_vec.push_back(false);
      }
    }

    ROS_INFO("The inlier count %zu ", inlier_count);


    // only accept the hypothesis with more than
    if (inlier_count < setting.min_inlier_count)
      continue;

    // 3. only pick up inliers, and use normal optimization to refine the result
    // generate inlier-only normal graph
    if (setting.flag_reoptimize_inlier_only) {
      NonlinearFactorGraph graph_normal_inlier;
      for (size_t j = 0; j < nr_measure; j++) {
        if (hypothesis_inlier_vec.at(j))
          graph_normal_inlier.push_back(graph_normal.at(j));
      }

      Values init_value_inlier;
      init_value_inlier.insert(nodekey, initial_EM);
      //value = LevenbergMarquardtOptimizer(graph_normal_inlier, init_value_inlier, opt_paramL).optimize();
      value = GaussNewtonOptimizer(graph_normal_inlier, init_value_inlier, opt_param).optimize();
    }

    // generate the hypothesis
    Hypothesis hypothesis;
    hypothesis.relative_pose = value.at<PoseD>(nodekey);
    hypothesis.inlier_vec = hypothesis_inlier_vec;
    hypothesis.nr_inlier = inlier_count;

    hypothesis_vec.push_back(hypothesis);
  }

  return hypothesis_vec;
}


}   // namespace mast
