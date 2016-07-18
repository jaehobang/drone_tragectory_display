/*
 * @file CRProcess.cpp
 * @brief hypothesis prior by chinese restaurant process
 * @author Jing Dong
 * @date Sep 18, 2014
 */

#include <joint_mapping/CRProcess.h>
#include <joint_mapping/Hypothesis.h>

#include <numeric>

using namespace std;
using namespace gtsam;

namespace comap {

/* ************************************************************************* */
// get hypothesis prior for a single hypothesis
// TODO: cell-count version
double getHypothesisPriorCRP(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value,
    const Hypothesis& hypothesis,
    const CRPSetting& setting) {

  // pose-count version
  size_t place_count = 2 * hypothesis.inlier_vec.size() - hypothesis.nr_inlier;
  return CRProcessFinalProb(place_count, setting.alpha);
}

/* ************************************************************************* */
// given unique place number, output probability
// see vadim's journal paper
double CRProcessFinalProb(size_t place_count, double alpha) {
  double final_prob = 1.0;
  for (size_t i = 1; i <= place_count; i++)
    final_prob = final_prob * (alpha / (alpha + static_cast<double>(place_count - 1)));
  return final_prob;
}

/* ************************************************************************* */
// normalized CRP prior (in place)
void normalizePrior(std::vector<double>& hprior) {
  double sum = accumulate(hprior.begin(), hprior.end(), 0.0);
  for (size_t i = 0; i < hprior.size(); i++)
    hprior[i] = hprior[i] / sum;
}

}   // namespace mast
