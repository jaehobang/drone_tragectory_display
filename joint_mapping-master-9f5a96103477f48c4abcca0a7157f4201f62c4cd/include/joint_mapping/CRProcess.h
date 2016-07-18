/*
 * @file CRProcess.h
 * @brief hypothesis prior by chinese restaurant process 
 * @author Jing Dong
 * @date Sep 18, 2014
 */

#pragma once

#include <gtsam/nonlinear/Values.h>

#include <vector>

namespace comap {

class Hypothesis;
class HypothesisSelectSetting;

/* ************************************************************************* */
// settings

struct CRPSetting {

  // whether use grid based method
  bool flag_grid;
  // grid size
  double grid_size;
  // alpha value
  double alpha;

  // default setting
  CRPSetting() :
    flag_grid(false),
    grid_size(1.0),
    alpha(300) {}

  virtual ~CRPSetting() {}
};

/* ************************************************************************* */
// methods

// get hypothesis prior for a single hypothesis
double getHypothesisPriorCRP(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value,
    const Hypothesis& hypothesis,
    const CRPSetting& setting);

// given unique place number, output probability
double CRProcessFinalProb(size_t place_count, double alpha);

// normalized CRP prior (in place)
void normalizePrior(std::vector<double>& hprior);

}   // namespace mast
