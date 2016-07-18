/**
 * @file   Saliency_impl.h
 * @brief  calculate Scan Saliency (template implementation)
 * @author Jing Dong
 * @date   Jun 30, 2014
 */

#pragma once

#include <gtsam/geometry/Pose2.h>

#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>

#include <vector>
#include <iostream>

namespace comap {

/* ************************************************************************* */
// calculate saliency
template <class MATCHER>
double ScanSaliency<MATCHER>::calculateScanSaliency(const Eigen::MatrixXd& scan) {

  // preparation
  boost::normal_distribution<double> normalDistrib(0.0, 1.0);
  boost::variate_generator<boost::mt19937, boost::normal_distribution<double> >
    randnum(random_gen_, normalDistrib);

  // run ntimes_ ICP matching use random init pose
  std::vector<Eigen::Vector3d> trans_results;
  std::vector<Eigen::Matrix3d> cov_results;
  for (unsigned int i = 0; i < setting_.ntimes; i++) {

    // generate random init pose
    gtsam::Pose2 initpose(randnum()*setting_.sigma_t, randnum()*setting_.sigma_t,
        randnum()*setting_.sigma_theta);

    // perform ICP
    MatchResult result = matcher_.matchPointClouds(scan, scan, initpose);

    if (!result.status)
      continue;

    Eigen::Vector3d xvec(3);
    xvec << result.delta_pose.x(), result.delta_pose.y(), result.delta_pose.theta();

    trans_results.push_back(xvec);
    cov_results.push_back(result.cov);
  }

  // get x mean
  Eigen::Vector3d xmean = Eigen::MatrixXd::Zero(3,1);
  for (size_t i = 0; i < trans_results.size(); i++)
    xmean = xmean + trans_results.at(i);
  xmean = xmean / static_cast<double>(trans_results.size());

  // remove mean from trans_results
  for (size_t i = 0; i < trans_results.size(); i++)
    trans_results.at(i) = trans_results.at(i) - xmean;

  // get Saliency
  Eigen::Matrix3d R = Eigen::MatrixXd::Zero(3,3);
  for (size_t i = 0; i < trans_results.size(); i++)
    R = R + cov_results.at(i) + trans_results.at(i) * trans_results.at(i).transpose();
  R = R / static_cast<double>(trans_results.size());

  return 1.0 / R.trace();
}

}   // namespace mast
