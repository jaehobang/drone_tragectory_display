/**
 * @file   ICPMatcher.h
 * @brief  a 2D wrapper class to call libpointmatcher ICP matcher
 * @author Jing Dong
 * @date   Jun 19, 2014
 */

#pragma once

#include <joint_mapping/Agent_Definitions.h>
#include <joint_mapping/Settings.h>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>

#include <pointmatcher/PointMatcher.h>

#include <eigen3/Eigen/Dense>


#include <vector>

namespace comap {

struct MatchResult {

  bool status;
  double inlier_ratio;
  PoseD delta_pose;
  Eigen::MatrixXd cov;

  MatchResult() : status(false), inlier_ratio(0.0), delta_pose(), cov() {}
  virtual ~MatchResult() {}
};


/* ************************************************************************* */
// class load data from txt file, load for a single robot
class ICPMatcher {

private:

  typedef PointMatcher<double> PMatcher;
  ICPParameter param_;
  PMatcher::ICP icp_;

public:
  // constrcutor with default setting
  ICPMatcher() : icp_() {
    icp_.setDefault();
    param_.sensor_sdv = 0.01;
  }
  // constructor with yaml file configure input
  ICPMatcher(std::string filename, ICPParameter param);
  // constructor from PointMatcher setting
  ICPMatcher(const PointMatcherSetting& setting);

  // deconstructor
  virtual ~ICPMatcher() {}


  // return a relative pose2, using boost optional class in case of failure
  // with optional initial pose estimation value input
  MatchResult matchPointClouds(const std::vector<gtsam::Point2> &query,
      const std::vector<gtsam::Point2> &ref, const gtsam::Pose2 &initpose = gtsam::Pose2());

  // faster mode: if batch meory copy possible, call this to save time dealing with memory
  // Be notice that we use homo 2D corrdinate here
  MatchResult matchPointClouds(const Eigen::MatrixXd &queryMat,
      const Eigen::MatrixXd &refMat, const gtsam::Pose2 &initpose = gtsam::Pose2());

private:
  // compute covariance, using input format from errorminizer
  Eigen::Matrix3d estimateCovariance(
      const PMatcher::DataPoints& reading,
      const PMatcher::DataPoints& reference,
      const PMatcher::Matches& matches,
      const PMatcher::OutlierWeights& outlierWeights,
      const PMatcher::TransformationParameters& trans);
};

}  // namespace comap
