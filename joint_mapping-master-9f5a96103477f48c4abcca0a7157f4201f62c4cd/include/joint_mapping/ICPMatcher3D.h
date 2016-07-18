/**
 * @file   ICPMatcher3D.h
 * @brief  a 3D wrapper class to call libpointmatcher ICP matcher
 * @author Vibhav Ganesh
 * @date   Feb 7
 */

#pragma once

#include <joint_mapping/Agent_Definitions.h>
#include <joint_mapping/Settings.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>

#include <pointmatcher/PointMatcher.h>

#include <eigen3/Eigen/Dense>


#include <vector>

namespace comap {

struct MatchResult {

  bool status;
  double inlier_ratio;
  gtsam::Pose3 delta_pose;
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

  // faster mode: if batch meory copy possible, call this to save time dealing with memory
  // Be notice that we use homo 2D corrdinate here
  MatchResult matchPointClouds(const Eigen::MatrixXd &queryMat,
      const Eigen::MatrixXd &refMat, const gtsam::Pose3 &initpose = gtsam::Pose3());

};

}  // namespace comap
