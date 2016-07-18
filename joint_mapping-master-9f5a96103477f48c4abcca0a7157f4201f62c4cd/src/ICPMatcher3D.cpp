/**
 * @file   ICPMatcher.cpp
 * @brief  a 3D wrapper class to call libpointmatcher ICP matcher
 * @author Vibhav Ganesh
 * @date   Feb 9. 2016
 */

#include <joint_mapping/ICPMatcher3D.h>
#include <joint_mapping/point_cloud.h>

#include <cmath>
#include <iostream>
#include <fstream>

#define _USE_MATH_DEFINES

using namespace std;
using namespace gtsam;

namespace comap {

/* ************************************************************************* */
// constructor from PointMatcher setting
ICPMatcher::ICPMatcher(const PointMatcherSetting& setting) : icp_() {

  // param
  param_ = ICPParameter(setting.sensor_sdv);
  // default setting
  icp_.setDefault();

  /*
  // clear up not used default setting
  icp_.readingDataPointsFilters.clear();
  icp_.readingStepDataPointsFilters.clear();
  icp_.outlierFilters.clear();
  icp_.transformationCheckers.clear();

  // read data filter
  if (!setting.read_maxdens_flag) {
    // not use max dens
    PMatcher::DataPointsFilter* randfilter =
        PMatcher::get().DataPointsFilterRegistrar.create(
        "RandomSamplingDataPointsFilter",
        PointMatcherSupport::map_list_of
        ("prob", PointMatcherSupport::toParam(setting.read_randsample_ratio)));

    icp_.readingDataPointsFilters.push_back(randfilter);

  } else {
    // use max dens
    PMatcher::DataPointsFilter* samplefilter =
        PMatcher::get().DataPointsFilterRegistrar.create(
        "SamplingSurfaceNormalDataPointsFilter",
        PointMatcherSupport::map_list_of
        ("keepNormals", "0")
        ("keepDensities", "1"));
    PMatcher::DataPointsFilter* densfilter =
        PMatcher::get().DataPointsFilterRegistrar.create(
        "MaxDensityDataPointsFilter",
        PointMatcherSupport::map_list_of
        ("maxDensity", PointMatcherSupport::toParam(setting.read_maxdens_density)));

    icp_.readingDataPointsFilters.push_back(samplefilter);
    icp_.readingDataPointsFilters.push_back(densfilter);
  }

  if (setting.read_maxdist_flag) {
    // use max distance filter
    PMatcher::DataPointsFilter* distfilter =
        PMatcher::get().DataPointsFilterRegistrar.create(
        "MaxDistDataPointsFilter",
        PointMatcherSupport::map_list_of
        ("maxDist", PointMatcherSupport::toParam(setting.read_maxdist_dist)));

    icp_.readingDataPointsFilters.push_back(distfilter);
  }

  // reference

  // outlier filter
  PMatcher::OutlierFilter* trimoutlierfilter =
      PMatcher::get().OutlierFilterRegistrar.create(
      "TrimmedDistOutlierFilter",
      PointMatcherSupport::map_list_of
      ("ratio", PointMatcherSupport::toParam(setting.outlier_trim_ratio)));

  icp_.outlierFilters.push_back(trimoutlierfilter);

  if (setting.outlier_maxdist_flag) {
    // use max dist outlier filter
    PMatcher::OutlierFilter* maxdistoutlierfilter =
        PMatcher::get().OutlierFilterRegistrar.create(
        "MaxDistOutlierFilter",
        PointMatcherSupport::map_list_of
        ("maxDist", PointMatcherSupport::toParam(setting.outlier_maxdist_dist)));

    icp_.outlierFilters.push_back(maxdistoutlierfilter);
  }

  // transformation checkers
  PMatcher::TransformationChecker* iterchecker =
      PMatcher::get().TransformationCheckerRegistrar.create(
      "CounterTransformationChecker",
      PointMatcherSupport::map_list_of
      ("maxIterationCount", PointMatcherSupport::toParam(setting.checker_maxiter)));
  PMatcher::TransformationChecker* diffchecker =
      PMatcher::get().TransformationCheckerRegistrar.create(
      "DifferentialTransformationChecker",
      PointMatcherSupport::map_list_of
      ("minDiffTransErr", PointMatcherSupport::toParam(setting.checker_diff_trans))
      ("minDiffRotErr", PointMatcherSupport::toParam(setting.checker_diff_rot)));

  icp_.transformationCheckers.push_back(iterchecker);
  icp_.transformationCheckers.push_back(diffchecker);
  */
}

// constructor with file configure input
ICPMatcher::ICPMatcher(std::string filename, ICPParameter param) : param_(param), icp_() {

  ifstream ifs(filename.c_str());

  // check whether config file opened
  if (!ifs) {
    // cannot read
    cerr << "[WARNING:ICPMatcher] cannot load config file: " << filename;
    cerr << ", will use default ICP setting" << endl;
    icp_.setDefault();

  } else {
    icp_.loadFromYaml(ifs);;
  }
}


/* ************************************************************************* */
// faster mode: if batch meory copy possible, call this to save time dealing with memory
MatchResult ICPMatcher::matchPointClouds(const sensor_msgs::PointCloud2& query_cloud
    const sesor_msgs &ref_cloud, const gtsam::Pose3 &initpose) {

  MatchResult result;

  const PMatcher::DataPoints query = PointMatcher_ros::rosMsgToPointMatcherCloud(query_cloud);
  const PMatcher::DataPoints ref = PointMatcher_ros::rosMsgToPointMatcherCloud(ref_cloud);;

  // perform icp and catch possible converge error
  PMatcher::TransformationParameters trans;
  try {
    trans = icp_(query, ref, initpose.matrix());

    // check whether reach iteration
    if ((*icp_.transformationCheckers.begin())->getConditionVariables()[0] >=
        (*icp_.transformationCheckers.begin())->getLimits()[0]) {
      result.status = false;
      return result;
    }

  } catch (PMatcher::ConvergenceError error) {
    // catch error, cannot match
    result.status = false;
    return result;
  }


  // prepare the result strcuture
  result.status = true;
  result.delta_pose = Pose3(trans);
  result.inlier_ratio = icp_.errorMinimizer->getPointUsedRatio();
  result.cov = icp_.errorMinimzer->getCovariance();

  return result;
}

}  // namespace comap
