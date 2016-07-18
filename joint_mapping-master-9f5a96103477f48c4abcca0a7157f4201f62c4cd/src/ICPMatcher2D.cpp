/**
 * @file   ICPMatcher.cpp
 * @brief  a 2D wrapper class to call libpointmatcher ICP matcher
 * @author Jing Dong
 * @date   Jun 19, 2014
 */

#include <joint_mapping/ICPMatcher2D.h>

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
// return a relative pose2, using boost optional class in case of failure
MatchResult ICPMatcher::matchPointClouds(const std::vector<gtsam::Point2> &query,
    const std::vector<gtsam::Point2> &ref, const gtsam::Pose2 &initpose) {

  MatchResult result;

  // check input data
  if (query.size() != ref.size())
    cerr << "[WARNING:ICPMatcher] input query and reference should have same length" << endl;

  // transfer GTSAM Point2 objects to Eigen Matrix input
  Eigen::Matrix<double, 3, -1> queryMat(3, query.size()), refMat(3, query.size());
  for (size_t idx = 0; idx < query.size(); idx++) {
    // trans query
    queryMat(0, idx) = query.at(idx).x();
    queryMat(1, idx) = query.at(idx).y();
    queryMat(2, idx) = 1;
    // trnas ref
    refMat(0, idx) = ref.at(idx).x();
    refMat(1, idx) = ref.at(idx).y();
    refMat(2, idx) = 1;
  }

  return this->matchPointClouds(queryMat, refMat, initpose);
}

/* ************************************************************************* */
// faster mode: if batch meory copy possible, call this to save time dealing with memory
MatchResult ICPMatcher::matchPointClouds(const Eigen::MatrixXd &queryMat,
    const Eigen::MatrixXd &refMat, const gtsam::Pose2 &initpose) {

  MatchResult result;

  // pipe data in
  PMatcher::DataPoints::Labels label;
  label.push_back(PMatcher::DataPoints::Label("x",1));
  label.push_back(PMatcher::DataPoints::Label("y",1));
  label.push_back(PMatcher::DataPoints::Label("pad",1));

  const PMatcher::DataPoints query(queryMat, label);
  const PMatcher::DataPoints ref(refMat, label);

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


  // compute covariance, prepare all things needed
  Eigen::Matrix3d cov;
  // filtered point cloud
  PMatcher::DataPoints query_after_filter(query), ref_after_filter(ref);
  icp_.readingDataPointsFilters.apply(query_after_filter);
  icp_.referenceDataPointsFilters.apply(ref_after_filter);
  // apply
  PMatcher::DataPoints stepReading(query_after_filter);
  icp_.matcher->init(ref_after_filter);
  icp_.transformations.apply(stepReading, trans);

  // matches
  PMatcher::Matches matches = icp_.matcher->findClosests(stepReading);
  // outlier weight
  PMatcher::OutlierWeights outlier_weight =
      icp_.outlierFilters.compute(stepReading, ref_after_filter, matches);

  cov = this->estimateCovariance(query_after_filter, ref_after_filter, matches,
      outlier_weight, trans);


  // prepare the result strcuture
  result.status = true;
  result.delta_pose = Pose2(trans);
  result.inlier_ratio = icp_.errorMinimizer->getPointUsedRatio();
  result.cov = cov;

  return result;
}

/* ************************************************************************* */
// compute covariance, using input format from errorminizer
Eigen::Matrix3d ICPMatcher::estimateCovariance(
    const PMatcher::DataPoints& reading,
    const PMatcher::DataPoints& reference,
    const PMatcher::Matches& matches,
    const PMatcher::OutlierWeights& outlierWeights,
    const PMatcher::TransformationParameters& trans) {

  // prepare vars
  size_t max_nr_point = outlierWeights.cols();

  Eigen::Matrix3d d2J_dx2 = Eigen::MatrixXd::Zero(3,3); // Hessian
  Eigen::MatrixXd d2J_dxdz_reading = Eigen::MatrixXd::Zero(3,max_nr_point); // Jacobian
  Eigen::MatrixXd d2J_dxdz_reference = Eigen::MatrixXd::Zero(3,max_nr_point); // Jacobian
  Eigen::Vector2d reading_point(2);
  Eigen::Vector2d reference_point(2);
  Eigen::Vector2d normal(2);
  Eigen::Vector2d reading_direction(2);
  Eigen::Vector2d reference_direction(2);

  // normals of ref
  Eigen::MatrixXd normals = reference.getDescriptorCopyByName("normals");

  if (normals.rows() < 2)    // Make sure there are normals in DataPoints
    throw std::runtime_error("[ERROR:ICPMatcher] normals of reference not exist");

  Pose2 trans_pose = Pose2(trans);
  double theta = trans_pose.theta();
  double x = trans_pose.x();
  double y = trans_pose.y();

  size_t valid_points_count = 0;

  // calculate each point
  for (size_t i = 0; i < max_nr_point; ++i) {

    // only select inlier point
    if (outlierWeights(0, i) > 0.0) {

      // prepare data
      reading_point = reading.features.block<2, 1>(0, i);
      //cout << reading_point << endl;
      int reference_idx = matches.ids(0, i);
      reference_point = reference.features.block<2, 1>(0, reference_idx);
      //cout << reference_point << endl;

      normal = normals.block<2, 1>(0, reference_idx);
      //cout << normal << endl;
      //normal = normal/normal.norm();

      double reading_range = reading_point.norm();
      reading_direction = reading_point / reading_range;
      double reference_range = reference_point.norm();
      reference_direction = reference_point / reference_range;

      // the covariance paper gives the following:
      // cov(x) = inv(d2J_dx2) * d2J_dxdz * cov(z) * d2J_dxdz' * inv(d2J_dx2)
      // cov(z) is diagonal so process later
      Eigen::Matrix3d tmp_mat3(3,3);
      Eigen::Vector3d tmp_vec3(3);

      // 1. d2J_dx2
      tmp_mat3(0,0) = 2.0 * normal(0)*normal(0);
      tmp_mat3(1,1) = 2.0 * normal(1)*normal(1);
      tmp_mat3(1,0) = 2.0 * normal(0)*normal(1);  tmp_mat3(0,1) = tmp_mat3(1,0);
      double a_cvx_a_svy = cos(reference_direction(0)) + sin(reference_direction(1));
      double m_svx_a_cvy = -sin(reference_direction(0)) + cos(reference_direction(1));
      double tmp1 = reference_range * (normal(1)*m_svx_a_cvy + normal(0)*a_cvx_a_svy);
      tmp_mat3(2,0) = 2.0 * normal(0) * tmp1;
      tmp_mat3(2,1) = 2.0 * normal(1) * tmp1;
      tmp_mat3(0,2) = tmp_mat3(2,0);  tmp_mat3(1,2) = tmp_mat3(2,1);
      tmp_mat3(2,2) = 2.0 * tmp1 * tmp1;

      d2J_dx2 += tmp_mat3;

      // d2J_dxdz_reading
      tmp1 = -normal(0)*reading_direction(0) - normal(1)*reading_direction(1);
      tmp_vec3(0) = 2.0 * normal(0) * tmp1;
      tmp_vec3(1) = 2.0 * normal(1) * tmp1;
      tmp_vec3(2) = 2.0 * tmp1 *reference_range * (normal(1)*m_svx_a_cvy + normal(0)*a_cvx_a_svy);

      d2J_dxdz_reading.block(0,valid_points_count,3,1) = tmp_vec3;


      // d2J_dxdz_reference
      double a_ctvx_a_stvy = cos(theta)*reference_direction(0) + sin(theta)*reference_direction(1);
      double m_stvx_a_ctvy = -sin(theta)*reference_direction(0) + cos(theta)*reference_direction(1);
      tmp1 = normal(1)*m_stvx_a_ctvy + normal(0)*a_ctvx_a_stvy;
      tmp_vec3(0) = 2.0 * normal(0) * tmp1;
      tmp_vec3(1) = 2.0 * normal(1) * tmp1;
      double tmp2 = normal(1)*m_svx_a_cvy + normal(0)*a_cvx_a_svy;
      tmp_vec3(2) = 2.0*reference_range * tmp2 * tmp1
          + 2.0*tmp2 * (normal(0)*(reference_range*a_ctvx_a_stvy - reading_range*reading_direction(0) + x)
              + normal(1)*(reference_range*m_stvx_a_ctvy - reading_range*reading_direction(1) + y));

      d2J_dxdz_reference.block(0,valid_points_count,3,1) = tmp_vec3;

      // valid counter
      valid_points_count++;
    }
  }

  // only cut out the valid d2J_dxdz
  Eigen::MatrixXd d2J_dxdz = Eigen::MatrixXd::Zero(3, 2 * valid_points_count);
  d2J_dxdz.block(0,0,3,valid_points_count) = d2J_dxdz_reading.block(0,0,3,valid_points_count);
  d2J_dxdz.block(0,valid_points_count,3,valid_points_count) = d2J_dxdz_reference.block(0,0,3,valid_points_count);


  Eigen::Matrix3d inv_J = d2J_dx2.inverse();
  Eigen::Matrix3d cov = param_.sensor_sdv * param_.sensor_sdv
      * inv_J * (d2J_dxdz * d2J_dxdz.transpose()) * inv_J;

  return cov;
}

}  // namespace mast
