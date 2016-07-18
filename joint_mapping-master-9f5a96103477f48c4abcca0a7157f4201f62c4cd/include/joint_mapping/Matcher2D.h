/*
*
  Abstract Class for Matchers. ICP or Feature based Matching.

  Get two scans and tell how the line up. These scans can be 2D or 3D

  Will be used to detect both local loop closure and foreign robot data correspondence
*
*/

#pragma once

#include <joint_mapping/Agent_Definitions.h>
#include <joint_mapping/Settings.h>
#include <joint_mapping/FLIRT.h>
#include <joint_mapping/FLANN.h>
#include <joint_mapping/FeaturePoint.h>
#include <joint_mapping/PeakFinder.h>
#include <joint_mapping/LaserScan2D.h>

#include <ransac/Ransac.h>

#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>

#include <eigen3/Eigen/Dense>

#include <joint_mapping/ICPMatcher2D.h>

#include <vector>

namespace comap {

/* ************************************************************************* */
// loop closure result struct
struct LoopResult2D {
  size_t loop_idx;
  gtsam::Pose2 delta_pose;
};

/* ************************************************************************* */
// definition of FeaturePoint

// feature point, point with rotation
typedef gtsam::Pose2 FeaturePoint;
typedef std::vector<FeaturePoint> FeaturePoints;

// feature descriptor (just compatible to Beta-Grid descriptor)
typedef boost::shared_array<double> FeatureDescriptor;
typedef std::vector<FeatureDescriptor> FeatureDescriptors;

// match index pair
typedef std::pair<int, int> MatchPair;

/* ************************************************************************* */
// save FeaturePoints position to file
bool writeMATLABfileFeaturePoints(std::string filename, const FeaturePoints& featpoints);

/* ************************************************************************* */
// loop closure detector, performing the following steps:
//
// For a local training scan:
// 1.   add it in kdtree
//
// For a remote query scan:
// 1.   perform k-nn search by kdtree, get the matching feature index
// 2.   get the matched local scans' index by clustering, put in histogram
// 3.   get max peak as loop closure

/* ************************************************************************* */
// loop closure detector setting
/* ************************************************************************* */
// loop closure detector
class LoopClosureDetector {

private:
  // setting
  LoopClosureDetectorSetting setting_;

  // utils
  FeatureDetector feature_detector_;
  DescriptorGenerator descriptor_gen_;
  KnnMatcher knn_matcher_;
  PeakFinder<size_t> peak_finder_;

  // data caches
  std::vector<size_t> put_in_scan_idx_;  // frame index put in cache
  std::vector<std::vector<InterestPoint *> > train_point_cache_;
  std::vector<FeatureDescriptors> train_descriptor_cache_;
  std::vector<InterestPoint *> query_point_;
  FeatureDescriptors query_descriptor_;

public:
  // constructors
  LoopClosureDetector(const LoopClosureDetectorSetting& setting = LoopClosureDetectorSetting()) :
      setting_(setting),
      feature_detector_(setting_.feature_detect_setting),
      descriptor_gen_(setting_.descriptor_setting),
      knn_matcher_(setting_.knn_matcher_setting),
      peak_finder_(setting_.peak_finder_setting) {}

  // decontructor
  virtual ~LoopClosureDetector() {}

  // add a local training scan
  // please use filtered scan if you want.
  // @idx frame index
  // @return features added in
  unsigned int addTrainingScan(size_t idx, const Eigen::MatrixXd& scan);

  // give a query scan and output a vector of matched frames
  // please use filtered scan if you want.
  // @return matched frame index
  std::vector<size_t> queryMatchedScan(const Eigen::MatrixXd& scan);

  // get internal data for other usage, given frame index
  inline std::vector<InterestPoint *> train_feature_points(size_t scan_idx) const {
    std::vector<InterestPoint *> fpoints = train_point_cache_.at(this->find_scan_index(scan_idx));
    return fpoints;
  }
  inline FeatureDescriptors train_feature_descriptors(size_t scan_idx) const {
    FeatureDescriptors fdp = train_descriptor_cache_.at(this->find_scan_index(scan_idx));
    return fdp;
  }
  // query just has cached last query
  inline std::vector<InterestPoint *> query_feature_points() const { return query_point_; }
  inline FeatureDescriptors query_feature_descriptors() const { return query_descriptor_; }

private:
  // find a scan index from saved scan index
  inline size_t find_scan_index(size_t scan_idx) const {
    vector<size_t>::const_iterator it_find = std::find(put_in_scan_idx_.begin(), put_in_scan_idx_.end(), scan_idx);
    if (it_find == put_in_scan_idx_.end()) {
      throw std::runtime_error("cannot find scan index in trained scan");
      return 0;
    } else {
      return it_find - put_in_scan_idx_.begin();
    }
  }
};

/* ************************************************************************* */
// loop closure matcher, performing the following steps:
//
// when have a local scan and a remote scan matched:
// 1.   use 2-nn match 2 frames, features, run flann get feature matching by Lowe thresh
// 2.   run ransac to all matched local scans,
// 3.   use ransac result as init pose, run icp, get final relative pose

/* ************************************************************************* */

// matcher output result
struct LoopClosureMatchResult {
  bool flag_success;
  gtsam::Pose2 relative_pose;
  std::vector<MatchPair> feature_match;
  std::vector<bool> ransac_mask;
  size_t ransac_inliers;
};


/* ************************************************************************* */
// loop closure detector
class LoopClosureMatcher {

private:
  LoopClosureMatcherSetting setting_;
  ICPMatcher icp_matcher_;

public:
  // constructors
  LoopClosureMatcher(const LoopClosureMatcherSetting& setting = LoopClosureMatcherSetting()) :
      setting_(setting), icp_matcher_(setting_.icp_config) {}
  // decontructor
  virtual ~LoopClosureMatcher() {}

  // output the result pose by flann+ransac+icp
  // in this step scan will be only used in icp, so please use none-filtered scan
  // output pose: matched training scan -> query scan (this is ref !!! @_@)
  LoopClosureMatchResult match2scans(
      const Eigen::MatrixXd& query_scan,
      const std::vector<InterestPoint *>& query_points,
      const FeatureDescriptors& query_descriptors,
      const Eigen::MatrixXd& local_scan,
      const std::vector<InterestPoint *>& local_points,
      const FeatureDescriptors& local_descriptors);
};

/* ************************************************************************* */
// Associate 2 scans' features by FLANN
std::vector<MatchPair> associateDescriptors(const FeatureDescriptors& feat1,
    const FeatureDescriptors& feat2, unsigned int feature_size = 72,
    double match_lowe_ratio = 0.6);

class Matcher2D {

private:
  ICPMatcher icp_matcher_consec_;
  LoopClosureDetector loop_detector_;
  LoopClosureMatcher loop_matcher_;

  size_t min_cache_size;
  size_t trained_scan_count_;
  std::map<size_t, Eigen::MatrixXd> local_scan_cache_;

public:

  Matcher2D(const PointMatcherSetting icp_config, const LoopClosureDetectorSetting loop_detect_setting, const LoopClosureMatcherSetting loop_match_setting, size_t min_c_size) {
    icp_matcher_consec_ = ICPMatcher(icp_config);
    loop_detector_ = LoopClosureDetector(loop_detect_setting);
    loop_matcher_ = LoopClosureMatcher(loop_match_setting);
    trained_scan_count_ = 0;
    min_cache_size = min_c_size;
  }

  size_t get_trained_count() { return trained_scan_count_; }

  std::vector<LoopResult2D> findLoopClosure(LaserScan2D& remote_scan);

  gtsam::NonlinearFactorGraph findLocalLoopClosure( const PoseD slam_pose, LaserScan2D& scan);

  unsigned int addLocalScan(LaserScan2D& scan, size_t idx) {
    local_scan_cache_[idx] = scan.homo();
    loop_detector_.addTrainingScan(idx, scan.filtered());
    trained_scan_count_++;
  }

  Eigen::MatrixXd getScanAtPose(size_t idx) {
    return local_scan_cache_[idx];
  }
};

}  // namespace comap


/* *************************************************************************** */
// 2-point RANSAC method to estimate (3 dim by 2 points => over-constrain problem)
// note: this engine won't consider key point rotaion
namespace Ransac {

// pose estimator class wrapped by RANSAC engine
struct PoseEstimator {

  typedef gtsam::Pose2 Model;
  typedef gtsam::Point2Pair Datum;
  typedef std::vector<gtsam::Point2Pair> Datums;
  static const size_t setSize = 2;

  // get inlier numbers
  static size_t inliers(const Datums& putatives, const Model& model,
      double sigma, Mask& mask, double *ignored);

  // final refine function
  static Model refine(const Datums& putatives, const Mask& mask,
      boost::optional<Model> bestModel = boost::none);

};      // class PoseEstimator

/* *************************************************************************** */
// estimate pose using 2-point RANSAC

Result ransacPose(const std::vector<gtsam::Point2Pair> &ps, double confidence,
    double sigma, gtsam::Pose2 &pose, std::vector<bool> &inliers,
    verbosityLevel verbose = SILENT, size_t maxIt = 0, bool paranoid = true);

}
