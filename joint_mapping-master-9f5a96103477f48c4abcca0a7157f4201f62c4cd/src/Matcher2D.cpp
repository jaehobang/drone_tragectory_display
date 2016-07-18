
#include <joint_mapping/Matcher2D.h>
#include <ransac/Ransac-inl.h>

#include <flann/flann.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point2.h>

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace gtsam;

/* Utils for loop closure functions */
namespace comap {

/* ************************************************************************* */
// add a local training scan
unsigned int LoopClosureDetector::addTrainingScan(size_t idx,
    const Eigen::MatrixXd& scan) {

  // get descriptors to put in kdtree
  const LaserReading laser_reading = transScan2LaserReading(scan, setting_.down_sampling);

  std::vector<InterestPoint *> interest_points;
  unsigned int count_feature = feature_detector_.detect(laser_reading, interest_points);

  FeatureDescriptors descp = descriptor_gen_.describe(interest_points, laser_reading);

  // put in k-nn
  knn_matcher_.addTraining(idx, descp);

  // cache feature points and descriptors
  train_point_cache_.push_back(interest_points);
  train_descriptor_cache_.push_back(descp);
  // cache scan index

  put_in_scan_idx_.push_back(idx);

  return count_feature;
}

/* ************************************************************************* */
// give a query scan and output a vector of matched frames
std::vector<size_t> LoopClosureDetector::queryMatchedScan(const Eigen::MatrixXd& scan) {

  // only return if cached size is big enough
  if (put_in_scan_idx_.size() < setting_.min_cache_size) {
    cout << "Cache size too small" << endl;
    return vector<size_t>();
  }

  // get descriptors to put in kdtree
  const LaserReading laser_reading = transScan2LaserReading(scan, setting_.down_sampling);

  feature_detector_.detect(laser_reading, query_point_);
  query_descriptor_ = descriptor_gen_.describe(query_point_, laser_reading);

  // get k-nn query result
  vector<vector<unsigned int> > knn_query_result = knn_matcher_.knnSearch(query_descriptor_);

  // put k-nn result in 1-dim histogram
  vector<size_t> knn_result_hist;
  knn_result_hist.assign(put_in_scan_idx_.size(), 0);
  /*
  put in scan index contains a list of all the scans that have been cached. It uses the local_scan_cache a map of pose_count to scan. we build a histogram over the scans in our cache
  and finds the scans where most of the descriptors are found as peaks
  */

  for (size_t i = 0; i < knn_query_result.size(); i++) {
    for (size_t j = 0; j < setting_.knn_matcher_setting.knn; j++) {
      size_t frame_idx = find(put_in_scan_idx_.begin(), put_in_scan_idx_.end(),
          knn_query_result[i][j]) - put_in_scan_idx_.begin();
      if (frame_idx >= knn_result_hist.size())
        throw std::runtime_error("cannot find index searched by k-nn in pre-cached frame index");
      knn_result_hist[frame_idx]++;
    }
  }

  // search peak
  vector<Peak> peak_result = peak_finder_.findPeaks(knn_result_hist);

  vector<size_t> peak_idx;
  for (size_t i = 0; i < peak_result.size(); i++) {
    peak_idx.push_back(put_in_scan_idx_.at(peak_result[i].idx));
  }

  return peak_idx;
}

/* ************************************************************************* */
// output the result pose by flann+ransac+icp
LoopClosureMatchResult LoopClosureMatcher::match2scans(
    const Eigen::MatrixXd& query_scan,
    const std::vector<InterestPoint *>& query_points,
    const FeatureDescriptors& query_descriptors,
    const Eigen::MatrixXd& local_scan,
    const std::vector<InterestPoint *>& local_points,
    const FeatureDescriptors& local_descriptors) {

  LoopClosureMatchResult result;

  // 1. flann match features
  result.feature_match = associateDescriptors(query_descriptors,
      local_descriptors, setting_.feature_size, setting_.match_lowe_ratio);

  // 2. preapre putatives for ransac
  vector<Point2Pair> putatives;
  putatives.resize(result.feature_match.size());
  FeaturePoints query_featpoints = transInterestPoint2FeaturePoints(query_points);
  FeaturePoints local_featpoints = transInterestPoint2FeaturePoints(local_points);

  // for ransac putatives, local is first and query is second
  for (size_t i = 0; i < result.feature_match.size(); i++) {
    int query_idx = result.feature_match[i].first;
    int local_idx = result.feature_match[i].second;
    putatives[i] = make_pair(local_featpoints.at(local_idx).t(),
        query_featpoints.at(query_idx).t());
  }

  if (result.feature_match.size() <= 3) {
    result.flag_success = false;
    return result;
  }

  // 3. run ransac
  result.ransac_mask.assign(result.feature_match.size(), true);
  Pose2 init_pose;


  Ransac::Result ransac_result = Ransac::ransacPose(putatives, setting_.ransac_confidence,
      setting_.ransac_sigma, init_pose, result.ransac_mask, Ransac::SILENT, setting_.ransac_maxiter);

  result.ransac_inliers = ransac_result.inlierCount;

  cout << "Ransac finished and had " << result.ransac_inliers << " inliers " << endl;

  // 4. check min inlier count thresh
  if (result.ransac_inliers < setting_.ransac_min_inliers) {
    cout << "Not enough inliers" << endl;
    result.flag_success = false;
    return result;

  } else {
    // run ICP to refine the pose
    MatchResult icp_result = icp_matcher_.matchPointClouds(local_scan, query_scan, init_pose);

    if (icp_result.status) {
      // catched by icp
      result.flag_success = true;
      result.relative_pose = icp_result.delta_pose;
      return result;

    } else {
      cout << "ICP did not finish correctly" << endl;
      result.flag_success = false;
      return result;
    }
  }
}

/* ************************************************************************* */
// Associate 2 scans' features by FLANN
std::vector<MatchPair> associateDescriptors(const FeatureDescriptors& feat1,
    const FeatureDescriptors& feat2, unsigned int feature_size, double match_lowe_ratio) {

  // local setting
  const unsigned int tree_nr = 4;

  // prepare data
  double* feat1cache = new double[feature_size * feat1.size()];
  double* feat2cache = new double[feature_size * feat2.size()];
  if (feat1cache == NULL || feat2cache == NULL)
    throw std::runtime_error("memory allocation error");

  for (size_t i = 0; i < feat1.size(); i++)
    for (size_t j = 0; j < feature_size; j++)
      feat1cache[j + i*feature_size] = feat1[i][j];
  for (size_t i = 0; i < feat2.size(); i++)
    for (size_t j = 0; j < feature_size; j++)
      feat2cache[j + i*feature_size] = feat2[i][j];

  flann::Matrix<double> queryMat(feat1cache, feat1.size(), feature_size);
  flann::Matrix<double> trainMat(feat2cache, feat2.size(), feature_size);

  // training kd-tree
  flann::Index<flann::L2<double> > matcher(trainMat, flann::KDTreeIndexParams(tree_nr));
  matcher.buildIndex();

  // get query knn index
  std::vector<std::vector<size_t> > feat_idx_cache;
  std::vector<std::vector<double> > dist_cache;

  matcher.knnSearch(queryMat, feat_idx_cache, dist_cache, 2, flann::SearchParams());

  // get matching result by checking distance Lowe's thresh
  vector<MatchPair> matchpair;
  for (size_t i = 0; i < feat_idx_cache.size(); i++)
    if (dist_cache.at(i)[0] < match_lowe_ratio * dist_cache.at(i)[1] * 2)
      matchpair.push_back(make_pair(i, feat_idx_cache.at(i)[0]));

  // free memory
  delete[] queryMat.ptr();
  delete[] trainMat.ptr();

  return matchpair;
}

gtsam::NonlinearFactorGraph Matcher2D::findLocalLoopClosure(
  const PoseD slam_pose, LaserScan2D& scan) {

  NonlinearFactorGraph graph;
#if 0
  // get looped index
  vector<LoopResult2d> loop_result;
  loop_result = this->findLoopClosure(scan);

  // perform small EM only after init
  if (local_smallEM_.flag_init) {
    for (size_t i = 0; i < loop_result.size(); i++) {
      Pose2 relpose = loop_result[i].delta_pose;
      pair<size_t, size_t> relidx = make_pair(loop_result[i].loop_idx, pose_count_);
      // inlier
      if (pose_count_ - loop_result[i].loop_idx > setting_.local_loop_interval &&
          local_smallEM_.perform(relpose, relidx, curr_values_, isam_.getFactorsUnsafe())) {

        cout << "local loop detected! " << endl;
        cout << "robot_" << ID_ << ": [" << loop_result[i].loop_idx << ", " << pose_count_ << "]" << endl;
        cout << "Press Enter to continue ... " << endl;
        cin.ignore(1);

        // matched: insert between robot factor
        graph.push_back(BetweenFactor<Pose2>(Symbol(ID_, loop_result[i].loop_idx), Symbol(ID_, pose_count_),
            relpose, setting_.loop_default_model));
      }
    }

  } else {
    // only init small EM after certain count
    if (local_measure_poses_.size() >= setting_.local_loop_count_smallEM) {
      local_smallEM_.init(local_measure_poses_, local_measure_index_, Pose2());
      local_measure_poses_.clear();
      local_measure_index_.clear();

    // insert in local cache
    } else {
      for (size_t i = 0; i < loop_result.size(); i++) {
        local_measure_poses_.push_back(loop_result[i].delta_pose);
        local_measure_index_.push_back(make_pair(loop_result[i].loop_idx, pose_count_));
      }
    }
  }
#endif
  return graph;
}


/* ************************************************************************* */
// use FLIRT-ANN-RANSAC based dector and matcher to fine loop closure
// testing function: use it before loop built
std::vector<LoopResult2D> Matcher2D::findLoopClosure(LaserScan2D& remote_scan) {

  vector<LoopResult2D> loop_result_vec;

  if (trained_scan_count_ >= min_cache_size) {

    // 1. detect
    vector<size_t> detected_loop = loop_detector_.queryMatchedScan(remote_scan.filtered());

    // 2. try match detected
    for (size_t i = 0; i < detected_loop.size(); i++) {
      LoopClosureMatchResult match_result;

      try {
        match_result = loop_matcher_.match2scans(
            remote_scan.homo(),
            loop_detector_.query_feature_points(),
            loop_detector_.query_feature_descriptors(),
            local_scan_cache_[detected_loop[i]],
            loop_detector_.train_feature_points(detected_loop[i]),
            loop_detector_.train_feature_descriptors(detected_loop[i]));
      } catch (std::runtime_error err) {
        cerr << "[ERROR: Matcher2D] Loop closure cannot match" << err.what() << endl;
        continue;
      }

      if (match_result.flag_success) {
        LoopResult2D loop_result;
        loop_result.loop_idx = detected_loop[i];
        // inverse here since matcher output's ref is query, but agent input need local as ref
        loop_result.delta_pose = match_result.relative_pose.inverse();
        loop_result_vec.push_back(loop_result);
      } else {
        //cerr << "[ERROR: Matcher2D] match flag is false" << endl;
      }
    }
  }

  return loop_result_vec;
}

} // namespace comap

namespace Ransac {

/* *************************************************************************** */
// get inlier numbers
size_t PoseEstimator::inliers(const Datums& putatives, const Model& model,
    double sigma, Mask& mask, double *ignored) {

  size_t inlier_count = 0;

  for (size_t i = 0; i < putatives.size(); i++)
    if (putatives.at(i).second.distance(model.transform_from(putatives.at(i).first)) < sigma)
      inlier_count++;
    else
      mask[i] = false;

  // return inlier count
  return inlier_count;
}

/* *************************************************************************** */
// refine the plane given set of inlier points
gtsam::Pose2 PoseEstimator::refine(const Datums& ps, const Mask& mask,
    boost::optional<Model> bestModel) {

  // filter inlier
  std::vector<Point2Pair> putatives;
  for (size_t i = 0; i < ps.size(); i++) {
    if (!mask[i]) continue;
    putatives.push_back(ps[i]);
  }

  // check inlier size
  if (putatives.size() < 2)
    throw std::runtime_error("minimal pose solver must have input inlier size >= 2");

  // method of 'A Method for Registration of 3­D Shapes', by Besl and McKay, 1992
  // TODO: not sure this is correct in 2D
  // 1. find centroids of each dataset
  Vector2 cent1 = zero(2);
  Vector2 cent2 = zero(2);
  for (size_t i = 0; i < putatives.size(); i++) {
    cent1 += putatives[i].first.vector();
    cent2 += putatives[i].second.vector();
  }
  cent1 = cent1 / static_cast<double>(putatives.size());
  cent2 = cent2 / static_cast<double>(putatives.size());

  // 2. SVD
  Matrix H = zeros(2,2);
  for (size_t i = 0; i < putatives.size(); i++)
    H = H + (putatives[i].first.vector() - cent1) *
        (putatives[i].second.vector() - cent1).transpose();

  Matrix U,V;
  Vector S;
  svd(H, U, S, V);

  // 3. get rotation matrix
  Matrix R = V * U.transpose();
  if (R.determinant() < 0) {
    V(0,1) = -V(0,1);
    V(1,1) = -V(1,1);
    R = V * U.transpose();
  }

  // 4. translation
  Vector2 t = -R * cent1 + cent2;
  return Pose2(Rot2(atan2(R(1,0), R(0,0))), Point2(t));
}

/* *************************************************************************** */
// estimate pose using 2-point RANSAC
Result ransacPose(const std::vector<gtsam::Point2Pair> &ps, double confidence,
    double sigma, gtsam::Pose2 &pose, std::vector<bool> &inliers,
    verbosityLevel verbose, size_t maxIt, bool paranoid) {

  if (ps.size() < 3)
    throw std::runtime_error("RANSAC pose solver must have input vector size >= 3");

  Parameters parameters(confidence, sigma, verbose, maxIt, false,
        paranoid);

  return ransac<PoseEstimator>(ps, pose, inliers, parameters);
}
} // namespace ransac
