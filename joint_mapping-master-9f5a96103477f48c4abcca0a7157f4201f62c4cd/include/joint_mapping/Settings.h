/* File Containing all the settings */

#pragma once

#include <Eigen/Dense>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Key.h>

#include <joint_mapping/Agent_Definitions.h>
#include <joint_mapping/CRProcess.h>
#include <joint_mapping/Package.h>

#include <joint_mapping/FLIRT.h>

#include <vector>

namespace comap {

/* ************************************************************************* */
// Clustering setting
struct ClusteringSetting {

  // max number of cluster
  unsigned int max_cluster;
  // min number of poses each cluster
  unsigned int min_nr_candidates;
  // xy threshold of cluster: to make sure the number of pose in cluster
  double cluster_th_xy;
  // theta threshold of cluster
  double cluster_th_theta;
  // xy clustering resolution: to distinguish cluster
  double cluster_res_xy;
  // theta clustering resolution
  double cluster_res_theta;

  // default setting
  ClusteringSetting() :
    max_cluster(10),
    min_nr_candidates(10),
    cluster_th_xy(2.0),
    cluster_th_theta(0.2),
    cluster_res_xy(3.0),
    cluster_res_theta(0.5)
    {}

  virtual ~ClusteringSetting() {}
};

/* ************************************************************************* */
struct RelativePoseEMSetting {

  // inlier noise model, also used for loop closure in Agent.cpp
  gtsam::noiseModel::Diagonal::shared_ptr inlier_model;
  // outlier noise model
  gtsam::noiseModel::Diagonal::shared_ptr outlier_model;
  // inlier prior
  double inlier_prior;
  // flag of zero probability bump
  bool flag_bump_at_zero;
  // minimum probability threshold of true inlier
  double min_inlier_prob;
  // minimum number of inliers make a hypothesis valid
  unsigned int min_inlier_count;
  // re-optimize by inlier only
  bool flag_reoptimize_inlier_only;

  // default setting
  RelativePoseEMSetting() :
    inlier_prior(0.5),
    flag_bump_at_zero(false),
    min_inlier_prob(0.8),
    min_inlier_count(10),
    flag_reoptimize_inlier_only(false) {
    Eigen::VectorXd temp(6);
    temp << 0.2, 0.2, 0.2, 0.02, 0.02, 0.02;
    inlier_model = gtsam::noiseModel::Diagonal::Sigmas(temp);
    Eigen::VectorXd temp2(6);
    temp2 << 500, 500, 500, 200, 200, 200;
    outlier_model = gtsam::noiseModel::Diagonal::Sigmas(temp2);
  }

  virtual ~RelativePoseEMSetting() {}
};

/* ************************************************************************* */
struct KnnMatcherSetting {

  // number of trees
  unsigned int tree_nr;
  // feature array size
  unsigned int feature_size;
  // n of k-nn
  unsigned int knn;

  // default setting
  KnnMatcherSetting() :
    tree_nr(4),
    feature_size(128),
    knn(1) {}

  virtual ~KnnMatcherSetting() {}
};

struct HypothesisMergeSetting {

  // position resolution
  double position_res;
  // theta resolution
  double theta_res;
  // accept relative inlier number ratio compare to the best hypothesis
  double relative_inlier_ratio;

  // default setting
  HypothesisMergeSetting() :
    position_res(2.0),
    theta_res(0.2),
    relative_inlier_ratio(0.5) {}

  virtual ~HypothesisMergeSetting() {}
};

/* ************************************************************************* */
// peak finder setting
struct PeakFinderSetting {

  // smoother step (for i: 0 as not smooth, 1 as smooth in [i-1, i+1])
  unsigned int smooth_step;
  // smoother weight param
  double smooth_weight;
  // peak ratio to neighbour, such as peak v[i] > ratio * v[i-1] & ratio * v[i+1]
  // for ratio > 1.0, bigger means more sharp; for < 1.0, accept few not peaks
  double peak_ratio;
  // accept thresh, only accept peaks above this ratio * average value (after smoothing)
  double min_peak_thresh_ratio;
  // whether accept all point above below ratio
  bool flag_accept_all_above;
  // accept thresh, accept all point above this ratio * average value
  double min_accept_thresh_ratio;

  // default setting
  PeakFinderSetting() :
    smooth_step(3),
    smooth_weight(0.5),
    peak_ratio(1.0),
    min_peak_thresh_ratio(1.5),
    flag_accept_all_above(true),
    min_accept_thresh_ratio(3.0) {}

  virtual ~PeakFinderSetting() {}
};

struct HypothesisSelectSetting {

  // global setting
  // max hypothesis perform selection: only take these have highest inlier number
  unsigned int max_hypothesis_checked;
  // probability ratio of best/second best hyopthesis to accept best one
  double prob_ratio_btw_best_second;
  // accepted lowest prior prob
  double min_prior_prob;

  // hypothesis prior part:
  // flag whether use CRP prior
  bool hprior_use_CRP;
  // cell size for counting cell in m
  double hprior_cell_size;
  // chinese restaurant process setting
  CRPSetting hprior_CRP_setting;

  // relative pose transformation prior part:
  gtsam::noiseModel::Diagonal::shared_ptr tprior_model;

  // measurement likelihood part:
  // inlier noise model
  gtsam::noiseModel::Diagonal::shared_ptr measlike_inlier_model;
  // outlier noise model
  gtsam::noiseModel::Diagonal::shared_ptr measlike_outlier_model;

  // default setting
  HypothesisSelectSetting() :
    max_hypothesis_checked(5),
    prob_ratio_btw_best_second(2.0),
    min_prior_prob(0.2),
    hprior_use_CRP(false),
    hprior_cell_size(2.0),
    hprior_CRP_setting() {
    Eigen::VectorXd temp(6);
    temp << 0.2, 0.2, 0.2, 0.02, 0.02, 0.02;
    Eigen::VectorXd temp2(6);
    temp2 << 500, 500, 500, 200, 200, 200;
    Eigen::VectorXd temp3(6);
    temp3 << 10.0, 10.0, 10.0, 3.1416/2, 3.1416/2, 3.1416/2;

    tprior_model = gtsam::noiseModel::Diagonal::Sigmas(temp3);
    measlike_inlier_model = gtsam::noiseModel::Diagonal::Sigmas(temp);
    measlike_outlier_model = gtsam::noiseModel::Diagonal::Sigmas(temp2);
  }

  virtual ~HypothesisSelectSetting() {}
};

struct ICPParameter {

  double sensor_sdv;    // range sensor standard deviation

  // constructors
  ICPParameter() : sensor_sdv(0.01) {}
  ICPParameter(double sdv) : sensor_sdv(sdv) {}
  ICPParameter(const ICPParameter& param) : sensor_sdv(param.sensor_sdv) {}
  virtual ~ICPParameter() {}
};

/* ************************************************************************* */
// setting for libpointmatcher
struct PointMatcherSetting : public ICPParameter{

  // reading filter:
  // random sampling filter: sample ratio
  double read_randsample_ratio;
  // flag of whether use max density filter
  bool read_maxdens_flag;
  // max density for density filter
  double read_maxdens_density;
  // flag of whether use max distance filter
  bool read_maxdist_flag;
  // max distance for distance filter
  double read_maxdist_dist;

  // outlier filter
  // accpet ratio of inlier
  double outlier_trim_ratio;
  // flag of whether use max dist outlier filter
  bool outlier_maxdist_flag;
  // max distance to apply outlier filter
  double outlier_maxdist_dist;

  // transformation checker
  // iteration checker: max iteration
  unsigned int checker_maxiter;
  // differential checker: translation
  double checker_diff_trans;
  // rotation
  double checker_diff_rot;

  // other parameter will use default

  // default setting
  PointMatcherSetting() :
    read_randsample_ratio(0.4),
    read_maxdens_flag(false),
    read_maxdens_density(1000),
    read_maxdist_flag(true),
    read_maxdist_dist(8.0),
    outlier_trim_ratio(0.95),
    outlier_maxdist_flag(false),
    outlier_maxdist_dist(1.0),
    checker_maxiter(40),
    checker_diff_trans(1e-3),
    checker_diff_rot(1e-4) {}

  virtual ~PointMatcherSetting() {}
};

/* ************************************************************************* */

// loop closure detector setting
struct LoopClosureDetectorSetting {

  // factor of down-sampling pre detection, 1 for no down-sampling
  unsigned int down_sampling;
  // minimal cache size to start returning result
  unsigned int min_cache_size;
  // feature detector setting
  FeatureDetectorSetting feature_detect_setting;
  // descriptor setting
  DescriptorGeneratorSetting descriptor_setting;
  // k-nn macher setting
  KnnMatcherSetting knn_matcher_setting;
  // matched features peak finder settings
  PeakFinderSetting peak_finder_setting;

  // default setting
  LoopClosureDetectorSetting() :
    down_sampling(1),
    min_cache_size(5),
    feature_detect_setting(),
    descriptor_setting(),
    knn_matcher_setting(),
    peak_finder_setting() {
    knn_matcher_setting.knn = 10;    // modify to 3-nn, default is 1-nn
  }

  virtual ~LoopClosureDetectorSetting() {}
};

struct LoopClosureMatcherSetting {

  // feature vector size
  unsigned int feature_size;
  // flann matching Lowe ratio
  double match_lowe_ratio;
  // ransac inlier noise level
  double ransac_sigma;
  // ransac confidence
  double ransac_confidence;
  // ransac max iteration
  double ransac_maxiter;
  // min inlier number to accept this match
  unsigned int ransac_min_inliers;
  // icp matcher setting
  PointMatcherSetting icp_config;

  // default setting
  LoopClosureMatcherSetting() :
    feature_size(128),
    match_lowe_ratio(0.8),
    ransac_sigma(0.2),
    ransac_confidence(0.9),
    ransac_maxiter(20),
    ransac_min_inliers(3),
    icp_config() {}

  virtual ~LoopClosureMatcherSetting() {}
};

/* ************************************************************************* */
// small EM setting
struct SmallEMSetting {

  // flag of whether use updated inlier model depends on the covariance
  bool flag_update_model;
  // inlier noise model, may be updated by pose covariance
  gtsam::noiseModel::Diagonal::shared_ptr inlier_model;
  // outlier noise model
  gtsam::noiseModel::Diagonal::shared_ptr outlier_model;
  // inlier prior
  double inlier_prior;
  // minimum probability threshold of true inlier
  double min_inlier_prob;

  // default setting
  SmallEMSetting() :
    flag_update_model(false),
    inlier_prior(0.5),
    min_inlier_prob(0.8) {
    Eigen::VectorXd temp(6);
    temp << 0.2, 0.2, 0.2, 0.02, 0.02, 0.02;
    Eigen::VectorXd temp2(6);
    temp2 << 500, 500, 500, 200, 200, 200;

    inlier_model = gtsam::noiseModel::Diagonal::Sigmas(temp);
    outlier_model = gtsam::noiseModel::Diagonal::Sigmas(temp2);
  }

  virtual ~SmallEMSetting() {}
};

} // end comap
