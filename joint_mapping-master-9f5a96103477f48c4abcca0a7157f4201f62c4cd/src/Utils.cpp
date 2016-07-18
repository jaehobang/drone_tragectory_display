/**
 * @file MAST_cpp_utils.cc
 * @brief some utils to use MAST_cpp lib
 * @author Jing Dong
 * @date July 17, 2014
 */

#include <joint_mapping/Utils.h>

namespace pu = parameter_utils;

using namespace std;

/* ************************************************************************** */
// convert ros-message to comap-package
comap::AgentPackage msgToPack(const joint_mapping::AgentPackageMsg& msg) {

  comap::AgentPackage agent_pack;

  // odometry information
  agent_pack.robot_name = static_cast<std::string>(msg.robot_name);
  agent_pack.robot_ID = static_cast<unsigned char>(msg.id);
  agent_pack.pose_count = static_cast<size_t>(msg.pose_count);
  agent_pack.curr_pose = gtsam::Pose3(
     gtsam::Rot3::quaternion(msg.pose.orientation.w,
                      msg.pose.orientation.x,
                      msg.pose.orientation.y,
                      msg.pose.orientation.z),
     gtsam::Point3(msg.pose.position.x,
                   msg.pose.position.y,
                   msg.pose.position.z));
  agent_pack.delta_odometry = gtsam::Pose3(
     gtsam::Rot3::quaternion(msg.odom.orientation.w,
                      msg.odom.orientation.x,
                      msg.odom.orientation.y,
                      msg.odom.orientation.z),
     gtsam::Point3(msg.odom.position.x,
                   msg.odom.position.y,
                   msg.odom.position.z));

  // informative scan
  agent_pack.informative_scan = msg.flag_scan;
  if (msg.flag_scan) {
    /*
    Eigen::MatrixXd scan_eigen(2, msg.scan.points.size());
    for (unsigned int ii = 0; ii < msg.scan.points.size(); ++ii) {
      scan_eigen(0, ii) = msg.scan.points[ii].x;
      scan_eigen(1, ii) = msg.scan.points[ii].y;
    }
    agent_pack.scan = scan_eigen;
    */
    agent_pack.scan = msg.scan;
  }

  // matched measurements
  agent_pack.flag_matches = msg.flag_match;
  if (msg.flag_match) {

    // matched measurements size check: should be equal
    unsigned int match_size = msg.match_robot_id1.size();
    if (msg.match_robot_id2.size() != match_size ||
        msg.match_index1.size() != match_size ||
        msg.match_index2.size() != match_size ||
        msg.match_pose.size() != match_size) {
      throw std::string("received vectors' size are not the same");
    }

    agent_pack.match_robot_id.reserve(match_size);
    agent_pack.match_robot_names.reserve(match_size);
    agent_pack.match_meas_index.reserve(match_size);
    agent_pack.match_measurement.reserve(match_size);

    for (unsigned int ii = 0; ii < match_size; ++ii) {
      unsigned char robot_id1 = boost::lexical_cast<unsigned char>(msg.match_robot_id1[ii]);
      unsigned char robot_id2 = boost::lexical_cast<unsigned char>(msg.match_robot_id2[ii]);
      std::string robot_name1 = boost::lexical_cast<std::string>(msg.match_robot_name1[ii]);
      std::string robot_name2 = boost::lexical_cast<std::string>(msg.match_robot_name2[ii]);
      size_t match_id1 = static_cast<size_t>(msg.match_index1[ii]);
      size_t match_id2 = static_cast<size_t>(msg.match_index2[ii]);
      /*gtsam::Pose2 measure(msg.match_pose[ii].position.x, msg.match_pose[ii].position.y,
          gr::getYaw(msg.match_pose[ii].orientation));
          */
      gtsam::Pose3 measure(
         gtsam::Rot3::quaternion(msg.match_pose[ii].orientation.w,
                          msg.match_pose[ii].orientation.x,
                          msg.match_pose[ii].orientation.y,
                          msg.match_pose[ii].orientation.z),
         gtsam::Point3(msg.match_pose[ii].position.x,
                       msg.match_pose[ii].position.y,
                       msg.match_pose[ii].position.z));

      agent_pack.match_robot_id.push_back(std::make_pair(robot_id1, robot_id2));
      agent_pack.match_robot_names.push_back(std::make_pair(robot_name1, robot_name2));
      agent_pack.match_meas_index.push_back(std::make_pair(match_id1, match_id2));
      agent_pack.match_measurement.push_back(measure);
    }
  }

  return agent_pack;
}

/* ************************************************************************** */
// convert comap-package to ros-message
joint_mapping::AgentPackageMsg packToMsg(const comap::AgentPackage& pack) {

  joint_mapping::AgentPackageMsg agent_msg;

  // indexing info
  agent_msg.id = static_cast<uint8_t>(pack.robot_ID);
  agent_msg.pose_count = static_cast<uint32_t>(pack.pose_count);
  agent_msg.robot_name = pack.robot_name;

  gtsam::Vector3 rpy = pack.curr_pose.rotation().rpy();
  tf::Quaternion q;
  q.setRPY(rpy(0), rpy(1), rpy(2));
  geometry_msgs::Quaternion qmsg;
  tf::quaternionTFToMsg(q,qmsg);

  // pose
  //geometry_msgs::Quaternion q2 = gr::ZYXToQuatMsg(pack.curr_pose.rotation().rpy());

  agent_msg.pose.orientation = qmsg;
  agent_msg.pose.position.x = pack.curr_pose.x();
  agent_msg.pose.position.y = pack.curr_pose.y();
  agent_msg.pose.position.z = pack.curr_pose.z();

  // odom
  rpy = pack.delta_odometry.rotation().rpy();
  q.setRPY(rpy(0), rpy(1), rpy(2));
  tf::quaternionTFToMsg(q,qmsg);

  //q = gr::ZYXToQuatMsg(pack.delta_odometry.rotation().rpy());

  agent_msg.odom.orientation = qmsg;
  agent_msg.odom.position.x = pack.delta_odometry.x();
  agent_msg.odom.position.y = pack.delta_odometry.y();
  agent_msg.odom.position.z = pack.delta_odometry.z();

  // informative scan
  agent_msg.flag_scan = pack.informative_scan;
  if (pack.informative_scan) {
    agent_msg.scan = pack.scan;
  }

  // matched measurement
  agent_msg.flag_match = pack.flag_matches;
  if (pack.flag_matches) {
    for (unsigned int ii = 0; ii < pack.match_robot_id.size(); ii++) {
      agent_msg.match_robot_id1.push_back(pack.match_robot_id.at(ii).first);
      agent_msg.match_robot_id2.push_back(pack.match_robot_id.at(ii).second);
      agent_msg.match_robot_name1.push_back(pack.match_robot_names.at(ii).first);
      agent_msg.match_robot_name2.push_back(pack.match_robot_names.at(ii).second);
      agent_msg.match_index1.push_back(pack.match_meas_index.at(ii).first);
      agent_msg.match_index2.push_back(pack.match_meas_index.at(ii).second);
      // measurement pose
      geometry_msgs::Pose pose;

      rpy = pack.match_measurement.at(ii).rotation().rpy();
      q.setRPY(rpy(0), rpy(1), rpy(2));
      tf::quaternionTFToMsg(q,qmsg);

      pose.orientation = qmsg;
      pose.position.x = pack.match_measurement.at(ii).x();
      pose.position.y = pack.match_measurement.at(ii).y();
      pose.position.z = pack.match_measurement.at(ii).z();
      agent_msg.match_pose.push_back(pose);
    }
  }

  return agent_msg;
}

/* ************************************************************************** */
// load robot agent setting from a ros handle
comap::AgentSetting loadAgentSetting(const ros::NodeHandle& n) {

  comap::AgentSetting setting;
  bool read_success = true;

  // Call parameter_utils::get to load settings as rosparams, set in the launch file

  // icp setting
  setting.icp_config_consec = loadPointMatcherSetting("distributed_mapping/icp_config_consec", n);

  // loop detection setting
  setting.loop_detect_setting = loadLoopClosureDetectorSetting("distributed_mapping/loop_detect_setting", n);

  // loop matching setting
  setting.loop_match_setting = loadLoopClosureMatcherSetting("distributed_mapping/loop_match_setting", n);

  // small EM setting
  setting.smallEM_setting = loadSmallEMSetting("distributed_mapping/smallEM_setting", n);

  // iSAM2 setting
  if (!pu::get("distributed_mapping/isam_param/relinearizeSkip", setting.isam_param.relinearizeSkip)) read_success = false;
  double relinearizeThreshold;
  if (!pu::get("distributed_mapping/isam_param/relinearizeThreshold", relinearizeThreshold)) read_success = false;
  setting.isam_param.relinearizeThreshold = relinearizeThreshold;

  // Clustering setting
  setting.cluster_setting = loadClusteringSetting("distributed_mapping/cluster_setting", n);

  // RelativePoseEM setting
  setting.relativeEM_setting = loadRelativePoseEMSetting("distributed_mapping/relativeEM_setting", n);

  // hypothesis merge setting
  setting.hypo_merge_setting = loadHypothesisMergeSetting("distributed_mapping/hypo_merge_setting", n);

  // hypothesis selection setting
  setting.hypo_select_setting = loadHypothesisSelectSetting("distributed_mapping/hypo_select_setting", n);

  setting.matcher_setting = loadMatcherSetting("distributed_mapping/matcher", n);

  // other overall setting
  if (!pu::get("distributed_mapping/find_loop_min_localcache", setting.min_cache_size)) read_success = false;
  int min_iter = 5;
  if (!pu::get("distributed_mapping/find_loop_min_inter",min_iter)) read_success = false;
  setting.train_min_iter = min_iter;
  int min_pose = 10;
  if (!pu::get("distributed_mapping/min_pose_count",min_pose)) read_success = false;
  setting.min_pose_count = min_pose;
  if (!pu::get("distributed_mapping/find_loop_min_distance", setting.train_min_distance)) read_success = false;
  if (!pu::get("distributed_mapping/local_loop_count_smallEM", setting.local_loop_count_smallEM)) read_success = false;
  if (!pu::get("distributed_mapping/local_loop_interval", setting.local_loop_interval)) read_success = false;

  double model_trans, model_rot;
  if (!pu::get("distributed_mapping/icp_default_model_trans", model_trans)) read_success = false;
  if (!pu::get("distributed_mapping/icp_default_model_rot", model_rot)) read_success = false;
  Eigen::VectorXd noise(6);
  noise << model_trans, model_trans, model_trans, model_rot, model_rot, model_rot;
  setting.icp_default_model = gtsam::noiseModel::Diagonal::Sigmas(noise);
  if (!pu::get("distributed_mapping/loop_default_model_trans", model_trans)) read_success = false;
  if (!pu::get("distributed_mapping/loop_default_model_rot", model_rot)) read_success = false;
  Eigen::VectorXd loop_noise(6);
  loop_noise << model_trans, model_trans, model_trans, model_rot, model_rot, model_rot;
  setting.loop_default_model = gtsam::noiseModel::Diagonal::Sigmas(loop_noise);

  // other settings
  //if (!pu::get("distributed_mapping/output_path", setting.output_path)) read_success = false;

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}

/* ************************************************************************** */
// PontMatcherSetting loader
/*
comap::PointMatcherSetting loadPointMatcherSetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::PointMatcherSetting setting;

  if (!pu::get(param + "/read_randsample_ratio", setting.read_randsample_ratio)) read_success = false;
  if (!pu::get(param + "/read_maxdens_flag", setting.read_maxdens_flag)) read_success = false;
  if (setting.read_maxdens_flag)
    if (!pu::get(param + "/read_maxdens_density", setting.read_maxdens_density)) read_success = false;
  if (!pu::get(param + "/read_maxdist_flag", setting.read_maxdist_flag)) read_success = false;
  if (setting.read_maxdist_flag)
    if (!pu::get(param + "/read_maxdist_dist", setting.read_maxdist_dist)) read_success = false;
  if (!pu::get(param + "/outlier_trim_ratio", setting.outlier_trim_ratio)) read_success = false;
  if (!pu::get(param + "/outlier_maxdist_flag", setting.outlier_maxdist_flag)) read_success = false;
  if (setting.outlier_maxdist_flag)
    if (!pu::get(param + "/outlier_maxdist_dist", setting.outlier_maxdist_dist)) read_success = false;
  if (!pu::get(param + "/checker_maxiter", setting.checker_maxiter)) read_success = false;
  if (!pu::get(param + "/checker_diff_trans", setting.checker_diff_trans)) read_success = false;
  if (!pu::get(param + "/checker_diff_rot", setting.checker_diff_rot)) read_success = false;

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}
*/

/* ************************************************************************** */
// SaliencySetting loader
/*
comap::ScanSaliencySetting loadScanSaliencySetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::ScanSaliencySetting setting;

  if (!pu::get(param + "/ntimes", setting.ntimes)) read_success = false;
  if (!pu::get(param + "/sigma_t", setting.sigma_t)) read_success = false;
  if (!pu::get(param + "/sigma_theta", setting.sigma_theta)) read_success = false;

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}
*/

/* ************************************************************************** */
// CLustering setting loader
comap::ClusteringSetting loadClusteringSetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::ClusteringSetting setting;

  if (!pu::get(param + "/max_cluster", setting.max_cluster)) read_success = false;
  if (!pu::get(param + "/min_nr_candidates", setting.min_nr_candidates)) read_success = false;
  if (!pu::get(param + "/cluster_th_xy", setting.cluster_th_xy)) read_success = false;
  if (!pu::get(param + "/cluster_th_theta", setting.cluster_th_theta)) read_success = false;
  if (!pu::get(param + "/cluster_res_xy", setting.cluster_res_xy)) read_success = false;
  if (!pu::get(param + "/cluster_res_theta", setting.cluster_res_theta)) read_success = false;

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}

/* ************************************************************************** */
// relative pose EM setting
comap::RelativePoseEMSetting loadRelativePoseEMSetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::RelativePoseEMSetting setting;

  if (!pu::get(param + "/inlier_prior", setting.inlier_prior)) read_success = false;
  if (!pu::get(param + "/flag_bump_at_zero", setting.flag_bump_at_zero)) read_success = false;
  if (!pu::get(param + "/min_inlier_prob", setting.min_inlier_prob)) read_success = false;
  if (!pu::get(param + "/min_inlier_count", setting.min_inlier_count)) read_success = false;
  if (!pu::get(param + "/flag_reoptimize_inlier_only", setting.flag_reoptimize_inlier_only)) read_success = false;
  double model_trans, model_rot;
  if (!pu::get(param + "/inlier_model_trans", model_trans)) read_success = false;
  if (!pu::get(param + "/inlier_model_rot", model_rot)) read_success = false;
  Eigen::VectorXd noise(6);
  noise << model_trans, model_trans, model_trans, model_rot, model_rot, model_rot;
  setting.inlier_model = gtsam::noiseModel::Diagonal::Sigmas(noise);
  if (!pu::get(param + "/outlier_model_trans", model_trans)) read_success = false;
  if (!pu::get(param + "/outlier_model_rot", model_rot)) read_success = false;
  Eigen::VectorXd out_noise(6);
  out_noise << model_trans, model_trans, model_trans, model_rot, model_rot, model_rot;
  setting.outlier_model = gtsam::noiseModel::Diagonal::Sigmas(out_noise);

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}

/* ************************************************************************** */
// hypothesis merge setting
comap::HypothesisMergeSetting loadHypothesisMergeSetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::HypothesisMergeSetting setting;

  if (!pu::get(param + "/position_res", setting.position_res)) read_success = false;
  if (!pu::get(param + "/theta_res", setting.theta_res)) read_success = false;
  if (!pu::get(param + "/relative_inlier_ratio", setting.relative_inlier_ratio)) read_success = false;

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}

/* ************************************************************************** */
// hypothesis selection setting
comap::HypothesisSelectSetting loadHypothesisSelectSetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::HypothesisSelectSetting setting;

  if (!pu::get(param + "/max_hypothesis_checked", setting.max_hypothesis_checked)) read_success = false;
  if (!pu::get(param + "/prob_ratio_btw_best_second", setting.prob_ratio_btw_best_second)) read_success = false;
  if (!pu::get(param + "/min_prior_prob", setting.min_prior_prob)) read_success = false;
  if (!pu::get(param + "/hprior_use_CRP", setting.hprior_use_CRP)) read_success = false;
  if (!pu::get(param + "/hprior_cell_size", setting.hprior_cell_size)) read_success = false;
  // CRP setting
  if (setting.hprior_use_CRP)
    setting.hprior_CRP_setting = loadCRPSetting(param + "/hprior_CRP_setting", n);

  double model_trans, model_rot;
  if (!pu::get(param + "/tprior_model_trans", model_trans)) read_success = false;
  if (!pu::get(param + "/tprior_model_rot", model_rot)) read_success = false;
  Eigen::VectorXd tpr(6);
  tpr << model_trans, model_trans, model_trans, model_rot, model_rot, model_rot;
  setting.tprior_model = gtsam::noiseModel::Diagonal::Sigmas(tpr);

  if (!pu::get(param + "/measlike_inlier_model_trans", model_trans)) read_success = false;
  if (!pu::get(param + "/measlike_inlier_model_rot", model_rot)) read_success = false;

  Eigen::VectorXd noise(6);
  noise << model_trans, model_trans, model_trans, model_rot, model_rot, model_rot;
  setting.measlike_inlier_model = gtsam::noiseModel::Diagonal::Sigmas(noise);
  if (!pu::get(param + "/measlike_outlier_model_trans", model_trans)) read_success = false;
  if (!pu::get(param + "/measlike_outlier_model_rot", model_rot)) read_success = false;
  Eigen::VectorXd out_noise(6);
  out_noise << model_trans, model_trans, model_trans, model_rot, model_rot, model_rot;
  setting.measlike_outlier_model = gtsam::noiseModel::Diagonal::Sigmas(out_noise);

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}

/* ************************************************************************** */
/*
comap::FeatureDetectorSetting loadFeatureDetectorSetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::FeatureDetectorSetting setting;

  if (!pu::get(param + "/minPeak", setting.minPeak)) read_success = false;
  if (!pu::get(param + "/minPeakDistance", setting.minPeakDistance)) read_success = false;
  if (!pu::get(param + "/scale", setting.scale)) read_success = false;
  if (!pu::get(param + "/baseSigma", setting.baseSigma)) read_success = false;
  if (!pu::get(param + "/sigmaStep", setting.sigmaStep)) read_success = false;
  if (!pu::get(param + "/dmst", setting.dmst)) read_success = false;
  if (!pu::get(param + "/useMaxRange", setting.useMaxRange)) read_success = false;

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}
*/
/* ************************************************************************** */
/*
comap::DescriptorGeneratorSetting loadDescriptorGeneratorSetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::DescriptorGeneratorSetting setting;

  if (!pu::get(param + "/minRho", setting.minRho)) read_success = false;
  if (!pu::get(param + "/maxRho", setting.maxRho)) read_success = false;
  if (!pu::get(param + "/binRho", setting.binRho)) read_success = false;
  if (!pu::get(param + "/binPhi", setting.binPhi)) read_success = false;

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}
*/
/* ************************************************************************** */
comap::KnnMatcherSetting loadKnnMatcherSetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::KnnMatcherSetting setting;

  if (!pu::get(param + "/tree_nr", setting.tree_nr)) read_success = false;
  if (!pu::get(param + "/feature_size", setting.feature_size)) read_success = false;
  if (!pu::get(param + "/knn", setting.knn)) read_success = false;

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}
/* ************************************************************************** */
comap::PeakFinderSetting loadPeakFinderSetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::PeakFinderSetting setting;

  if (!pu::get(param + "/smooth_step", setting.smooth_step)) read_success = false;
  if (!pu::get(param + "/smooth_weight", setting.smooth_weight)) read_success = false;
  if (!pu::get(param + "/peak_ratio", setting.peak_ratio)) read_success = false;
  if (!pu::get(param + "/min_peak_thresh_ratio", setting.min_peak_thresh_ratio)) read_success = false;
  if (!pu::get(param + "/flag_accept_all_above", setting.flag_accept_all_above)) read_success = false;
  if (!pu::get(param + "/min_accept_thresh_ratio", setting.min_accept_thresh_ratio)) read_success = false;

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}

/* ************************************************************************** */
comap::LoopClosureDetectorSetting loadLoopClosureDetectorSetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::LoopClosureDetectorSetting setting;

  if (!pu::get(param + "/down_sampling", setting.down_sampling)) read_success = false;

  // sub-method settings
  setting.feature_detect_setting = loadFeatureDetectorSetting(param + "/feature_detect_setting", n);
  setting.descriptor_setting = loadDescriptorGeneratorSetting(param + "/descriptor_setting", n);
  setting.knn_matcher_setting = loadKnnMatcherSetting(param + "/knn_matcher_setting", n);
  setting.peak_finder_setting = loadPeakFinderSetting(param + "/peak_finder_setting", n);

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}
/* ************************************************************************** */
comap::LoopClosureMatcherSetting loadLoopClosureMatcherSetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::LoopClosureMatcherSetting setting;

  if (!pu::get(param + "/feature_size", setting.feature_size)) read_success = false;
  if (!pu::get(param + "/match_lowe_ratio", setting.match_lowe_ratio)) read_success = false;
  if (!pu::get(param + "/ransac_sigma", setting.ransac_sigma)) read_success = false;
  if (!pu::get(param + "/ransac_confidence", setting.ransac_confidence)) read_success = false;
  if (!pu::get(param + "/ransac_maxiter", setting.ransac_maxiter)) read_success = false;
  if (!pu::get(param + "/ransac_min_inliers", setting.ransac_min_inliers)) read_success = false;

  // sub-method settings
  //setting.icp_config = loadPointMatcherSetting(param + "/icp_config", n);

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}

/* ************************************************************************** */
comap::SmallEMSetting loadSmallEMSetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::SmallEMSetting setting;

  if (!pu::get(param + "/inlier_prior", setting.inlier_prior)) read_success = false;
  if (!pu::get(param + "/min_inlier_prob", setting.min_inlier_prob)) read_success = false;
  double model_trans, model_rot;
  if (!pu::get(param + "/inlier_model_trans", model_trans)) read_success = false;
  if (!pu::get(param + "/inlier_model_rot", model_rot)) read_success = false;
  Eigen::VectorXd noise(6);
  noise << model_trans, model_trans, model_trans, model_rot, model_rot, model_rot;
  setting.inlier_model = gtsam::noiseModel::Diagonal::Sigmas(noise);
  if (!pu::get(param + "/outlier_model_trans", model_trans)) read_success = false;
  if (!pu::get(param + "/outlier_model_rot", model_rot)) read_success = false;
  Eigen::VectorXd out_noise(6);
  out_noise << model_trans, model_trans, model_trans, model_rot, model_rot, model_rot;
  setting.outlier_model = gtsam::noiseModel::Diagonal::Sigmas(out_noise);

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}

/* ************************************************************************** */
comap::CRPSetting loadCRPSetting(const std::string& param, const ros::NodeHandle& n) {

  bool read_success = true;
  comap::CRPSetting setting;

  if (!pu::get(param + "/flag_grid", setting.flag_grid)) read_success = false;
  if (!pu::get(param + "/grid_size", setting.grid_size)) read_success = false;
  if (!pu::get(param + "/alpha", setting.alpha)) read_success = false;

  if (!read_success)
    ROS_ERROR("failed to load Agent parameters");

  return setting;
}

comap::Matcher3DSetting loadMatcherSetting(const std::string& param, const ros::NodeHandle&n ) {

  comap::Matcher3DSetting setting;
  bool read_success = true;

  // Load frames
  std::string fixed_frame_id, dma_frame_id;
  if (!pu::get("frame_id/fixed", fixed_frame_id)) read_success = false;
  if (!pu::get("frame_id/dma", dma_frame_id)) read_success = false;

  // Lilter, normal, feature, and keypoint parameters
  if (!pu::get(param + "/pcl/voxel_grid_leaf_size", setting.voxel_grid_leaf_size)) read_success = false;
  if (!pu::get(param + "/pcl/match_grid_leaf_size", setting.match_grid_leaf_size)) read_success = false;
  if (!pu::get(param + "/pcl/normal_radius", setting.normal_radius)) read_success = false;
  if (!pu::get(param + "/pcl/keypoint/min_scale", setting.min_scale)) read_success = false;
  if (!pu::get(param + "/pcl/keypoint/n_octaves", setting.n_octaves)) read_success = false;
  if (!pu::get(param + "/pcl/keypoint/n_scales_per_octave", setting.n_scales_per_octave)) read_success = false;
  if (!pu::get(param + "/pcl/keypoint/min_contrast", setting.min_contrast)) read_success = false;
  if (!pu::get(param + "/pcl/iss/support_radius", setting.support_radius)) read_success = false;
  if (!pu::get(param + "/pcl/iss/nms_radius", setting.nms_radius)) read_success = false;
  if (!pu::get(param + "/pcl/feature_radius", setting.feature_radius)) read_success = false;
  if (!pu::get(param + "/pcl/registration/max_iterations", setting.max_iterations)) read_success = false;
  if (!pu::get(param + "/pcl/registration/feature_max_iterations", setting.feature_max_iterations)) read_success = false;
  if (!pu::get(param + "/pcl/registration/n_samples", setting.n_samples)) read_success = false;
  if (!pu::get(param + "/pcl/registration/correspondence_rand", setting.correspondence_rand)) read_success = false;
  if (!pu::get(param + "/pcl/registration/sim_threshold", setting.sim_threshold)) read_success = false;
  if (!pu::get(param + "/pcl/registration/max_correspondence_dist", setting.max_correspondence_dist)) read_success = false;
  if (!pu::get(param + "/pcl/registration/feature_max_correspondence_dist", setting.max_correspondence_dist)) read_success = false;
  if (!pu::get(param + "/pcl/registration/inlier_fraction", setting.inlier_fraction)) read_success = false;
  if (!pu::get(param + "/pcl/registration/min_sample_distance", setting.min_sample_distance)) read_success = false;
  if (!pu::get(param + "/pcl/registration/error_threshold", setting.error_threshold)) read_success = false;
  if (!pu::get(param + "/pcl/registration/convergence_threshold", setting.convergence_threshold)) read_success = false;
  if (!pu::get(param + "/pcl/registration/visualize_scan_matching", setting.visualize_scan_matching)) read_success = false;
  if (!pu::get(param + "/pcl/registration/feature_align", setting.feature_align)) read_success = false;

  float x,y,z;
  if (!pu::get(param + "/pcl/crop_box/x_limit", x)) read_success = false;
  if (!pu::get(param + "/pcl/crop_box/y_limit", y)) read_success = false;
  if (!pu::get(param + "/pcl/crop_box/z_limit", z)) read_success = false;

  if (!pu::get(param + "/z_range", setting.z_range)) read_success = false;

  Eigen::Vector4f max_pt(x,y,z, 1);
  setting.max_pt = max_pt;

  Eigen::Vector4f min_pt(-x, -y, -z, 1);
  setting.min_pt = min_pt;

  if (!pu::get(param + "/pcl/slice/x_limit", x)) read_success = false;
  if (!pu::get(param + "/pcl/slice/y_limit", y)) read_success = false;
  if (!pu::get(param + "/pcl/slice/z_limit", z)) read_success = false;

  Eigen::Vector4f max_pt_slice(x,y,z, 1);
  setting.max_pt_slice = max_pt_slice;

  Eigen::Vector4f min_pt_slice(-x, -y, -z, 1);
  setting.min_pt_slice = min_pt_slice;

  if (!pu::get(param + "/pcl/column/x_limit", x)) read_success = false;
  if (!pu::get(param + "/pcl/column/y_limit", y)) read_success = false;
  if (!pu::get(param + "/pcl/column/z_limit", z)) read_success = false;

  Eigen::Vector4f max_pt_col(x,y,z, 1);
  setting.max_pt_col = max_pt_col;

  Eigen::Vector4f min_pt_col(-x, -y, -z, 1);
  setting.min_pt_col = min_pt_col;

  int min_cache;
  if (!pu::get(param + "/min_cache_size", min_cache)) read_success = false;
  setting.min_cache_size = min_cache;

  int tree_nr;
  if (!pu::get(param + "/tree_nr", tree_nr)) read_success = false;
  setting.tree_nr = tree_nr;

  int K;
  if (!pu::get(param + "/K", K)) read_success = false;
  setting.K = K;

  int peak_threshold;
  if (!pu::get(param + "/peak_threshold", K)) read_success = false;
  setting.peak_threshold = peak_threshold;

  if (!pu::get(param + "/flann_radius", setting.flann_radius)) read_success = false;

  if (!pu::get(param + "/sensor_inverted", setting.sensor_inverted)) read_success = false;

  if (!pu::get("world_frame", setting.world_frame)) read_success = false;

  if (!read_success)
    ROS_ERROR("Failed to load matcher3d parameters");

  return setting;

}
