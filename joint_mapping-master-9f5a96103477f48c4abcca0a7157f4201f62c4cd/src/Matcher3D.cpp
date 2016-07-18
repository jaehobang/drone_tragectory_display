#include <joint_mapping/Matcher3D.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

#include <iostream>
#include <istream>

// TODO make use of OpenMP versions of pcl things
// TODO Split point cloud manipulation from matcher stuff

namespace pu = parameter_utils;

namespace comap {

  void Matcher3D::twoDSlice(PointCloudT::Ptr in, PointCloudT::Ptr out) {
    // Use pass through filter to get everything out of a certain z range
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (in);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.1, setting_.z_range);
    pass.filter (*out);

  }

  // The point cloud is a 2D scan so now we use the 2d stuff
  sensor_msgs::PointCloud2::Ptr Matcher3D::addLocalScan2D(sensor_msgs::PointCloud2& scan_cloud, PoseD pose, size_t pose_count) {
    PCLPointCloudStamped scan;
    scan_cloud.header.frame_id = setting_.world_frame;

    ROSToPCL(scan_cloud, scan);
    PointCloudT::Ptr target_ptr(new PointCloudT);
    pcl::copyPointCloud(scan.points, *target_ptr);
    std::cout << " The original has : "<< target_ptr->points.size() << std::endl;

    //Convert pcl scan to eigen
    Eigen::MatrixXd scan_eigen(2,target_ptr->size());
    for (unsigned int i = 0; i < target_ptr->size(); i++) {
      scan_eigen(0,i) = target_ptr->points[i].x;
      scan_eigen(1,i) = target_ptr->points[i].y;
    }
    LaserScan2D scan_2d;
    scan_2d.initialize(scan_eigen);

    pose_cache_[pose_count] = pose;
    matcher2d_->addLocalScan(scan_2d, pose_count);
  }

  std::vector<LoopResult> Matcher3D::findLoopClosure2D(sensor_msgs::PointCloud2 query_cloud, std::string foreign_name, size_t foreign_pose_count, PoseD foreign_pose) {
    PCLPointCloudStamped scan;
    query_cloud.header.frame_id = setting_.world_frame;

    ROSToPCL(query_cloud, scan);
    PointCloudT::Ptr target_ptr(new PointCloudT);
    pcl::copyPointCloud(scan.points, *target_ptr);
    std::cout << " The original has : "<< target_ptr->points.size() << std::endl;

    //Convert pcl scan to eigen
    Eigen::MatrixXd scan_eigen(2,target_ptr->size());
    for (unsigned int i = 0; i < target_ptr->size(); i++) {
      scan_eigen(0,i) = target_ptr->points[i].x;
      scan_eigen(1,i) = target_ptr->points[i].y;
    }
    LaserScan2D scan_2d;
    scan_2d.initialize(scan_eigen);

    std::vector<LoopResult2D> results = matcher2d_->findLoopClosure(scan_2d);

    std::vector<LoopResult> results3d;
    for (int i=0; i < results.size(); i++){
      LoopResult result3d;
      results3d.scan_idx = results[i].loop_idx;

      gtsam::Pose3 dpose(results[i].delta_pose);
      // Use the delta height to pouplate the delta_pose
      // TODO add roll and pitch
      double dz = pose_cache_[results[i].loop_idx].z() - foreign_pose.z();
      PoseD height_fix = PoseD(Rot3(), Point3(0,0,dz));
      dpose.compose(height_fix);
      result3d.delta_pose = dpose;
      result3d.success = true;

      results3d.push_back(result3d);
    }
    return results3d;
  }

  sensor_msgs::PointCloud2::Ptr Matcher3D::addLocalScan3D(sensor_msgs::PointCloud2& scan_cloud, PoseD pose, size_t pose_count) {

    Logger::instance()->startTimer("Local_Descriptor");
    PointCloudT::Ptr ref_downsampled(new PointCloudT);
    PointCloudNT::Ptr ref_normals(new PointCloudNT);
    PointCloudWST::Ptr ref_keypoints(new PointCloudWST);
    FeatureCloudT::Ptr ref_descriptors(new FeatureCloudT);

    std::string old_frame = scan_cloud.header.frame_id;
    scan_cloud.header.frame_id = setting_.world_frame;
    std_msgs::Header header = getDownsampledFromROS(scan_cloud, ref_downsampled);

    pcl::transformPointCloud(*ref_downsampled, *ref_downsampled, base_to_sensor);

    computePFHFeaturePoints(ref_downsampled, ref_normals, ref_keypoints, ref_descriptors);
    Logger::instance()->endTimer();

    if (ref_descriptors->size() > 0) {
      sensor_msgs::PointCloud2 keypoints_ros;
      PointCloudT keypoints_noscale;
      pcl::copyPointCloud(*ref_keypoints, keypoints_noscale);
      PCLToROS(keypoints_noscale, header, keypoints_ros);

      keypoints_ros.header.frame_id = old_frame;
      Logger::instance()->sendLocalKeyPoints(keypoints_ros);

      Logger::instance()->sendScanPoints(scan_cloud.width,
					 ref_downsampled->points.size(),
					 ref_keypoints->points.size());

      Logger::instance()->startTimer("Adding_Local_to_KDTree");
      flann::Matrix<float> mat = transDescriptors2Matrix(ref_descriptors);
      if (!flann_init_) {
	pmatcher_ = MatcherType_Ptr(new MatcherType(mat,
						    flann::KDTreeIndexParams()));
	pmatcher_->buildIndex();
	flann_init_ = true;
      } else {
	pmatcher_->addPoints(mat);
      }
      Logger::instance()->endTimer();

      trained_scan_count_++;

      std::vector<size_t> this_scan;
      this_scan.assign(ref_descriptors->points.size(), pose_count);
      point_to_idx_.insert(point_to_idx_.end(), this_scan.begin(), this_scan.end());
      scan_idx_.push_back(pose_count);
      local_scan_cache_[pose_count] = scan_cloud;
      local_pcl_cache_[pose_count] = ref_downsampled;
      local_normals_cache_[pose_count] = ref_normals;
      local_keypoints_cache_[pose_count] = ref_keypoints;
      local_descriptors_cache_[pose_count] = ref_descriptors;
    }

    sensor_msgs::PointCloud2::Ptr downsampled_ros(new sensor_msgs::PointCloud2);
    PCLToROS(*ref_downsampled, header, *downsampled_ros);

    return downsampled_ros;
  }

  std::vector<LoopResult> Matcher3D::findLoopClosure(sensor_msgs::PointCloud2 query_cloud, std::string foreign_name, size_t foreign_pose_count) {

    std::vector<LoopResult> loops;
    if (trained_scan_count_ < setting_.min_cache_size) {
      std::cout << " Not enought trained scans " << std::endl;
      return loops;
    }

    Logger::instance()->startTimer("Descriptors_Foreign_Cloud");

    PointCloudT::Ptr query_downsampled(new PointCloudT);
    PointCloudNT::Ptr query_normals(new PointCloudNT);
    PointCloudWST::Ptr query_keypoints(new PointCloudWST);
    FeatureCloudT::Ptr query_descriptors(new FeatureCloudT);

    // The query is already downsampled and transformed in the the correct frame
    query_cloud.header.frame_id = setting_.world_frame;
    PCLPointCloudStamped out;
    ROSToPCL(query_cloud, out);
    pcl::copyPointCloud(out.points, *query_downsampled);
    computePFHFeaturePoints(query_downsampled, query_normals, query_keypoints, query_descriptors);

    Logger::instance()->endTimer();

    if (query_descriptors->size() == 0)
      return loops;

    Logger::instance()->startTimer("KNN_Search");
    flann::Matrix<float> mat = transDescriptors2Matrix(query_descriptors);
    std::vector<unsigned int> query_result_hist;
    query_result_hist.assign(scan_idx_.size(), 0);
    std::vector<std::vector<long unsigned int> > knnSearchIdx;
    std::vector<std::vector<float> > knnSquaredDistances;
    try {
      flann::SearchParams searchParams;
      searchParams.max_neighbors = setting_.K;
      pmatcher_->radiusSearch(mat, knnSearchIdx, knnSquaredDistances,
                              setting_.flann_radius, searchParams);
    } catch(  std::exception& e) {
      std::cout << "There was an error with knnsearch" << std::endl;
      std::cout << e.what() << std::endl;
    }

    try {
      // TODO test that these points are good matches
      for (size_t i = 0; i < knnSearchIdx.size(); i++) {
        for (size_t j = 0; j < knnSearchIdx[i].size(); j++) {
          size_t scan_pose_count = point_to_idx_[knnSearchIdx[i][j]];
          size_t index = find(scan_idx_.begin(), scan_idx_.end(),
                              scan_pose_count) - scan_idx_.begin();
          if (index < query_result_hist.size()) {
            query_result_hist[index]++;
          }
        }// for loop through K points
      } // for loop through the descriptors
    } catch( std::exception& e) {
      std::cout << "Error with making query result hist " << std::endl;
      std::cout << e.what() << std::endl;
    }

    Logger::instance()->endTimer();
    Logger::instance()->sendKnnHist(query_result_hist);

    // The scan indexes of the peaks** are potential matches.
    std::vector<size_t> potential_scan_matches = getPotentialMatches(query_result_hist, knnSearchIdx.size());

    for (size_t i = 0; i < potential_scan_matches.size(); i++) {
      Logger::instance()->startTimer("Registration");
      size_t pose_count = potential_scan_matches[i];
      //std::cout << "Matching between local " << pose_count << " and foreign " << foreign_pose_count << std::endl;
      LoopResult result = consensusRegistration(local_pcl_cache_[pose_count],
                                                query_downsampled,
                                                local_keypoints_cache_[pose_count],
                                                query_keypoints,
                                                local_descriptors_cache_[pose_count],
                                                query_descriptors);
      Logger::instance()->endTimer();

      if (result.success) {
        result.scan_idx = potential_scan_matches[i];
        loops.push_back(result);
        Logger::instance()->sendMatchedPointClouds(
                                                   robot_name_,
                                                   foreign_name,
                                                   result.scan_idx,
                                                   foreign_pose_count,
                                                   local_scan_cache_[result.scan_idx],
                                                   query_cloud,
                                                   result.delta_pose.x(),
                                                   result.delta_pose.y(),
                                                   result.delta_pose.rotation().yaw()
                                                  );

        /*
        if (robot_name_ == "rasl1") {
          pcl::io::savePCDFileASCII ("/home/rasl/data/scans/matched_local_"+boost::lexical_cast<std::string>(pose_count)+"_"+boost::lexical_cast<std::string>(foreign_pose_count) +".pcd", *local_pcl_cache_[pose_count]);
          pcl::io::savePCDFileASCII ("/home/rasl/data/scans/matched_foreign_"+boost::lexical_cast<std::string>(pose_count)+"_"+boost::lexical_cast<std::string>(foreign_pose_count) +".pcd", *query_downsampled);
        }
        */

      } /* else {
        // we didn't find a match
        if (robot_name_ == "rasl1") {
          pcl::io::savePCDFileASCII ("/home/rasl/data/scans/unmatched_local_"+boost::lexical_cast<std::string>(pose_count)+"_"+boost::lexical_cast<std::string>(foreign_pose_count) +".pcd", *local_pcl_cache_[pose_count]);
          pcl::io::savePCDFileASCII ("/home/rasl/data/scans/unmatched_foreign_"+boost::lexical_cast<std::string>(pose_count)+"_"+boost::lexical_cast<std::string>(foreign_pose_count) +".pcd", *query_downsampled);
        }
      }
      */
    }

    return loops;
  }

  std::vector<size_t> Matcher3D::getPotentialMatches(std::vector<unsigned int> query_result_hist, size_t search_size) {

    std::vector<size_t> potential_matches;

    // Peak threshold allows us to only select matches that have at least that fraction amount of total
    size_t threshold = setting_.peak_threshold * setting_.K * search_size;

    // Start with simply choosing the single best match
    size_t best_idx = 0;
    size_t best_count = 0;
    for (size_t i = 0; i < query_result_hist.size(); i++) {
      if (query_result_hist[i] > best_count) {
        best_idx = i;
        best_count = query_result_hist[i];
      }
      /*
         if (query_result_hist[i] > threshold) {
         potential_matches.push_back(scan_idx_[i]);
         }
         */
    }
    potential_matches.push_back(scan_idx_[best_idx]);

    return potential_matches;
  }

  LoopResult Matcher3D::consensusRegistration(const PointCloudT::Ptr& source_points,
                                              const PointCloudT::Ptr& target_points,
                                              const PointCloudWST::Ptr& source_keypoints,
                                              const PointCloudWST::Ptr& target_keypoints,
                                              const FeatureCloudT::Ptr& source_descriptors,
                                              const FeatureCloudT::Ptr& target_descriptors)
  {
    LoopResult result;
    PointCloudWST::Ptr init_aligned(new PointCloudWST);
    PointCloudT::Ptr aligned_source(new PointCloudT);
    PointCloudT::Ptr aligned(new PointCloudT);

    //Disabled since the keypoints are not aligned well (the data has no interesting features up/down)

    Eigen::Matrix4f initial_T;
    if (setting_.feature_align && source_keypoints->points.size() > 0 && target_keypoints->points.size() > 0) {
      pcl::SampleConsensusPrerejective<PointWST, PointWST, FeatureT> align;
      align.setInputSource(source_keypoints);
      align.setSourceFeatures(source_descriptors);
      align.setInputTarget(target_keypoints);
      align.setTargetFeatures(target_descriptors);

      align.setMaximumIterations(setting_.feature_max_iterations);
      align.setNumberOfSamples(setting_.n_samples);
      align.setCorrespondenceRandomness(2);
      align.setSimilarityThreshold(setting_.sim_threshold);
      align.setMaxCorrespondenceDistance(setting_.feature_max_correspondence_dist);
      align.setInlierFraction(setting_.inlier_fraction);

      // Perform the registration
      align.align(*init_aligned);

      if (align.hasConverged() && align.getFitnessScore() < setting_.error_threshold*2) {
        ROS_INFO("%s: Feature match successful", robot_name_.c_str());
        initial_T = align.getFinalTransformation();
        pcl::transformPointCloud(*source_points, *aligned_source, initial_T);
      } else {
        ROS_WARN("%s: Feature based Registration failed to converge", robot_name_.c_str());
        std::cout << " Only got " << align.getInliers().size() << std::endl;
        std::cout << " and a score of " << align.getFitnessScore() << std::endl;
        pcl::copyPointCloud(*source_points, *aligned_source);
        initial_T = Eigen::Matrix4f::Identity();
      }
    } else {
      pcl::copyPointCloud(*source_points, *aligned_source);
      initial_T = Eigen::Matrix4f::Identity();
    }

    Eigen::Matrix4f icp_transformation;
    Eigen::Matrix4f transformation;
    if (findGICPTransform(aligned_source, target_points, aligned, icp_transformation)) {
      transformation = icp_transformation * initial_T;
    } else {
      ROS_WARN("%s: ICP Registration failed to converge", robot_name_.c_str());
      result.scan_idx = 0;
      result.success = false;
      return result;
    }

    gtsam::Pose3 delta_pose(transformation.cast<double>());
    delta_pose = delta_pose.inverse();
    result.delta_pose = delta_pose;
    result.scan_idx = 0;
    result.success = true;

    pcl::transformPointCloud(*source_points, *aligned, transformation);

    delta_pose.translation().print("The delta translation is : ");
    std::cout << " and yaw of " << delta_pose.rotation().yaw() << std::endl;

    Logger::instance()->sendRelPoseEstimate(delta_pose.x(),
                                            delta_pose.y(),
                                            delta_pose.rotation().yaw());

    if (setting_.visualize_scan_matching) {

      pcl::visualization::PCLVisualizer viewer ("Registration");

      // Define R,G,B colors for the point cloud
      pcl::visualization::PointCloudColorHandlerCustom<PointT> source_cloud_color_handler (source_points, 255, 0, 0);
      // We add the point cloud to the viewer and pass the color handler
      viewer.addPointCloud (source_points, source_cloud_color_handler, "original_cloud");
      pcl::visualization::PointCloudColorHandlerCustom<PointT> target_cloud_color_handler (target_points, 0, 255, 0); // Green
      viewer.addPointCloud (target_points, target_cloud_color_handler, "target_cloud");

      pcl::visualization::PointCloudColorHandlerCustom<PointWST> source_key_cloud_color_handler (source_keypoints, 255, 0, 0);
      // We add the point cloud to the viewer and pass the color handler
      viewer.addPointCloud (source_keypoints, source_key_cloud_color_handler, "original_key_cloud");
      pcl::visualization::PointCloudColorHandlerCustom<PointWST> target_key_cloud_color_handler (target_keypoints, 100, 200, 0); // Green
      viewer.addPointCloud (target_keypoints, target_key_cloud_color_handler, "target_key_cloud");

      pcl::visualization::PointCloudColorHandlerCustom<PointT> transformed_cloud_color_handler (aligned, 0, 0, 255); // Blue
      viewer.addPointCloud (aligned, transformed_cloud_color_handler, "transformed_cloud");

      pcl::visualization::PointCloudColorHandlerCustom<PointT> sacia_cloud_color_handler (aligned_source, 120, 120, 120); // Blue
      viewer.addPointCloud (aligned_source, sacia_cloud_color_handler, "sacia_cloud");

      viewer.addCoordinateSystem (1.0, 0);
      viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "original_key_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "target_key_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sacia_cloud");

      while(!viewer.wasStopped()) {
        viewer.spinOnce();
      }
    }

    return result;
  }


  bool Matcher3D::findGICPTransform(const PointCloudT::Ptr& input,
                                  const PointCloudT::Ptr& target,
                                  const PointCloudT::Ptr& aligned,
                                  Eigen::Matrix4f& final_transform)
  {

    /*
       1. Take pseudo 2D slice and get initial x,y, yaw
       2. Take column slice of both and transform input by initial x, y, yaw
       3. Use ICP again and match for height, roll and pitch
       */

    PointCloudT::Ptr input_slice(new PointCloudT);
    PointCloudT::Ptr input_column(new PointCloudT);
    PointCloudT::Ptr target_slice(new PointCloudT);
    PointCloudT::Ptr target_column(new PointCloudT);

    cropBox(input, input_slice, setting_.min_pt_slice, setting_.max_pt_slice);
    cropBox(target, target_slice, setting_.min_pt_slice, setting_.max_pt_slice);

    cropBox(input, input_column, setting_.min_pt_col, setting_.max_pt_col);
    cropBox(target, target_column, setting_.min_pt_col, setting_.max_pt_col);

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setCorrespondenceRandomness(setting_.correspondence_rand);
    gicp.setMaxCorrespondenceDistance (setting_.max_correspondence_dist);
    gicp.setRANSACOutlierRejectionThreshold (setting_.max_correspondence_dist);
    gicp.setTransformationEpsilon(1e-8);
    gicp.setMaximumIterations (setting_.max_iterations);
    gicp.setEuclideanFitnessEpsilon (setting_.convergence_threshold);

    gicp.setInputSource (input_slice);
    gicp.setInputTarget (target_slice);

    gicp.align (*aligned);

    if (gicp.hasConverged()) {
      std::cout << "ICP Converged with fitness score of " << gicp.getFitnessScore() << std::endl;
      Eigen::Matrix4f init_transform =  gicp.getFinalTransformation();

      PointCloudT::Ptr input_column_transformed (new PointCloudT);
      pcl::transformPointCloud (*input_column, *input_column_transformed, init_transform);

      gicp.setInputSource(input_column_transformed);
      gicp.setInputTarget(target_column);

      gicp.align (*aligned);

      if (gicp.hasConverged() &&  gicp.getFitnessScore() < setting_.error_threshold) {
        std::cout << "Second ICP converged with fitness score of " << gicp.getFitnessScore() << std::endl;
        Eigen::Matrix4f transform = gicp.getFinalTransformation();

        final_transform = (transform * init_transform);
        return true;
      }

    } else {
      std::cout << "ICP did not converge, had fitness score of " << gicp.getFitnessScore() << std::endl;
    }
    return false;
  }


  // Take away the rotation and crop that z
  sensor_msgs::PointCloud2 Matcher3D::crop_z( sensor_msgs::PointCloud2& scan, PoseD pose ) {

    PCLPointCloudStamped scan_pcl;
    ROSToPCL(scan, scan_pcl);

    PointCloudT::Ptr input_scan(new PointCloudT);
    PointCloudT::Ptr rotated_scan(new PointCloudT);
    PointCloudT::Ptr cropped_scan(new PointCloudT);

    pcl::copyPointCloud(scan_pcl.points, *input_scan);

    // Invert the transformation so we get a level response but zero the transform
    Eigen::Matrix4f transformation= pose.inverse().matrix().cast<float>();
    transformation(0,3) = 0;
    transformation(1,3) = 0;
    transformation(2,3) = 0;

    pcl::transformPointCloud(*input_scan, *rotated_scan, transformation);
    twoDSlice(rotated_scan, cropped_scan);

    transformation = transformation.inverse();

    pcl::transformPointCloud(*cropped_scan, *input_scan, transformation);

    sensor_msgs::PointCloud2 out;
    PCLToROS(*input_scan, scan.header, out);
    return out;
  }

  sensor_msgs::PointCloud2 Matcher3D::project( sensor_msgs::PointCloud2& scan,
                                    PoseD robot_pose,
                                    PoseD relative_pose)
  {
    scan.header.frame_id = setting_.world_frame;
    Eigen::Matrix4f total = relative_pose.matrix().cast<float>() * robot_pose.matrix().cast<float>() ;
    sensor_msgs::PointCloud2 out;
    pcl_ros::transformPointCloud(total, scan, out);
    out.header.frame_id = setting_.world_frame;

    return out;
  }

  std_msgs::Header Matcher3D::getDownsampledFromROS(const sensor_msgs::PointCloud2& scan_cloud,
                                                    PointCloudT::Ptr downsampled)
  {
    PCLPointCloudStamped scan;
    ROSToPCL(scan_cloud, scan);
    PointCloudT::Ptr target_ptr(new PointCloudT);
    pcl::copyPointCloud(scan.points, *target_ptr);
    std::cout << " The original has : "<< target_ptr->points.size() << std::endl;
    voxelGridDownsample(target_ptr, downsampled, setting_.voxel_grid_leaf_size);
    // TODO Try different downsampling techniques
    std::cout << " The downsampled version has : " << downsampled->points.size() << std::endl;
    return scan.header;
  }

  void Matcher3D::computeFPFHFeaturePoints(PointCloudT::Ptr downsampled_in,
                                           PointCloudWST::Ptr keypoints_out,
                                           FastFeatureCloudT::Ptr descriptors_out)
  {
    PointCloudNT::Ptr normals(new PointCloudNT);
    computeSurfaceNormals(downsampled_in, normals);
    PointCloudI::Ptr keypoints(new PointCloudI);
    detectISSKeyPoints(downsampled_in, keypoints);
    pcl::copyPointCloud(*keypoints, *keypoints_out);
    //detectKeyPoints(downsampled_in, normals, keypoints_out);
    PointCloudT::Ptr keypoints_noscale(new PointCloudT);
    pcl::copyPointCloud(*keypoints_out, *keypoints_noscale);
    computeFPFHFeatures(downsampled_in, keypoints_noscale, normals, descriptors_out);
  }

  void Matcher3D::computePFHFeaturePoints(PointCloudT::Ptr downsampled_in,
                                          PointCloudNT::Ptr normals_out,
                                          PointCloudWST::Ptr keypoints_out,
                                          FeatureCloudT::Ptr descriptors_out)
  {
    computeSurfaceNormals(downsampled_in, normals_out);
    std::cout << "Computer surface normals num:  " << normals_out->points.size() << std::endl;
    //detectKeyPoints(downsampled_in, normals_out, keypoints_out);
    PointCloudI::Ptr keypoints(new PointCloudI);
    detectISSKeyPoints(downsampled_in, keypoints);
    pcl::copyPointCloud(*keypoints, *keypoints_out);
    std::cout << "Number of keypoints: " <<  keypoints_out->points.size() << std::endl;
    PointCloudT::Ptr keypoints_noscale(new PointCloudT);
    pcl::copyPointCloud(*keypoints_out, *keypoints_noscale);
    computePFHFeatures(downsampled_in, keypoints_noscale, normals_out, descriptors_out);
    std::cout << "Descriptors count : " << descriptors_out->points.size() << std::endl;
  }

  void Matcher3D::cropBox(const PointCloudT::Ptr in,
               const PointCloudT::Ptr out,
               Eigen::Vector4f min_pt,
               Eigen::Vector4f max_pt)
  {
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setMax(max_pt);
    crop_box.setMin(min_pt);

    crop_box.setInputCloud(in);
    crop_box.filter(*out);
  }

  void Matcher3D::voxelGridDownsample(const PointCloudT::Ptr& in,
                                      PointCloudT::Ptr& out,
                                      float leaf_size)
  {
    /* Because of size of scan, we use crop box to limit the size and then filter */
    PointCloudT::Ptr cropped(new PointCloudT);
    cropBox(in, cropped, setting_.min_pt, setting_.max_pt);

    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setLeafSize(leaf_size,
                           leaf_size,
                           leaf_size);
    voxel_grid.setInputCloud(cropped);
    voxel_grid.filter(*out);
  }

  void Matcher3D::computeSurfaceNormals(const PointCloudT::Ptr& in,
                                        PointCloudNT::Ptr& out)
  {
    pcl::NormalEstimation<PointT, PointNT> norm_est;
    norm_est.setSearchMethod(pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
    norm_est.setRadiusSearch(setting_.normal_radius);
    norm_est.setInputCloud(in);
    norm_est.compute (*out);
  }

  void Matcher3D::detectKeyPoints(const PointCloudI::Ptr& in,
                                  PointCloudI::Ptr& out)
  {
    pcl::SIFTKeypoint<PointI, PointI> sift_detect;
    sift_detect.setSearchMethod(pcl::search::KdTree<PointI>::Ptr (new pcl::search::KdTree<PointI>));
    sift_detect.setScales(setting_.min_scale, setting_.n_octaves, setting_.n_scales_per_octave);
    sift_detect.setMinimumContrast(setting_.min_contrast);
    sift_detect.setInputCloud(in);
    sift_detect.compute(*out);
  }

  void Matcher3D::detectISSKeyPoints(const PointCloudT::Ptr& in,
                          const PointCloudI::Ptr& out)
  {
    PointCloudT::Ptr keypoints (new PointCloudT());
    pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
    iss_detector.setSalientRadius (setting_.support_radius);
    iss_detector.setNonMaxRadius (setting_.nms_radius);
    iss_detector.setInputCloud (in);
    iss_detector.compute (*keypoints);
    pcl::copyPointCloud(*keypoints, *out);
  }

  void Matcher3D::detectKeyPoints(const PointCloudT::Ptr& in,
                                  const PointCloudNT::Ptr& normals,
                                  PointCloudWST::Ptr& out)
  {
    // Have to copy points from in to normal
    if (in->size() > 0 && in->size() == normals->size()) {
      for (size_t i = 0; i < normals->points.size(); ++i) {
        normals->points[i].x = in->points[i].x;
        normals->points[i].y = in->points[i].y;
        normals->points[i].z = in->points[i].z;
      }

      pcl::SIFTKeypoint<PointNT, pcl::PointWithScale> sift_detect;
      sift_detect.setSearchMethod(pcl::search::KdTree<PointNT>::Ptr (new pcl::search::KdTree<PointNT>));
      sift_detect.setScales(setting_.min_scale, setting_.n_octaves, setting_.n_scales_per_octave);
      sift_detect.setMinimumContrast(setting_.min_contrast);
      sift_detect.setInputCloud(normals);
      sift_detect.compute(*out);
    }
  }

  void Matcher3D::computeFPFHFeatures(const PointCloudT::Ptr& in_surface,
                                      const PointCloudT::Ptr& in_points,
                                      const PointCloudNT::Ptr& in_normals,
                                      const FastFeatureCloudT::Ptr& out_descriptors)
  {
    if (in_points->size() > 0 && in_surface->size() == in_normals->size()) {
      pcl::FPFHEstimation<PointT, PointNT, FastFeatureT> fpfh_est;
      fpfh_est.setInputCloud (in_points);

      for (size_t i = 0; i < in_normals->points.size(); ++i) {
        in_normals->points[i].x = in_surface->points[i].x;
        in_normals->points[i].y = in_surface->points[i].y;
        in_normals->points[i].z = in_surface->points[i].z;
      }

      fpfh_est.setInputNormals (in_normals);
      fpfh_est.setSearchSurface (in_surface);
      fpfh_est.setSearchMethod(pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
      fpfh_est.setRadiusSearch (setting_.feature_radius);
      fpfh_est.compute (*out_descriptors);
    }
  }

  void Matcher3D::computePFHFeatures(const PointCloudT::Ptr& in_surface,
                                     const PointCloudT::Ptr& in_points,
                                     const PointCloudNT::Ptr& in_normals,
                                     FeatureCloudT::Ptr& out)
  {
    if (in_points->size() > 0 && in_surface->size() == in_normals->size()) {
      for (size_t i = 0; i < in_normals->points.size(); ++i) {
        in_normals->points[i].x = in_surface->points[i].x;
        in_normals->points[i].y = in_surface->points[i].y;
        in_normals->points[i].z = in_surface->points[i].z;
      }

      FeatureEstimationT pfh_est;
      pfh_est.setSearchMethod(pcl::search::KdTree<PointT>::Ptr (new pcl::search::KdTree<PointT>));
      pfh_est.setRadiusSearch(setting_.feature_radius);
      pfh_est.setSearchSurface(in_surface);
      pfh_est.setInputNormals(in_normals);
      pfh_est.setInputCloud(in_points);
      pfh_est.compute(*out);
    }
  }

  flann::Matrix<float> Matcher3D::transDescriptors2Matrix(FeatureCloudT::Ptr descriptors) {
    // Feature size is 125 or 33 for Fast
    size_t feature_size = 125;
    float* cache = new float[feature_size * descriptors->points.size()];
    if (cache == NULL)
      throw std::runtime_error("memory allocation error");

    // copy memory
    for (size_t i = 0; i < descriptors->points.size(); i++)
      for (size_t j = 0; j < feature_size; j++)
        cache[i*feature_size + j] = descriptors->points[i].histogram[j];

    flann::Matrix<float> mat =  flann::Matrix<float>(cache, descriptors->points.size(), feature_size);
    delete cache;
    return mat;
  }

  void Matcher3D::ROSToPCL(const sensor_msgs::PointCloud2& in,
                           PCLPointCloudStamped& out)
  {
    // Slow - need to do two conversions to satisfy ROS Hydro point clouds
    out.header = in.header;
    pcl::PCLPointCloud2 pcl2;
    pcl_conversions::toPCL(in, pcl2);
    pcl::fromPCLPointCloud2(pcl2, out.points);
  }

  void Matcher3D::PCLToROS(const PointCloudT& in,
                           std_msgs::Header header,
                           sensor_msgs::PointCloud2& out)
  {
    // Slow - need to do two conversions to satisfy ROS Hydro point clouds
    out.header = header;
    pcl::PCLPointCloud2 pcl2;
    pcl::toPCLPointCloud2(in, pcl2);
    pcl_conversions::fromPCL(pcl2, out);
    //sensor_msgs::convertPointCloud2ToPointCloud(ros_pcld2, ros_pcld);
  }

  gtsam::NonlinearFactorGraph Matcher3D::findLocalLoopClosure(
                  const PoseD slam_pose, comap::Scan& scan) {
    gtsam::NonlinearFactorGraph graph;

    // for now, ignore local loop closures, but

    return graph;
  }
} // comap end
