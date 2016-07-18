#ifndef MATCHER
#define MATCHER

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <parameter_utils/ParameterUtils.h>

//#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <flann/flann.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>

#include <pcl/registration/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_box.h>

#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include <joint_mapping/Agent_Definitions.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose3.h>

#include <joint_mapping/Logger.h>
#include <joint_mapping/Matcher2D.h>

namespace comap {

  typedef pcl::PointXYZ PointT;
  typedef pcl::PointXYZI PointI;
  typedef pcl::PointXYZRGBA PointCT;
  typedef pcl::PointNormal PointNT;
  typedef pcl::PointWithScale PointWST;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef pcl::PointCloud<PointI> PointCloudI;
  typedef pcl::PointCloud<PointNT> PointCloudNT;
  typedef pcl::PointCloud<PointCT> PointCloudCT;
  typedef pcl::PointCloud<PointWST> PointCloudWST;
  //typedef pcl::FPFHSignature125 FeatureT;
  //typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
  typedef pcl::PFHSignature125 FeatureT;
  typedef pcl::FPFHSignature33 FastFeatureT;
  typedef pcl::PFHEstimation<PointT,PointNT,FeatureT> FeatureEstimationT;
  typedef pcl::PointCloud<FeatureT> FeatureCloudT;
  typedef pcl::PointCloud<FastFeatureT> FastFeatureCloudT;

  struct Matcher3DSetting {
    // PCL filter parameters
    float voxel_grid_leaf_size;
    float match_grid_leaf_size;
    float normal_radius;
    float min_scale;
    int n_octaves;
    int n_scales_per_octave;
    float min_contrast;
    float feature_radius;
    int max_iterations;
    int feature_max_iterations;
    int n_samples;
    int correspondence_rand;
    float sim_threshold;
    float max_correspondence_dist;
    float feature_max_correspondence_dist;
    float inlier_fraction;
    float flann_radius;

    float support_radius;
    float nms_radius;

    float min_sample_distance;
    float convergence_threshold;
    float error_threshold;

    bool visualize_scan_matching;
    bool feature_align;

    std::string world_frame;

    size_t tree_nr;
    size_t min_cache_size;
    size_t K;

    float z_range;

    size_t peak_threshold;
    bool sensor_inverted;

    // Used in pass through filter to limit size of scan
    Eigen::Vector4f max_pt;
    Eigen::Vector4f min_pt;

    Eigen::Vector4f max_pt_slice;
    Eigen::Vector4f min_pt_slice;

    Eigen::Vector4f max_pt_col;
    Eigen::Vector4f min_pt_col;

  };

  struct LoopResult {
    // if scan_idx = -1, match didn't work
    bool success;
    size_t scan_idx;
    gtsam::Pose3 delta_pose;
  };

  struct PCLPointCloudStamped
  {
    std_msgs::Header header;
    PointCloudT points;
  };

  // Wrapper class that
  class Matcher3D {

  private:

    size_t trained_scan_count_;
    // Maps from pose_count_ to filtered/down-sampled point cloud
    std::map<size_t, sensor_msgs::PointCloud2> local_scan_cache_;
    std::map<size_t, PointCloudT::Ptr> local_pcl_cache_;
    std::map<size_t, PointCloudNT::Ptr> local_normals_cache_;
    std::map<size_t, PointCloudWST::Ptr> local_keypoints_cache_;
    std::map<size_t, FeatureCloudT::Ptr> local_descriptors_cache_;
    std::vector<size_t> point_to_idx_;
    std::vector<size_t> scan_idx_;

    std::map<size_t, PoseD> pose_cache_;

    Eigen::Affine3f base_to_sensor;
    std::string robot_name_;

    typedef flann::Index<flann::L2<float> > MatcherType;
    typedef boost::shared_ptr<MatcherType> MatcherType_Ptr;
    typedef boost::shared_ptr<const MatcherType> MatcherType_ConstPtr;


    bool flann_init_;
    MatcherType_Ptr pmatcher_;
    //  FeatureCloudT::Ptr feature_cloud_;

    std::vector<size_t> point_idx_to_pose_count_;

    //geometry_msgs::PoseWithCovarianceStamped::ConstPtr getIncrementalEstimate();
    //geometry_msgs::PoseStamped::ConstPtr getIntegratedEstimate();

    // Supply these PCL wrappers publicly; this class can be used for its utilities as well
    void voxelGridDownsample(const PointCloudT::Ptr& in,
                             PointCloudT::Ptr& out,
                             float leaf_size);

    void twoDSlice(PointCloudT::Ptr in, PointCloudT::Ptr out);

    void computeSurfaceNormals(const PointCloudT::Ptr& in,
                               PointCloudNT::Ptr& out);

    void detectKeyPoints(const PointCloudT::Ptr& in,
                         const PointCloudNT::Ptr& normals,
                         PointCloudWST::Ptr& out);

    void detectKeyPoints(const PointCloudI::Ptr& in,
                         PointCloudI::Ptr& out);

    void detectISSKeyPoints(const PointCloudT::Ptr& in,
                            const PointCloudI::Ptr& out);

    void computePFHFeatures(const PointCloudT::Ptr& in_surface,
                            const PointCloudT::Ptr& in_points,
                            const PointCloudNT::Ptr& in_normals,
                            FeatureCloudT::Ptr& out_descriptors);
    void computeFPFHFeatures(const PointCloudT::Ptr& in_surface,
                             const PointCloudT::Ptr& in_points,
                             const PointCloudNT::Ptr& in_normals,
                             const FastFeatureCloudT::Ptr& out_descriptors);

    void findFeatureCorrespondences(const FeatureCloudT::Ptr& source_descriptors,
                                    const FeatureCloudT::Ptr& target_descriptors,
                                    std::vector<int>& correspondences_out,
                                    std::vector<float>& correspondence_scores_out);

    void computeFPFHFeaturePoints(PointCloudT::Ptr downsampled_in,
                              PointCloudWST::Ptr keypoints_out,
                              FastFeatureCloudT::Ptr descriptors_out);

    void computePFHFeaturePoints(PointCloudT::Ptr downsampled_in,
                              PointCloudNT::Ptr normals_out,
                              PointCloudWST::Ptr keypoints_out,
                              FeatureCloudT::Ptr descriptors_out);
    bool findGICPTransform(const PointCloudT::Ptr& input,
                           const PointCloudT::Ptr& target,
                           const PointCloudT::Ptr& aligned,
                           Eigen::Matrix4f& final_transform);

    LoopResult consensusRegistration(const PointCloudT::Ptr& target_points,
                                     const PointCloudT::Ptr& reference_points,
                                     const PointCloudWST::Ptr& source_keypoints,
                                     const PointCloudWST::Ptr& target_keypoints,
                                     const FeatureCloudT::Ptr& target_descriptors,
                                     const FeatureCloudT::Ptr& reference_descriptors);

    void cropBox(const PointCloudT::Ptr in,
                 const PointCloudT::Ptr out,
                 Eigen::Vector4f min_pt,
                 Eigen::Vector4f max_pt);

    std::vector<size_t> getPotentialMatches(std::vector<unsigned int> query_result_hist,
                                            size_t search_size);

    flann::Matrix<float> transDescriptors2Matrix(FeatureCloudT::Ptr descriptors);

    std_msgs::Header getDownsampledFromROS(const sensor_msgs::PointCloud2& scan_cloud,
                                           PointCloudT::Ptr downsampled);

    void ROSToPCL(const sensor_msgs::PointCloud2& in,
                  PCLPointCloudStamped& out);

    void PCLToROS(const PointCloudT& in,
                  std_msgs::Header header,
                  sensor_msgs::PointCloud2& out);

    /*
       geometry_msgs::PoseWithCovarianceStamped::Ptr incremental_estimate;
       geometry_msgs::PoseStamped::Ptr integrated_estimate;

       std::string name;
       bool cloud_set;
       */

  public:
    Matcher3D();
    ~Matcher3D(){}

    Matcher3DSetting setting_;

    Matcher2D::Ptr matcher2d_;

    Matcher3D(Matcher3DSetting setting, std::string name) : setting_(setting), trained_scan_count_(0),
    flann_init_(false),
    pmatcher_(), robot_name_(name){
      base_to_sensor = Eigen::Affine3f::Identity();
      float pitch = 0;
      if (setting.sensor_inverted)
        pitch = 1.5707;
      base_to_sensor.rotate (Eigen::AngleAxisf (pitch, Eigen::Vector3f::UnitY()));
    }

    void initialize(Matcher3DSetting setting, std::string name, const PointMatcherSetting icp_config, const LoopClosureDetectorSetting loop_detect_setting, const LoopClosureMatcherSetting loop_match_setting) {
      setting_ = setting;
      trained_scan_count_ = 0;
      flann_init_ = false;
      robot_name_ = name;

      matcher2d_ = boost::shared_ptr<Matcher2D>(new Matcher2D(icp_config, loop_detect_setting, loop_match_setting, setting.min_cache_size));

    }

    /* Meant for local scans (these are the keyframes essentially)
       1. Decompose scan into keypoints and descriptors
       2. Store descriptors into octree for fast search later on
       */
    sensor_msgs::PointCloud2::Ptr addLocalScan(sensor_msgs::PointCloud2& scan, PoseD pose, size_t pose_count);

    sensor_msgs::PointCloud2::Ptr addLocalScan2D(sensor_msgs::PointCloud2& scan, PoseD pose, size_t pose_count);

    /* Given scan from foreign agent (or new local scan)
       1. Decompose into key points and descriptors for those points
       2. Search in octree for k nearest scans for each keypoint -> histogram
       3. Pick scan at peaks of histogram and try align with those scans
       4. Return the index of the scan query matched with and the relative pose
       between them.
       */
    std::vector<LoopResult> findLoopClosure(sensor_msgs::PointCloud2 query,
                                            std::string foreign_name,
                                            size_t foreign_pose_count);

    std::vector<LoopResult> findLoopClosure2D(sensor_msgs::PointCloud2 query,
                                            std::string foreign_name,
                                            size_t foreign_pose_count,
                                            PoseD foreign_pose);

    size_t get_num_trained_scan() { return trained_scan_count_; }

    sensor_msgs::PointCloud2 crop_z( sensor_msgs::PointCloud2& scan, PoseD pose );

    sensor_msgs::PointCloud2 project( sensor_msgs::PointCloud2& scan,
                                      PoseD robot_pose,
                                      PoseD relative_pose);

    gtsam::NonlinearFactorGraph findLocalLoopClosure( const PoseD slam_pose, comap::Scan& scan);

  };

  typedef Matcher3D Matcher;
}
#endif
