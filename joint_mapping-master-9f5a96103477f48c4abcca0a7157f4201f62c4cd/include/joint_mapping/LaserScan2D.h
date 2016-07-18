/*

Extends Scan and implements saliency and feature extraction

*/

#pragma once

#include <joint_mapping/Scan.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pointmatcher/PointMatcher.h>
#include <eigen3/Eigen/Dense>

#include <vector>

#include <iostream>

namespace comap {

class LaserScan2D: public Scan2D {

private:

  typedef PointMatcher<double> PMatcher;

  PMatcher::DataPointsFilter* pfilter_;
  double voxel_size;

  void initialize_filter() {
    voxel_size = 0.1;
    pfilter_ =
      PMatcher::get().DataPointsFilterRegistrar.create(
      "VoxelGridDataPointsFilter",
      PointMatcherSupport::map_list_of
      ("vSizeX", PointMatcherSupport::toParam(voxel_size))
      ("vSizeY", PointMatcherSupport::toParam(voxel_size))
      ("useCentroid", PointMatcherSupport::toParam(1)));
  }


protected:

  void calculate_homo() {
    scan_homo = Eigen::MatrixXd(3, point_count);
    scan_homo.block(0,0,2,point_count) = scan.block(0,0,2,point_count);
    scan_homo.block(2,0,1,point_count) = Eigen::MatrixXd::Ones(1,point_count);

    homography_calculated = true;
  }

  void calculate_filter() {
    PMatcher::DataPoints::Labels label;
    label.push_back(PMatcher::DataPoints::Label("x", 1));
    label.push_back(PMatcher::DataPoints::Label("y", 1));
    label.push_back(PMatcher::DataPoints::Label("pad", 1));

    // a local processable copy
    PMatcher::DataPoints scan_dp(scan_homo, label);

    // in-place apply filter
    pfilter_->inPlaceFilter(scan_dp);

    scan_homo_filtered = scan_dp.features;
    filter_calculated = true;
  }

public:

  LaserScan2D() {}
  ~LaserScan2D() {}

  void initialize(Eigen::MatrixXd scan_matrix) {
    point_count = scan_matrix.cols();
    scan = scan_matrix;
    initialize_filter();
    homography_calculated = false;
    filter_calculated = false;
  }

  void initialize(sensor_msgs::PointCloud cloud) {
    scan = Eigen::MatrixXd(2, cloud.points.size());
    for (unsigned int ii = 0; ii < cloud.points.size(); ++ii)
    {
      scan(0, ii) = cloud.points[ii].x;
      scan(1, ii) = cloud.points[ii].y;
    }

    point_count = cloud.points.size();
    initialize_filter();
    homography_calculated = false;
    filter_calculated = false;
  }

  double get_saliency() {
    return 1.0;
  }

};
}
