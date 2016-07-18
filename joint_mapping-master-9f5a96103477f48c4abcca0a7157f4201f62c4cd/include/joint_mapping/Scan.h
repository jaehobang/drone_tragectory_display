/*

Abstract Class representing a scan. Can be 2D laser scan from hokyu or 3D Scan2D from puck

*/

#pragma once

#include <joint_mapping/Agent_Definitions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <eigen3/Eigen/Dense>

#include <iostream>

namespace comap {

class Scan2D {

public:

  virtual ~Scan2D() {}

  Eigen::MatrixXd homo() {
    if (!homography_calculated) {
      this->calculate_homo();
    }

    return scan_homo;
  }
  virtual void initialize(Eigen::MatrixXd scan_matrix) = 0;
  virtual void initialize(sensor_msgs::PointCloud cloud) = 0;
//  virtual void initialize(sensor_msgs::PointCloud2 cloud);
  virtual double get_saliency() = 0;
  size_t num_points() { return point_count; }

  Eigen::MatrixXd filtered() {
    if (!homography_calculated) {
      this->calculate_homo();
    }

    if (!filter_calculated) {
      this->calculate_filter();
    }

    return scan_homo_filtered;
  }

  void setPose(PoseD pose_i) { pose = pose_i; }

protected:
  PoseD pose;
  size_t pose_count;
  size_t point_count; // the old scan cols
  Eigen::MatrixXd scan;
  Eigen::MatrixXd scan_homo;
  Eigen::MatrixXd scan_homo_filtered;

  bool homography_calculated;
  bool filter_calculated;

  virtual void calculate_homo() = 0;
  virtual void calculate_filter() = 0;

};
}
