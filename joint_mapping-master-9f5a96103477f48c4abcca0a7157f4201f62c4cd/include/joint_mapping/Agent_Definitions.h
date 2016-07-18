
#pragma once

#include <ros/ros.h>

#include <gtsam/nonlinear/Values.h>
// #include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <sensor_msgs/PointCloud2.h>

namespace comap {

// graph with init values together
typedef std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> GraphValues;

// Allows it to be easily changed
typedef gtsam::Pose3 PoseD;

//typedef LaserScan2D DerivedScan;
typedef sensor_msgs::PointCloud2::Ptr DerivedScan;

// only for 3D
typedef DerivedScan Scan;

}
