/**
 * @file   FeaturePoint.h
 * @brief  Feature Point and Descriptor classes
 * @author Jing Dong
 * @date   Aug 23, 2014
 */

#pragma once

#include <gtsam/geometry/Pose2.h>

#include <boost/shared_array.hpp>

#include <vector>
#include <fstream>

namespace comap {

/* ************************************************************************* */
// definition of data structures:

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

}  // namespace mast
