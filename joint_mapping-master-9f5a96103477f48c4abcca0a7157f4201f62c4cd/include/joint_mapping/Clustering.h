/*
 * @file Clustering.h
 * @brief clustering tool when generating hypothesis
 * @author Jing Dong
 * @date July 7, 2014
 */

#pragma once

#include <gtsam/nonlinear/Values.h>
#include <joint_mapping/Settings.h>

#include <vector>

namespace comap {

// preform clustering (vadim's histogram method), without EM and merging
// input: given all relative poses, and setting
// output: clustered poses
std::vector<PoseD> clusterRelativePoses(const std::vector<PoseD>& all_poses,
    const ClusteringSetting& setting);


// use measurements and local trajectory to get original relative poses
std::vector<PoseD> originRelativePose(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<PoseD>& measure_poses,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value);


/* ************************************************************************* */
// clustering a single channel (x/y/theta), output possible poes
std::vector<double> cluster_vector(const std::vector<double>& data, double res, size_t cluster_number);

// histogram for double scalar class
struct Histogram {

  // normalized count
  std::vector<double> count;
  // bin centers
  std::vector<double> bin_center;

  // utils
  void print();
};

// get normalized histogram for a double vector, given optional resolution
// if resolution not give, use bin number = data number
Histogram histogram(const std::vector<double>& data, double res = 0.0);


// pair index for index-remained sorting
typedef std::pair<size_t, size_t> uintidx;  // first one is sorted
bool comparator_uint(const uintidx& l, const uintidx& r);

// pair index for index-remained sorting
typedef std::pair<double, size_t> doubleidx;
bool comparator_double(const doubleidx& l, const doubleidx& r);


}   // namespace mast
