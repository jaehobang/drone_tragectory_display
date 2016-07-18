/*
 * @file Clustering.cpp
 * @brief clustering tool when generating hypothesis
 * @author Jing Dong
 * @date July 7, 2014
 */

#include <joint_mapping/Clustering.h>

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace gtsam;
using namespace std;

namespace comap {

/* ************************************************************************* */
// preform clustering (vadim's histogram method), without EM and merging
// input: given all relative poses, and setting
// output: clustered poses
// TODO: this method is too bad :( is there any better method to do this ?????????
std::vector<gtsam::Pose2> clusterRelativePoses(const std::vector<gtsam::Pose2>& all_poses,
    const ClusteringSetting& setting) {

  size_t nr_poses = all_poses.size();

  // decompose the pose vector to 3 scalar vector, to prefrom scalar clustering
  vector<double> x_data, y_data, theta_data;
  x_data.reserve(nr_poses);
  y_data.reserve(nr_poses);
  theta_data.reserve(nr_poses);

  for (size_t i = 0; i < nr_poses; i++) {
    x_data.push_back(all_poses.at(i).x());
    y_data.push_back(all_poses.at(i).y());
    theta_data.push_back(all_poses.at(i).theta());
  }

  vector<double> x_candidate = cluster_vector(x_data, setting.cluster_res_xy, setting.max_cluster);
  vector<double> y_candidate = cluster_vector(y_data, setting.cluster_res_xy, setting.max_cluster);
  vector<double> theta_candidate = cluster_vector(theta_data, setting.cluster_res_theta, setting.max_cluster);

  vector<Pose2> candidate_poses;
  vector<pair<size_t, size_t> > candidate_idxcount;

  // scan for the 'cross sections' of each candidates
  // TODO: this is not efficient enough ?!
  for (size_t xidx = 0; xidx < x_candidate.size(); xidx++) {
    for (size_t yidx = 0; yidx < y_candidate.size(); yidx++) {
      for (size_t thetaidx = 0; thetaidx < theta_candidate.size(); thetaidx++) {
        // for each datapose: check distance & angle
        // only keep hypothesis which has enough neighborhood in range
        size_t neighborhood_count = 0;
        for (size_t count = 0; count < nr_poses; count++) {
          // in the cluster
          if ((Point2(x_candidate.at(xidx), y_candidate.at(yidx)) - all_poses.at(count).t()).norm() < setting.cluster_th_xy &&
              abs(theta_candidate.at(thetaidx) - all_poses.at(count).theta()) < setting.cluster_th_theta) {
            neighborhood_count++;
          }
        }

        // reach candidate number requirement
        if (neighborhood_count >= setting.min_nr_candidates) {
          candidate_poses.push_back(Pose2(x_candidate.at(xidx), y_candidate.at(yidx), theta_candidate.at(thetaidx)));
          candidate_idxcount.push_back(make_pair(neighborhood_count, candidate_poses.size()-1));
        }
      }
    }
  }

  // check size, if exceed max sort and get best
  if (candidate_poses.size() > setting.max_cluster) {
    vector<Pose2> candidate_poses_limit;
    // index-remained sort
    sort(candidate_idxcount.begin(), candidate_idxcount.end(), comparator_uint);
    for (size_t i = 1; i <= setting.max_cluster; i++)
      candidate_poses_limit.push_back(candidate_poses.at(candidate_idxcount.at(candidate_poses.size()-i).second));
    return candidate_poses_limit;
  }

  return candidate_poses;
}

/* ************************************************************************* */
// use measurements and local trajectory to get original relative poses
std::vector<gtsam::Pose2> originRelativePose(
    unsigned char local_ID, unsigned char agent_ID,
    const std::vector<gtsam::Pose2>& measure_poses,
    const std::vector<std::pair<size_t, size_t> >& measure_index,
    const gtsam::Values& local_value,
    const gtsam::Values& agent_value) {


  size_t nr_measure = measure_poses.size();
  vector<Pose2> origin_relpose;
  origin_relpose.reserve(nr_measure);

  for (size_t i = 0; i < nr_measure; i++) {
    Pose2 ori1_T_val1 = local_value.at<Pose2>(Symbol(local_ID, measure_index.at(i).first));
    Pose2 ori2_T_val2 = agent_value.at<Pose2>(Symbol(agent_ID, measure_index.at(i).second));
/*
    cout << "Local ID: " << local_ID << " at index: " << measure_index.at(i).first << endl;
    cout << "Foreign ID: " << agent_ID << " at index: " << measure_index.at(i).second << endl;
    ori1_T_val1.print("Origin 1 to val 1: ");
    ori2_T_val2.print("Origin 2 to val 2: ");
    measure_poses.at(i).print("The relative pose: ");
*/

    Pose2 val1_T_val2 = measure_poses.at(i);
    origin_relpose.push_back((ori1_T_val1 * val1_T_val2) * (ori2_T_val2.inverse()));
  }

  return origin_relpose;
}

/* ************************************************************************* */
// clustering a single channel (x/y/theta), output possible poes
std::vector<double> cluster_vector(const std::vector<double>& data, double res, size_t cluster_number) {

  Histogram hist = histogram(data);
  size_t hist_count = hist.count.size();

  //hist.print();

  // create doubleidx vector for index-remained sorting
  vector<doubleidx> hist_idx;
  hist_idx.reserve(hist_count);
  for (size_t i = 0; i < hist_count; i++)
    hist_idx.push_back(make_pair(hist.count.at(i), i));

  // index-remained sort
  sort(hist_idx.begin(), hist_idx.end(), comparator_double);

  // get largest cluster
  vector<double> cluster;
  cluster.push_back(hist.bin_center.at(hist_idx.at(hist_count - 1).second));

  // from second (optional, if bin count is zero)
  for (size_t i = 1; i < hist_count; i++) {

    // check cluster size:
    if (cluster.size() >= cluster_number)
      break;

    size_t candidate_idx = hist_idx.at(hist_count - i - 1).second;
    double candidate_count = hist_idx.at(hist_count - i - 1).first;
    double candidate_value = hist.bin_center.at(candidate_idx);

    // check whether reach zero count : stop
    if (!(candidate_count > 0.0))
      break;

    // check whether too close to previous value : skip
    bool is_skip = false;
    for (size_t j = 0; j < cluster.size(); j++) {
      if (abs(cluster.at(j) - candidate_value) < res) {
        is_skip = true;
        break;
      }
    }
    if (is_skip)
      continue;

    // push candidate in clustered value
    cluster.push_back(candidate_value);
  }

  return cluster;
}

/* ************************************************************************* */
// get normalized histogram for a double vector, given resolution
Histogram histogram(const std::vector<double>& data, double res) {

  Histogram hist;

  // bin number
  double maxnr = *(max_element(data.begin(), data.end()));
  double minnr = *(min_element(data.begin(), data.end()));
  size_t nr_bins;
  // optional res = 0.0
  if (res > 0.0) {
    nr_bins = static_cast<size_t>(ceil((maxnr - minnr) / res));
    // check whether 0 bin happens
    if (nr_bins == 0)
      nr_bins = 1;
  } else {
    nr_bins = data.size();
    res = (maxnr - minnr) / static_cast<size_t>(nr_bins);
    // check res > 0, given an arbitry value if <= 0
    if (res <= 1e-9)
      res = 1e-4;
  }

  // prepare hist
  hist.bin_center.reserve(nr_bins);
  for (size_t i = 0; i < nr_bins; i++)
    hist.bin_center.push_back((static_cast<double>(i) + 0.5) * res + minnr);
  hist.count.assign(nr_bins, 0.0);

  // accumulate hist
  double pre_count = 1.0 / static_cast<double>(data.size());
  for (size_t i = 0; i < data.size(); i++) {
    size_t bin_idx = static_cast<size_t>((data.at(i) - minnr) / res);
    // limit range
    if (bin_idx == nr_bins)
      bin_idx = nr_bins - 1;
    hist.count.at(bin_idx) += pre_count;
  }

  return hist;
}

/* ************************************************************************* */
// print histogram
void Histogram::print() {

  cout << "Histogram:" << endl;

  cout << "value:";
  for (size_t i = 0; i < bin_center.size(); i++)
    cout << setw(10) << bin_center.at(i);
  cout << endl;

  cout << "count:";
  for (size_t i = 0; i < count.size(); i++)
    cout << setw(10) << count.at(i);
  cout << endl;
}

/* ************************************************************************* */
// pair index for index-remained sorting
bool comparator_double(const doubleidx& l, const doubleidx& r) { return l.first < r.first; }
bool comparator_uint(const uintidx& l, const uintidx& r) { return l.first < r.first; }


}   // namespace mast
