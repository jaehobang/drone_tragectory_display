/**
 * @file   PeakFinder.h
 * @brief  find peaks in given vector, type defined in templated param
 * @author Jing Dong
 * @date   Aug 23, 2014
 */

#pragma once

#include <joint_mapping/Settings.h>

#include <algorithm>
#include <numeric>
#include <cmath>
#include <vector>

#include <iostream>
using namespace std;

namespace comap {

// peak point class
struct Peak {
  size_t idx;
  double value;
};

/* ************************************************************************* */
// loop closure detector
template <typename T>
class PeakFinder {

private:
  PeakFinderSetting setting_;

public:
  // constructors
  PeakFinder(const PeakFinderSetting& setting = PeakFinderSetting()) :
      setting_(setting) {}
  // decontructor
  virtual ~PeakFinder() {}

  /* ************************************************************************* */
  // find peak, return peak index and result value
  std::vector<Peak> findPeaks(const std::vector<T>& input) const {

    // smoothed result
    std::vector<double> smooth_result = this->smoother(input, setting_.smooth_step);
/*
    cout << "smooth result: ";
    for (size_t i = 0; i < smooth_result.size(); i++)
      cout << smooth_result[i] << ", ";
    cout << endl;
*/
    // scan for peaks
    std::vector<size_t> peak_idx;
    for (size_t i = 0; i < input.size(); i++) {

      if (i > 0 && i < input.size() - 1) {
        // middle
        if (smooth_result[i] > (setting_.peak_ratio * smooth_result[i-1]) &&
            smooth_result[i] > (setting_.peak_ratio * smooth_result[i+1]))
          peak_idx.push_back(i);
      } else if (i == 0) {
        // first
        if (smooth_result[i] > (setting_.peak_ratio * smooth_result[i+1]))
          peak_idx.push_back(i);
      } else {
        // last
        if (smooth_result[i] > (setting_.peak_ratio * smooth_result[i-1]))
          peak_idx.push_back(i);
      }
    }

    // threshed by value = avg * ratio
    double vec_avg = std::accumulate(smooth_result.begin(), smooth_result.end(), 0.0) /
        static_cast<double>(smooth_result.size());
    double min_peak_thresh = vec_avg * setting_.min_peak_thresh_ratio;
    double min_accpt_thresh = vec_avg * setting_.min_accept_thresh_ratio;

    std::vector<Peak> result;

    std::vector<size_t> inserted_above;
    // insert all above
    if (setting_.flag_accept_all_above) {
      for (size_t i = 0; i < smooth_result.size(); i++) {
        if (smooth_result.at(i) > min_accpt_thresh) {
          Peak peak;
          peak.idx = i;
          peak.value = smooth_result.at(i); // smoothed value
          result.push_back(peak);
          inserted_above.push_back(i);
        }
      }
    }

    // insert peaks
    for (size_t i = 0; i < peak_idx.size(); i++) {
      if (smooth_result.at(peak_idx[i]) > min_peak_thresh &&
          std::find(inserted_above.begin(), inserted_above.end(), peak_idx[i]) == inserted_above.end()) {
        Peak peak;
        peak.idx = peak_idx[i];
        peak.value = smooth_result.at(peak_idx[i]); // smoothed value
        result.push_back(peak);
      }
    }
    return result;
  }

private:
  /* ************************************************************************* */
  // vector smoother
  std::vector<double> smoother(const std::vector<T>& input, size_t smooth_step) const {

    std::vector<double> smooth_result;
    smooth_result.resize(input.size());

    int input_size = static_cast<int>(input.size());    // avoid comparison warning
    int signed_step = static_cast<int>(smooth_step);

    // design a smoother: single weight param is w
    // for i has w^0 weight, i+1&i-1 has w^1, i+2&i-2 has w^2, ...
    // for w == 0.5, i has 1.0, i+1&i-1 has 0.5, i+2&i-2 has 0.25, ...

    for (int i = 0; i < input_size; i++) {
      // get lower and upper bonds
      int lower_idx = std::max(0, i-signed_step);
      int upper_idx = std::min(input_size-1, i+signed_step);

      double smooth_weighter = 0;
      double smooth_accumulator = 0.0;
      for (int j = lower_idx; j <= upper_idx; j++) {

        double weight = pow(setting_.smooth_weight, fabs(static_cast<double>(i - j)));

        smooth_accumulator += static_cast<double>(input.at(j)) * weight;
        smooth_weighter += weight;
      }

      // put result in
      smooth_result.at(i) = smooth_accumulator / smooth_weighter;
    }
    return smooth_result;
  }

};

}  // namespace mast
