/**
 * @file   Saliency.h
 * @brief  calculate Scan Saliency
 * @author Jing Dong
 * @date   Jun 30, 2014
 */

#pragma once

#include <joint_mapping/ICPMatcher2D.h>

#include <eigen3/Eigen/Dense>

#include <boost/random/mersenne_twister.hpp>

namespace comap {

struct ScanSaliencySetting {

  // iteration checked
  unsigned int ntimes;
  // sigma in translation
  double sigma_t;
  // sigma in translation
  double sigma_theta;

  // default setting
  ScanSaliencySetting() :
    ntimes(20),
    sigma_t(0.5),
    sigma_theta(10/180.0*3.1416) {}

  virtual ~ScanSaliencySetting() {}
};

/* ************************************************************************* */
// Scan Saliency class
template <class MATCHER>
class ScanSaliency {

private:
  ScanSaliencySetting setting_;
  MATCHER matcher_;
  boost::mt19937 random_gen_;

public:
  // constructor
  ScanSaliency() : setting_(),
      matcher_(PointMatcherSetting()), random_gen_() {}

  ScanSaliency(std::string filename, ICPParameter param) :
      setting_(), matcher_(filename, param), random_gen_() {}

  ScanSaliency(const PointMatcherSetting& pmsetting, const ScanSaliencySetting& setting) :
      setting_(), matcher_(pmsetting), random_gen_() {}

  virtual ~ScanSaliency() {}

  // calculate saliency use defualt parameter
  double calculateScanSaliency(const Eigen::MatrixXd& scan);
};

}   // namespace mast

#include <joint_mapping/Saliency2D_impl.h>
