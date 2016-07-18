/**
 * @file   FLIRT.h
 * @brief  a wrapper to flirt lib
 * @author Jing Dong
 * @date   Aug 21, 2014
 */

#pragma once


#include <joint_mapping/FeaturePoint.h>

// they are all flirt headers
#include <sensors/LaserReading.h>
#include <feature/CurvatureDetector.h>
//#include <feature/RangeDetector.h>
#include <utils/SimpleMinMaxPeakFinder.h>
#include <feature/BetaGrid.h>
//#include <feature/ShapeContext.h>

#include <gtsam/geometry/Point2.h>

#include <eigen3/Eigen/Dense>

#include <vector>

namespace comap{

/* ************************************************************************* */
// feature detector setting
struct FeatureDetectorSetting {
  // peak finder setting
  double minPeak;
  double minPeakDistance;
  // curvature detector setting
  unsigned int scale;
  double baseSigma;
  unsigned int sigmaStep;
  unsigned int dmst;
  bool useMaxRange;

  // default setting
  FeatureDetectorSetting() :
    minPeak(0.2),
    minPeakDistance(0.0001),
    scale(4),
    baseSigma(0.2),
    sigmaStep(1.0),
    dmst(2),
    useMaxRange(false) {}

  virtual ~FeatureDetectorSetting() {}
};

/* ************************************************************************* */
// feature detector, use Curvature Detector
class FeatureDetector {

private:
  SimpleMinMaxPeakFinder* peakfinder_;
  CurvatureDetector* detector_;
  //RangeDetector detector_;

public:
  // constructors
  FeatureDetector(const FeatureDetectorSetting& setting = FeatureDetectorSetting()) {
    peakfinder_ = new SimpleMinMaxPeakFinder(setting.minPeak, setting.minPeakDistance);
    detector_ = new CurvatureDetector(peakfinder_, setting.scale, setting.baseSigma,
        setting.sigmaStep, setting.dmst);
    detector_->setUseMaxRange(setting.useMaxRange);
  }

  // decontructor
  virtual ~FeatureDetector() {}

  // detect features and output interesting points
  // @return number of point detected
  // @scan laser reading scan
  // @points reference of interesting points
  unsigned int detect(const LaserReading& scan, std::vector<InterestPoint *>& points) const {
    return detector_->detect(scan, points);
  }

};

/* ************************************************************************* */
// feature descriptor setting
struct DescriptorGeneratorSetting {
  // Beta-Grid setting
  double minRho;
  double maxRho;
  unsigned int binRho;
  unsigned int binPhi;

  // default setting
  DescriptorGeneratorSetting() :
    minRho(0.02),
    maxRho(1.0),
    binRho(8),
    binPhi(16) {}

  virtual ~DescriptorGeneratorSetting() {}
};

/* ************************************************************************* */
// feature descriptor, use Beta-Grid descriptor
class DescriptorGenerator {

private:
  BetaGridGenerator* descriptor_gen_;

public:
  // constructors
  DescriptorGenerator(const DescriptorGeneratorSetting& setting = DescriptorGeneratorSetting()) {
    descriptor_gen_ = new BetaGridGenerator(
        setting.minRho, setting.maxRho, setting.binRho, setting.binPhi);
  }

  // decontructor
  virtual ~DescriptorGenerator() {}

  // get FeatureDescriptors given
  // @return descriptors
  // @points input interesting point
  // @scan laser reading scan
  FeatureDescriptors describe(const std::vector<InterestPoint *>& points, const LaserReading& scan) const;

};

/* ************************************************************************* */
// input a scan, get features & descriptors in gtsam format
// @return number of features
// @scan input scan in Eigen format
// @points feature points
// @descp descriptors
unsigned int getFeatureAndDescriptor(
    const Eigen::MatrixXd& scan,
    FeaturePoints& points,
    FeatureDescriptors& descp,
    const FeatureDetectorSetting& detect_setting = FeatureDetectorSetting(),
    const DescriptorGeneratorSetting& descriptor_setting = DescriptorGeneratorSetting());

// utils
// translate data struct
// @down_sampling is how many times to down sampling, 1 means no down sampling
LaserReading transScan2LaserReading(const Eigen::MatrixXd& scan, size_t down_sampling = 1);

// translate InterestPoint to FeaturePoints
FeaturePoints transInterestPoint2FeaturePoints(const std::vector<InterestPoint *>& points);

// translate histogram to shared_array
FeatureDescriptor transHist2Array(const std::vector<std::vector<double> >& hist);


}  // namespace mast
