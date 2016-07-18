/**
 * @file   FLIRT.cpp
 * @brief  a wrapper to flirt lib
 * @author Jing Dong
 * @date   Aug 21, 2014
 */

#include <joint_mapping/FLIRT.h>

#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;
using namespace gtsam;

namespace comap {

/* ************************************************************************* */
// get FeatureDescriptors given
FeatureDescriptors DescriptorGenerator::describe(const std::vector<InterestPoint *>& points,
    const LaserReading& scan) const {

  FeatureDescriptors descriptors;
  descriptors.resize(points.size());

  for (size_t i = 0; i < points.size(); i++) {

    // discribe
    Descriptor* desp = descriptor_gen_->describe(*points[i], scan);

    // histogram
    BetaGrid* desp_betegrid = dynamic_cast<BetaGrid*>(desp);
    vector<vector<double> > hist = desp_betegrid->getHistogram();

    descriptors[i] = transHist2Array(hist);
  }

  return descriptors;
}


/* ************************************************************************* */
// input a scan, get features & descriptors in gtsam format
unsigned int getFeatureAndDescriptor(
    const Eigen::MatrixXd& scan,
    FeaturePoints& points,
    FeatureDescriptors& descp,
    const FeatureDetectorSetting& detect_setting,
    const DescriptorGeneratorSetting& descriptor_setting) {

  unsigned int count_feature;
  FeatureDetector feature_detector(detect_setting);
  DescriptorGenerator descriptor_generator(descriptor_setting);

  // translate input
  const LaserReading reading = transScan2LaserReading(scan);

  // detect interest point
  std::vector<InterestPoint *> interest_points;
//  Timer t2("detect");
//  t2.tic();

  count_feature = feature_detector.detect(reading, interest_points);

 // t2.toc();

  // get descriptor
  //Timer t3("describe");
  //t3.tic();

  descp = descriptor_generator.describe(interest_points, reading);

  //t3.toc();

  // translate keypoints
  points = transInterestPoint2FeaturePoints(interest_points);

  return count_feature;
}


/* ************************************************************************* */
// utils to translate data struct
LaserReading transScan2LaserReading(const Eigen::MatrixXd& scan, size_t down_sampling) {

  // traslate x/y to rho/phi
  size_t nr_scan = scan.cols();

  // down-sampleing version
  vector<double> phi_vec, rho_vec;
  phi_vec.resize(nr_scan / down_sampling);
  rho_vec.resize(nr_scan / down_sampling);

  for (size_t i = 0; i < nr_scan/down_sampling; i++) {
    phi_vec[i] = atan2(scan(1, down_sampling*i), scan(0, down_sampling*i));
    if (fabs(scan(0, down_sampling*i)) > 1e-3)
      rho_vec[i] = scan(0, down_sampling*i) / cos(phi_vec[i]);
    else
      rho_vec[i] = scan(1, down_sampling*i) / sin(phi_vec[i]);
  }

  // put in LaserReading constructor
  return LaserReading(phi_vec, rho_vec);
}

/* ************************************************************************* */
// translate InterestPoint to FeaturePoints
FeaturePoints transInterestPoint2FeaturePoints(const std::vector<InterestPoint *>& points) {

  FeaturePoints featpoints;
  featpoints.resize(points.size());

  for (size_t i = 0; i < points.size(); i++)
    featpoints[i] = Pose2(points[i]->getPosition().x, points[i]->getPosition().y, points[i]->getPosition().theta);

  return featpoints;
}

/* ************************************************************************* */
// translate histogram to shared_array
FeatureDescriptor transHist2Array(const std::vector<std::vector<double> >& hist) {

  // get size
  unsigned int histsize = hist.size();
  unsigned int binsize =  hist[0].size();

  FeatureDescriptor disp(new double[histsize * binsize]);
  for (size_t i = 0; i < histsize; i++)
    for (size_t j = 0; j < binsize; j++)
      disp[i * binsize + j] = hist[i][j];

  return disp;
}

}  // namespace mast
