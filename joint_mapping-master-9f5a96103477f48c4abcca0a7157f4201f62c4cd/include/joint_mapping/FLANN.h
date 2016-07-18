/**
 * @file   FLANN.h
 * @brief  a wrapper to FLANN lib
 * @author Jing Dong
 * @date   Aug 22, 2014
 */

#pragma once

#include <joint_mapping/FLIRT.h>

#include <flann/flann.hpp>

#include <joint_mapping/Settings.h>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include <vector>

#include <iostream>

namespace comap {


/* ************************************************************************* */
// KnnMatcher, use randomized kdtree in FLANN
class KnnMatcher {

private:

  typedef flann::Index<flann::L2<double> > MatcherType;
  typedef boost::shared_ptr<MatcherType> MatcherType_Ptr;
  typedef boost::shared_ptr<const MatcherType> MatcherType_ConstPtr;

  KnnMatcherSetting setting_;
  bool is_init_;        // inidcator of whether initialized index
  MatcherType_Ptr pmatcher_;
  std::vector<unsigned int> idx_cache_;     // index cache to retrieve scan index
  std::vector<flann::Matrix<double> > mat_cache_;   // matrix cache, save for future delete

public:
  // constructors
  KnnMatcher(const KnnMatcherSetting& setting = KnnMatcherSetting()) :
      setting_(setting), is_init_(false),
      pmatcher_(new MatcherType(flann::KDTreeIndexParams(setting_.tree_nr))) {}

  // decontructor
  virtual ~KnnMatcher() {
    // free training matrix space
    for (size_t i = 0; i < mat_cache_.size(); i++)
      delete[] mat_cache_[i].ptr();
  }

  /* ************************************************************************* */
  // not first time use: add points in index
  void addTraining(unsigned int scan_idx, const FeatureDescriptors& feat) {

    // convert to flann::Matrix
    flann::Matrix<double> mat = this->transFeature2Matrix(feat);

    // if first time: init the trainer
    if (!is_init_) {
      pmatcher_->buildIndex(mat);
      is_init_ = true;
    } else {
      pmatcher_->addPoints(mat);
    }

    // cache scan index
    std::vector<unsigned int> this_scan;
    this_scan.assign(feat.size(), scan_idx);
    idx_cache_.insert(idx_cache_.end(), this_scan.begin(), this_scan.end());

/*
    std::cout << "idx_cache_: ";
    for (size_t i = 0; i < idx_cache_.size(); i++)
      std::cout << idx_cache_.at(i) << ",";
    std::cout << std::endl;
*/

    // cache Matrix
    mat_cache_.push_back(mat);
    //delete[] mat.ptr();
  }

  /* ************************************************************************* */
  // perform knn-search
  // @return scan-idx in vector[i][j], the (j+1)th closest feature for feat[i]
  std::vector<std::vector<unsigned int> > knnSearch(const FeatureDescriptors& feat) const {

    // check whether trained
    if (idx_cache_.size() == 0)
      throw std::runtime_error("kd-tree not trained yet");

    // scan index cache: pre-allocate
    std::vector<std::vector<unsigned int> > scan_idx_cache;
    scan_idx_cache.resize(feat.size());
    for (size_t i = 0; i < feat.size(); i++)
      scan_idx_cache[i].resize(setting_.knn);

    // feature-idx cache, distance cache
    std::vector<std::vector<size_t> > feat_idx_cache;
    std::vector<std::vector<double> > dist_cache;

    // convert to flann::Matrix
    flann::Matrix<double> mat = this->transFeature2Matrix(feat);

    pmatcher_->knnSearch(mat, feat_idx_cache, dist_cache,
        setting_.knn, flann::SearchParams());

    // feat_idx => scan_idx
    for (size_t i = 0; i < feat.size(); i++)
      for (size_t j = 0; j < setting_.knn; j++)
        //cout << "result: " << feat_idx_cache[i][j] << endl;
        scan_idx_cache[i][j] = idx_cache_[feat_idx_cache[i][j]];

    // free Matrix
    delete[] mat.ptr();

    return scan_idx_cache;
  }

  /* ************************************************************************* */
private:
  // translate descriptors to flann::Matrix
  flann::Matrix<double> transFeature2Matrix(const FeatureDescriptors& feat) const {

    // pre-allocate mem for data cache
    double* cache = new double[setting_.feature_size * feat.size()];
    if (cache == NULL)
      throw std::runtime_error("memory allocation error");

    // copy memory
    for (size_t i = 0; i < feat.size(); i++)
      for (size_t j = 0; j < setting_.feature_size; j++)
        cache[j + i*setting_.feature_size] = feat[i][j];
/*
    flann::Matrix<double> mat(cache, feat.size(), setting_.feature_size);
    for (size_t i = 0 ; i < feat.size(); i++) {
      for (size_t j = 0; j < setting_.feature_size; j++) {
        cout << mat[i][j] << ", ";
      }
      cout << endl;
    }
*/
    return flann::Matrix<double>(cache, feat.size(), setting_.feature_size);
  }

  // translate a single descriptor to flann::Matrix
  flann::Matrix<double> transFeature2Matrix(const FeatureDescriptor& feat) const {
    return flann::Matrix<double>(feat.get(), 1, setting_.feature_size);
  }
};

}  // namespace mast
