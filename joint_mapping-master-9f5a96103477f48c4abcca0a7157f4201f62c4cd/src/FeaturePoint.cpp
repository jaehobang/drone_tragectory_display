/**
 * @file   FeaturePoint.cpp
 * @brief  Feature Point and Descriptor classes
 * @author Jing Dong
 * @date   Aug 23, 2014
 */

#include <joint_mapping/FeaturePoint.h>

namespace comap {

/* ************************************************************************* */
// save FeaturePoints position to file
bool writeMATLABfileFeaturePoints(std::string filename, const FeaturePoints& featpoints) {

  std::ofstream fs(filename.c_str(), std::ios::out);
  if (!fs)
    return false;

  for (size_t i = 0; i < featpoints.size(); i++)
    fs  << featpoints.at(i).x() << " "
        << featpoints.at(i).y() << " "
        << featpoints.at(i).theta() << std::endl;

  fs.close();
  return true;
}

}  // namespace mast
