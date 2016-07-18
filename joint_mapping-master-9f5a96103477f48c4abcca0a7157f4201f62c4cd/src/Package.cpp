/**
 * @file   Package.cpp
 * @brief  robot package class (define all I/O data contents)
 * @author Jing Dong
 * @date   July 2, 2014
 */

#include <joint_mapping/Package.h>

#include <iostream>

using namespace gtsam;
using namespace std;

namespace comap {

/* ************************************************************************* */
// printing function for debug
void AgentPackage::print() const {

  cout << "  package ID: " << robot_ID << endl;
  cout << "  package pose: " << pose_count << endl;
  cout << "  package scan: " << std::boolalpha << informative_scan << endl;
  cout << "  matched measure: " << std::boolalpha << flag_matches << endl;
  if (flag_matches)
    cout << "  measure size: " << match_measurement.size() << endl;
}

}   // namespace mast
