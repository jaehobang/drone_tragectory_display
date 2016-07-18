/**
 * @file   Package.h
 * @brief  robot package class (define all I/O data contents)
 * @author Jing Dong
 * @date   July 1, 2014
 */


///////////////////////////////////////////////
//  The current 3D version with pointcloud2
//  does not support serialization
////////////////////////////////////////////

#pragma once

#include <joint_mapping/Agent_Definitions.h>

#include <eigen3/Eigen/Dense>
#include <ros/serialization.h>
#include <boost/serialization/utility.hpp>
#include <sensor_msgs/PointCloud2.h>

#include <vector>

namespace comap {

/* ************************************************************************* */
// AgentPackage class: communication contents between robot
struct AgentPackage {

  // shared pointers
  typedef boost::shared_ptr<AgentPackage> Ptr;
  typedef boost::shared_ptr<const AgentPackage> ConstPtr;

  // identifications
  unsigned char robot_ID;    // robot id
  unsigned int pose_count;    // robot's current pose count

  std::string robot_name;

  // odometry information
  // current pose value esitmated by local isam
  PoseD curr_pose;
  // odometry output
  PoseD delta_odometry;

  // scan information
  // whether transfer informative scan
  bool informative_scan;
  // scan, if informative. If not, will be empty
  //Eigen::MatrixXd scan;
  sensor_msgs::PointCloud2 scan;

  // matching information:
  // this information is generated the agent which send package,
  // and should be insert into the status of agent's status which received package
  bool flag_matches;
  // measurement robor ID (use unsigned char ID), local first remote second
  std::vector<std::pair<unsigned char, unsigned char> > match_robot_id;
  std::vector<std::pair<std::string, std::string> > match_robot_names;
  // measurement index (use unsigned char ID)
  std::vector<std::pair<unsigned int, unsigned int> > match_meas_index;
  // matched measurements
  std::vector<PoseD> match_measurement;

  // utility functions
  // printing function for debug
  void print() const;

  /* ************************************************************************* */
private:
  // Serialization function, use if to minimize the size of the serialized package

  friend class boost::serialization::access;

  template<class Archive>
  void save(Archive & ar, const unsigned int version) const {
    ar << robot_ID;
    ar << pose_count;
    ar << robot_name;
    ar << curr_pose;
    ar << delta_odometry;

    ar << informative_scan;
    if (informative_scan) {
      // homebrew compact serialization of eigen
      // note: this only works for 2 row matrix
      /*
      const unsigned int cols = scan.cols(), rows = scan.rows();
      ar << rows;
      ar << cols;
      ar << boost::serialization::make_array(scan.data(), scan.size());
      */
      const unsigned int scan_size = ros::serialization::serializationLength(scan);
      boost::shared_array<uint8_t> buffer(new uint8_t[scan_size]);
      ros::serialization::OStream stream(buffer.get(), scan_size);
      ros::serialization::serialize(stream, scan);
      ar << scan_size;
      ar << boost::serialization::make_array(&buffer[0], scan_size);
    }

    ar << flag_matches;
    if (flag_matches) {
      const unsigned int match_size = match_robot_id.size();
      ar << match_size;
      ar << boost::serialization::make_array(&match_robot_id[0], match_size);
      ar << boost::serialization::make_array(&match_robot_names[0], match_size);
      ar << boost::serialization::make_array(&match_meas_index[0], match_size);
      ar << boost::serialization::make_array(&match_measurement[0], match_size);
    }
  }

  /* ************************************************************************* */
  template<class Archive>
  void load(Archive & ar, const unsigned int version) {
    ar >> robot_ID;
    ar >> pose_count;
    ar >> robot_name;
    ar >> curr_pose;
    ar >> delta_odometry;

    ar >> informative_scan;

    if (informative_scan) {
      // homebrew compact serialization of eigen
      /*
      unsigned int rows, cols;
      ar >> rows;
      ar >> cols;
      scan.resize(rows, cols);
      ar >> boost::serialization::make_array(scan.data(), scan.size());
      */
      unsigned int scan_size;
      ar >> scan_size;
      boost::shared_array<uint8_t> buffer(new uint8_t[scan_size]);
      ar >> boost::serialization::make_array(&buffer[0], scan_size);
      //ar >> buffer;
      ros::serialization::IStream stream(buffer.get(), scan_size);
      ros::serialization::Serializer<sensor_msgs::PointCloud2>::read(stream, scan);
    }

    ar >> flag_matches;
    if (flag_matches) {
      unsigned int match_size;
      ar >> match_size;
      match_robot_id.resize(match_size);
      match_robot_names.resize(match_size);
      match_meas_index.resize(match_size);
      match_measurement.resize(match_size);
      ar >> boost::serialization::make_array(&match_robot_id[0], match_size);
      ar >> boost::serialization::make_array(&match_robot_names[0], match_size);
      ar >> boost::serialization::make_array(&match_meas_index[0], match_size);
      ar >> boost::serialization::make_array(&match_measurement[0], match_size);
    }

  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()
};

}   // namespace mast
