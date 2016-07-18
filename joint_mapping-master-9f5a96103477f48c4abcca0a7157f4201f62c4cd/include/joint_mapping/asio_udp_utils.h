/** 
 * @file asio_udp_utils.h
 * @brief some wrapper to use asio_udp_device to do multi-robots communication
 * @author Jing Dong
 * @date July 25, 2014
 */
 
#ifndef __ASIO_UDP_UTILS_H
#define __ASIO_UDP_UTILS_H

// ROS package version
#include <joint_mapping/AgentPackageMsg.h>

// MAST_CPP package version
#include <joint_mapping/Package.h>

#include <ros/ros.h>


/* ************************************************************************** */
// ROS package version (ROS message data structure + ROS serialization utils)
// serialization
std::vector<uint8_t> serializePackageROS(const joint_mapping::AgentPackageMsg& package);
// deserialization
joint_mapping::AgentPackageMsg::ConstPtr deserializePackageROS(const uint8_t* buf, size_t buf_size);

/* ************************************************************************** */
// MAST_CPP package version (MAST_CPP message data structure + boost serialization utils)
// serialization
std::vector<uint8_t> serializePackageMAST(const comap::AgentPackage& package);
// deserialization
comap::AgentPackage::ConstPtr deserializePackageMAST(const uint8_t* buf, size_t buf_size);

// crc-32 calculator
uint32_t getCRC32(const std::string& input_str);


#endif
