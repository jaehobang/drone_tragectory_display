/**
 * @file MAST_cpp_utils.h
 * @brief some utils to use MAST_cpp lib
 * @author Jing Dong
 * @date July 17, 2014
 */

#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <fstream>

#include <joint_mapping/AgentPackageMsg.h>
#include <joint_mapping/Package.h>
#include <joint_mapping/Agent.h>

#include <parameter_utils/ParameterUtils.h>
#include <tf/transform_datatypes.h>

// message utilities
comap::AgentPackage msgToPack(const joint_mapping::AgentPackageMsg& msg);
joint_mapping::AgentPackageMsg packToMsg(const comap::AgentPackage& pack);

// param loader
comap::AgentSetting loadAgentSetting(const ros::NodeHandle& n);

// sub-param loader
/*
comap::PointMatcherSetting loadPointMatcherSetting(const std::string& param, const ros::NodeHandle& n);
comap::ScanSaliencySetting loadScanSaliencySetting(const std::string& param, const ros::NodeHandle& n);
*/
comap::ClusteringSetting loadClusteringSetting(const std::string& param, const ros::NodeHandle& n);
comap::RelativePoseEMSetting loadRelativePoseEMSetting(const std::string& param, const ros::NodeHandle& n);
comap::HypothesisMergeSetting loadHypothesisMergeSetting(const std::string& param, const ros::NodeHandle& n);
comap::HypothesisSelectSetting loadHypothesisSelectSetting(const std::string& param, const ros::NodeHandle& n);

//comap::LoopClosureDetectorSetting loadLoopClosureDetectorSetting(const std::string& param, const ros::NodeHandle& n);
//comap::LoopClosureMatcherSetting loadLoopClosureMatcherSetting(const std::string& param, const ros::NodeHandle& n);
//comap::FeatureDetectorSetting loadFeatureDetectorSetting(const std::string& param, const ros::NodeHandle& n);
//comap::DescriptorGeneratorSetting loadDescriptorGeneratorSetting(const std::string& param, const ros::NodeHandle& n);
comap::KnnMatcherSetting loadKnnMatcherSetting(const std::string& param, const ros::NodeHandle& n);
comap::PeakFinderSetting loadPeakFinderSetting(const std::string& param, const ros::NodeHandle& n);
comap::Matcher3DSetting loadMatcherSetting(const std::string& param, const ros::NodeHandle&n );

comap::SmallEMSetting loadSmallEMSetting(const std::string& param, const ros::NodeHandle& n);
comap::CRPSetting loadCRPSetting(const std::string& param, const ros::NodeHandle& n);
