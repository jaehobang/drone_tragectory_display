/** 
 * @file asio_udp_utils.cc
 * @brief some wrapper to use asio_udp_device to do multi-robots communication
 * @author Jing Dong
 * @date July 25, 2014
 */
 
#include <joint_mapping/asio_udp_utils.h>

#include <boost/crc.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

namespace ser = ros::serialization;

/* ************************************************************************* */
// ROS package version (ROS message data structure + ROS serialization utils)
// serialization
std::vector<uint8_t> serializePackageROS(const joint_mapping::AgentPackageMsg& package) {

  size_t serial_size = ros::serialization::serializationLength(package);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);

  ser::OStream stream(buffer.get(), serial_size);
  ser::Serializer<joint_mapping::AgentPackageMsg>::write(stream, package);
  //ser::serialize(stream, package);
  
  std::vector<uint8_t> output_vec(buffer.get(), buffer.get() + serial_size);
  
  //for (size_t i = 0; i < output_vec.size(); i++)
    //cout << std::hex << static_cast<int>(output_vec.at(i)) << std::dec << ", ";
    
  return output_vec;
}


/* ************************************************************************* */
// deserialization
joint_mapping::AgentPackageMsg::ConstPtr deserializePackageROS(const uint8_t* buf, size_t buf_size) {

  joint_mapping::AgentPackageMsg::Ptr package(new joint_mapping::AgentPackageMsg);
  
  // get avoid of const, since IStream required non-const
  // TODO: this is not right to do it, may have other solution
  uint8_t* buf_nonconst = const_cast<uint8_t*>(buf);

  ser::IStream stream(buf_nonconst, buf_size);
  
  ser::Serializer<joint_mapping::AgentPackageMsg>::read(stream, *package);
  //ser::deserialize(stream, *package);

  return package;
}


/* ************************************************************************** */
// MAST_CPP package version (MAST_CPP message data structure + boost serialization utils)
// serialization
std::vector<uint8_t> serializePackageMAST(const comap::AgentPackage& package) {

  // serialize
  std::ostringstream oss;
  boost::archive::binary_oarchive oa(oss, boost::archive::no_header);
  oa << package;

  // append crc
  uint32_t crc_result = getCRC32(oss.str());
  //std::cout << "CRC result: " << std::hex << crc_result << std::dec << std::endl;
  oa << crc_result;

  // pack
  const std::string str = oss.str();
  std::vector<uint8_t> buffer(str.begin(), str.end());
  
  return buffer;
}

/* ************************************************************************** */
// deserialization
comap::AgentPackage::ConstPtr deserializePackageMAST(const uint8_t* buf, size_t buf_size) {

  // deserialize
  comap::AgentPackage::Ptr package(new comap::AgentPackage);

  const std::string strpack(reinterpret_cast<const char *>(buf), buf_size - 4);
  uint32_t crc_result_calc = getCRC32(strpack);
  //std::cout << "CRC result calc: " << std::hex << crc_result_calc << std::dec << std::endl;

  // get received crc
  uint32_t crc_result_got = 0;
  for(unsigned int i = 1; i < 4; i++) {
    crc_result_got += static_cast<uint32_t>(*(buf + buf_size - i));
    crc_result_got = crc_result_got << 8;
  }
  crc_result_got += static_cast<uint32_t>(*(buf + buf_size - 4));
  //std::cout << "CRC result got : " << std::hex << crc_result_got << std::dec << std::endl;

  if (crc_result_got == crc_result_calc) {
    // deserialize
    std::istringstream iss(strpack);
    boost::archive::binary_iarchive ia(iss, boost::archive::no_header);
    ia >> (*package);
    
    return package;
    
  } else {
    // if crc fails, just return a null pointer
    ROS_WARN("Received package CRC checksum not passed");
    return comap::AgentPackage::Ptr();
  }
}

/* ************************************************************************** */
// get CRC-32 for std::string
uint32_t getCRC32(const std::string& input_str) {
  boost::crc_32_type result;
  result.process_bytes(input_str.data(), input_str.length());
  return result.checksum();
}

