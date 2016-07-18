#include <joint_mapping/RobotManager.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_mapping");
  ros::NodeHandle n("~");

  RobotManager r;
  if (!r.initialize(n))
  {
    ROS_ERROR("%s: failed to initialize mast_gt_cmu_example",
        ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
