uint8 id
uint32 pose_count
geometry_msgs/Pose pose
geometry_msgs/Pose odom
string robot_name

bool flag_scan
sensor_msgs/PointCloud2 scan

bool flag_match
uint8[] match_robot_id1
uint8[] match_robot_id2
string[] match_robot_name1
string[] match_robot_name2
uint32[] match_index1
uint32[] match_index2
geometry_msgs/Pose[] match_pose
