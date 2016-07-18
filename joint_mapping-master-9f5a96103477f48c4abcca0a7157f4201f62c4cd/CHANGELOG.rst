^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package joint_mapping
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.2 (2016-05-31)
------------------
* Updating crop z
* Updating map construction and network passing
* Updaing launch file
* Adding ISS keypoints
* Contributors: Vibhav Ganesh

0.1.1 (2016-05-05)
------------------
* Adding intensity based keypoint detector
* Updating parameters to get map working
* Debugging the velodyne scans
* Updating parameters for velodyne points
* Fixing merge conflict with velodyne launch file
* Removing pcl display from launch file
* Fixing package send bug
* Updating parameters
* New network parameters
* More removing tagged odom
* Removing dependence of tagged point clouds
* Adding parameter for sensor inverted axis
* Network now reads from param file
* Merge branch 'develop' of nmichael.frc.ri.cmu.edu:vnganesh/joint_mapping into develop
* Adding logging for network packets and started changing network to take in parameters from yaml
* Mapping now works, outputs 2D grid
* Another bug
* Fixing bugs with agent symbol
* Adding check for value existing before using it
* removing print statements for debug and adding check for values
* Adding lock to received package list
* Debugging lines for UDP msgs
* Adding brix launch files
* Merge branch 'develop' of nmichael.frc.ri.cmu.edu:vnganesh/joint_mapping into develop
* Adding teleop launch file
* Merge branch 'develop' of nmichael.frc.ri.cmu.edu:vnganesh/joint_mapping into develop
* initial map output, needs fixing
* Removing libpointmatcher and flirt
* Adding more udp launch files
* Adding udp launch files
* Switching from ros msg to udp packet
* Fixing merge conflict with simulation launch file
* Removing geometry_utils, adding 2D slice code, send scans to map subscriber
* Adding c+11 requirements and moving bagfiles to home
* Adding NDT Transform variant
* Adding a logger
* Testing different parameters and new peak finding
* Fixed bug with outlier rejection model not being read correctly
* Better refinement by ICP
* fixed bugs with em update
* Fixed bug with msg to package
* Works uptil after relpose is built
* Runs without error but performance problems restrict ability to find relative pose
* Updating with 3D registration
* Adding stuff for 3D points
* Fixed more bugs, everything runs smoothly now
* Runs minus bug in matcher
* Adding launch and config minus the bag
* Adding in launch and config files
* Everything compiles
* Almost stable point
* Fixing compiler errors
* Adding dependencies and renaming
* Adding init functions for the Local and Foreign Agent
* Updating with new architecture
* Updating refactor
* Adding package and loop closure code
* Removing more temp files
* Removing temp files
* Adding git ignore
* Initial commit of new code
* Contributors: Nathan Michael, Vibhav Ganesh
