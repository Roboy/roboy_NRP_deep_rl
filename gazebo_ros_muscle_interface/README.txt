Short description for testing the generic controller plugin.

1. Compile the GazeboRosPackages ROS workspace with catkin_make

2. Create a symlink for the test model in your ~/.gazebo/models folder:
cd ~/.gazebo/models
ln -s <package_path>/sdf test_model

3. start gazebo with your GazeboRosPackages workspace sourced:
cd <GazeboRosPackages>
source devel/setup.bash
rosrun gazebo_ros gazebo  (--verbose for debug output)

4. Insert the test model from the models list. It will show a robot with 4 links and 3 joints (see model.sdf in sdf folder).

5. Use ROS topic publisher to control the joints:
rostopic list    (list available topics)

example for setting velocity of joint2 to 0.2 meter per second:
rostopic pub /test_robot/my_joint2/cmd_vel std_msgs/Float64 "data: 0.2"

example for setting position of joint1 to 1.5 rad (joint angle):
rostopic pub /test_robot/my_joint1/cmd_pos std_msgs/Float64 "data: 1.5"

