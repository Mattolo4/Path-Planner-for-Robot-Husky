# Launching your path planner for Ass3

- Launch Gazebo Simulation:

`roslaunch robot_gazebo platform_building.launch tf_base_link:=base_link_real`


- Launch RViz for ROS visualization:

`roslaunch robot_gazebo rviz.launch`

- Start the move base (which also launches your RRT Planner):

`roslaunch advrob_controller move_base.launch`

- Start the planner by selecting a 2D Nav Goal in RViz

