cmake_minimum_required(VERSION 3.0.2)
project(logistics_gazebo_sim)
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs mavros_msgs gazebo_ros)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/generate_worlds scripts/generate_missions scripts/gazebo_marker_sync scripts/offboard_controller scripts/fleet_mission_player scripts/fleet_safety_monitor scripts/fleet_visualizer scripts/mission_recorder scripts/parcel_delivery_manager
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
