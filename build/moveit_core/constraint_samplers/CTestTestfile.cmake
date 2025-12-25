# CMake generated Testfile for 
# Source directory: /home/peru0002/ros2_ws/src/moveit2/moveit_core/constraint_samplers
# Build directory: /home/peru0002/ros2_ws/build/moveit_core/constraint_samplers
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test([=[test_constraint_samplers]=] "/usr/bin/python3" "-u" "/opt/ros/jazzy/share/ament_cmake_test/cmake/run_test.py" "/home/peru0002/ros2_ws/build/moveit_core/test_results/moveit_core/test_constraint_samplers.gtest.xml" "--package-name" "moveit_core" "--output-file" "/home/peru0002/ros2_ws/build/moveit_core/ament_cmake_gmock/test_constraint_samplers.txt" "--command" "/home/peru0002/ros2_ws/build/moveit_core/constraint_samplers/test_constraint_samplers" "--gtest_output=xml:/home/peru0002/ros2_ws/build/moveit_core/test_results/moveit_core/test_constraint_samplers.gtest.xml")
set_tests_properties([=[test_constraint_samplers]=] PROPERTIES  LABELS "gmock" REQUIRED_FILES "/home/peru0002/ros2_ws/build/moveit_core/constraint_samplers/test_constraint_samplers" TIMEOUT "60" WORKING_DIRECTORY "/home/peru0002/ros2_ws/build/moveit_core/constraint_samplers" _BACKTRACE_TRIPLES "/opt/ros/jazzy/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/jazzy/share/ament_cmake_gmock/cmake/ament_add_gmock_test.cmake;98;ament_add_test;/opt/ros/jazzy/share/ament_cmake_gmock/cmake/ament_add_gmock.cmake;90;ament_add_gmock_test;/home/peru0002/ros2_ws/src/moveit2/moveit_core/constraint_samplers/CMakeLists.txt;42;ament_add_gmock;/home/peru0002/ros2_ws/src/moveit2/moveit_core/constraint_samplers/CMakeLists.txt;0;")
subdirs("../gmock")
