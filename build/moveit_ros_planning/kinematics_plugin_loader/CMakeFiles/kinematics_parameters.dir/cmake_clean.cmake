file(REMOVE_RECURSE
  "include/kinematics_parameters.hpp"
  "include/moveit_ros_planning/kinematics_parameters.hpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/kinematics_parameters.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
