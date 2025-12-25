file(REMOVE_RECURSE
  "include/kdl_kinematics_parameters.hpp"
  "include/moveit_kinematics/kdl_kinematics_parameters.hpp"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/kdl_kinematics_parameters.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
