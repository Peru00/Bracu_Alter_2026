#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "moveit_kinematics::moveit_cached_ik_kinematics_base" for configuration "Release"
set_property(TARGET moveit_kinematics::moveit_cached_ik_kinematics_base APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(moveit_kinematics::moveit_cached_ik_kinematics_base PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmoveit_cached_ik_kinematics_base.so.2.14.1"
  IMPORTED_SONAME_RELEASE "libmoveit_cached_ik_kinematics_base.so.2.14.1"
  )

list(APPEND _cmake_import_check_targets moveit_kinematics::moveit_cached_ik_kinematics_base )
list(APPEND _cmake_import_check_files_for_moveit_kinematics::moveit_cached_ik_kinematics_base "${_IMPORT_PREFIX}/lib/libmoveit_cached_ik_kinematics_base.so.2.14.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
