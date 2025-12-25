#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "moveit_planners_stomp::stomp_moveit_plugin" for configuration "Release"
set_property(TARGET moveit_planners_stomp::stomp_moveit_plugin APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(moveit_planners_stomp::stomp_moveit_plugin PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libstomp_moveit_plugin.so"
  IMPORTED_SONAME_RELEASE "libstomp_moveit_plugin.so"
  )

list(APPEND _cmake_import_check_targets moveit_planners_stomp::stomp_moveit_plugin )
list(APPEND _cmake_import_check_files_for_moveit_planners_stomp::stomp_moveit_plugin "${_IMPORT_PREFIX}/lib/libstomp_moveit_plugin.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
