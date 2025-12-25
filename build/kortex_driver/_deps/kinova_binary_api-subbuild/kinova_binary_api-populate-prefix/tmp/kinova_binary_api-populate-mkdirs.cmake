# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/peru0002/ros2_ws/build/kortex_driver/_deps/kinova_binary_api-src"
  "/home/peru0002/ros2_ws/build/kortex_driver/_deps/kinova_binary_api-build"
  "/home/peru0002/ros2_ws/build/kortex_driver/_deps/kinova_binary_api-subbuild/kinova_binary_api-populate-prefix"
  "/home/peru0002/ros2_ws/build/kortex_driver/_deps/kinova_binary_api-subbuild/kinova_binary_api-populate-prefix/tmp"
  "/home/peru0002/ros2_ws/build/kortex_driver/_deps/kinova_binary_api-subbuild/kinova_binary_api-populate-prefix/src/kinova_binary_api-populate-stamp"
  "/home/peru0002/ros2_ws/build/kortex_driver/_deps/kinova_binary_api-subbuild/kinova_binary_api-populate-prefix/src"
  "/home/peru0002/ros2_ws/build/kortex_driver/_deps/kinova_binary_api-subbuild/kinova_binary_api-populate-prefix/src/kinova_binary_api-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/peru0002/ros2_ws/build/kortex_driver/_deps/kinova_binary_api-subbuild/kinova_binary_api-populate-prefix/src/kinova_binary_api-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/peru0002/ros2_ws/build/kortex_driver/_deps/kinova_binary_api-subbuild/kinova_binary_api-populate-prefix/src/kinova_binary_api-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
