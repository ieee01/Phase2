cmake_minimum_required(VERSION 3.8)
project(cstam_gazebo)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds urdf models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
