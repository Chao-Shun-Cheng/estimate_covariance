cmake_minimum_required(VERSION 3.0.2)
project(estimate_covariance)

find_package(autoware_build_flags REQUIRED)
find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  autoware_msgs
  roscpp
  tf
)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
   CATKIN_DEPENDS 
   autoware_msgs 
   roscpp
   tf
)

set(ROSLINT_CPP_OPTS "--filter=-build/c++14")

include_directories(
  ${catkin_INCLUDE_DIRS}
  include)

add_executable(estimate_covariance
        src/estimate_covariance.cpp
        )
target_link_libraries(estimate_covariance
        ${catkin_LIBRARIES}
        )
add_dependencies(estimate_covariance
        ${catkin_EXPORTED_TARGETS}
        )

