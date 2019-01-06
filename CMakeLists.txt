cmake_minimum_required(VERSION 2.8.3)
project(ndt_omp)

# -mavx causes a lot of errors!!
add_definitions(-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_CXX_FLAGS "-std=c++11 -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
#set(CMAKE_CXX_FLAGS "-std=c++11 -O0")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

# 寻找sophus
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ndt_omp
)

###########
## Build ##
###########
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(ndt_omp
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

add_executable(align
  apps/align.cpp
)
add_dependencies(align
  ndt_omp
)
target_link_libraries(align
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
  ${Sophus_LIBRARIES}
)

add_executable(match
  apps/match.cpp
)
add_dependencies(match
  ndt_omp
)
target_link_libraries(match
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
  ${Sophus_LIBRARIES}
)

add_executable(odom_kitti
  apps/odom_kitti.cpp
)
add_dependencies(odom_kitti
  ndt_omp
)
target_link_libraries(odom_kitti
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
  ${Sophus_LIBRARIES}
)

add_executable(workflow_scanner
  apps/workflow_scanner.cpp
)
add_dependencies(workflow_scanner
  ndt_omp
)
target_link_libraries(workflow_scanner
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ndt_omp
  ${Sophus_LIBRARIES}
)