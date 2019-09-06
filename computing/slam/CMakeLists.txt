cmake_minimum_required(VERSION 2.8.3)
project(slam)

IF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    SET(PCL_OPENMP_PACKAGES pcl_omp_registration)
ENDIF (NOT (PCL_VERSION VERSION_LESS "1.7.2"))

find_package(OpenMP)
if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

find_package(CUDA)

if (CUDA_FOUND)
    add_definitions(-DCUDA_FOUND)
    list(APPEND PCL_OPENMP_PACKAGES ndt_gpu)
endif ()

find_package(Eigen3 REQUIRED)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

# G2O
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake-modules")
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIR})
link_libraries(${G2O_LIBRARIES})
if(G2O_FOUND)
    message(STATUS "Find G2O Sucesse!!" ...)
    message(STATUS "G2O_INCLUDE_DIR : ${G2O_INCLUDE_DIR}" ...)
else()
    message(WARNING "Can not Find G2O!!!" ...)
endif()

# CSparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
link_libraries(${CSPARSE_LIBRARY})
if(CSPARSE_FOUND)
    message(STATUS "Find CSPARSE Sucesse!! ${CSPARSE_LIBRARY} "...)
else()
    message(WARNING "Find CSPARSE Fail!!!"...)
endif()

# Cholmod
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})
link_libraries(${CHOLMOD_LIBRARY})
if(CHOLMOD_FOUND)
    message(STATUS "Find CHOLMOD Sucesse!! ${CHOLMOD_LIBRARY} "...)
else()
    message(WARNING "Find CHOLMOD Fail!!!"...)
endif()


add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  ndt_cpu
  ndt_tku
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
  velodyne_pointcloud
  ndt_tku
  ndt_cpu
  interactive_markers
  ${PCL_OPENMP_PACKAGES}
)

catkin_package(
        CATKIN_DEPENDS std_msgs velodyne_pointcloud ndt_tku ndt_cpu ${PCL_OPENMP_PACKAGES}
        DEPENDS PCL
)

add_library(slam3dlib
        include/slam/VertexSE3WithData.h
        include/slam/SLAM3DSystem.h
        include/slam/EKFSystem.h
        include/slam/PoseEKFSystem.h
        include/slam/InteractiveMarkersAPI.h
        src/SLAM3DSystem.cpp
        src/EKFSystem.cpp
        src/PoseEKFSystem.cpp
        src/InteractiveMarkersAPI.cpp
        )



###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(3d_slam_node src/3d_slam.cpp)
target_link_libraries(3d_slam_node
        ${catkin_LIBRARIES}
        ${G2O_LIBRARIES}
        slam3dlib)
add_dependencies(3d_slam_node ${catkin_EXPORTED_TARGETS})

add_executable(heuristic_slam_node src/heuristic_slam.cpp)
target_link_libraries(heuristic_slam_node
        ${catkin_LIBRARIES}
        ${G2O_LIBRARIES}
        slam3dlib)
add_dependencies(heuristic_slam_node ${catkin_EXPORTED_TARGETS})

if (CUDA_FOUND)
    target_include_directories(3d_slam_node PUBLIC ${CUDA_INCLUDE_DIRS})
    target_include_directories(heuristic_slam_node PUBLIC ${CUDA_INCLUDE_DIRS})
endif ()


if (NOT (PCL_VERSION VERSION_LESS "1.7.2"))
    set_target_properties(3d_slam_node PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
    set_target_properties(heuristic_slam_node PROPERTIES COMPILE_DEFINITIONS "USE_PCL_OPENMP")
endif (NOT (PCL_VERSION VERSION_LESS "1.7.2"))

add_executable(combine_3lidar_node src/combine_3lidar.cpp)
target_link_libraries(combine_3lidar_node ${catkin_LIBRARIES})
add_dependencies(combine_3lidar_node ${catkin_EXPORTED_TARGETS})

add_executable(combine_2lidar_node src/combine_2lidar.cpp)
target_link_libraries(combine_2lidar_node ${catkin_LIBRARIES})
add_dependencies(combine_2lidar_node ${catkin_EXPORTED_TARGETS})

add_executable(test_node src/test.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})
add_dependencies(test_node ${catkin_EXPORTED_TARGETS})

add_executable(test2_node src/test2.cpp)
target_link_libraries(test2_node ${catkin_LIBRARIES})
add_dependencies(test2_node ${catkin_EXPORTED_TARGETS})