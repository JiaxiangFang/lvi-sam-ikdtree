cmake_minimum_required(VERSION 2.8.3)
project(lvi_sam_ikdtree)

######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

######################
### Packages
######################
find_package(catkin REQUIRED COMPONENTS
    tf
    roscpp
    rospy
    roslib
    # msg
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    # cv
    cv_bridge
    # pcl
    pcl_conversions
    # msg generation
    message_generation
)

find_package(OpenMP REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
# Some ROS images ship GTSAM cmake files but the corresponding shared objects may be missing/renamed.
# If ROS gtsam runtime is unavailable, force CMake to resolve GTSAM from /usr/local.
if(EXISTS "/opt/ros/noetic/lib/cmake/GTSAM/GTSAMConfig.cmake" AND
   NOT EXISTS "/opt/ros/noetic/lib/x86_64-linux-gnu/libgtsam.so.4.2.0" AND
   EXISTS "/usr/local/lib/cmake/GTSAM/GTSAMConfig.cmake")
    set(GTSAM_DIR "/usr/local/lib/cmake/GTSAM" CACHE PATH "GTSAM config directory" FORCE)
endif()
find_package(GTSAM REQUIRED QUIET)
find_package(Boost REQUIRED COMPONENTS filesystem program_options system timer)


######################
### Message generation
######################
add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

######################
### Catkin
######################
catkin_package(
    DEPENDS PCL GTSAM
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ikdtree 
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${OpenCV_LIBRARY_DIRS}
    ${GTSAM_LIBRARY_DIRS}
)

######################
### visual odometry
######################
file(GLOB visual_feature_files
    "src/visual_odometry/visual_feature/*.cpp"
    "src/visual_odometry/visual_feature/camera_models/*.cc"
)
file(GLOB visual_odometry_files
    "src/visual_odometry/visual_estimator/*.cpp"
    "src/visual_odometry/visual_estimator/factor/*.cpp"
    "src/visual_odometry/visual_estimator/initial/*.cpp"
    "src/visual_odometry/visual_estimator/utility/*.cpp"
)
file(GLOB visual_loop_files
    "src/visual_odometry/visual_loop/*.cpp"
    "src/visual_odometry/visual_loop/utility/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DBoW/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DUtils/*.cpp"
    "src/visual_odometry/visual_loop/ThirdParty/DVision/*.cpp"
    "src/visual_odometry/visual_feature/camera_models/*.cc"
)
# Visual Feature Tracker
add_executable(${PROJECT_NAME}_visual_feature ${visual_feature_files})
target_link_libraries(${PROJECT_NAME}_visual_feature ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
# Visual Odometry
add_executable(${PROJECT_NAME}_visual_odometry ${visual_odometry_files})
target_link_libraries(${PROJECT_NAME}_visual_odometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 
# Visual Lopp
add_executable(${PROJECT_NAME}_visual_loop ${visual_loop_files})
target_link_libraries(${PROJECT_NAME}_visual_loop ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES}) 

######################
### lidar odometry
######################

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/lidar_odometry/imuPreintegration.cpp src/lidar_odometry/mahonyMine.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam Boost::timer)
# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/lidar_odometry/imageProjection.cpp src/lidar_odometry/mahonyMine.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/lidar_odometry/featureExtraction.cpp src/lidar_odometry/mahonyMine.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization src/lidar_odometry/mapOptmization_ikdtree.cpp src/ikdtree/ikd_Tree.cpp src/lidar_odometry/mahonyMine.cpp)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization PRIVATE ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam Boost::timer)

######################
### Regression tests
######################
if(CATKIN_ENABLE_TESTING)
    add_executable(${PROJECT_NAME}_regression_core tests/regression_core.cpp src/lidar_odometry/mahonyMine.cpp)
    target_include_directories(${PROJECT_NAME}_regression_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
    option(LVI_TEST_SANITIZERS "Enable address/undefined sanitizers for dependency-light regression tests" ON)
    if(LVI_TEST_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${PROJECT_NAME}_regression_core PRIVATE -O1 -fno-omit-frame-pointer -fsanitize=address,undefined)
        target_link_libraries(${PROJECT_NAME}_regression_core -fsanitize=address,undefined)
    endif()
    foreach(case inverse_sqrt sample_rates imu_guards timestamps rotation_history depth_rejections depth_geometry depth_frame)
        add_test(NAME ${PROJECT_NAME}_${case} COMMAND ${PROJECT_NAME}_regression_core ${case})
        set_tests_properties(${PROJECT_NAME}_${case} PROPERTIES TIMEOUT 30)
    endforeach()

    add_executable(${PROJECT_NAME}_ikdtree_snapshot_test tests/ikdtree_snapshot_test.cpp src/ikdtree/ikd_Tree.cpp)
    target_link_libraries(${PROJECT_NAME}_ikdtree_snapshot_test ${PCL_LIBRARIES} pthread)
    add_test(NAME ${PROJECT_NAME}_ikdtree_snapshot COMMAND ${PROJECT_NAME}_ikdtree_snapshot_test)
    set_tests_properties(${PROJECT_NAME}_ikdtree_snapshot PROPERTIES TIMEOUT 60)
endif()
