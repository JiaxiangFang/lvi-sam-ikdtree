#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

const int WINDOW_SIZE = 10;


const double FOCAL_LENGTH = 460.0;
const int NUM_OF_CAM = 1;
const int NUM_OF_F = 1000;
// #define UNIT_SPHERE_ERROR 1


extern double INIT_DEPTH;
extern double MIN_PARALLAX;
extern int ESTIMATE_EXTRINSIC;

extern double ACC_N, ACC_W;
extern double GYR_N, GYR_W;

extern std::vector<Eigen::Matrix3d> RIC;
extern std::vector<Eigen::Vector3d> TIC;
extern Eigen::Vector3d G;

extern double BIAS_ACC_THRESHOLD;
extern double BIAS_GYR_THRESHOLD;
extern double SOLVER_TIME;
extern int NUM_ITERATIONS;
extern std::string EX_CALIB_RESULT_PATH;
extern std::string PROJECT_NAME;
extern std::string IMU_TOPIC;
extern double TD;
extern double TR;
extern int ESTIMATE_TD;
extern int ROLLING_SHUTTER;
extern double ROW, COL;

extern int USE_LIDAR;
extern int ALIGN_CAMERA_LIDAR_COORDINATE;

// ===================== IMU -> LiDAR 外参（可配置，替代原硬编码 q_cam_to_lidar(0,1,0,0)）=====================
// 含义：把一个点的坐标从 IMU 坐标系变换到 LiDAR 坐标系，即 lidar_T_imu = (旋转 RPY, 平移 XYZ)。
// 数值约定与 yaml 键 imu_to_lidar_* 一一对应；原数据集填 (平移=0, ry=π) 即可复现原版行为。
extern double IMU_TO_LIDAR_TX;     // IMU->LiDAR 平移 X（米）
extern double IMU_TO_LIDAR_TY;     // IMU->LiDAR 平移 Y（米）
extern double IMU_TO_LIDAR_TZ;     // IMU->LiDAR 平移 Z（米）
extern double IMU_TO_LIDAR_ROLL;   // IMU->LiDAR 绕 X 轴旋转 roll（弧度）
extern double IMU_TO_LIDAR_PITCH;  // IMU->LiDAR 绕 Y 轴旋转 pitch（弧度）
extern double IMU_TO_LIDAR_YAW;    // IMU->LiDAR 绕 Z 轴旋转 yaw（弧度）

void readParameters(ros::NodeHandle &n);

enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,
    SIZE_SPEEDBIAS = 9,
    SIZE_FEATURE = 1
};

enum StateOrder
{
    O_P = 0,
    O_R = 3,
    O_V = 6,
    O_BA = 9,
    O_BG = 12
};

enum NoiseOrder
{
    O_AN = 0,
    O_GN = 3,
    O_AW = 6,
    O_GW = 9
};
