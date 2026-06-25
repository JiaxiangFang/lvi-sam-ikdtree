#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <ros/ros.h>
#include <map>
#include "../feature_manager.h"

using namespace Eigen;
using namespace std;

class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>>& _points, 
                   const vector<float> &_lidar_initialization_info,
                   double _t):
        t{_t}, is_key_frame{false}, reset_id{-1}, gravity{9.805}
        {
            points = _points;
            
            // reset id in case lidar odometry relocate
            reset_id = (int)round(_lidar_initialization_info[0]);
            // Pose
            T.x() = _lidar_initialization_info[1];
            T.y() = _lidar_initialization_info[2];
            T.z() = _lidar_initialization_info[3];
            // Rotation
            Eigen::Quaterniond Q = Eigen::Quaterniond(_lidar_initialization_info[7],
                                                      _lidar_initialization_info[4],
                                                      _lidar_initialization_info[5],
                                                      _lidar_initialization_info[6]);
            R = Q.normalized().toRotationMatrix();
            // Velocity
            V.x() = _lidar_initialization_info[8];
            V.y() = _lidar_initialization_info[9];
            V.z() = _lidar_initialization_info[10];
            // Acceleration bias
            Ba.x() = _lidar_initialization_info[11];
            Ba.y() = _lidar_initialization_info[12];
            Ba.z() = _lidar_initialization_info[13];
            // Gyroscope bias
            Bg.x() = _lidar_initialization_info[14];
            Bg.y() = _lidar_initialization_info[15];
            Bg.z() = _lidar_initialization_info[16];
            // Gravity
            gravity = _lidar_initialization_info[17];
        };

        map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>> > > points;
        double t;
        
        IntegrationBase *pre_integration;
        bool is_key_frame;

        // Lidar odometry info
        int reset_id;
        Vector3d T;
        Matrix3d R;
        Vector3d V;
        Vector3d Ba;
        Vector3d Bg;
        double gravity;
};


bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);


class odometryRegister
{
public:

    ros::NodeHandle n;
    // vins_world 与激光里程计 world 之间相差绕 Z 轴 180°，用于把激光 odom 对齐到 vins_world
    tf::Quaternion vins_world_tf_odom;          // tf 形式（作用于姿态）
    Eigen::Quaterniond vins_world_q_odom;       // eigen 形式（作用于位置/速度）

    ros::Publisher pub_latest_odometry; 

    odometryRegister(ros::NodeHandle n_in):
    n(n_in)
    {
        // 绕 Z 轴 180°：(w,x,y,z) = (0,0,0,1)
        vins_world_tf_odom = tf::createQuaternionFromRPY(0, 0, M_PI);
        vins_world_q_odom = Eigen::Quaterniond(0, 0, 0, 1);
        // pub_latest_odometry = n.advertise<nav_msgs::Odometry>("odometry/test", 1000);
    }

    // convert odometry from ROS Lidar frame to VINS camera frame
    vector<float> getOdometry(deque<nav_msgs::Odometry>& odomQueue, double img_time)
    {
        vector<float> odometry_channel;
        odometry_channel.resize(18, -1); // reset id(1), P(3), Q(4), V(3), Ba(3), Bg(3), gravity(1)

        nav_msgs::Odometry odomCur;
        
        // pop old odometry msg
        while (!odomQueue.empty()) 
        {
            if (odomQueue.front().header.stamp.toSec() < img_time - 0.05)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
        {
            return odometry_channel;
        }

        // find the odometry time that is the closest to image time
        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            odomCur = odomQueue[i];

            if (odomCur.header.stamp.toSec() < img_time - 0.002) // 500Hz imu
                continue;
            else
                break;
        }

        // time stamp difference still too large
        if (abs(odomCur.header.stamp.toSec() - img_time) > 0.05)
        {
            return odometry_channel;
        }

        // ===================== 外参驱动：把激光 odom（LiDAR 位姿）转换为 VINS 的 IMU 位姿 =====================
        // 原版硬编码 q_lidar_to_cam(0,1,0,0) + 位置绕 Z 轴 180°，只适用于原传感器装配。
        // 现改为由 imu_to_lidar 外参构造，支持任意安装方式；原数据集填 (平移=0, ry=π) 即等价。
        tf::Quaternion q_odom_lidar;
        tf::quaternionMsgToTF(odomCur.pose.pose.orientation, q_odom_lidar);

        //   odom_T_lidar：LiDAR 在激光 odom world 下的位姿
        //   lidar_T_imu ：把点从 IMU 系变换到 LiDAR 系（外参，与 visualization 中一致）
        //   odom_T_imu  ：IMU 在激光 odom world 下的位姿
        tf::Transform odom_T_lidar = tf::Transform(q_odom_lidar,
            tf::Vector3(odomCur.pose.pose.position.x, odomCur.pose.pose.position.y, odomCur.pose.pose.position.z));
        tf::Transform lidar_T_imu = tf::Transform(
            tf::createQuaternionFromRPY(IMU_TO_LIDAR_ROLL, IMU_TO_LIDAR_PITCH, IMU_TO_LIDAR_YAW),
            tf::Vector3(IMU_TO_LIDAR_TX, IMU_TO_LIDAR_TY, IMU_TO_LIDAR_TZ));
        tf::Transform odom_T_imu = odom_T_lidar * lidar_T_imu;

        // 姿态：再绕 Z 轴 180° 对齐到 vins_world
        tf::Quaternion world_q_imu = vins_world_tf_odom * odom_T_imu.getRotation();
        tf::quaternionTFToMsg(world_q_imu, odomCur.pose.pose.orientation);

        // 位置/速度：取 IMU 位姿的平移，并绕 Z 轴 180° 对齐到 vins_world
        Eigen::Vector3d p_eigen(odom_T_imu.getOrigin().x(), odom_T_imu.getOrigin().y(), odom_T_imu.getOrigin().z());
        Eigen::Vector3d v_eigen(odomCur.twist.twist.linear.x, odomCur.twist.twist.linear.y, odomCur.twist.twist.linear.z);
        Eigen::Vector3d p_eigen_new = vins_world_q_odom * p_eigen;
        Eigen::Vector3d v_eigen_new = vins_world_q_odom * v_eigen;

        odomCur.pose.pose.position.x = p_eigen_new.x();
        odomCur.pose.pose.position.y = p_eigen_new.y();
        odomCur.pose.pose.position.z = p_eigen_new.z();

        odomCur.twist.twist.linear.x = v_eigen_new.x();
        odomCur.twist.twist.linear.y = v_eigen_new.y();
        odomCur.twist.twist.linear.z = v_eigen_new.z();

        // odomCur.header.stamp = ros::Time().fromSec(img_time);
        // odomCur.header.frame_id = "vins_world";
        // odomCur.child_frame_id = "vins_body";
        // pub_latest_odometry.publish(odomCur);

        odometry_channel[0] = odomCur.pose.covariance[0];
        odometry_channel[1] = odomCur.pose.pose.position.x;
        odometry_channel[2] = odomCur.pose.pose.position.y;
        odometry_channel[3] = odomCur.pose.pose.position.z;
        odometry_channel[4] = odomCur.pose.pose.orientation.x;
        odometry_channel[5] = odomCur.pose.pose.orientation.y;
        odometry_channel[6] = odomCur.pose.pose.orientation.z;
        odometry_channel[7] = odomCur.pose.pose.orientation.w;
        odometry_channel[8]  = odomCur.twist.twist.linear.x;
        odometry_channel[9]  = odomCur.twist.twist.linear.y;
        odometry_channel[10] = odomCur.twist.twist.linear.z;
        odometry_channel[11] = odomCur.pose.covariance[1];
        odometry_channel[12] = odomCur.pose.covariance[2];
        odometry_channel[13] = odomCur.pose.covariance[3];
        odometry_channel[14] = odomCur.pose.covariance[4];
        odometry_channel[15] = odomCur.pose.covariance[5];
        odometry_channel[16] = odomCur.pose.covariance[6];
        odometry_channel[17] = odomCur.pose.covariance[7];

        return odometry_channel;
    }
};
