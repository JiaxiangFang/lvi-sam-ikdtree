#pragma once

#include <cstdio>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camera_models/CameraFactory.h"
#include "camera_models/CataCamera.h"
#include "camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"
#include "depth_fitting.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

bool inBorder(const cv::Point2f &pt);

void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
void reduceVector(vector<int> &v, vector<uchar> status);

class FeatureTracker
{
  public:
    FeatureTracker();

    void readImage(const cv::Mat &_img,double _cur_time);

    void setMask();

    void addPoints();

    bool updateID(unsigned int i);

    void readIntrinsicParameter(const string &calib_file);

    void showUndistortion(const string &name);

    void rejectWithF();

    void undistortedPoints();

    cv::Mat mask;
    cv::Mat fisheye_mask;
    cv::Mat prev_img, cur_img, forw_img;
    vector<cv::Point2f> n_pts;
    vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
    vector<cv::Point2f> prev_un_pts, cur_un_pts;
    vector<cv::Point2f> pts_velocity;
    vector<int> ids;
    vector<int> track_cnt;
    map<int, cv::Point2f> cur_un_pts_map;
    map<int, cv::Point2f> prev_un_pts_map;
    camodocal::CameraPtr m_camera;
    double cur_time;
    double prev_time;

    static int n_id;
};


class DepthRegister
{
public:

    ros::NodeHandle n;
    // publisher for visualization
    ros::Publisher pub_depth_feature;
    ros::Publisher pub_depth_image;
    ros::Publisher pub_depth_cloud;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    int num_bins;
    float bin_res_deg;
    int depth_knn_k;
    vector<vector<PointType>> pointsArray;

    DepthRegister(ros::NodeHandle n_in):
    n(n_in)
    {
        // messages for RVIZ visualization
        pub_depth_feature = n.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/vins/depth/depth_feature", 5);
        pub_depth_image =   n.advertise<sensor_msgs::Image>      (PROJECT_NAME + "/vins/depth/depth_image",   5);
        pub_depth_cloud =   n.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/vins/depth/depth_cloud",   5);

        num_bins = max(1, (int)round(180.0 / DEPTH_SPHERE_RESOLUTION_DEG));
        bin_res_deg = 180.0f / (float)num_bins;
        depth_knn_k = max(3, DEPTH_KNN_K);

        pointsArray.resize(num_bins);
        for (int i = 0; i < num_bins; ++i)
            pointsArray[i].resize(num_bins);
    }

    sensor_msgs::ChannelFloat32 get_depth(const ros::Time& stamp_cur, const cv::Mat& imageCur, 
                                          const pcl::PointCloud<PointType>::Ptr& depthCloud,
                                          const camodocal::CameraPtr& camera_model ,
                                          const vector<geometry_msgs::Point32>& features_2d)
    {
        // 0.1 initialize depth for return
        sensor_msgs::ChannelFloat32 depth_of_point;
        depth_of_point.name = "depth";
        depth_of_point.values.resize(features_2d.size(), -1);

        // 0.2  check if depthCloud available
        if (depthCloud->size() == 0)
        {
            if (DEPTH_DEBUG_LOG)
                ROS_WARN_THROTTLE(1.0, "[DepthFit] skip: depthCloud empty (no lidar depth accumulated yet, check use_lidar / point_cloud_topic)");
            return depth_of_point;
        }

        // 0.3 look up transform at current image time
        try{
            listener.waitForTransform("vins_world", "vins_body_ros", stamp_cur, ros::Duration(0.01));
            listener.lookupTransform("vins_world", "vins_body_ros", stamp_cur, transform);
        } 
        catch (tf::TransformException ex){
            if (DEPTH_DEBUG_LOG)
                ROS_WARN_THROTTLE(1.0, "[DepthFit] skip: tf vins_world->vins_body_ros unavailable (VINS not initialized yet?)");
            return depth_of_point;
        }

        double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
        xCur = transform.getOrigin().x();
        yCur = transform.getOrigin().y();
        zCur = transform.getOrigin().z();
        tf::Matrix3x3 m(transform.getRotation());
        m.getRPY(rollCur, pitchCur, yawCur);
        Eigen::Affine3f transNow = pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

        // 0.4 transform cloud from global frame to camera frame
        pcl::PointCloud<PointType>::Ptr depth_cloud_local(new pcl::PointCloud<PointType>());
        // transNow = vins_world_T_lidar，求逆把世界系点云转回“激光原始轴、原点在相机光心”的局部系
        pcl::transformPointCloud(*depthCloud, *depth_cloud_local, transNow.inverse());

        // ===================== 轴归一化：激光原始轴 -> 相机对齐的标准轴(x前 y左 z上) =====================
        // depth_cloud_local 当前在“激光原始轴”，而下面的特征射线由相机系换算得到（也是 x 前的右手系）。
        // M3DGR 的 lidar_to_cam 是 ~90° 旋转，两者并非轴对齐，若不旋转对齐，kd-tree 深度匹配会完全错位。
        // 这里把激光点旋到与特征射线一致的标准轴，确保后续深度关联正确（仅旋转，平移已在 node 中处理）。
        Eigen::Affine3f cam_R_lidar_original = pcl::getTransformation(0, 0, 0, L_C_RX, L_C_RY, L_C_RZ); // 仅旋转：lidar->cam
        Eigen::Matrix4f lidar_normal_R_cam;   // 相机(z前x右y下) -> 标准轴(x前y左z上)
        lidar_normal_R_cam << 0, 0, 1, 0,
                             -1, 0, 0, 0,
                              0, -1, 0, 0,
                              0, 0, 0, 1;
        Eigen::Affine3f normal_R_original;
        normal_R_original = lidar_normal_R_cam * cam_R_lidar_original.matrix(); // 激光原始轴 -> 标准轴
        pcl::transformPointCloud(*depth_cloud_local, *depth_cloud_local, normal_R_original);
        const Eigen::Affine3f original_T_normal = depth_fitting::normalToOriginal(
            normal_R_original.linear(), Eigen::Vector3f(L_C_TX, L_C_TY, L_C_TZ));

        // 0.5 project undistorted normalized (z) 2d features onto a unit sphere
        pcl::PointCloud<PointType>::Ptr features_3d_sphere(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)features_2d.size(); ++i)
        {
            // normalize 2d feature to a unit sphere
            Eigen::Vector3f feature_cur(features_2d[i].x, features_2d[i].y, features_2d[i].z); // z always equal to 1
            feature_cur.normalize(); 
            // convert to ROS standard
            PointType p;
            p.x =  feature_cur(2);
            p.y = -feature_cur(0);
            p.z = -feature_cur(1);
            p.intensity = -1; // intensity will be used to save depth
            features_3d_sphere->push_back(p);
        }

        // 3. project depth cloud on a range image, filter points satcked in the same region
        cv::Mat rangeImage = cv::Mat(num_bins, num_bins, CV_32F, cv::Scalar::all(FLT_MAX));

        for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
        {
            PointType p = depth_cloud_local->points[i];
            // filter points not in camera view
            if (!pcl::isFinite(p) || p.x <= 1e-6f || abs(p.y / p.x) > 10 || abs(p.z / p.x) > 10)
                continue;
            // find row id in range image
            float row_angle = atan2(p.z, sqrt(p.x * p.x + p.y * p.y)) * 180.0 / M_PI + 90.0; // degrees, bottom -> up, 0 -> 360
            int row_id = round(row_angle / bin_res_deg);
            // find column id in range image
            float col_angle = atan2(p.x, p.y) * 180.0 / M_PI; // degrees, left -> right, 0 -> 360
            int col_id = round(col_angle / bin_res_deg);
            // id may be out of boundary
            if (row_id < 0 || row_id >= num_bins || col_id < 0 || col_id >= num_bins)
                continue;
            // only keep points that's closer
            float dist = pointDistance(p);
            if (dist < rangeImage.at<float>(row_id, col_id))
            {
                rangeImage.at<float>(row_id, col_id) = dist;
                pointsArray[row_id][col_id] = p;
            }
        }

        // 4. extract downsampled depth cloud from range image
        pcl::PointCloud<PointType>::Ptr depth_cloud_local_filter2(new pcl::PointCloud<PointType>());
        for (int i = 0; i < num_bins; ++i)
        {
            for (int j = 0; j < num_bins; ++j)
            {
                if (rangeImage.at<float>(i, j) != FLT_MAX)
                    depth_cloud_local_filter2->push_back(pointsArray[i][j]);
            }
        }
        *depth_cloud_local = *depth_cloud_local_filter2;
        if (pub_depth_cloud.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr depth_cloud_publish(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*depth_cloud_local, *depth_cloud_publish, original_T_normal);
            publishCloud(&pub_depth_cloud, depth_cloud_publish, stamp_cur, "vins_body_ros");
        }

        // 5. project depth cloud onto a unit sphere
        pcl::PointCloud<PointType>::Ptr depth_cloud_unit_sphere(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
        {
            PointType p = depth_cloud_local->points[i];
            float range = pointDistance(p);
            p.x /= range;
            p.y /= range;
            p.z /= range;
            p.intensity = range;
            depth_cloud_unit_sphere->push_back(p);
        }
        if (depth_cloud_unit_sphere->size() < 10)
        {
            if (DEPTH_DEBUG_LOG)
                ROS_WARN_THROTTLE(1.0, "[DepthFit] skip: too few depth points in view (%d < 10)", (int)depth_cloud_unit_sphere->size());
            return depth_of_point;
        }

        // 6. create a kd-tree using the spherical depth cloud
        pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>());
        kdtree->setInputCloud(depth_cloud_unit_sphere);

        // 7. find the feature depth using kd-tree
        // ===================== 改进二: 鲁棒几何聚类的视觉特征深度拟合 =====================
        // 在单位球 KNN 候选基础上, 依次执行:
        //   2.2 视线夹角约束: 剔除与特征视线方向偏离过大的伪关联点;
        //   2.3 深度聚类前景筛选: 按深度升序做跳变检测, 仅保留最近的前景点簇;
        //   2.4 PCA 自适应: 按特征值分布区分平面/线性/离散, 分别拟合或放弃深度.
        // 被视线/前景簇/PCA拒绝的点直接无效；仅可信前景上的数值退化允许受限3-NN兜底。
        TicToc t_depth;
        // 诊断计数 (仅在 DEPTH_DEBUG_LOG 开启时有意义)
        int cnt_total = (int)features_3d_sphere->size();
        int cnt_no_neighbor = 0;   // KNN 候选不足/超距
        int cnt_sight_reject = 0;  // 视线约束后候选不足
        int cnt_cluster_reject = 0;// 前景簇点数不足
        int cnt_plane = 0;         // PCA 判定平面并成功
        int cnt_line = 0;          // PCA 判定线性并成功
        int cnt_scatter = 0;       // PCA 判定离散放弃
        int cnt_geom_fail = 0;     // 几何解退化/不合理
        int cnt_fallback = 0;      // 退化兜底成功
        int cnt_success = 0;       // 最终成功 (几何+兜底)

        vector<int> pointSearchInd;
        vector<float> pointSearchSqDis;
        float dist_sq_threshold = pow(sin(bin_res_deg / 180.0 * M_PI) * 5.0, 2);
        float cos_sight_threshold = cos(DEPTH_SIGHT_ANGLE_DEG / 180.0 * M_PI);
        const depth_fitting::Options fitting_options{DEPTH_USE_ROBUST_FITTING != 0,
            dist_sq_threshold, cos_sight_threshold, static_cast<float>(DEPTH_CLUSTER_GAP),
            static_cast<float>(DEPTH_PCA_SCATTER_THRESH)};
        for (int i = 0; i < (int)features_3d_sphere->size(); ++i)
        {
            int found = kdtree->nearestKSearch(features_3d_sphere->points[i], depth_knn_k,
                                               pointSearchInd, pointSearchSqDis);
            std::vector<depth_fitting::Neighbor> neighbors;
            neighbors.reserve(found);
            for (int j = 0; j < found; ++j)
            {
                const auto& point = depth_cloud_unit_sphere->points[pointSearchInd[j]];
                neighbors.push_back({Eigen::Vector3f(point.x, point.y, point.z), point.intensity, pointSearchSqDis[j]});
            }
            const auto result = depth_fitting::fit(Eigen::Vector3f(features_3d_sphere->points[i].x,
                features_3d_sphere->points[i].y, features_3d_sphere->points[i].z), std::move(neighbors), fitting_options);
            switch (result.outcome)
            {
                case depth_fitting::Outcome::NoNeighbor: ++cnt_no_neighbor; break;
                case depth_fitting::Outcome::SightRejected: ++cnt_sight_reject; break;
                case depth_fitting::Outcome::ClusterRejected: ++cnt_cluster_reject; break;
                case depth_fitting::Outcome::ScatterRejected: ++cnt_scatter; break;
                case depth_fitting::Outcome::GeometryRejected: ++cnt_geom_fail; break;
                case depth_fitting::Outcome::Plane: ++cnt_plane; break;
                case depth_fitting::Outcome::Line: ++cnt_line; break;
                case depth_fitting::Outcome::Fallback: ++cnt_fallback; break;
            }
            const float s = result.range;
            if (s > 0.5f)
            {
                // convert feature into cartesian space if depth is available
                features_3d_sphere->points[i].x *= s;
                features_3d_sphere->points[i].y *= s;
                features_3d_sphere->points[i].z *= s;
                // the obtained depth here is for unit sphere, VINS estimator needs depth for normalized feature (by value z), (lidar x = camera z)
                features_3d_sphere->points[i].intensity = features_3d_sphere->points[i].x;
                cnt_success++;
            }
        }

        if (DEPTH_DEBUG_LOG)
        {
            // 用 ROS_WARN(stderr) 而非 ROS_INFO(stdout): roslaunch 子节点 stdout 在管道下为块缓冲,
            // 经 grep/stdbuf 也难实时刷新; stderr 行缓冲, 保证统计行始终可见.
            ROS_WARN("[DepthFit] feat=%d success=%d (plane=%d line=%d fallback=%d) | "
                     "reject: noNbr=%d sight=%d cluster=%d scatter=%d geomFail=%d | cost=%.2fms",
                     cnt_total, cnt_success, cnt_plane, cnt_line, cnt_fallback,
                     cnt_no_neighbor, cnt_sight_reject, cnt_cluster_reject, cnt_scatter,
                     cnt_geom_fail, t_depth.toc());
        }

        // 逐帧统计落盘 CSV (供消融实验定量分析): DEPTH_STATS_CSV 非空时追写一行.
        if (!DEPTH_STATS_CSV.empty())
        {
            static std::ofstream csv_ofs;
            static bool csv_inited = false;
            if (!csv_inited)
            {
                csv_ofs.open(DEPTH_STATS_CSV.c_str(), std::ios::out | std::ios::trunc);
                if (csv_ofs.is_open())
                    csv_ofs << "stamp,robust,feat,success,plane,line,fallback,"
                               "noNbr,sight,cluster,scatter,geomFail,cost_ms\n";
                csv_inited = true;
            }
            if (csv_ofs.is_open())
            {
                csv_ofs << std::fixed << std::setprecision(6) << stamp_cur.toSec() << ","
                        << DEPTH_USE_ROBUST_FITTING << ","
                        << cnt_total << "," << cnt_success << "," << cnt_plane << ","
                        << cnt_line << "," << cnt_fallback << "," << cnt_no_neighbor << ","
                        << cnt_sight_reject << "," << cnt_cluster_reject << ","
                        << cnt_scatter << "," << cnt_geom_fail << ","
                        << std::setprecision(3) << t_depth.toc() << "\n";
                csv_ofs.flush();
            }
        }

        // visualize features in cartesian 3d space (including the feature without depth (default 1))
        // 修复: features_3d_sphere 当前位于 lidar_normal(相机朝向 FLU)轴且以相机光心为原点,
        // 直接以 vins_body_ros(lidar_original)发布会让特征点比激光地图多转一个 normal_R_original(~58°),
        // 表现为"特征点不指向前方/比地图倾斜". 这里逆旋转回 lidar_original 轴向并补回相机光心偏移 L_C_T,
        // 使其经 world<-vins_body_ros TF 后与激光地图在 world 中对齐. (纯可视化, 不影响喂给VINS的深度值)
        pcl::PointCloud<PointType>::Ptr features_3d_publish(new pcl::PointCloud<PointType>());
        pcl::transformPointCloud(*features_3d_sphere, *features_3d_publish, original_T_normal);
        publishCloud(&pub_depth_feature, features_3d_publish, stamp_cur, "vins_body_ros");
        
        // update depth value for return
        for (int i = 0; i < (int)features_3d_sphere->size(); ++i)
        {
            if (features_3d_sphere->points[i].intensity > 3.0)
                depth_of_point.values[i] = features_3d_sphere->points[i].intensity;
        }

        // visualization project points on image for visualization (it's slow!)
        if (pub_depth_image.getNumSubscribers() != 0)
        {
            vector<cv::Point2f> points_2d;
            vector<float> points_distance;

            for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
            {
                // convert points from 3D to 2D
                Eigen::Vector3d p_3d(-depth_cloud_local->points[i].y,
                                     -depth_cloud_local->points[i].z,
                                      depth_cloud_local->points[i].x);
                Eigen::Vector2d p_2d;
                camera_model->spaceToPlane(p_3d, p_2d);
                
                points_2d.push_back(cv::Point2f(p_2d(0), p_2d(1)));
                points_distance.push_back(pointDistance(depth_cloud_local->points[i]));
            }

            cv::Mat showImage, circleImage;
            cv::cvtColor(imageCur, showImage, cv::COLOR_GRAY2RGB);
            circleImage = showImage.clone();
            for (int i = 0; i < (int)points_2d.size(); ++i)
            {
                float r, g, b;
                getColor(points_distance[i], 50.0, r, g, b);
                cv::circle(circleImage, points_2d[i], 0, cv::Scalar(r, g, b), 5);
            }
            cv::addWeighted(showImage, 1.0, circleImage, 0.7, 0, showImage); // blend camera image and circle image

            cv_bridge::CvImage bridge;
            bridge.image = showImage;
            bridge.encoding = "rgb8";
            sensor_msgs::Image::Ptr imageShowPointer = bridge.toImageMsg();
            imageShowPointer->header.stamp = stamp_cur;
            pub_depth_image.publish(imageShowPointer);
        }

        return depth_of_point;
    }

    void getColor(float p, float np, float&r, float&g, float&b) 
    {
        float inc = 6.0 / np;
        float x = p * inc;
        r = 0.0f; g = 0.0f; b = 0.0f;
        if ((0 <= x && x <= 1) || (5 <= x && x <= 6)) r = 1.0f;
        else if (4 <= x && x <= 5) r = x - 4;
        else if (1 <= x && x <= 2) r = 1.0f - (x - 1);

        if (1 <= x && x <= 3) g = 1.0f;
        else if (0 <= x && x <= 1) g = x - 0;
        else if (3 <= x && x <= 4) g = 1.0f - (x - 3);

        if (3 <= x && x <= 5) b = 1.0f;
        else if (2 <= x && x <= 3) b = x - 2;
        else if (5 <= x && x <= 6) b = 1.0f - (x - 5);
        r *= 255.0;
        g *= 255.0;
        b *= 255.0;
    }
};