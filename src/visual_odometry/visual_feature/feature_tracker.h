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
            if (p.x < 0 || abs(p.y / p.x) > 10 || abs(p.z / p.x) > 10)
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
        publishCloud(&pub_depth_cloud, depth_cloud_local, stamp_cur, "vins_body_ros");

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
        // 当严格几何路径失败时, 退化兜底到原始 3-NN 平面拟合, 保证深度覆盖率不低于基线.
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
        for (int i = 0; i < (int)features_3d_sphere->size(); ++i)
        {
            int found = kdtree->nearestKSearch(features_3d_sphere->points[i], depth_knn_k,
                                               pointSearchInd, pointSearchSqDis);
            // 至少需要 3 个近邻, 且最近邻在合理角距范围内 (否则该特征附近没有可靠激光点)
            if (found < 3 || pointSearchSqDis[0] > dist_sq_threshold)
            {
                cnt_no_neighbor++;
                continue;
            }

            // 单位视线方向 (特征点投影到单位球后的方向向量)
            Eigen::Vector3f V(features_3d_sphere->points[i].x,
                              features_3d_sphere->points[i].y,
                              features_3d_sphere->points[i].z);

            float s = -1.0f;        // 最终深度 (沿视线方向), <0 表示未求得
            bool geom_ok = false;   // 严格几何路径是否成功

            // ---------- 2.2 视线夹角约束 ----------
            // 候选激光点方向也是单位向量, 与 V 的点积即视线夹角余弦; 夹角过大则视为伪关联剔除.
            // DEPTH_USE_ROBUST_FITTING=0 时跳过整个鲁棒几何路径, 所有特征退回原始 3-NN 平面拟合,
            // 用于消融实验 (同一二进制内只切换"改进二开/关"这一个变量).
            vector<pair<float, int>> cand; // (depth=range, index in depth_cloud_unit_sphere)
            if (DEPTH_USE_ROBUST_FITTING)
            {
            cand.reserve(found);
            for (int j = 0; j < found; ++j)
            {
                const PointType& q = depth_cloud_unit_sphere->points[pointSearchInd[j]];
                float cosang = V.x() * q.x + V.y() * q.y + V.z() * q.z;
                if (cosang < cos_sight_threshold)
                    continue;
                cand.push_back(make_pair(q.intensity, pointSearchInd[j]));
            }

            if ((int)cand.size() >= 3)
            {
                // ---------- 2.3 深度聚类前景筛选 ----------
                // 按深度升序排列后检测深度跳变, 一旦相邻深度差超过阈值即判定空间不连续,
                // 依据“前景优先”原则截断, 仅保留距相机最近的前景点簇.
                sort(cand.begin(), cand.end(),
                     [](const pair<float, int>& a, const pair<float, int>& b){ return a.first < b.first; });
                int cluster_end = 1;
                for (; cluster_end < (int)cand.size(); ++cluster_end)
                {
                    if (cand[cluster_end].first - cand[cluster_end - 1].first > DEPTH_CLUSTER_GAP)
                        break;
                }

                if (cluster_end >= 3)
                {
                    // 收集前景簇的 3D 笛卡尔坐标 (单位方向 * range) 并求质心
                    vector<Eigen::Vector3f> pts;
                    pts.reserve(cluster_end);
                    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
                    for (int j = 0; j < cluster_end; ++j)
                    {
                        const PointType& q = depth_cloud_unit_sphere->points[cand[j].second];
                        Eigen::Vector3f P(q.x * q.intensity, q.y * q.intensity, q.z * q.intensity);
                        pts.push_back(P);
                        centroid += P;
                    }
                    centroid /= (float)pts.size();
                    float min_depth = cand[0].first;
                    float max_depth = cand[cluster_end - 1].first;

                    // ---------- 2.4 PCA 自适应点云结构感知 ----------
                    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
                    for (const auto& P : pts)
                    {
                        Eigen::Vector3f d = P - centroid;
                        cov += d * d.transpose();
                    }
                    cov /= (float)pts.size();
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(cov);
                    Eigen::Vector3f evals = es.eigenvalues();          // 升序: λ3 <= λ2 <= λ1
                    float l3 = max(evals(0), 0.0f);
                    float l2 = max(evals(1), 0.0f);
                    float l1 = max(evals(2), 1e-9f);
                    float linearity  = (l1 - l2) / l1;                 // 线性度
                    float planarity  = (l2 - l3) / l1;                 // 平面度
                    float scattering =  l3 / l1;                       // 散乱度 (三者之和=1)

                    if (scattering > DEPTH_PCA_SCATTER_THRESH ||
                        (scattering >= planarity && scattering >= linearity))
                    {
                        // 离散无序 (树叶 / 高噪区): 放弃 LiDAR 深度, 交由视觉后端三角化
                        cnt_scatter++;
                    }
                    else if (planarity >= linearity)
                    {
                        // 平面分布 (墙面 / 地面): 法向为最小特征值方向, 求视线射线与平面交点
                        Eigen::Vector3f normal = es.eigenvectors().col(0);
                        float denom = normal.dot(V);
                        if (fabs(denom) >= 1e-6f)
                        {
                            s = normal.dot(centroid) / denom;
                            if (s > 0.5f)
                            {
                                if (s < min_depth) s = min_depth;
                                if (s > max_depth) s = max_depth;
                                geom_ok = true;
                                cnt_plane++;
                            }
                        }
                    }
                    else
                    {
                        // 线性分布 (杆状物 / 树枝): 主方向为最大特征值方向,
                        // 求视线射线 (过原点, 方向 V) 与空间直线 (过 centroid, 方向 dir) 的公垂线对应深度.
                        Eigen::Vector3f dir = es.eigenvectors().col(2);
                        float b = V.dot(dir);
                        float denom = 1.0f - b * b;                    // a*c - b^2, 其中 a=c=1
                        if (fabs(denom) >= 1e-6f)                       // 射线与直线非平行
                        {
                            float d_ = -V.dot(centroid);
                            float e_ = -dir.dot(centroid);
                            s = (b * e_ - d_) / denom;                 // c=1
                            if (s > 0.5f)
                            {
                                if (s < min_depth) s = min_depth;
                                if (s > max_depth) s = max_depth;
                                geom_ok = true;
                                cnt_line++;
                            }
                        }
                    }
                    if (!geom_ok && s <= 0.5f && scattering <= DEPTH_PCA_SCATTER_THRESH)
                        cnt_geom_fail++;
                }
                else
                {
                    cnt_cluster_reject++;
                }
            }
            else
            {
                cnt_sight_reject++;
            }
            } // end if (DEPTH_USE_ROBUST_FITTING)

            // ---------- 退化兜底: 原始 3-NN 平面拟合 ----------
            // 严格几何路径失败 (但不是被判定为离散噪声) 时, 回退到 LVI-SAM 原始策略,
            // 保证深度覆盖率不低于基线, 避免改进反而丢失大量可用深度.
            // (消融关闭时所有特征都从这里取深度, 等价于原始算法)
            if (!geom_ok)
            {
                float r1 = depth_cloud_unit_sphere->points[pointSearchInd[0]].intensity;
                Eigen::Vector3f A(depth_cloud_unit_sphere->points[pointSearchInd[0]].x * r1,
                                  depth_cloud_unit_sphere->points[pointSearchInd[0]].y * r1,
                                  depth_cloud_unit_sphere->points[pointSearchInd[0]].z * r1);
                float r2 = depth_cloud_unit_sphere->points[pointSearchInd[1]].intensity;
                Eigen::Vector3f B(depth_cloud_unit_sphere->points[pointSearchInd[1]].x * r2,
                                  depth_cloud_unit_sphere->points[pointSearchInd[1]].y * r2,
                                  depth_cloud_unit_sphere->points[pointSearchInd[1]].z * r2);
                float r3 = depth_cloud_unit_sphere->points[pointSearchInd[2]].intensity;
                Eigen::Vector3f C(depth_cloud_unit_sphere->points[pointSearchInd[2]].x * r3,
                                  depth_cloud_unit_sphere->points[pointSearchInd[2]].y * r3,
                                  depth_cloud_unit_sphere->points[pointSearchInd[2]].z * r3);

                Eigen::Vector3f N = (A - B).cross(B - C);
                float denom = N(0) * V(0) + N(1) * V(1) + N(2) * V(2);
                if (pointSearchSqDis[2] < dist_sq_threshold && fabs(denom) >= 1e-6f)
                {
                    float sf = (N(0) * A(0) + N(1) * A(1) + N(2) * A(2)) / denom;
                    float min_depth = min(r1, min(r2, r3));
                    float max_depth = max(r1, max(r2, r3));
                    if (!(max_depth - min_depth > 2 || sf <= 0.5))
                    {
                        if (sf - max_depth > 0) sf = max_depth;
                        else if (sf - min_depth < 0) sf = min_depth;
                        s = sf;
                        geom_ok = true;
                        cnt_fallback++;
                    }
                }
            }

            if (geom_ok && s > 0.5f)
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
        Eigen::Affine3f original_T_normal = Eigen::Affine3f::Identity();
        original_T_normal.linear() = normal_R_original.linear().transpose();
        original_T_normal.translation() = Eigen::Vector3f((float)L_C_TX, (float)L_C_TY, (float)L_C_TZ);
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