#pragma once

#include <cstdio>
#include <iostream>
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

    const int num_bins = 720;               // NOTE: num_bins:360 -> 720
    vector<vector<PointType>> pointsArray;

    DepthRegister(ros::NodeHandle n_in):
    n(n_in)
    {
        // messages for RVIZ visualization
        pub_depth_feature = n.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/vins/depth/depth_feature", 5);
        pub_depth_image =   n.advertise<sensor_msgs::Image>      (PROJECT_NAME + "/vins/depth/depth_image",   5);
        pub_depth_cloud =   n.advertise<sensor_msgs::PointCloud2>(PROJECT_NAME + "/vins/depth/depth_cloud",   5);

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
            return depth_of_point;

        // 0.3 look up transform at current image time
        try{
            listener.waitForTransform("vins_world", "vins_body_ros", stamp_cur, ros::Duration(0.01));
            listener.lookupTransform("vins_world", "vins_body_ros", stamp_cur, transform);
        } 
        catch (tf::TransformException ex){
            // ROS_ERROR("image no tf");
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
        pcl::transformPointCloud(*depthCloud, *depth_cloud_local, transNow.inverse());

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
        float bin_res = 180.0 / (float)num_bins;            // NOTE: bin_res:0.5 -> 0.25度
        cv::Mat rangeImage = cv::Mat(num_bins, num_bins, CV_32F, cv::Scalar::all(FLT_MAX));

        for (int i = 0; i < (int)depth_cloud_local->size(); ++i)
        {
            PointType p = depth_cloud_local->points[i];
            // filter points not in camera view
            if (p.x < 0 || abs(p.y / p.x) > 10 || abs(p.z / p.x) > 10)
                continue;
            
            float row_angle = atan2(p.z, sqrt(p.x * p.x + p.y * p.y)) * 180.0 / M_PI + 90.0;    // row id
            int row_id = round(row_angle / bin_res);
            
            float col_angle = atan2(p.x, p.y) * 180.0 / M_PI;                                   // col id
            int col_id = round(col_angle / bin_res);
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
            return depth_of_point;

        // 6. create a kd-tree using the spherical depth cloud
        pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>());
        kdtree->setInputCloud(depth_cloud_unit_sphere);

        // 7. find the feature depth using kd-tree
        int K = 7;
        vector<int> pointSearchInd;
        vector<float> pointSearchSqDis;
        float dist_sq_threshold = pow(sin(bin_res / 180.0 * M_PI) * 5.0, 2);
        for (int i = 0; i < (int)features_3d_sphere->size(); ++i)
        {
            // NOTE: 鲁棒估计步骤 1：宽进严出（K近邻搜索 + 角度阈值过滤）
            vector<PointType> angular_inliers;
            kdtree->nearestKSearch(features_3d_sphere->points[i], K, pointSearchInd, pointSearchSqDis);
            for(size_t j = 0; j < pointSearchInd.size(); ++j)
            {
                if(pointSearchSqDis[j] > dist_sq_threshold) break;
                angular_inliers.push_back(features_3d_sphere->points[pointSearchInd[j]]);
            }
            if(angular_inliers.size()<3) continue;

            // NOTE: 鲁棒估计步骤 2：前景优先（深度排序 + 跳变检测）
            vector<PointType> final_candidates;
            std::sort(angular_inliners.begin(), angular_inliners.end(),
                [](const PointType& a, const PointType& b) -> bool {
                    return a.intensity < b.intensity;
                });
            final_candidates.push_back(angular_inliners[0]);
            for(size_t j = 1; j < angular_inliners.size(); ++j)
            {
                if(fabs(angular_inliers[j] - angular_inliers[j - 1]) > 1.0) break;
                final_candidates.push_back(angular_inliers[j]);
            }

            if(final_candidates.size() < 3) continue;
            {
                // convert feature into cartesian space if depth is available
                features_3d_sphere->points[i].x *= s;
                features_3d_sphere->points[i].y *= s;
                features_3d_sphere->points[i].z *= s;
                // the obtained depth here is for unit sphere, VINS estimator needs depth for normalized feature (by value z), (lidar x = camera z)
                features_3d_sphere->points[i].intensity = features_3d_sphere->points[i].x;
            }
        }

        // visualize features in cartesian 3d space (including the feature without depth (default 1))
        publishCloud(&pub_depth_feature, features_3d_sphere, stamp_cur, "vins_body_ros");
        
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