#include "parameters.h"

std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::string POINT_CLOUD_TOPIC;
std::string PROJECT_NAME;

std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

double L_C_TX;
double L_C_TY;
double L_C_TZ;
double L_C_RX;
double L_C_RY;
double L_C_RZ;

double DEPTH_SPHERE_RESOLUTION_DEG;
int DEPTH_KNN_K;
double DEPTH_SIGHT_ANGLE_DEG;
double DEPTH_CLUSTER_GAP;
double DEPTH_PCA_SCATTER_THRESH;
int DEPTH_DEBUG_LOG;
int DEPTH_USE_ROBUST_FITTING;
std::string DEPTH_STATS_CSV;

int USE_LIDAR;
int LIDAR_SKIP;


void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    n.getParam("vins_config_file", config_file);
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    // project name
    fsSettings["project_name"] >> PROJECT_NAME;
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);

    // sensor topics
    fsSettings["image_topic"]       >> IMAGE_TOPIC;
    fsSettings["imu_topic"]         >> IMU_TOPIC;
    fsSettings["point_cloud_topic"] >> POINT_CLOUD_TOPIC;

    // lidar configurations
    fsSettings["use_lidar"] >> USE_LIDAR;
    fsSettings["lidar_skip"] >> LIDAR_SKIP;

    // feature and image settings
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];

    L_C_TX = fsSettings["lidar_to_cam_tx"];
    L_C_TY = fsSettings["lidar_to_cam_ty"];
    L_C_TZ = fsSettings["lidar_to_cam_tz"];
    L_C_RX = fsSettings["lidar_to_cam_rx"];
    L_C_RY = fsSettings["lidar_to_cam_ry"];
    L_C_RZ = fsSettings["lidar_to_cam_rz"];

    DEPTH_SPHERE_RESOLUTION_DEG = 0.5;
    DEPTH_KNN_K = 7;
    DEPTH_SIGHT_ANGLE_DEG = 2.0;
    DEPTH_CLUSTER_GAP = 1.0;
    DEPTH_PCA_SCATTER_THRESH = 0.2;
    DEPTH_DEBUG_LOG = 0;
    DEPTH_USE_ROBUST_FITTING = 1;
    DEPTH_STATS_CSV = "";

    if (!fsSettings["depth_sphere_resolution_deg"].empty())
        DEPTH_SPHERE_RESOLUTION_DEG = (double)fsSettings["depth_sphere_resolution_deg"];
    if (!fsSettings["depth_knn_k"].empty())
        DEPTH_KNN_K = (int)fsSettings["depth_knn_k"];
    if (!fsSettings["depth_sight_angle_deg"].empty())
        DEPTH_SIGHT_ANGLE_DEG = (double)fsSettings["depth_sight_angle_deg"];
    if (!fsSettings["depth_cluster_gap"].empty())
        DEPTH_CLUSTER_GAP = (double)fsSettings["depth_cluster_gap"];
    if (!fsSettings["depth_pca_scatter_thresh"].empty())
        DEPTH_PCA_SCATTER_THRESH = (double)fsSettings["depth_pca_scatter_thresh"];
    if (!fsSettings["depth_debug_log"].empty())
        DEPTH_DEBUG_LOG = (int)fsSettings["depth_debug_log"];
    if (!fsSettings["depth_use_robust_fitting"].empty())
        DEPTH_USE_ROBUST_FITTING = (int)fsSettings["depth_use_robust_fitting"];
    if (!fsSettings["depth_stats_csv"].empty())
        fsSettings["depth_stats_csv"] >> DEPTH_STATS_CSV;

    if (DEPTH_SPHERE_RESOLUTION_DEG <= 0.0)
    {
        ROS_WARN("Invalid depth_sphere_resolution_deg (%.3f), fallback to 0.5", DEPTH_SPHERE_RESOLUTION_DEG);
        DEPTH_SPHERE_RESOLUTION_DEG = 0.5;
    }
    if (DEPTH_KNN_K < 3)
    {
        ROS_WARN("depth_knn_k (%d) is too small, clamp to 3", DEPTH_KNN_K);
        DEPTH_KNN_K = 3;
    }
    if (DEPTH_SIGHT_ANGLE_DEG <= 0.0)
        DEPTH_SIGHT_ANGLE_DEG = 2.0;
    if (DEPTH_CLUSTER_GAP <= 0.0)
        DEPTH_CLUSTER_GAP = 1.0;
    if (DEPTH_PCA_SCATTER_THRESH <= 0.0)
        DEPTH_PCA_SCATTER_THRESH = 0.2;

    // fisheye mask
    FISHEYE = fsSettings["fisheye"];
    if (FISHEYE == 1)
    {
        std::string mask_name;
        fsSettings["fisheye_mask"] >> mask_name;
        FISHEYE_MASK = pkg_path + mask_name;
    }

    // camera config
    CAM_NAMES.push_back(config_file);

    WINDOW_SIZE = 20;
    STEREO_TRACK = false;
    FOCAL_LENGTH = 460;
    PUB_THIS_FRAME = false;

    if (FREQ == 0)
        FREQ = 100;

    fsSettings.release();
    usleep(100);
}

float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

void publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    if (thisPub->getNumSubscribers() == 0)
        return;
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    thisPub->publish(tempCloud); 
}