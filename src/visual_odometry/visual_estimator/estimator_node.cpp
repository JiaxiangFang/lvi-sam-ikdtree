#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cstdint>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"
#include "../../lidar_odometry/imu_time.h"


Estimator estimator;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;

// global variable saving the lidar odometry
deque<nav_msgs::Odometry> odomQueue;
odometryRegister *odomRegister;

std::mutex m_buf;
std::mutex m_state;
std::mutex m_estimator;
std::mutex m_odom;
std::atomic<bool> stop_requested{false};
std::uint64_t estimator_epoch = 0;  // Reset holds m_estimator + m_buf.
double synchronization_td = 0.0;    // Protected by m_buf.

double latest_time = -1.0;
Eigen::Vector3d tmp_P = Eigen::Vector3d::Zero();
Eigen::Quaterniond tmp_Q = Eigen::Quaterniond::Identity();
Eigen::Vector3d tmp_V = Eigen::Vector3d::Zero();
Eigen::Vector3d tmp_Ba = Eigen::Vector3d::Zero();
Eigen::Vector3d tmp_Bg = Eigen::Vector3d::Zero();
Eigen::Vector3d acc_0 = Eigen::Vector3d::Zero();
Eigen::Vector3d gyr_0 = Eigen::Vector3d::Zero();
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = -1;
// The IMU callback reads a predictor snapshot, never a concurrently optimized Estimator.
Eigen::Vector3d tmp_gravity = Eigen::Vector3d::Zero();
bool prediction_ready = false;
int prediction_failure_count = 0;

void requestStop()
{
    {
        // Synchronize with the wait predicate to avoid a lost shutdown wakeup.
        std::lock_guard<std::mutex> lock(m_buf);
        stop_requested.store(true);
    }
    con.notify_all();
}

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    if (!std::isfinite(dt) || dt <= 0.0)
        return;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - tmp_gravity;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - tmp_gravity;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    init_imu = 0;
    tmp_gravity = estimator.g;
    prediction_failure_count = estimator.failureCount;
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());
}

std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (ros::ok())
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements;

        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + synchronization_td))
        {
            return measurements;
        }

        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + synchronization_td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + synchronization_td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front());
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

void resetEstimator()
{
    // Consistent lock order with process(): estimator -> buffers -> predictor.
    std::lock_guard<std::mutex> estimator_lock(m_estimator);
    std::lock_guard<std::mutex> buffer_lock(m_buf);
    std::lock_guard<std::mutex> state_lock(m_state);
    while (!feature_buf.empty()) feature_buf.pop();
    while (!imu_buf.empty()) imu_buf.pop();
    estimator.clearState();
    estimator.setParameter();
    synchronization_td = estimator.td;
    ++estimator_epoch;
    current_time = -1;
    last_imu_t = -1;
    init_feature = 0;
    init_imu = 1;
    prediction_ready = false;
    tmp_Q = Eigen::Quaterniond::Identity();
    tmp_P.setZero();
    tmp_V.setZero();
    {
        std::lock_guard<std::mutex> odom_lock(m_odom);
        odomQueue.clear();
    }
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    const auto& a = imu_msg->linear_acceleration;
    const auto& w = imu_msg->angular_velocity;
    if (!std::isfinite(a.x) || !std::isfinite(a.y) || !std::isfinite(a.z) ||
        !std::isfinite(w.x) || !std::isfinite(w.y) || !std::isfinite(w.z))
    {
        ROS_WARN_THROTTLE(1.0, "Dropping non-finite visual IMU measurement");
        return;
    }
    const double stamp = imu_msg->header.stamp.toSec();
    bool gap = false;
    {
        std::lock_guard<std::mutex> lock(m_buf);
        if (stop_requested.load()) return;
        if (stamp <= last_imu_t)
        {
            ROS_WARN_THROTTLE(1.0, "imu message in disorder!");
            return;
        }
        gap = last_imu_t >= 0.0 && !imu_safety::validInterval(stamp - last_imu_t);
    }
    if (gap)
    {
        ROS_WARN("IMU gap: reset visual estimator instead of integrating missing data");
        resetEstimator();
    }
    {
        std::lock_guard<std::mutex> lock(m_buf);
        if (stop_requested.load()) return;
        last_imu_t = stamp;
        imu_buf.push(imu_msg);
        if (imu_buf.size() > imu_safety::kMaxQueueSize)
        {
            imu_buf.pop();
            ROS_WARN_THROTTLE(5.0, "Visual IMU queue capped; check image input");
        }
        // Keep insertion and prediction in one buffer/state critical section:
        // the worker must not repropagate past this stamp before publication.
        std::lock_guard<std::mutex> lg(m_state);
        if (prediction_ready && stamp > latest_time)
        {
            predict(imu_msg);
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, imu_msg->header, prediction_failure_count);
        }
    }
    con.notify_one();
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    m_odom.lock();
    odomQueue.push_back(*odom_msg);
    if (odomQueue.size() > imu_safety::kMaxQueueSize)
        odomQueue.pop_front();
    m_odom.unlock();
}

void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (feature_msg->channels.size() < 6)
        return;
    for (std::size_t i = 0; i < 6; ++i)
        if (feature_msg->channels[i].values.size() != feature_msg->points.size())
            return;
    std::lock_guard<std::mutex> lock(m_buf);
    if (stop_requested.load())
        return;
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    feature_buf.push(feature_msg);
    con.notify_one();
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        resetEstimator();
        con.notify_all();
    }
    return;
}

// thread: visual-inertial odometry
void process()
{
    while (!stop_requested.load())
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return stop_requested.load() || !ros::ok() || !(measurements = getMeasurements()).empty();
                 });
        if (stop_requested.load() || !ros::ok())
            break;
        const auto batch_epoch = estimator_epoch;
        lk.unlock();

        std::unique_lock<std::mutex> estimator_lock(m_estimator);
        if (batch_epoch != estimator_epoch)
            continue;  // A restart invalidated the batch while the worker waited.
        for (auto &measurement : measurements)
        {
            if (stop_requested.load())
                break;
            auto img_msg = measurement.second;

            // 1. IMU pre-integration
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td;
                if (t <= img_t)
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time;
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("imu: dt:%f a: %f %f %f w: %f %f %f\n",dt, dx, dy, dz, rx, ry, rz);
                }
                else
                {
                    double dt_1 = img_t - current_time;
                    double dt_2 = t - img_t;
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                    //printf("dimu: dt:%f a: %f %f %f w: %f %f %f\n",dt_1, dx, dy, dz, rx, ry, rz);
                }
            }

            // 2. VINS Optimization
            // TicToc t_s;
            map<int, vector<pair<int, Eigen::Matrix<double, 8, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM;
                int camera_id = v % NUM_OF_CAM;
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                double depth = img_msg->channels[5].values[i];

                ROS_ASSERT(z == 1);
                Eigen::Matrix<double, 8, 1> xyz_uv_velocity_depth;
                xyz_uv_velocity_depth << x, y, z, p_u, p_v, velocity_x, velocity_y, depth;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity_depth);
            }

            // Get initialization info from lidar odometry
            vector<float> initialization_info;
            m_odom.lock();
            initialization_info = odomRegister->getOdometry(odomQueue, img_msg->header.stamp.toSec() + estimator.td);
            m_odom.unlock();


            estimator.processImage(image, initialization_info, img_msg->header);
            // double whole_t = t_s.toc();
            // printStatistics(estimator, whole_t);

            // 3. Visualization
            std_msgs::Header header = img_msg->header;
            pubOdometry(estimator, header);
            pubKeyPoses(estimator, header);
            pubCameraPose(estimator, header);
            pubPointCloud(estimator, header);
            pubTF(estimator, header);
            pubKeyframe(estimator);
        }
        std::lock_guard<std::mutex> buffer_lock(m_buf);
        std::lock_guard<std::mutex> state_lock(m_state);
        synchronization_td = estimator.td;
        prediction_ready = estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR;
        if (prediction_ready)
            update();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins");
    ros::NodeHandle n;
    ROS_INFO("\033[1;32m----> Visual Odometry Estimator Started.\033[0m");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);

    readParameters(n);
    estimator.setParameter();
    synchronization_td = estimator.td;

    registerPub(n);

    odomRegister = new odometryRegister(n);

    ros::Subscriber sub_imu     = n.subscribe(IMU_TOPIC,      5000, imu_callback,  ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_odom    = n.subscribe("odometry/imu", 5000, odom_callback);
    ros::Subscriber sub_image   = n.subscribe(PROJECT_NAME + "/vins/feature/feature", 1, feature_callback);
    ros::Subscriber sub_restart = n.subscribe(PROJECT_NAME + "/vins/feature/restart", 1, restart_callback);
    if (!USE_LIDAR)
        sub_odom.shutdown();

    std::thread measurement_process{process};

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    // roscpp has no on_shutdown callback API. The spinner returns after ROS
    // shutdown; wake the measurement worker before joining it here.
    requestStop();
    if (measurement_process.joinable())
        measurement_process.join();
    delete odomRegister;

    return 0;
}