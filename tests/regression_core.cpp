#include "lidar_odometry/mahonyMine.h"
#include "lidar_odometry/imu_time.h"
#include "visual_odometry/visual_feature/depth_fitting.h"
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>

#define CHECK(value) do { if (!(value)) throw std::runtime_error(#value); } while (false)

void initialize(Mahony& filter, float dt)
{
    for (int i = 0; i < static_cast<int>(0.2f / dt) + 2; ++i)
        filter.MahonyAHRSupdateIMU(0, 0, 0, 0, 0, 9.80511f, dt);
    CHECK(filter.isInitialized());
}

void inverseSquareRoot()
{
    for (float value : {1e-12f, 0.01f, 0.25f, 1.0f, 4.0f, 100.0f, 1e12f})
        CHECK(std::fabs(Mahony::invSqrt(value) * std::sqrt(value) - 1.0f) < 2e-6f);
    CHECK(Mahony::invSqrt(0.0f) == 0.0f);
    CHECK(Mahony::invSqrt(-1.0f) == 0.0f);
    CHECK(Mahony::invSqrt(std::numeric_limits<float>::quiet_NaN()) == 0.0f);
    CHECK(Mahony::invSqrt(std::numeric_limits<float>::infinity()) == 0.0f);
}

void sampleRates()
{
    for (int rate : {100, 200, 500, 1000})
    {
        Mahony filter;
        const float dt = 1.0f / rate;
        initialize(filter, dt);
        for (int i = 0; i < rate; ++i)
            filter.MahonyAHRSupdateIMU(0, 0, 1, 0, 0, 9.80511f, dt);
        CHECK(std::fabs(2.0f * std::atan2(filter.getQuaternionZ(), filter.getQuaternionW()) - 1.0f) < 1e-4f);
        const float norm = std::sqrt(filter.getQuaternionW() * filter.getQuaternionW() +
            filter.getQuaternionX() * filter.getQuaternionX() + filter.getQuaternionY() * filter.getQuaternionY() +
            filter.getQuaternionZ() * filter.getQuaternionZ());
        CHECK(std::fabs(norm - 1.0f) < 1e-6f);
    }
}

void imuGuards()
{
    Mahony filter;
    for (int i = 0; i < 100; ++i)
        filter.MahonyAHRSupdateIMU(0, 0, 1, 0, 0, 9.80511f, 0.005f);
    CHECK(!filter.isInitialized());
    for (int i = 0; i < 100; ++i)
        filter.MahonyAHRSupdateIMU(0, 0, 0, 0, 0, 20.0f, 0.005f);
    CHECK(!filter.isInitialized());
    initialize(filter, 0.005f);
    const float before = filter.getQuaternionW();
    for (float dt : {0.0f, -0.01f, 1.0f, std::numeric_limits<float>::quiet_NaN()})
        filter.MahonyAHRSupdateIMU(0, 0, 1, 0, 0, 9.80511f, dt);
    filter.MahonyAHRSupdateIMU(std::numeric_limits<float>::infinity(), 0, 0, 0, 0, 9.80511f, 0.005f);
    CHECK(filter.getQuaternionW() == before);
    filter.reset();
    CHECK(!filter.isInitialized());
}

void timestamps()
{
    imu_safety::TimestampTracker clock;
    using Status = imu_safety::TimestampTracker::Status;
    double dt = 0;
    CHECK(clock.observe(1.0, dt) == Status::First);
    CHECK(clock.observe(1.005, dt) == Status::Valid);
    CHECK(std::fabs(dt - 0.005) < 1e-10);
    CHECK(clock.observe(1.005, dt) == Status::Invalid);
    CHECK(clock.observe(0.0, dt) == Status::Invalid);
    CHECK(clock.observe(std::numeric_limits<double>::quiet_NaN(), dt) == Status::Invalid);
    CHECK(clock.observe(2.0, dt) == Status::Gap);
    CHECK(clock.observe(2.005, dt) == Status::Valid);
}

void rotationHistory()
{
    imu_safety::RotationHistory history;
    for (int i = 0; i <= 2000; ++i)
        CHECK(history.append(i * 0.001, 0, 0, 1));
    CHECK(history.size() == 2001);
    float x, y, z;
    CHECK(history.interpolate(1.2345, x, y, z));
    CHECK(std::fabs(z - 1.2345f) < 1e-5f);
    CHECK(!history.append(2.0, 0, 0, 1));
    CHECK(!history.append(1.0, 0, 0, 1));
    CHECK(!history.append(3.0, 0, 0, 1));
    CHECK(history.size() == 2001);
    history.clear();
    CHECK(!history.interpolate(1, x, y, z));
}

depth_fitting::Neighbor point(float x, float y, float z)
{
    const Eigen::Vector3f position(x, y, z);
    const Eigen::Vector3f direction = position.normalized();
    return {direction, position.norm(), (direction - Eigen::Vector3f::UnitX()).squaredNorm()};
}

depth_fitting::Options options()
{
    return {true, 0.02f, static_cast<float>(std::cos(2.0 * 3.141592653589793 / 180.0)), 1.0f, 0.2f};
}

void depthRejections()
{
    using depth_fitting::Outcome;
    const Eigen::Vector3f ray = Eigen::Vector3f::UnitX();
    auto config = options();
    const std::vector<depth_fitting::Neighbor> plane{point(5, -.05, -.05), point(5, .05, -.05), point(5, 0, .05)};
    config.minSightCosine = std::cos(0.001f);
    auto rejected = depth_fitting::fit(ray, plane, config);
    CHECK(rejected.outcome == Outcome::SightRejected && rejected.range < 0);
    config.robust = false;
    CHECK(depth_fitting::fit(ray, plane, config).outcome == Outcome::Fallback);

    config = options();
    const std::vector<depth_fitting::Neighbor> edge{point(4, .01, 0), point(5.5, -.02, -.02),
        point(5.5, .02, -.02), point(5.5, 0, .02)};
    rejected = depth_fitting::fit(ray, edge, config);
    CHECK(rejected.outcome == Outcome::ClusterRejected && rejected.range < 0);

    std::vector<depth_fitting::Neighbor> cube;
    for (float x : {4.95f, 5.05f})
        for (float y : {-0.05f, 0.05f})
            for (float z : {-0.05f, 0.05f}) cube.push_back(point(x, y, z));
    rejected = depth_fitting::fit(ray, cube, config);
    CHECK(rejected.outcome == Outcome::ScatterRejected && rejected.range < 0);
    config.robust = false;
    CHECK(depth_fitting::fit(ray, cube, config).outcome == Outcome::Fallback);
}

void depthGeometry()
{
    using depth_fitting::Outcome;
    const Eigen::Vector3f ray = Eigen::Vector3f::UnitX();
    const auto config = options();
    auto result = depth_fitting::fit(ray,
        {point(5, -.05, -.05), point(5, .05, -.05), point(5, -.05, .05), point(5, .05, .05)}, config);
    CHECK(result.outcome == Outcome::Plane && std::fabs(result.range - 5.0f) < 0.002f);
    result = depth_fitting::fit(ray, {point(5, -.05, 0), point(5, 0, 0), point(5, .05, 0)}, config);
    CHECK(result.outcome == Outcome::Line && std::fabs(result.range - 5.0f) < 0.002f);
    result = depth_fitting::fit(ray, {point(5, 0, 0), point(5.05, 0, 0), point(5.1, 0, 0)}, config);
    CHECK(result.outcome == Outcome::GeometryRejected && result.range < 0);
}

void depthFrame()
{
    const Eigen::Matrix3f rotation = Eigen::AngleAxisf(1.1f, Eigen::Vector3f(1, 2, 3).normalized()).toRotationMatrix();
    const Eigen::Vector3f offset(.1f, -.4f, .2f);
    const Eigen::Vector3f original(5, 1, 2);
    const Eigen::Vector3f normal = rotation * (original - offset);
    CHECK((depth_fitting::normalToOriginal(rotation, offset) * normal - original).norm() < 1e-5f);
}

int main(int argc, char** argv)
{
    const std::map<std::string, std::function<void()>> tests{
        {"inverse_sqrt", inverseSquareRoot}, {"sample_rates", sampleRates}, {"imu_guards", imuGuards},
        {"timestamps", timestamps}, {"rotation_history", rotationHistory}, {"depth_rejections", depthRejections},
        {"depth_geometry", depthGeometry}, {"depth_frame", depthFrame}};
    try
    {
        CHECK(argc == 2);
        tests.at(argv[1])();
        std::cout << "PASS " << argv[1] << std::endl;
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "FAIL: " << error.what() << std::endl;
        return 1;
    }
}