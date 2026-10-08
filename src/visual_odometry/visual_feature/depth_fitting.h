#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <vector>

namespace depth_fitting
{
enum class Outcome { NoNeighbor, SightRejected, ClusterRejected, ScatterRejected, GeometryRejected, Plane, Line, Fallback };

struct Neighbor
{
    Eigen::Vector3f direction;
    float range;
    float squaredDistance;
};

struct Options
{
    bool robust;
    float maxSquaredDistance;
    float minSightCosine;
    float clusterGap;
    float maxScattering;
};

struct Result
{
    Outcome outcome;
    float range;
};

inline Result fitThreePointPlane(const Eigen::Vector3f& ray, const std::vector<Neighbor>& support,
                                float maxSquaredDistance)
{
    if (support.size() < 3 || support[2].squaredDistance >= maxSquaredDistance)
        return {Outcome::GeometryRejected, -1.0f};
    const Eigen::Vector3f a = support[0].direction * support[0].range;
    const Eigen::Vector3f b = support[1].direction * support[1].range;
    const Eigen::Vector3f c = support[2].direction * support[2].range;
    const Eigen::Vector3f normal = (a - b).cross(b - c);
    const float denominator = normal.dot(ray);
    if (!std::isfinite(denominator) || std::fabs(denominator) < 1e-6f)
        return {Outcome::GeometryRejected, -1.0f};
    const float range = normal.dot(a) / denominator;
    const float low = std::min(support[0].range, std::min(support[1].range, support[2].range));
    const float high = std::max(support[0].range, std::max(support[1].range, support[2].range));
    if (!std::isfinite(range) || range <= 0.5f || high - low > 2.0f)
        return {Outcome::GeometryRejected, -1.0f};
    return {Outcome::Fallback, std::max(low, std::min(high, range))};
}

// Keep the geometric decision independent of ROS/TF so the actual acceptance
// and fallback paths can be regression-tested with synthetic neighborhoods.
inline Result fit(const Eigen::Vector3f& ray, std::vector<Neighbor> neighbors, const Options& options)
{
    if (!ray.allFinite() || ray.squaredNorm() < 1e-12f)
        return {Outcome::NoNeighbor, -1.0f};
    const Eigen::Vector3f direction = ray.normalized();
    neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(), [](const Neighbor& point) {
        return !point.direction.allFinite() || !std::isfinite(point.range) || point.range <= 0.0f ||
               !std::isfinite(point.squaredDistance) || point.squaredDistance < 0.0f;
    }), neighbors.end());
    const auto angularOrder = [](const Neighbor& a, const Neighbor& b) { return a.squaredDistance < b.squaredDistance; };
    std::stable_sort(neighbors.begin(), neighbors.end(), angularOrder);
    if (neighbors.size() < 3 || neighbors.front().squaredDistance > options.maxSquaredDistance)
        return {Outcome::NoNeighbor, -1.0f};
    if (!options.robust)
        return fitThreePointPlane(direction, neighbors, options.maxSquaredDistance);

    neighbors.erase(std::remove_if(neighbors.begin(), neighbors.end(), [&](const Neighbor& point) {
        return direction.dot(point.direction) < options.minSightCosine;
    }), neighbors.end());
    if (neighbors.size() < 3)
        return {Outcome::SightRejected, -1.0f};  // Terminal rejection: never fall back.

    std::stable_sort(neighbors.begin(), neighbors.end(), [](const Neighbor& a, const Neighbor& b) { return a.range < b.range; });
    std::size_t end = 1;
    while (end < neighbors.size() && neighbors[end].range - neighbors[end - 1].range <= options.clusterGap)
        ++end;
    neighbors.resize(end);
    if (neighbors.size() < 3)
        return {Outcome::ClusterRejected, -1.0f};

    const float low = neighbors.front().range;
    const float high = neighbors.back().range;
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for (const auto& point : neighbors)
        centroid += point.direction * point.range;
    centroid /= static_cast<float>(neighbors.size());
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto& point : neighbors)
    {
        const Eigen::Vector3f delta = point.direction * point.range - centroid;
        covariance += delta * delta.transpose();
    }
    covariance /= static_cast<float>(neighbors.size());
    if (!covariance.allFinite())
        return {Outcome::GeometryRejected, -1.0f};
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
        return {Outcome::GeometryRejected, -1.0f};
    const float l3 = std::max(solver.eigenvalues()(0), 0.0f);
    const float l2 = std::max(solver.eigenvalues()(1), 0.0f);
    const float l1 = std::max(solver.eigenvalues()(2), 1e-9f);
    const float linearity = (l1 - l2) / l1;
    const float planarity = (l2 - l3) / l1;
    const float scattering = l3 / l1;
    if (scattering > options.maxScattering || (scattering >= planarity && scattering >= linearity))
        return {Outcome::ScatterRejected, -1.0f};

    float range = -1.0f;
    Outcome outcome = Outcome::Plane;
    if (planarity >= linearity)
    {
        const Eigen::Vector3f normal = solver.eigenvectors().col(0);
        const float denominator = normal.dot(direction);
        if (std::fabs(denominator) >= 1e-6f)
            range = normal.dot(centroid) / denominator;
    }
    else
    {
        outcome = Outcome::Line;
        const Eigen::Vector3f axis = solver.eigenvectors().col(2);
        const float b = direction.dot(axis);
        const float denominator = 1.0f - b * b;
        if (std::fabs(denominator) >= 1e-6f)
            range = (direction.dot(centroid) - b * axis.dot(centroid)) / denominator;
    }
    if (std::isfinite(range) && range > 0.5f)
        return {outcome, std::max(low, std::min(high, range))};

    // Only a trusted, non-scattered foreground cluster may use the numerical
    // fallback. Discarded sight/background points can never be reintroduced.
    std::stable_sort(neighbors.begin(), neighbors.end(), angularOrder);
    return fitThreePointPlane(direction, neighbors, options.maxSquaredDistance);
}

inline Eigen::Affine3f normalToOriginal(const Eigen::Matrix3f& normal_R_original,
                                      const Eigen::Vector3f& cameraOffset)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.linear() = normal_R_original.transpose();
    transform.translation() = cameraOffset;
    return transform;
}
}  // namespace depth_fitting