#include "ikd_Tree.h"
#include <iostream>
#include <memory>
#include <stdexcept>

bool splitPlaneDeletion(float perturbation)
{
    using Tree = KD_TREE<pcl::PointXYZI>;
    auto tree = std::make_shared<Tree>(0.5f, 0.6f, 0.2f);
    Tree::PointVector points(3), snapshot;
    points[0].x = 0.0f; points[0].y = 0.0f; points[0].z = 0.0f;
    points[1].x = 0.0f; points[1].y = 0.1f; points[1].z = 0.0f;
    points[2].x = 2.0f; points[2].y = 0.0f; points[2].z = 0.0f;
    tree->Build(points);
    if (!tree->Root_Node || !tree->Root_Node->left_son_ptr || tree->Root_Node->division_axis != 0)
        return false;
    // Both the root and its left child have x=0; a right-only search misses it.
    auto target = tree->Root_Node->left_son_ptr->point;
    target.x += perturbation;
    Tree::PointVector removed{target};
    tree->Delete_Points(removed);
    tree->Snapshot(snapshot);
    return snapshot.size() == 2;
}

int main()
{
    if (!splitPlaneDeletion(0.0f) || !splitPlaneDeletion(EPSS * 0.5f))
    {
        std::cerr << "FAIL deletion at an equal/tolerance-overlapping split plane" << std::endl;
        return 6;
    }
    using Tree = KD_TREE<pcl::PointXYZI>;
    auto tree = std::make_shared<Tree>(0.5f, 0.6f, 0.2f);
    Tree::PointVector points, snapshot;
    tree->Snapshot(snapshot);
    if (!snapshot.empty()) return 1;
    for (int i = 0; i < 5000; ++i)
    {
        pcl::PointXYZI point;
        point.x = (i % 100) * 0.3f;
        point.y = (i / 100) * 0.3f;
        point.z = 0;
        point.intensity = i;
        points.push_back(point);
    }
    tree->Build(points);
    tree->Snapshot(snapshot);
    if (snapshot.size() != points.size()) return 2;

    // Deletion crosses the asynchronous rebuild threshold. The public calls
    // are serialized like mapping; the internal rebuild thread remains live.
    for (int batch = 0; batch < 30; ++batch)
    {
        Tree::PointVector removed(points.begin() + batch * 100, points.begin() + (batch + 1) * 100);
        tree->Delete_Points(removed);
        tree->Snapshot(snapshot);
        if (snapshot.size() != points.size() - (batch + 1) * 100)
        {
            std::cerr << "FAIL deletion batch " << batch << ": snapshot=" << snapshot.size()
                      << ", expected=" << points.size() - (batch + 1) * 100 << std::endl;
            return 3;
        }
        for (const auto& point : snapshot)
            if (point.intensity < (batch + 1) * 100) return 4;
    }
    Tree::PointVector recalled(points.begin(), points.begin() + 100);
    tree->Add_Points(recalled, false);
    tree->Snapshot(snapshot);
    if (snapshot.size() != 2100) return 5;
    std::cout << "PASS snapshot: empty/build/delete/rebuild/recall" << std::endl;
    return 0;
}