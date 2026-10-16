#pragma once

#include <string>
#include <utility>
#include <vector>

#include "nav_map.hpp"

// 路径图节点（现场用 tools/mark_points.py 打点生成）
struct WaypointNode {
    std::string name;
    Point2D p;
};

// 禁小陀螺区（斜坡/起伏路段）：轴对齐矩形，现场标定
struct SpinForbiddenZone {
    std::string name;
    double x_min = 0.0;
    double x_max = 0.0;
    double y_min = 0.0;
    double y_max = 0.0;

    bool contains(double x, double y) const {
        return x >= x_min && x <= x_max && y >= y_min && y <= y_max;
    }
};

// config/waypoint_graph.yaml：节点 + 无向边 + 禁转区。
// 规划时逐边用 NavMap::lineClear 校验，不可通行的边直接跳过（启动时打印统计）。
class WaypointGraph {
public:
    bool load(const std::string &path);

    bool ready() const { return !nodes_.empty(); }
    const char *status() const { return status_.c_str(); }
    const std::string &path() const { return path_; }

    const std::vector<WaypointNode> &nodes() const { return nodes_; }
    const std::vector<std::pair<int, int>> &edges() const { return edges_; }
    const std::vector<SpinForbiddenZone> &spinZones() const { return spin_zones_; }

    int findByName(const std::string &name) const;
    int nearest(const Point2D &p) const;
    // 离目标最近、且「该节点 -> 目标」直线可通行的节点；找不到返回 -1
    int nearestReachable(const Point2D &goal, const NavMap &map, double margin) const;
    // Dijkstra（边上做可通行校验）；成功时 out = 节点下标序列（含首尾）
    bool plan(int start, int goal, const NavMap &map, double margin, std::vector<int> &out) const;
    // 统计当前地图下不可通行的边数（启动自检）
    int blockedEdges(const NavMap &map, double margin, std::string *detail = nullptr) const;
    const SpinForbiddenZone *spinZoneAt(double x, double y) const;

private:
    bool edgeClear(const NavMap &map, int a, int b, double margin) const;

    std::string status_ = "not loaded";
    std::string path_;
    std::vector<WaypointNode> nodes_;
    std::vector<std::pair<int, int>> edges_;
    std::vector<SpinForbiddenZone> spin_zones_;
};
