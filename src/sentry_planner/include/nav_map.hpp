#pragma once

#include <cstdint>
#include <string>
#include <vector>

// 世界系/车体系通用的二维点（单位 m）
struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

// 离线占据栅格（由 tools/build_nav_map.py 从 map.pcd 生成，格式见 tools/nav_map.py）。
//
// 坐标约定与 ROS map_server 一致：
//   origin 是「左下角格子的左下角」的世界坐标（不是格心）；
//   第 0 行是【最上面】一行（y 最大）；
//   x = origin_x + (col + 0.5) * res
//   y = origin_y + (height - 1 - row + 0.5) * res
//
// cost: 0=空闲 100=占据（原始，未膨胀）；dist: 到最近占据格中心的距离 m；
// slope: 1=斜坡/台阶（禁小陀螺的几何参考，实际操作以 waypoint_graph 里手标的区域为准）。
class NavMap {
public:
    bool load(const std::string &path);

    bool ready() const { return ready_; }
    const char *status() const { return status_.c_str(); }
    const std::string &source() const { return source_; }

    int width() const { return width_; }
    int height() const { return height_; }
    double resolution() const { return resolution_; }

    bool worldToCell(double x, double y, int &col, int &row) const;
    void cellToWorld(int col, int row, double &x, double &y) const;
    bool inBounds(int col, int row) const {
        return col >= 0 && row >= 0 && col < width_ && row < height_;
    }

    // 到最近障碍格中心的距离（m）；地图外或未加载返回 0
    float clearance(double x, double y) const;
    bool isFree(double x, double y, double margin) const { return clearance(x, y) >= margin; }
    bool isSlope(double x, double y) const;

    // 线段上是否处处满足 clearance >= margin（按半格步长采样，含两端点）
    bool lineClear(double x0, double y0, double x1, double y1, double margin) const;
    // 线段上最小 clearance 及其位置（用于诊断「这条边为什么不可通行」）
    float minClearanceOnSegment(double x0, double y0, double x1, double y1,
                                double *worst_x = nullptr, double *worst_y = nullptr) const;

private:
    bool ready_ = false;
    std::string status_ = "not loaded";
    std::string source_;

    int width_ = 0;
    int height_ = 0;
    double resolution_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;

    std::vector<uint8_t> cost_;
    std::vector<float> dist_;
    std::vector<uint8_t> slope_;
};
