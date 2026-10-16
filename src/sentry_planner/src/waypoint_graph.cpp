#include "waypoint_graph.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>

#include <yaml-cpp/yaml.h>

int WaypointGraph::findByName(const std::string &name) const {
    for(std::size_t i = 0; i < nodes_.size(); ++i){
        if(nodes_[i].name == name){
            return static_cast<int>(i);
        }
    }
    return -1;
}

int WaypointGraph::nearest(const Point2D &p) const {
    int best = -1;
    double best_dist = std::numeric_limits<double>::infinity();
    for(std::size_t i = 0; i < nodes_.size(); ++i){
        const double d = std::hypot(nodes_[i].p.x - p.x, nodes_[i].p.y - p.y);
        if(d < best_dist){
            best_dist = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

int WaypointGraph::nearestReachable(const Point2D &goal, const NavMap &map, double margin) const {
    int best = -1;
    double best_dist = std::numeric_limits<double>::infinity();
    for(std::size_t i = 0; i < nodes_.size(); ++i){
        const double d = std::hypot(nodes_[i].p.x - goal.x, nodes_[i].p.y - goal.y);
        if(d >= best_dist){
            continue;
        }
        if(!map.lineClear(nodes_[i].p.x, nodes_[i].p.y, goal.x, goal.y, margin)){
            continue;
        }
        best_dist = d;
        best = static_cast<int>(i);
    }
    return best;
}

const SpinForbiddenZone *WaypointGraph::spinZoneAt(double x, double y) const {
    for(const auto &zone : spin_zones_){
        if(zone.contains(x, y)){
            return &zone;
        }
    }
    return nullptr;
}

bool WaypointGraph::edgeClear(const NavMap &map, int a, int b, double margin) const {
    if(a < 0 || b < 0 || a >= static_cast<int>(nodes_.size()) || b >= static_cast<int>(nodes_.size())){
        return false;
    }
    return map.lineClear(nodes_[a].p.x, nodes_[a].p.y, nodes_[b].p.x, nodes_[b].p.y, margin);
}

int WaypointGraph::blockedEdges(const NavMap &map, double margin, std::string *detail) const {
    int blocked = 0;
    for(const auto &edge : edges_){
        if(edgeClear(map, edge.first, edge.second, margin)){
            continue;
        }
        ++blocked;
        if(detail != nullptr){
            double worst_x = 0.0;
            double worst_y = 0.0;
            const float worst = map.minClearanceOnSegment(
                nodes_[edge.first].p.x, nodes_[edge.first].p.y,
                nodes_[edge.second].p.x, nodes_[edge.second].p.y, &worst_x, &worst_y);
            std::ostringstream oss;
            oss << nodes_[edge.first].name << "->" << nodes_[edge.second].name
                << "(最差 " << worst << "m @ (" << worst_x << ", " << worst_y << ")) ";
            *detail += oss.str();
        }
    }
    return blocked;
}

bool WaypointGraph::plan(int start, int goal, const NavMap &map, double margin,
                         std::vector<int> &out) const {
    out.clear();
    const int count = static_cast<int>(nodes_.size());
    if(start < 0 || goal < 0 || start >= count || goal >= count){
        return false;
    }
    if(start == goal){
        out.push_back(start);
        return true;
    }

    std::vector<std::vector<std::pair<int, double>>> adjacency(count);
    for(const auto &edge : edges_){
        const int a = edge.first;
        const int b = edge.second;
        if(!edgeClear(map, a, b, margin)){
            continue;
        }
        const double weight = std::hypot(nodes_[a].p.x - nodes_[b].p.x, nodes_[a].p.y - nodes_[b].p.y);
        adjacency[a].emplace_back(b, weight);
        adjacency[b].emplace_back(a, weight);
    }

    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> best(count, infinity);
    std::vector<int> parent(count, -1);
    using Entry = std::pair<double, int>;   // (距离, 节点)
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    best[start] = 0.0;
    queue.emplace(0.0, start);
    while(!queue.empty()){
        const auto [distance, node] = queue.top();
        queue.pop();
        if(distance > best[node]){
            continue;
        }
        if(node == goal){
            break;
        }
        for(const auto &[next, weight] : adjacency[node]){
            const double candidate = distance + weight;
            if(candidate < best[next]){
                best[next] = candidate;
                parent[next] = node;
                queue.emplace(candidate, next);
            }
        }
    }
    if(parent[goal] < 0 && start != goal){
        return false;
    }
    for(int node = goal; node >= 0; node = parent[node]){
        out.push_back(node);
        if(node == start){
            break;
        }
    }
    if(out.empty() || out.back() != start){
        out.clear();
        return false;
    }
    std::reverse(out.begin(), out.end());
    return true;
}

bool WaypointGraph::load(const std::string &path) {
    nodes_.clear();
    edges_.clear();
    spin_zones_.clear();
    status_ = "not loaded";
    path_ = path;

    YAML::Node root;
    try{
        root = YAML::LoadFile(path);
    }catch(const std::exception &ex){
        status_ = std::string("cannot parse ") + path + ": " + ex.what();
        return false;
    }

    try{
        for(const auto &entry : root["nodes"]){
            WaypointNode node;
            node.name = entry["name"].as<std::string>();
            node.p.x = entry["x"].as<double>();
            node.p.y = entry["y"].as<double>();
            if(node.name.empty()){
                status_ = "node without name in " + path;
                return false;
            }
            if(findByName(node.name) >= 0){
                status_ = "duplicate node name " + node.name;
                return false;
            }
            nodes_.push_back(node);
        }
        for(const auto &entry : root["edges"]){
            if(!entry.IsSequence() || entry.size() != 2){
                status_ = "edge must be [name, name]";
                return false;
            }
            const std::string a = entry[0].as<std::string>();
            const std::string b = entry[1].as<std::string>();
            const int index_a = findByName(a);
            const int index_b = findByName(b);
            if(index_a < 0 || index_b < 0){
                status_ = "edge references unknown node: " + a + " / " + b;
                return false;
            }
            if(index_a == index_b){
                status_ = "self edge on " + a;
                return false;
            }
            edges_.emplace_back(index_a, index_b);
        }
        for(const auto &entry : root["spin_forbidden_zones"]){
            SpinForbiddenZone zone;
            zone.name = entry["name"] ? entry["name"].as<std::string>() : std::string("zone");
            zone.x_min = entry["x_min"].as<double>();
            zone.x_max = entry["x_max"].as<double>();
            zone.y_min = entry["y_min"].as<double>();
            zone.y_max = entry["y_max"].as<double>();
            if(zone.x_min > zone.x_max || zone.y_min > zone.y_max){
                status_ = "bad extent for zone " + zone.name;
                return false;
            }
            spin_zones_.push_back(zone);
        }
    }catch(const std::exception &ex){
        status_ = std::string("bad field in ") + path + ": " + ex.what();
        nodes_.clear();
        edges_.clear();
        spin_zones_.clear();
        return false;
    }

    status_ = nodes_.empty() ? "empty graph (no nodes)" : "ok";
    return !nodes_.empty();
}
