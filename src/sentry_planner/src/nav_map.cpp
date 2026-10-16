#include "nav_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>

namespace {

constexpr const char *kMagic = "NAVMAP1";

std::size_t seekValue(const std::string &header, const std::string &key, std::size_t &out) {
    const std::string needle = "\"" + key + "\"";
    std::size_t pos = header.find(needle);
    if(pos == std::string::npos){
        return std::string::npos;
    }
    pos += needle.size();
    while(pos < header.size() && (header[pos] == ' ' || header[pos] == ':')){
        ++pos;
    }
    out = pos;
    return pos;
}

bool headerNumber(const std::string &header, const std::string &key, double &out) {
    std::size_t pos = 0;
    if(seekValue(header, key, pos) == std::string::npos){
        return false;
    }
    char *end = nullptr;
    const double value = std::strtod(header.c_str() + pos, &end);
    if(end == header.c_str() + pos){
        return false;
    }
    out = value;
    return true;
}

bool headerString(const std::string &header, const std::string &key, std::string &out) {
    std::size_t pos = 0;
    if(seekValue(header, key, pos) == std::string::npos || pos >= header.size() || header[pos] != '"'){
        return false;
    }
    const std::size_t end = header.find('"', pos + 1);
    if(end == std::string::npos){
        return false;
    }
    out = header.substr(pos + 1, end - pos - 1);
    return true;
}

// 解析 "arrays":[["cost","u1"],["dist","f4"],["slope","u1"]] 的名字/类型顺序
// （容忍 JSON 里的空白：["cost", "u1"] 与 ["cost","u1"] 都要能读）
bool headerArrays(const std::string &header, std::vector<std::pair<std::string, std::string>> &out) {
    std::size_t pos = 0;
    if(seekValue(header, "arrays", pos) == std::string::npos){
        return false;
    }
    out.clear();
    while((pos = header.find("[\"", pos)) != std::string::npos){
        const std::size_t name_end = header.find('"', pos + 2);
        if(name_end == std::string::npos){
            return false;
        }
        std::size_t cursor = name_end + 1;
        while(cursor < header.size() &&
              (header[cursor] == ' ' || header[cursor] == ',' || header[cursor] == '\n')){
            ++cursor;
        }
        if(cursor >= header.size() || header[cursor] != '"'){
            return false;
        }
        const std::size_t type_end = header.find('"', cursor + 1);
        if(type_end == std::string::npos){
            return false;
        }
        out.emplace_back(header.substr(pos + 2, name_end - pos - 2),
                         header.substr(cursor + 1, type_end - cursor - 1));

        // 该对的 ']' 之后：若是 ',' 说明还有下一对；若是 ']' 说明 arrays 列表结束
        std::size_t look = type_end + 1;
        while(look < header.size() && (header[look] == ' ' || header[look] == '\n')){
            ++look;
        }
        if(look >= header.size() || header[look] != ']'){
            return false;
        }
        ++look;
        while(look < header.size() && (header[look] == ' ' || header[look] == '\n')){
            ++look;
        }
        if(look < header.size() && header[look] == ']'){   // arrays 列表结束
            break;
        }
        pos = look;
    }
    return !out.empty();
}

std::size_t typeSize(const std::string &type) {
    if(type == "u1" || type == "i1"){
        return 1;
    }
    if(type == "u2" || type == "i2"){
        return 2;
    }
    if(type == "f4" || type == "u4" || type == "i4"){
        return 4;
    }
    if(type == "f8"){
        return 8;
    }
    return 0;
}

}  // namespace

bool NavMap::load(const std::string &path) {
    ready_ = false;
    status_ = "not loaded";
    cost_.clear();
    dist_.clear();
    slope_.clear();

    std::ifstream file(path, std::ios::binary);
    if(!file){
        status_ = "cannot open " + path;
        return false;
    }
    std::string blob((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    if(blob.size() < 16){
        status_ = "file too small: " + path;
        return false;
    }

    const std::size_t magic_end = blob.find('\n');
    if(magic_end == std::string::npos || blob.compare(0, magic_end, kMagic) != 0){
        status_ = "bad magic (expect " + std::string(kMagic) + "): " + path;
        return false;
    }
    const std::size_t header_begin = magic_end + 1;
    const std::size_t header_end = blob.find('\n', header_begin);
    if(header_end == std::string::npos){
        status_ = "missing header line: " + path;
        return false;
    }
    const std::string header = blob.substr(header_begin, header_end - header_begin);

    double width = 0.0, height = 0.0, resolution = 0.0;
    if(!headerNumber(header, "width", width) || !headerNumber(header, "height", height) ||
       !headerNumber(header, "resolution", resolution) || width <= 0.0 || height <= 0.0 ||
       resolution <= 0.0){
        status_ = "bad width/height/resolution: " + header;
        return false;
    }

    std::size_t origin_pos = 0;
    if(seekValue(header, "origin", origin_pos) == std::string::npos || header[origin_pos] != '['){
        status_ = "bad origin: " + header;
        return false;
    }
    double origin[2] = {0.0, 0.0};
    {
        const char *cursor = header.c_str() + origin_pos + 1;
        char *end = nullptr;
        for(double &value : origin){
            value = std::strtod(cursor, &end);
            if(end == cursor){
                status_ = "bad origin numbers: " + header;
                return false;
            }
            cursor = end;
            while(*cursor == ' ' || *cursor == ','){
                ++cursor;
            }
        }
    }

    std::vector<std::pair<std::string, std::string>> arrays;
    if(!headerArrays(header, arrays)){
        status_ = "bad arrays list: " + header;
        return false;
    }

    std::size_t cells = 0;
    std::size_t expected_bytes = 0;
    for(const auto &entry : arrays){
        const std::size_t item = typeSize(entry.second);
        if(item == 0){
            status_ = "unsupported array type " + entry.second + " for " + entry.first;
            return false;
        }
        if(entry.first != "cost" && entry.first != "dist" && entry.first != "slope"){
            status_ = "unknown array " + entry.first;
            return false;
        }
        expected_bytes += item;
    }
    cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    if(cells == 0){
        status_ = "empty grid";
        return false;
    }
    const std::size_t payload_begin = header_end + 1;
    const std::size_t payload_size = blob.size() - payload_begin;
    if(payload_size < cells * expected_bytes){
        std::ostringstream oss;
        oss << "payload too small: " << payload_size << " < " << cells * expected_bytes;
        status_ = oss.str();
        return false;
    }

    std::size_t offset = payload_begin;
    bool have_cost = false;
    bool have_dist = false;
    bool have_slope = false;
    for(const auto &entry : arrays){
        const std::size_t item = typeSize(entry.second);
        if(entry.first == "cost" && entry.second == "u1"){
            cost_.assign(blob.begin() + offset, blob.begin() + offset + cells);
            have_cost = true;
        }else if(entry.first == "dist" && entry.second == "f4"){
            dist_.resize(cells);
            std::memcpy(dist_.data(), blob.data() + offset, cells * sizeof(float));
            have_dist = true;
        }else if(entry.first == "slope" && entry.second == "u1"){
            slope_.assign(blob.begin() + offset, blob.begin() + offset + cells);
            have_slope = true;
        }else{
            status_ = "unexpected layout for " + entry.first + " (" + entry.second + ")";
            return false;
        }
        offset += cells * item;
    }
    // 缺数组必须报错退出：静默补零会让「距离场全是 0 → 所有路径都判不可通行」，很难查
    if(!have_cost){
        status_ = "missing cost array";
        return false;
    }
    if(!have_dist){
        status_ = "missing dist array (距离场缺失)";
        return false;
    }
    if(!have_slope){
        status_ = "missing slope array";
        return false;
    }

    width_ = static_cast<int>(width);
    height_ = static_cast<int>(height);
    resolution_ = resolution;
    origin_x_ = origin[0];
    origin_y_ = origin[1];
    headerString(header, "source", source_);
    ready_ = true;
    status_ = "ok";
    return true;
}

bool NavMap::worldToCell(double x, double y, int &col, int &row) const {
    const double cx = std::floor((x - origin_x_) / resolution_);
    const double cy = std::floor((y - origin_y_) / resolution_);
    const double row_from_bottom = cy;
    const double row_from_top = static_cast<double>(height_) - 1.0 - row_from_bottom;
    col = static_cast<int>(cx);
    row = static_cast<int>(row_from_top);
    return inBounds(col, row);
}

void NavMap::cellToWorld(int col, int row, double &x, double &y) const {
    x = origin_x_ + (static_cast<double>(col) + 0.5) * resolution_;
    y = origin_y_ + (static_cast<double>(height_ - 1 - row) + 0.5) * resolution_;
}

float NavMap::clearance(double x, double y) const {
    if(!ready_){
        return 0.0f;
    }
    int col = 0, row = 0;
    if(!worldToCell(x, y, col, row)){
        return 0.0f;      // 地图外一律视为不可通行
    }
    return dist_[static_cast<std::size_t>(row) * width_ + col];
}

bool NavMap::isSlope(double x, double y) const {
    if(!ready_){
        return false;
    }
    int col = 0, row = 0;
    if(!worldToCell(x, y, col, row)){
        return false;
    }
    return slope_[static_cast<std::size_t>(row) * width_ + col] != 0;
}

float NavMap::minClearanceOnSegment(double x0, double y0, double x1, double y1,
                                    double *worst_x, double *worst_y) const {
    if(!ready_){
        return std::numeric_limits<float>::infinity();
    }
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double length = std::hypot(dx, dy);
    const double step = std::max(resolution_ * 0.5, 1e-3);
    const int samples = static_cast<int>(std::ceil(length / step));
    float worst = std::numeric_limits<float>::infinity();
    for(int i = 0; i <= samples; ++i){
        const double ratio = (samples == 0) ? 0.0 : static_cast<double>(i) / samples;
        const double x = x0 + dx * ratio;
        const double y = y0 + dy * ratio;
        const float value = clearance(x, y);
        if(value < worst){
            worst = value;
            if(worst_x != nullptr){
                *worst_x = x;
            }
            if(worst_y != nullptr){
                *worst_y = y;
            }
        }
    }
    return worst;
}

bool NavMap::lineClear(double x0, double y0, double x1, double y1, double margin) const {
    if(!ready_){
        return true;      // 没有地图时不拦（台架/仿真）
    }
    return minClearanceOnSegment(x0, y0, x1, y1) >= margin;
}
