#include "PointCloudProcess.h"

std::vector<TimestampedPoint> PointCloudProcessor::throughOutlier(std::vector<TimestampedPoint>& data,
    char feature) {
    MeanVariance meanVar = PointCloudProcessor::computeMeanVariance(data, feature);
    double mean = meanVar.mean, variance = meanVar.variance;
    std::cout << "mean=" << mean << " var = " << variance << std::endl;
    std::vector<TimestampedPoint> result;
    for (const auto& p : data) {
        float value = 0.0f;
        switch (feature) {
        case 'x': value = p.x; break;
        case 'y': value = p.y; break;
        case 'z': value = p.z; break;
        }
        if (value >= mean - 0.4 * variance && value <= mean + variance) {
            result.push_back(p);
        }
    }
    return result;
}


std::vector<TimestampedPoint> PointCloudProcessor::statisticalOutlier(std::vector<TimestampedPoint>& data,
    int k,
    float std_dev) {
    // 参数校验
    if (data.empty()) throw std::invalid_argument("Empty input data");
    if (k <= 0) throw std::invalid_argument("k must be positive");
    if (data.size() < k + 1UL) throw std::invalid_argument("Insufficient points");

    // 转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->resize(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        cloud->points[i].x = data[i].x;
        cloud->points[i].y = data[i].y;
        cloud->points[i].z = data[i].z;
    }

    // 构建KD树
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 计算每个点的平均距离
    std::vector<double> avg_distances(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i) {
        std::vector<int> indices;
        indices.resize(k + 1);
        std::vector<float> dists_sq;
        dists_sq.resize(k + 1);

        if (kdtree.nearestKSearch(cloud->points[i], k + 1, indices, dists_sq) < k + 1) {
            throw std::runtime_error("Insufficient neighbors found");
        }

        double sum_dist = 0.0;
        for (int j = 1; j <= k; ++j) {
            sum_dist += std::sqrt(dists_sq[j]);
        }
        avg_distances[i] = sum_dist / k;
    }

    // 计算均值和方差（Welford算法）
    double mean = 0.0, m2 = 0.0;
    size_t count = 0;
    for (double d : avg_distances) {
        ++count;
        double delta = d - mean;
        mean += delta / count;
        m2 += delta * (d - mean);
    }
    double variance = m2 / avg_distances.size();
    const double stddev = std::sqrt(variance);
    const double threshold = mean + std_dev * stddev;

    // 过滤离群点
    std::vector<TimestampedPoint> result;
    result.reserve(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        if (avg_distances[i] < threshold) {
            result.push_back(data[i]);
        }
    }

    return result;
}

MeanVariance PointCloudProcessor::computeMeanVariance(const std::vector<TimestampedPoint>& data, char axis) {
    if (data.empty()) {
        throw std::invalid_argument("数据不能为空");
    }

    // 根据轴选择对应的成员指针
    double TimestampedPoint::* member_ptr;
    switch (axis) {
    case 'x': member_ptr = &TimestampedPoint::x; break;
    case 'y': member_ptr = &TimestampedPoint::y; break;
    case 'z': member_ptr = &TimestampedPoint::z; break;
    default: throw std::invalid_argument("无效的轴，必须是 'x', 'y' 或 'z'");
    }

    // 计算总和
    double sum = 0.0;
    for (const auto& point : data) {
        sum += point.*member_ptr;
    }
    const double mean = sum / data.size();

    // 计算方差（总体方差）
    double sum_diff_sq = 0.0;
    for (const auto& point : data) {
        const double diff = point.*member_ptr - mean;
        sum_diff_sq += diff * diff;
    }
    const double variance = sum_diff_sq / data.size();

    return { mean, sqrt(variance) };
}

TimestampedPoint PointCloudProcessor::weldValueByTimestamp(const std::vector<TimestampedPoint>& data, float percentage) {
    // 参数校验
    if (data.empty()) {
        throw std::invalid_argument("Input data cannot be empty");
    }
    if (percentage < 0.0f || percentage > 1.0f) {
        throw std::invalid_argument("Percentage must be between 0 and 1");
    }

    // 提取x,y坐标并计算统计量
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;
    const size_t n = data.size();

    for (const auto& pt : data) {
        sum_x += pt.x;
        sum_y += pt.y;
        sum_xy += pt.x * pt.y;
        sum_x2 += pt.x * pt.x;
    }

    // 计算线性回归参数 (y = m*x + b)
    const double denominator = n * sum_x2 - sum_x * sum_x;
    if (std::abs(denominator) < 1e-9) {
        throw std::runtime_error("Vertical line detected, cannot compute regression");
    }

    const double m = (n * sum_xy - sum_x * sum_y) / denominator; // 斜率
    const double b = (sum_y - m * sum_x) / n;                     // 截距

    // 计算各点到直线的距离
    std::vector<std::pair<double, size_t>> distances;
    for (size_t i = 0; i < n; ++i) {
        const double y_pred = m * data[i].x + b;
        const double dist = std::abs(data[i].y - y_pred);
        distances.emplace_back(dist, i);
    }

    // 按距离降序排序
    std::sort(distances.begin(), distances.end(),
        [](const auto& a, const auto& b) { return a.first > b.first; });

    // 计算需要选取的点数
    const size_t num_points = static_cast<size_t>(n * percentage);
    const size_t selected = std::max<size_t>(1, std::min(num_points, n));

    // 计算关键点均值
    double sum_key_x = 0.0, sum_key_y = 0.0, sum_key_z = 0.0;
    for (size_t i = 0; i < selected; ++i) {
        const auto& pt = data[distances[i].second];
        sum_key_x += pt.x;
        sum_key_y += pt.y;
        sum_key_z += pt.z;
    }

    return {
        data[0].timestamp,
        static_cast<float>(sum_key_x / selected),
        static_cast<float>(sum_key_y / selected),
        static_cast<float>(sum_key_z / selected),
        data[0].rx,
        data[0].ry,
        data[0].rz
    };
}

std::vector<std::vector<TimestampedPoint>> PointCloudProcessor::groupByTimestamp(
    const std::vector<TimestampedPoint>& data) {
    // 创建哈希表：key=时间戳，value=相同时间戳的点集合
    std::map<std::string, std::vector<TimestampedPoint>> groups_map;

    // 遍历数据并分组
    for (const auto& point : data) {
        groups_map[point.timestamp].push_back(point);
    }

    // 将哈希表转换为最终结果格式
    std::vector<std::vector<TimestampedPoint>> result;
    result.reserve(groups_map.size());

    // 将分组后的数据保存
    std::for_each(groups_map.begin(), groups_map.end(),
        [&result](const auto& entry) {
            result.push_back(entry.second);
        }
    );
    return result;
}

std::vector<TimestampedPoint> PointCloudProcessor::removeOutliersByDistance(const std::vector<TimestampedPoint>& data, int k, float threshold) {
    if (data.empty()) return {};

    // 1. 构建 PCL 点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& pt : data) {
        cloud->push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    }

    // 2. 构建 KD-Tree
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<TimestampedPoint> filtered_data;

    // 3. 遍历每个点，计算 k 个最近邻的平均距离
    for (size_t i = 0; i < cloud->size(); ++i) {
        std::vector<int> point_indices(k);
        std::vector<float> point_distances(k);

        // 查询 k 近邻（包括自身）
        if (kdtree.nearestKSearch(cloud->at(i), k, point_indices, point_distances) > 0) {
            // 计算平均距离
            float mean_distance = 0.0f;
            for (float dist : point_distances) {
                mean_distance += std::sqrt(dist);
            }
            mean_distance /= k;

            // 4. 根据阈值判断是否保留该点
            if (mean_distance < threshold) {
                filtered_data.push_back(data[i]);
            }
        }
    }

    return filtered_data;
}

std::vector<TimestampedPointXYZ> PointCloudProcessor::groupPointsToVector(const std::vector<TimestampedPoint>& points) {
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> grouped;

    for (const auto& pt : points) {
        if (grouped.find(pt.timestamp) == grouped.end()) {
            grouped[pt.timestamp] = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        }
        grouped[pt.timestamp]->points.emplace_back(pt.x, pt.y, pt.z);
    }

    std::vector<TimestampedPointXYZ> result;
    for (auto& pair : grouped) {
        const std::string& timestamp = pair.first;
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud = pair.second;

        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = true;

        result.push_back({ timestamp, cloud });
    }
    return result;
}

double PointCloudProcessor::perpendicularDistance(const pcl::PointXYZ& point, const pcl::PointXYZ& line_start, const pcl::PointXYZ& line_end) {
    double A = point.x - line_start.x;
    double B = point.y - line_start.y;
    double C = line_end.x - line_start.x;
    double D = line_end.y - line_start.y;

    double dot = A * C + B * D;
    double len_sq = C * C + D * D;
    double param = (len_sq != 0) ? dot / len_sq : -1;

    double xx, yy;
    if (param < 0) {
        xx = line_start.x;
        yy = line_start.y;
    }
    else if (param > 1) {
        xx = line_end.x;
        yy = line_end.y;
    }
    else {
        xx = line_start.x + param * C;
        yy = line_start.y + param * D;
    }

    double dx = point.x - xx;
    double dy = point.y - yy;
    return sqrt(dx * dx + dy * dy);
}

void PointCloudProcessor::RDP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int start_index, int end_index, double epsilon, std::vector<int>& output_indices) {
    if (end_index <= start_index + 1) return;

    double max_dist = 0.0;
    int index = start_index;

    for (int i = start_index + 1; i < end_index; ++i) {
        double dist = PointCloudProcessor::perpendicularDistance(cloud->points[i], cloud->points[start_index], cloud->points[end_index]);
        if (dist > max_dist) {
            index = i;
            max_dist = dist;
        }
    }

    if (max_dist > epsilon) {
        // 递归处理左右两段
        RDP(cloud, start_index, index, epsilon, output_indices);
        output_indices.push_back(index);
        RDP(cloud, index, end_index, epsilon, output_indices);
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudProcessor::simplifyPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double epsilon) {
    std::vector<int> key_indices;
    //key_indices.push_back(0);                   // 起点
    RDP(cloud, 0, cloud->size() - 1, epsilon, key_indices);
    //key_indices.push_back(cloud->size() - 1);   // 终点

    // 去重并排序
    sort(key_indices.begin(), key_indices.end());
    key_indices.erase(unique(key_indices.begin(), key_indices.end()), key_indices.end());

    // 构建简化点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr simplified(new pcl::PointCloud<pcl::PointXYZ>);
    for (int idx : key_indices) {
        simplified->points.push_back(cloud->points[idx]);
    }
    simplified->width = simplified->points.size();
    simplified->height = 1;
    simplified->is_dense = true;

    return simplified;
}