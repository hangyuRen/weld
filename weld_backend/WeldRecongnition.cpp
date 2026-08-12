#include "WeldRecongnition.h"

std::vector<TimestampedPoint> WeldRecongnition::timeCloud = []() {
    std::vector<TimestampedPoint> vec;
    vec.reserve(3000); // 分配初始容量
    return vec;
    }();

// 获取焊缝点 重载
void WeldRecongnition::getWeldSeam(std::vector<TimestampedPoint>& cloud, std::vector<weldStruct>& weld) {

    // 统计学滤波
    std::vector<TimestampedPoint> staData = PointCloudProcessor::statisticalOutlier(cloud);

    // 根据timestamp进行分组
    std::vector<std::vector<TimestampedPoint>> groupedData = PointCloudProcessor::groupByTimestamp(staData);

    // 计算焊缝点
    std::vector<TimestampedPoint> res;
    for (size_t i = 0; i < groupedData.size(); ++i) {
        TimestampedPoint point = PointCloudProcessor::weldValueByTimestamp(groupedData[i]);
        res.push_back(point);
    }

    std::cout << "焊缝点初步计算完成，数量: " << res.size() << std::endl;

    // 去除离群焊缝点
    std::vector<TimestampedPoint> filteredWeld = PointCloudProcessor::removeOutliersByDistance(res);

    std::cout << "焊缝点过滤完成，数量: " << filteredWeld.size() << std::endl;

    // 转换 `filteredWeld` 为 PCL 点云格式（焊缝点）
    for (const auto& pt : res) {
        weldStruct temp = { pt.timestamp, pcl::PointXYZ(pt.x, pt.y, pt.z), pt.rx, pt.ry, pt.rz };
        std::cout << "焊缝点: timestamp=" << temp.timestamp << " x=" << temp.point.x << " y=" << temp.point.y << " z=" << temp.point.z
            << " rx=" << temp.rx << " ry=" << temp.ry << " rz=" << temp.rz << std::endl;
        weld.push_back(temp);
    }
}

Eigen::Vector3d polyfit2(const Eigen::VectorXd& u, const Eigen::VectorXd& v) {
    Eigen::MatrixXd A(u.size(), 3);
    for (int i = 0; i < u.size(); ++i) {
        A(i, 0) = u(i) * u(i);
        A(i, 1) = u(i);
        A(i, 2) = 1.0;
    }
    return A.householderQr().solve(v);
}


Eigen::Vector3d polyfitSVD(const Eigen::VectorXd& u, const Eigen::VectorXd& v) {
    Eigen::MatrixXd A(u.size(), 3);
    for (int i = 0; i < u.size(); ++i) {
        A(i, 0) = u(i) * u(i);
        A(i, 1) = u(i);
        A(i, 2) = 1.0;
    }
    return A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(v);
}


std::vector<weldStruct> WeldRecongnition::solveWeldByPCA(const std::vector<TimestampedPoint>& points, float target_z, float& width)
{
    //  if (points.empty()) return {};

    //  // 1. 分组
    //  std::map<std::string, std::vector<const TimestampedPoint*>> groups;
    //  for (const auto& p : points) {
    //      groups[p.timestamp].push_back(&p);
    //  }

    //  std::vector<weldStruct> result;
    //  for (auto const& [ts, group] : groups) {
    //      if (group.size() < 30) continue;

    //      // 2. 构造数据矩阵并进行 PCA
    //      Eigen::MatrixXd mat(group.size(), 3);
    //      for (size_t i = 0; i < group.size(); ++i) {
    //          mat(i, 0) = group[i]->x; mat(i, 1) = group[i]->y; mat(i, 2) = group[i]->z;
    //      }

    //      Eigen::Vector3d mean = mat.colwise().mean();
    //      Eigen::MatrixXd centered = mat.rowwise() - mean.transpose();
    //      Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
    //      Eigen::MatrixXd pts_2d = centered * svd.matrixV().leftCols(2);

    //      // 3. 二次拟合 v = au^2 + bu + c
    //      Eigen::VectorXd u = pts_2d.col(0);
    //      Eigen::VectorXd v = pts_2d.col(1);
    //      Eigen::MatrixXd A(u.size(), 3);
    //      for (int i = 0; i < u.size(); ++i) {
    //          A(i, 0) = u(i) * u(i); A(i, 1) = u(i); A(i, 2) = 1.0;
    //      }
    //      Eigen::Vector3d coeffs = A.householderQr().solve(v);
    //      Eigen::VectorXd residuals = v - (A * coeffs);

    //      // 4. 提取偏差最大的 10% 并加权重心
    //      std::vector<size_t> indices(group.size());
    //      std::iota(indices.begin(), indices.end(), 0);
    //      std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
    //          return std::abs(residuals(a)) > std::abs(residuals(b));
    //          });

    //      int top_n = std::max(1, (int)group.size() / 10);
    //      float sw = 0, sx = 0, sy = 0, sz = 0;
    //      for (int i = 0; i < top_n; ++i) {
    //          size_t idx = indices[i];
    //          float w = residuals(idx) * residuals(idx) + 1e-6;
    //          sw += w; sx += group[idx]->x * w; sy += group[idx]->y * w; sz += group[idx]->z * w;
    //      }

    //      weldStruct ws;
    //      ws.point = { sx / sw, sy / sw, sz / sw + target_z };
          //ws.rx = group[0]->rx;
    //      ws.ry = group[0]->ry;
    //      ws.rz = group[0]->rz;
    //      ws.timestamp = ts;
    //      result.push_back(ws);
    //  }
    //  return result;
    if (points.empty()) return {};

    // 1. 分组 (ts_groups)
    std::map<std::string, std::vector<const TimestampedPoint*>> groups;
    for (const auto& p : points) {
        groups[p.timestamp].push_back(&p);
    }

    std::vector<weldStruct> weld_path;

    for (auto const& [ts, group] : groups) {
        if (group.size() < 60) continue;

        // 构造矩阵
        Eigen::MatrixXd pts_np(group.size(), 3);
        for (size_t i = 0; i < group.size(); ++i) {
            pts_np(i, 0) = group[i]->x;
            pts_np(i, 1) = group[i]->y;
            pts_np(i, 2) = group[i]->z;
        }

        // 2. PCA 处理
        Eigen::Vector3d mean = pts_np.colwise().mean();
        Eigen::MatrixXd centered = pts_np.rowwise() - mean.transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);

        Eigen::MatrixXd V = svd.matrixV();
        // --- 关键：强制方向一致性，确保 u 轴指向 X 正向，v 轴指向 Z 正向 ---
        if (V(0, 0) < 0) V.col(0) *= -1.0;
        if (V(2, 1) < 0) V.col(1) *= -1.0;

        Eigen::MatrixXd pts_2d = centered * V.leftCols(2);
        Eigen::VectorXd u = pts_2d.col(0);
        Eigen::VectorXd v = pts_2d.col(1);

        try {
            // --- A. 寻找中心点 ---
            Eigen::Vector3d coeffs_base = polyfitSVD(u, v);
            Eigen::VectorXd v_fit = Eigen::VectorXd::Zero(u.size());
            for (int i = 0; i < u.size(); ++i) {
                v_fit(i) = coeffs_base(0) * u(i) * u(i) + coeffs_base(1) * u(i) + coeffs_base(2);
            }
            Eigen::VectorXd residuals = v - v_fit;

            int core_idx = 0;
            residuals.array().abs().maxCoeff(&core_idx);

            // 2. 定义内部搜索 lambda (替换 get_local_curvature)
            // 逻辑：在指定范围内寻找曲率 a 最小的点 (最凸的拐点)
            auto get_curv_idx = [&](int start, int end) {
                int win = 7;
                double min_a = 1e9;
                int best_idx = (start + end) / 2; // 默认值防止范围太窄

                // 边界保护：确保窗口不会越界
                int actual_end = std::min(end, (int)u.size());
                for (int i = start; i <= actual_end - win; ++i) {
                    Eigen::VectorXd sub_u = u.segment(i, win);
                    Eigen::VectorXd sub_v = v.segment(i, win);

                    // 使用 BDCSVD 求解局部拟合
                    Eigen::Vector3d sc = polyfitSVD(sub_u, sub_v);

                    // 寻找二次项系数 a 的最小值 (向上凸起最剧烈的地方)
                    if (sc(0) < min_a) {
                        min_a = sc(0);
                        best_idx = i + win / 2;
                    }
                }
                return best_idx;
                };

            // 3. 搜索左右拐点
            int search_range = 50;
            // 左翼：[l_start, core_idx]
            int idxL = get_curv_idx(std::max(0, core_idx - search_range), core_idx);
            const TimestampedPoint* p_top_l = group[idxL];

            // 右翼：[core_idx, r_end]
            int idxR = get_curv_idx(core_idx, std::min((int)u.size(), core_idx + search_range));
            const TimestampedPoint* p_top_r = group[idxR];

            // 4. 计算 3D 物理宽度
            float weld_width = std::sqrt(std::pow(p_top_l->x - p_top_r->x, 2) +
                std::pow(p_top_l->y - p_top_r->y, 2) +
                std::pow(p_top_l->z - p_top_r->z, 2));

            // --- C. 计算加权中心 ---
            std::vector<size_t> indices(group.size());
            std::iota(indices.begin(), indices.end(), 0);
            std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
                return std::abs(residuals(a)) > std::abs(residuals(b));
                });

            int top_n = std::max(1, (int)group.size() / 10);
            double sw = 0, sx = 0, sy = 0, sz = 0;
            for (int i = 0; i < top_n; ++i) {
                size_t idx = indices[i];
                double w = residuals(idx) * residuals(idx) + 1e-9;
                sw += w;
                sx += group[idx]->x * w;
                sy += group[idx]->y * w;
                sz += group[idx]->z * w;
            }

            weldStruct ws;
            ws.point = { (float)(sx / sw), (float)(sy / sw) + 2, (float)(sz / sw + target_z) };
            ws.timestamp = ts;
            ws.width = weld_width;

            // 姿态均值
            float sum_rx = 0, sum_ry = 0, sum_rz = 0;
            for (int i = 0; i < top_n; ++i) {
                sum_rx += group[indices[i]]->rx; sum_ry += group[indices[i]]->ry; sum_rz += group[indices[i]]->rz;
            }
            ws.rx = sum_rx / top_n; ws.ry = sum_ry / top_n; ws.rz = sum_rz / top_n;

            weld_path.push_back(ws);

        }
        catch (...) { continue; }
    }

    // --- 鲁棒统计剔除异常值 (2 sigma) ---
    if (weld_path.empty()) return {};

    double sum_w = 0, sq_sum_w = 0;
    for (const auto& w : weld_path) { sum_w += w.width; sq_sum_w += w.width * w.width; }
    double mean_raw = sum_w / weld_path.size();
    double std_w = std::sqrt(std::abs(sq_sum_w / weld_path.size() - mean_raw * mean_raw));

    std::vector<weldStruct> clean_path;
    double sum_clean = 0;
    for (const auto& w : weld_path) {
        if (w.width > (mean_raw - 2 * std_w) && w.width < (mean_raw + 2 * std_w)) {
            clean_path.push_back(w);
            sum_clean += w.width;
        }
    }

    // 模仿 Python 打印格式
    printf("\nWeld Width Analysis:\n");
    printf("  Count: %d -> %d\n", (int)weld_path.size(), (int)clean_path.size());
    printf("  Raw Mean: %.3f mm | CLEAN AVERAGE: %.3f mm (Filtered %d outliers)\n\n",
        mean_raw, clean_path.empty() ? 0.0 : sum_clean / clean_path.size(),
        (int)(weld_path.size() - clean_path.size()));
    width = sum_clean / clean_path.size();
    return clean_path;
}

void WeldRecongnition::loadAndProcess(std::vector<TimestampedPoint>& rawPoints, double targetZ, std::vector<TimestampedPoint>& outCleanRaw, std::vector<weldStruct>& outWeld, const std::string& flag, float& width)
{
    // 1. 加载原始数据 (对应 points = [] ... points.append(p))
    /*std::vector<TimestampedPoint> rawPoints;
    loadTimestampedDataFromTxt(filePath, rawPoints);*/
    if (rawPoints.empty()) return;

    // 2. 预处理：全局去噪 (对应 clean = preprocess_clean(points))
    // 转换为 PCL 点云格式进行处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : rawPoints) {
        cloud->push_back(pcl::PointXYZ(static_cast<float>(p.x),
            static_cast<float>(p.y),
            static_cast<float>(p.z)));
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);

    if (flag._Equal("A")) {
        sor.setMeanK(10);            // A 组更密集，邻居数可以适当减少
        sor.setStddevMulThresh(1.5); // A 组更密集，标准差倍数可以适当降低
    }
    else if (flag._Equal("B")) {
        sor.setMeanK(5);            // B 组较稀疏，邻居数可以适当增加
        sor.setStddevMulThresh(10); // B 组较稀疏，标准差倍数可以适当增加
    }
    else {
        // 默认参数
        sor.setMeanK(10);
        sor.setStddevMulThresh(1.5);
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    sor.filter(inliers->indices);

    // 提取过滤后的“干净”原始点云
    outCleanRaw.clear();
    for (int idx : inliers->indices) {
        outCleanRaw.push_back(rawPoints[idx]);
    }

    // 3. 求解焊接点 (对应 weld = solve_weld_centers(clean, target_z))
    // 注意：这里传入的是已经去噪过的 outCleanRaw
    outWeld = solveWeldByPCA(outCleanRaw, targetZ, width);
}

bool WeldRecongnition::loadTimestampedDataFromTxt(const std::string& filename, std::vector<TimestampedPoint>& points) {
    std::ifstream infile(filename);
    std::string line;

    while (std::getline(infile, line)) {
        // 找到逗号的位置
        size_t commaPos = line.find(',');
        if (commaPos == std::string::npos) continue;

        // 获取timestamp
        std::string timestamp = line.substr(0, commaPos);

        // 获取剩下的 {...}
        std::string data = line.substr(commaPos + 1);
        // 去掉大括号
        data.erase(std::remove(data.begin(), data.end(), '{'), data.end());
        data.erase(std::remove(data.begin(), data.end(), '}'), data.end());

        std::istringstream ss(data);
        std::string token;
        std::vector<double> values;

        while (std::getline(ss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            }
            catch (...) {
                // 出现非法浮点数
                return false;
            }
        }

        if (values.size() >= 6) {
            TimestampedPoint point;
            point.timestamp = timestamp;
            point.x = values[0];
            point.y = values[1];
            point.z = values[2];
            point.rx = values[3];
            point.ry = values[4];
            point.rz = values[5];
            points.push_back(point);
        }
    }

    return true;
}


std::vector<weldStruct> WeldRecongnition::filterOverlappingPoints(const std::vector<weldStruct>& base_pts,
    const std::vector<weldStruct>& query_pts,
    double threshold) {
    if (base_pts.empty()) return query_pts;

    std::vector<weldStruct> filtered_query;
    double sq_threshold = threshold * threshold; // 使用平方比较，效率更高

    for (const auto& q : query_pts) {
        bool is_overlap = false;

        // 遍历 base 中的所有点进行距离校验
        for (const auto& b : base_pts) {
            double dx = q.point.x - b.point.x;
            double dy = q.point.y - b.point.y;
            double dz = q.point.z - b.point.z;
            double dist_sq = dx * dx + dy * dy + dz * dz;

            if (dist_sq < sq_threshold) {
                is_overlap = true;
                break; // 只要发现一个足够近的点，就判定为重叠
            }
        }

        if (!is_overlap) {
            filtered_query.push_back(q);
        }
    }
    return filtered_query;
}

std::vector<weldStruct> WeldRecongnition::fitAndResamplePath(const std::vector<weldStruct>& points, int num_samples) {
    if (points.empty()) return {};
    if (points.size() < 2) return points;

    // 1. 提取原始特征点并计算累积路径长度 s
    std::vector<double> s(points.size(), 0.0);
    for (size_t i = 1; i < points.size(); ++i) {
        double dx = points[i].point.x - points[i - 1].point.x;
        double dy = points[i].point.y - points[i - 1].point.y;
        double dz = points[i].point.z - points[i - 1].point.z;
        s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    double total_length = s.back();
    if (total_length < 1e-6) {
        return std::vector<weldStruct>(num_samples, points[0]);
    }

    std::vector<weldStruct> resampled;
    resampled.reserve(num_samples);

    // 2. 均匀取样 (包含 0 和 1)
    for (int i = 0; i < num_samples; ++i) {
        // 严格保留首尾点逻辑
        if (i == 0) {
            resampled.push_back(points.front());
            continue;
        }
        if (i == num_samples - 1) {
            resampled.push_back(points.back());
            continue;
        }

        // 计算目标长度位置
        double target_s = (double)i / (num_samples - 1) * total_length;

        // 查找目标 s 所在的区间 (类似 interp1d)
        auto it = std::lower_bound(s.begin(), s.end(), target_s);
        int idx = std::distance(s.begin(), it);

        if (idx == 0) idx = 1;
        int prev = idx - 1;

        // 计算线性插值比例 t
        double t = (target_s - s[prev]) / (s[idx] - s[prev]);

        weldStruct ws;
        ws.timestamp = "resampled";
        // 坐标插值
        ws.point.x = points[prev].point.x + t * (points[idx].point.x - points[prev].point.x);
        ws.point.y = points[prev].point.y + t * (points[idx].point.y - points[prev].point.y);
        ws.point.z = points[prev].point.z + t * (points[idx].point.z - points[prev].point.z);
        // 姿态插值
        ws.rx = points[prev].rx + t * (points[idx].rx - points[prev].rx);
        ws.ry = points[prev].ry + t * (points[idx].ry - points[prev].ry);
        ws.rz = points[prev].rz + t * (points[idx].rz - points[prev].rz);

        resampled.push_back(ws);
    }

    return resampled;
}

Eigen::Vector3d WeldRecongnition::fitCircle3D(const std::vector<weldStruct>& points)
{
    int n = points.size();
    if (n < 3) return Eigen::Vector3d(0, 0, 0);

    // --- 1. 转 Eigen ---
    Eigen::MatrixXd pts(n, 3);
    for (int i = 0; i < n; ++i) {
        pts(i, 0) = points[i].point.x;
        pts(i, 1) = points[i].point.y;
        pts(i, 2) = points[i].point.z;
    }

    // --- 2. 计算均值 ---
    Eigen::Vector3d mean = pts.colwise().mean();

    // 去中心化
    Eigen::MatrixXd centered = pts.rowwise() - mean.transpose();

    // --- 3. PCA (SVD) ---
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
    Eigen::Matrix3d V = svd.matrixV();

    // 取前两个主方向（平面）
    Eigen::Vector3d v1 = V.col(0);
    Eigen::Vector3d v2 = V.col(1);

    // --- 4. 投影到2D ---
    Eigen::VectorXd x(n), y(n);
    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d p = pts.row(i);
        x(i) = p.dot(v1);
        y(i) = p.dot(v2);
    }

    // --- 5. 拟合2D圆 ---
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd b(n);

    for (int i = 0; i < n; ++i) {
        A(i, 0) = 2 * x(i);
        A(i, 1) = 2 * y(i);
        A(i, 2) = 1.0;
        b(i) = x(i) * x(i) + y(i) * y(i);
    }

    Eigen::Vector3d c = A.colPivHouseholderQr().solve(b);

    double cx = c(0);
    double cy = c(1);

    // --- 6. 映射回3D ---
    Eigen::Vector3d center3D = mean + cx * v1 + cy * v2;

    return center3D;
}

std::vector<weldStruct> WeldRecongnition::offsetAlongBendRadius(
    const std::vector<weldStruct>& weld_pts,
    const Eigen::Vector3d& center,
    double offset = 3.0)
{
    std::vector<weldStruct> result;
    result.reserve(weld_pts.size());

    for (const auto& p : weld_pts) {
        Eigen::Vector3d P(p.point.x, p.point.y, p.point.z);

        Eigen::Vector3d dir = P - center;
        double norm = dir.norm();

        if (norm < 1e-6) {
            result.push_back(p);
            continue;
        }

        dir.normalize();

        weldStruct new_p = p;
        new_p.point.x += dir.x() * offset;
        new_p.point.y += dir.y() * offset;
        new_p.point.z += dir.z() * offset;
        new_p.rx = p.rx;
        new_p.ry = p.ry;
        new_p.rz = p.rz;

        result.push_back(new_p);
    }
    return result;
}