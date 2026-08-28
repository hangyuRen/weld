#include "WeldRecongnition.h"

std::vector<TimestampedPoint> WeldRecongnition::timeCloud = []() {
    std::vector<TimestampedPoint> vec;
    vec.reserve(3000); // �����ʼ����
    return vec;
    }();

// ��ȡ����� ����
void WeldRecongnition::getWeldSeam(std::vector<TimestampedPoint>& cloud, std::vector<weldStruct>& weld) {

    // ͳ��ѧ�˲�
    std::vector<TimestampedPoint> staData = PointCloudProcessor::statisticalOutlier(cloud);

    // ����timestamp���з���
    std::vector<std::vector<TimestampedPoint>> groupedData = PointCloudProcessor::groupByTimestamp(staData);

    // ���㺸���
    std::vector<TimestampedPoint> res;
    for (size_t i = 0; i < groupedData.size(); ++i) {
        TimestampedPoint point = PointCloudProcessor::weldValueByTimestamp(groupedData[i]);
        res.push_back(point);
    }

    std::cout << "��������������ɣ�����: " << res.size() << std::endl;

    // ȥ����Ⱥ�����
    std::vector<TimestampedPoint> filteredWeld = PointCloudProcessor::removeOutliersByDistance(res);

    std::cout << "����������ɣ�����: " << filteredWeld.size() << std::endl;

    // ת�� `filteredWeld` Ϊ PCL ���Ƹ�ʽ������㣩
    for (const auto& pt : res) {
        weldStruct temp = { pt.timestamp, pcl::PointXYZ(pt.x, pt.y, pt.z), pt.rx, pt.ry, pt.rz };
        std::cout << "�����: timestamp=" << temp.timestamp << " x=" << temp.point.x << " y=" << temp.point.y << " z=" << temp.point.z
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

    //  // 1. ����
    //  std::map<std::string, std::vector<const TimestampedPoint*>> groups;
    //  for (const auto& p : points) {
    //      groups[p.timestamp].push_back(&p);
    //  }

    //  std::vector<weldStruct> result;
    //  for (auto const& [ts, group] : groups) {
    //      if (group.size() < 30) continue;

    //      // 2. �������ݾ��󲢽��� PCA
    //      Eigen::MatrixXd mat(group.size(), 3);
    //      for (size_t i = 0; i < group.size(); ++i) {
    //          mat(i, 0) = group[i]->x; mat(i, 1) = group[i]->y; mat(i, 2) = group[i]->z;
    //      }

    //      Eigen::Vector3d mean = mat.colwise().mean();
    //      Eigen::MatrixXd centered = mat.rowwise() - mean.transpose();
    //      Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
    //      Eigen::MatrixXd pts_2d = centered * svd.matrixV().leftCols(2);

    //      // 3. ������� v = au^2 + bu + c
    //      Eigen::VectorXd u = pts_2d.col(0);
    //      Eigen::VectorXd v = pts_2d.col(1);
    //      Eigen::MatrixXd A(u.size(), 3);
    //      for (int i = 0; i < u.size(); ++i) {
    //          A(i, 0) = u(i) * u(i); A(i, 1) = u(i); A(i, 2) = 1.0;
    //      }
    //      Eigen::Vector3d coeffs = A.householderQr().solve(v);
    //      Eigen::VectorXd residuals = v - (A * coeffs);

    //      // 4. ��ȡƫ������ 10% ����Ȩ����
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

    // 1. ���� (ts_groups)
    std::map<std::string, std::vector<const TimestampedPoint*>> groups;
    for (const auto& p : points) {
        groups[p.timestamp].push_back(&p);
    }

    std::vector<weldStruct> weld_path;

    for (auto const& [ts, group] : groups) {
        if (group.size() < 60) continue;

        // �������
        Eigen::MatrixXd pts_np(group.size(), 3);
        for (size_t i = 0; i < group.size(); ++i) {
            pts_np(i, 0) = group[i]->x;
            pts_np(i, 1) = group[i]->y;
            pts_np(i, 2) = group[i]->z;
        }

        // 2. PCA ����
        Eigen::Vector3d mean = pts_np.colwise().mean();
        Eigen::MatrixXd centered = pts_np.rowwise() - mean.transpose();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);

        Eigen::MatrixXd V = svd.matrixV();
        // --- �ؼ���ǿ�Ʒ���һ���ԣ�ȷ�� u ��ָ�� X ����v ��ָ�� Z ���� ---
        if (V(0, 0) < 0) V.col(0) *= -1.0;
        if (V(2, 1) < 0) V.col(1) *= -1.0;

        Eigen::MatrixXd pts_2d = centered * V.leftCols(2);
        Eigen::VectorXd u = pts_2d.col(0);
        Eigen::VectorXd v = pts_2d.col(1);

        try {
            // --- A. Ѱ�����ĵ� ---
            Eigen::Vector3d coeffs_base = polyfitSVD(u, v);
            Eigen::VectorXd v_fit = Eigen::VectorXd::Zero(u.size());
            for (int i = 0; i < u.size(); ++i) {
                v_fit(i) = coeffs_base(0) * u(i) * u(i) + coeffs_base(1) * u(i) + coeffs_base(2);
            }
            Eigen::VectorXd residuals = v - v_fit;

            int core_idx = 0;
            residuals.array().abs().maxCoeff(&core_idx);

            // 2. �����ڲ����� lambda (�滻 get_local_curvature)
            // �߼�����ָ����Χ��Ѱ������ a ��С�ĵ� (��͹�Ĺյ�)
            auto get_curv_idx = [&](int start, int end) {
                int win = 7;
                double min_a = 1e9;
                int best_idx = (start + end) / 2; // Ĭ��ֵ��ֹ��Χ̫խ

                // �߽籣����ȷ�����ڲ���Խ��
                int actual_end = std::min(end, (int)u.size());
                for (int i = start; i <= actual_end - win; ++i) {
                    Eigen::VectorXd sub_u = u.segment(i, win);
                    Eigen::VectorXd sub_v = v.segment(i, win);

                    // ʹ�� BDCSVD ���ֲ����
                    Eigen::Vector3d sc = polyfitSVD(sub_u, sub_v);

                    // Ѱ�Ҷ�����ϵ�� a ����Сֵ (����͹������ҵĵط�)
                    if (sc(0) < min_a) {
                        min_a = sc(0);
                        best_idx = i + win / 2;
                    }
                }
                return best_idx;
                };

            // 3. �������ҹյ�
            int search_range = 50;
            // ������[l_start, core_idx]
            int idxL = get_curv_idx(std::max(0, core_idx - search_range), core_idx);
            const TimestampedPoint* p_top_l = group[idxL];

            // ������[core_idx, r_end]
            int idxR = get_curv_idx(core_idx, std::min((int)u.size(), core_idx + search_range));
            const TimestampedPoint* p_top_r = group[idxR];

            // 4. ���� 3D ��������
            float weld_width = std::sqrt(std::pow(p_top_l->x - p_top_r->x, 2) +
                std::pow(p_top_l->y - p_top_r->y, 2) +
                std::pow(p_top_l->z - p_top_r->z, 2));

            // --- C. �����Ȩ���� ---
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

            // ��̬��ֵ
            float sum_rx = 0, sum_ry = 0, sum_rz = 0;
            for (int i = 0; i < top_n; ++i) {
                sum_rx += group[indices[i]]->rx; sum_ry += group[indices[i]]->ry; sum_rz += group[indices[i]]->rz;
            }
            ws.rx = sum_rx / top_n; ws.ry = sum_ry / top_n; ws.rz = sum_rz / top_n;

            weld_path.push_back(ws);

        }
        catch (...) { continue; }
    }

    // --- ³��ͳ���޳��쳣ֵ (2 sigma) ---
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

    // ģ�� Python ��ӡ��ʽ
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
    // 1. ����ԭʼ���� (��Ӧ points = [] ... points.append(p))
    /*std::vector<TimestampedPoint> rawPoints;
    loadTimestampedDataFromTxt(filePath, rawPoints);*/
    if (rawPoints.empty()) return;

    // 2. Ԥ������ȫ��ȥ�� (��Ӧ clean = preprocess_clean(points))
    // ת��Ϊ PCL ���Ƹ�ʽ���д���
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& p : rawPoints) {
        cloud->push_back(pcl::PointXYZ(static_cast<float>(p.x),
            static_cast<float>(p.y),
            static_cast<float>(p.z)));
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);

    if (flag._Equal("A")) {
        sor.setMeanK(10);            // A ����ܼ����ھ��������ʵ�����
        sor.setStddevMulThresh(1.5); // A ����ܼ�����׼��������ʵ�����
    }
    else if (flag._Equal("B")) {
        sor.setMeanK(5);            // B ���ϡ�裬�ھ��������ʵ�����
        sor.setStddevMulThresh(10); // B ���ϡ�裬��׼��������ʵ�����
    }
    else {
        // Ĭ�ϲ���
        sor.setMeanK(10);
        sor.setStddevMulThresh(1.5);
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    sor.filter(inliers->indices);

    // ��ȡ���˺�ġ��ɾ���ԭʼ����
    outCleanRaw.clear();
    for (int idx : inliers->indices) {
        outCleanRaw.push_back(rawPoints[idx]);
    }

    // 3. ��⺸�ӵ� (��Ӧ weld = solve_weld_centers(clean, target_z))
    // ע�⣺���ﴫ������Ѿ�ȥ����� outCleanRaw
    outWeld = solveWeldByPCA(outCleanRaw, targetZ, width);
}

bool WeldRecongnition::loadTimestampedDataFromTxt(const std::string& filename, std::vector<TimestampedPoint>& points) {
    std::ifstream infile(filename);
    std::string line;

    while (std::getline(infile, line)) {
        // �ҵ����ŵ�λ��
        size_t commaPos = line.find(',');
        if (commaPos == std::string::npos) continue;

        // ��ȡtimestamp
        std::string timestamp = line.substr(0, commaPos);

        // ��ȡʣ�µ� {...}
        std::string data = line.substr(commaPos + 1);
        // ȥ��������
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
                // ���ַǷ�������
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
    double sq_threshold = threshold * threshold; // ʹ��ƽ���Ƚϣ�Ч�ʸ���

    for (const auto& q : query_pts) {
        bool is_overlap = false;

        // ���� base �е����е���о���У��
        for (const auto& b : base_pts) {
            double dx = q.point.x - b.point.x;
            double dy = q.point.y - b.point.y;
            double dz = q.point.z - b.point.z;
            double dist_sq = dx * dx + dy * dy + dz * dz;

            if (dist_sq < sq_threshold) {
                is_overlap = true;
                break; // ֻҪ����һ���㹻���ĵ㣬���ж�Ϊ�ص�
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

    // 1. ��ȡԭʼ�����㲢�����ۻ�·������ s
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

    // 2. ����ȡ�� (���� 0 �� 1)
    for (int i = 0; i < num_samples; ++i) {
        // �ϸ�����β���߼�
        if (i == 0) {
            resampled.push_back(points.front());
            continue;
        }
        if (i == num_samples - 1) {
            resampled.push_back(points.back());
            continue;
        }

        // ����Ŀ�곤��λ��
        double target_s = (double)i / (num_samples - 1) * total_length;

        // ����Ŀ�� s ���ڵ����� (���� interp1d)
        auto it = std::lower_bound(s.begin(), s.end(), target_s);
        int idx = std::distance(s.begin(), it);

        if (idx == 0) idx = 1;
        int prev = idx - 1;

        // �������Բ�ֵ���� t
        double t = (target_s - s[prev]) / (s[idx] - s[prev]);

        weldStruct ws;
        ws.timestamp = "resampled";
        // �����ֵ
        ws.point.x = points[prev].point.x + t * (points[idx].point.x - points[prev].point.x);
        ws.point.y = points[prev].point.y + t * (points[idx].point.y - points[prev].point.y);
        ws.point.z = points[prev].point.z + t * (points[idx].point.z - points[prev].point.z);
        // ��̬��ֵ
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

    // --- 1. ת Eigen ---
    Eigen::MatrixXd pts(n, 3);
    for (int i = 0; i < n; ++i) {
        pts(i, 0) = points[i].point.x;
        pts(i, 1) = points[i].point.y;
        pts(i, 2) = points[i].point.z;
    }

    // --- 2. �����ֵ ---
    Eigen::Vector3d mean = pts.colwise().mean();

    // ȥ���Ļ�
    Eigen::MatrixXd centered = pts.rowwise() - mean.transpose();

    // --- 3. PCA (SVD) ---
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(centered, Eigen::ComputeThinV);
    Eigen::Matrix3d V = svd.matrixV();

    // ȡǰ����������ƽ�棩
    Eigen::Vector3d v1 = V.col(0);
    Eigen::Vector3d v2 = V.col(1);

    // --- 4. ͶӰ��2D ---
    Eigen::VectorXd x(n), y(n);
    for (int i = 0; i < n; ++i) {
        Eigen::Vector3d p = pts.row(i);
        x(i) = p.dot(v1);
        y(i) = p.dot(v2);
    }

    // --- 5. ���2DԲ ---
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

    // --- 6. ӳ���3D ---
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

// =====================================================================
// 翻译自 algorithm2.py：扫描线斜率分析检测焊缝坡口左右拐角 + 圆拟合重采样
// 仅依赖 C++ 标准库 + Eigen（已有依赖），无新增外部依赖，不做绘图。
// =====================================================================
namespace {

    // 对应 np.median：拷贝排序，偶数长度取中间两数均值
    double medianValue(std::vector<double> v) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        const size_t n = v.size();
        if (n % 2 == 1) return v[n / 2];
        return 0.5 * (v[n / 2 - 1] + v[n / 2]);
    }

    // 对应 np.interp：xs 已升序，越界取端点值
    double interpolateLinear(const std::vector<double>& xs, const std::vector<double>& ys, double xq) {
        const int n = static_cast<int>(xs.size());
        if (n == 0) return 0.0;
        if (xq <= xs[0]) return ys[0];
        if (xq >= xs[n - 1]) return ys[n - 1];
        auto it = std::lower_bound(xs.begin(), xs.end(), xq);
        int idx = static_cast<int>(std::distance(xs.begin(), it));
        if (idx == 0) return ys[0];
        int i0 = idx - 1;
        double denom = xs[idx] - xs[i0];
        if (denom < 1e-12) return ys[i0];
        double t = (xq - xs[i0]) / denom;
        return ys[i0] + t * (ys[idx] - ys[i0]);
    }

    // 对应 _refine_corner：从 anchor 沿 direction(±1) 步进，返回仍“陡”的最后索引
    int refineCorner(const std::vector<double>& absSlope, int anchor, int direction, int sw, double threshold) {
        const int n = static_cast<int>(absSlope.size());
        int lastSteep = anchor;
        for (int step = 0; step <= 50; ++step) {
            int i = anchor + direction * step;
            if (i < sw || i >= n - sw) break;
            if (absSlope[i] > threshold) {
                lastSteep = i;
            }
            else if (absSlope[i] < threshold * 0.5) {
                break;
            }
        }
        return lastSteep;
    }

    // 对应 _mad_filter：按 x 排序，对 y、z 分别 MAD 去野
    std::vector<pcl::PointXYZ> madFilter(const std::vector<pcl::PointXYZ>& input, double sigma = 3.5) {
        std::vector<pcl::PointXYZ> pts = input;
        std::sort(pts.begin(), pts.end(),
            [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) { return a.x < b.x; });

        for (int col = 0; col < 2; ++col) { // 0 = y, 1 = z
            std::vector<double> vals(pts.size());
            for (size_t i = 0; i < pts.size(); ++i) vals[i] = (col == 0) ? pts[i].y : pts[i].z;

            double med = medianValue(vals);

            std::vector<double> devs(vals.size());
            for (size_t i = 0; i < vals.size(); ++i) devs[i] = std::abs(vals[i] - med);
            double mad = medianValue(devs);

            double scale;
            if (mad > 1e-9) {
                scale = 1.4826 * mad;
            }
            else {
                double mean = 0.0;
                for (double v : vals) mean += v;
                mean /= vals.size();
                double var = 0.0;
                for (double v : vals) { double d = v - mean; var += d * d; }
                var /= vals.size();
                scale = std::sqrt(var);
            }

            if (scale > 1e-9) {
                std::vector<pcl::PointXYZ> kept;
                kept.reserve(pts.size());
                for (size_t i = 0; i < pts.size(); ++i) {
                    if (std::abs(vals[i] - med) < sigma * scale) kept.push_back(pts[i]);
                }
                if (kept.size() >= 5) pts = kept;
            }
        }
        return pts;
    }

    // 对应 FeatureResult（仅保留 ok/索引/点）
    struct CornerResult {
        bool ok = false;
        int leftIdx = -1;
        int rightIdx = -1;
        pcl::PointXYZ left;
        pcl::PointXYZ right;
    };

    // 对应 process_scanline：单条扫描线内检测焊缝坡口左右拐角
    CornerResult processScanline(const std::vector<TimestampedPoint>& linePts) {
        CornerResult r;
        const int n = static_cast<int>(linePts.size());
        if (n < 30) return r;

        std::vector<double> z_arr(n), y_arr(n);
        for (int i = 0; i < n; ++i) {
            z_arr[i] = linePts[i].z;
            y_arr[i] = linePts[i].y;
        }

        // 0) 中值去噪：孤立 z 尖峰用局部中值替换
        const int dw = 5;
        for (int i = dw; i < n - dw; ++i) {
            std::vector<double> local;
            local.reserve(2 * dw + 1);
            for (int j = i - dw; j <= i + dw; ++j) local.push_back(z_arr[j]);
            double med = medianValue(local);
            double dev = std::abs(z_arr[i] - med);

            std::vector<double> devs;
            devs.reserve(local.size());
            for (double v : local) devs.push_back(std::abs(v - med));
            double local_mad = medianValue(devs) * 1.4826;
            double noise = std::max(local_mad, 0.01);
            if (dev > 5.0 * noise) z_arr[i] = med;
        }

        // 1) 平滑（边缘收缩的滑动平均）
        int win = std::max(5, n / 50);
        if (win % 2 == 0) win += 1;
        std::vector<double> z_s(n);
        {
            const int half = win / 2;
            for (int i = 0; i < n; ++i) {
                int lo = std::max(0, i - half);
                int hi = std::min(n, i + half + 1);
                double sum = 0.0;
                for (int j = lo; j < hi; ++j) sum += z_arr[j];
                z_s[i] = sum / (hi - lo);
            }
        }

        // 2) 斜率 dz/dy
        const int sw = std::max(3, n / 60);
        std::vector<double> slope(n, 0.0);
        for (int i = sw; i < n - sw; ++i) {
            double dy = y_arr[i + sw] - y_arr[i - sw];
            double dz = z_s[i + sw] - z_s[i - sw];
            if (std::abs(dy) > 1e-9) slope[i] = dz / dy;
        }
        std::vector<double> abs_slope(n);
        for (int i = 0; i < n; ++i) abs_slope[i] = std::abs(slope[i]);

        // 3) 基线噪声与阈值
        std::vector<double> valid(abs_slope.begin() + sw, abs_slope.begin() + (n - sw));
        if (valid.empty()) return r;
        double baseline = medianValue(valid);
        double threshold = std::max(baseline * 4.0, 0.08);

        // 4) 连续活跃区
        struct Region { int s; int e; };
        std::vector<Region> regions;
        bool in_region = false;
        int start = 0;
        for (int i = sw; i < n - sw; ++i) {
            if (abs_slope[i] > threshold && !in_region) {
                start = i; in_region = true;
            }
            else if (abs_slope[i] <= threshold && in_region) {
                regions.push_back({ start, i });
                in_region = false;
            }
        }
        if (in_region) regions.push_back({ start, n - sw });

        if (regions.size() < 2) return r;

        // 5) 合并相邻同号区
        std::vector<Region> merged;
        merged.push_back(regions[0]);
        for (size_t k = 1; k < regions.size(); ++k) {
            int s = regions[k].s, e = regions[k].e;
            int prev_s = merged.back().s, prev_e = merged.back().e;

            double dy_prev = y_arr[prev_e] - y_arr[prev_s];
            double dz_prev = z_s[prev_e] - z_s[prev_s];
            double slope_prev = std::abs(dy_prev) > 1e-9 ? dz_prev / dy_prev : 0.0;

            double dy_cur = y_arr[e] - y_arr[s];
            double dz_cur = z_s[e] - z_s[s];
            double slope_cur = std::abs(dy_cur) > 1e-9 ? dz_cur / dy_cur : 0.0;

            int gap = s - prev_e;
            bool same_sign = (slope_prev * slope_cur) > 0;

            if (gap < 3 * win && same_sign) {
                merged.back().e = e;
            }
            else {
                merged.push_back({ s, e });
            }
        }

        // 6) 按宽度与 z 幅值过滤候选边
        struct Cand { int s; int e; double amp; };
        std::vector<Cand> candidates;
        int min_width = std::max(3, win / 3);
        for (const auto& rg : merged) {
            int width = rg.e - rg.s;
            if (width < min_width) continue;
            double z_amp = std::abs(z_s[rg.e] - z_s[rg.s]);
            if (z_amp < 0.15) continue;
            candidates.push_back({ rg.s, rg.e, z_amp });
        }

        if (candidates.size() < 2) return r;

        // 7) 找最优对：一升一降、中间平顶
        int best_s1 = -1, best_s2 = -1, best_e2 = -1;
        double best_score = -1.0;
        for (size_t i = 0; i + 1 < candidates.size(); ++i) {
            int s1 = candidates[i].s, e1 = candidates[i].e;
            double amp1 = candidates[i].amp;
            int s2 = candidates[i + 1].s, e2 = candidates[i + 1].e;
            double amp2 = candidates[i + 1].amp;

            int gap = s2 - e1;

            double dy1 = y_arr[e1] - y_arr[s1];
            double dz1 = z_s[e1] - z_s[s1];
            double slope1 = std::abs(dy1) > 1e-9 ? dz1 / dy1 : 0.0;

            double dy2 = y_arr[e2] - y_arr[s2];
            double dz2 = z_s[e2] - z_s[s2];
            double slope2 = std::abs(dy2) > 1e-9 ? dz2 / dy2 : 0.0;

            if (slope1 * slope2 >= 0) continue;

            if (gap > 5) {
                int cnt = s2 - e1;
                if (cnt > 1) {
                    double mean = 0.0;
                    for (int j = e1; j < s2; ++j) mean += z_s[j];
                    mean /= cnt;
                    double var = 0.0;
                    for (int j = e1; j < s2; ++j) { double d = z_s[j] - mean; var += d * d; }
                    var /= cnt;
                    if (std::sqrt(var) > 0.3) continue;
                }
            }

            double score = (amp1 + amp2) * (1.0 / (1.0 + std::abs(gap - 10) / 50.0));
            if (score > best_score) {
                best_score = score;
                best_s1 = s1;
                best_s2 = s2;
                best_e2 = e2;
            }
        }

        if (best_score < 0.0) return r;

        // 8) 精修拐角到精确过渡点
        int cL = refineCorner(abs_slope, best_s1, -1, sw, threshold * 0.5);
        int cR = refineCorner(abs_slope, best_e2, 1, sw, threshold * 0.5);

        if (cL >= cR) return r;

        r.ok = true;
        r.leftIdx = cL;
        r.rightIdx = cR;
        r.left = pcl::PointXYZ((float)linePts[cL].x, (float)linePts[cL].y, (float)linePts[cL].z);
        r.right = pcl::PointXYZ((float)linePts[cR].x, (float)linePts[cR].y, (float)linePts[cR].z);
        return r;
    }

    // 对应 fit_and_resample：x-z 平面最小二乘圆拟合 + 按角度均匀重采样
    std::vector<pcl::PointXYZ> fitAndResampleCircle(const std::vector<pcl::PointXYZ>& input, int n_samples) {
        std::vector<pcl::PointXYZ> pts = input;
        std::sort(pts.begin(), pts.end(),
            [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) { return a.x < b.x; });

        // MAD 预滤波（对应 fit_and_resample 内部的 y/z MAD 去野）
        pts = madFilter(pts, 3.5);

        int n = static_cast<int>(pts.size());
        if (n < 5) return {};

        std::vector<double> xs(n), ys(n), zs(n);
        for (int i = 0; i < n; ++i) {
            xs[i] = pts[i].x; ys[i] = pts[i].y; zs[i] = pts[i].z;
        }

        double cx = 0.0, cz = 0.0, R = 0.0;

        // 迭代圆拟合去野（3 次）
        for (int iter = 0; iter < 3; ++iter) {
            Eigen::MatrixXd A(n, 3);
            Eigen::VectorXd b(n);
            for (int i = 0; i < n; ++i) {
                A(i, 0) = 2.0 * xs[i];
                A(i, 1) = 2.0 * zs[i];
                A(i, 2) = 1.0;
                b(i) = xs[i] * xs[i] + zs[i] * zs[i];
            }
            Eigen::Vector3d sol = A.colPivHouseholderQr().solve(b);
            cx = sol(0);
            cz = sol(1);
            double temp = sol(2) + cx * cx + cz * cz;
            R = temp > 0.0 ? std::sqrt(temp) : 0.0;

            std::vector<double> resid(n);
            for (int i = 0; i < n; ++i) {
                double dist = std::sqrt((xs[i] - cx) * (xs[i] - cx) + (zs[i] - cz) * (zs[i] - cz));
                resid[i] = std::abs(dist - R);
            }
            double med_r = medianValue(resid);
            std::vector<double> resid_dev(n);
            for (int i = 0; i < n; ++i) resid_dev[i] = std::abs(resid[i] - med_r);
            double mad_r = medianValue(resid_dev) * 1.4826;
            if (mad_r < 1e-9) break;

            double thresh = med_r + 3.0 * mad_r;
            std::vector<double> nxs, nys, nzs;
            nxs.reserve(n); nys.reserve(n); nzs.reserve(n);
            bool all_kept = true;
            int kept_cnt = 0;
            for (int i = 0; i < n; ++i) {
                if (resid[i] <= thresh) {
                    nxs.push_back(xs[i]); nys.push_back(ys[i]); nzs.push_back(zs[i]);
                    ++kept_cnt;
                }
                else {
                    all_kept = false;
                }
            }
            if (all_kept || kept_cnt < 10) break;
            xs.swap(nxs); ys.swap(nys); zs.swap(nzs);
            n = static_cast<int>(xs.size());
        }

        // 确定角度范围（弧 < 180°）
        std::vector<double> angles(n);
        for (int i = 0; i < n; ++i) angles[i] = std::atan2(zs[i] - cz, xs[i] - cx);

        int n_edge = std::min(5, n / 4);
        double med_first = medianValue(std::vector<double>(angles.begin(), angles.begin() + n_edge));
        double med_last = medianValue(std::vector<double>(angles.end() - n_edge, angles.end()));

        double ang_start, ang_end;
        if (med_first <= med_last) {
            ang_start = *std::min_element(angles.begin(), angles.begin() + n_edge);
            ang_end = *std::max_element(angles.end() - n_edge, angles.end());
        }
        else {
            ang_start = *std::max_element(angles.begin(), angles.begin() + n_edge);
            ang_end = *std::min_element(angles.end() - n_edge, angles.end());
        }

        // 沿弧均匀采样 n_samples 个点，y 沿 x 线性插值
        std::vector<pcl::PointXYZ> result;
        result.reserve(n_samples);
        for (int i = 0; i < n_samples; ++i) {
            double frac = (n_samples > 1) ? (double)i / (n_samples - 1) : 0.0;
            double a = ang_start + (ang_end - ang_start) * frac;
            double x_arc = cx + R * std::cos(a);
            double z_arc = cz + R * std::sin(a);
            double y_arc = interpolateLinear(xs, ys, x_arc);
            result.push_back(pcl::PointXYZ((float)x_arc, (float)y_arc, (float)z_arc));
        }
        return result;
    }

} // namespace

// 公开入口：按扫描线检测左右拐角 -> MAD 去噪 -> 圆拟合重采样 -> 输出左右焊缝点
void WeldRecongnition::detectWeldLeftRight(
    const std::vector<TimestampedPoint>& cloud,
    std::vector<weldStruct>& leftWeld,
    std::vector<weldStruct>& rightWeld,
    int num_samples)
{
    leftWeld.clear();
    rightWeld.clear();
    if (cloud.empty()) return;

    // 按输入顺序把连续相同 timestamp 的点归为一条扫描线（等价 Python main 的顺序分组）
    std::vector<std::vector<TimestampedPoint>> scanlines;
    bool first = true;
    std::string cur_ts;
    for (const auto& p : cloud) {
        if (first || cur_ts != p.timestamp) {
            first = false;
            cur_ts = p.timestamp;
            scanlines.push_back({});
        }
        scanlines.back().push_back(p);
    }

    std::vector<pcl::PointXYZ> left_raw, right_raw;
    for (const auto& line : scanlines) {
        CornerResult cr = processScanline(line);
        if (cr.ok) {
            left_raw.push_back(cr.left);
            right_raw.push_back(cr.right);
        }
    }

    auto toWeld = [](const std::vector<pcl::PointXYZ>& arc, const std::string& tag) {
        std::vector<weldStruct> out;
        out.reserve(arc.size());
        for (const auto& p : arc) {
            weldStruct w;
            w.timestamp = tag;
            w.point = p;
            w.rx = w.ry = w.rz = w.width = 0.0;
            out.push_back(w);
        }
        return out;
    };

    if (left_raw.size() >= 5) {
        std::vector<pcl::PointXYZ> clean = madFilter(left_raw);
        leftWeld = toWeld(fitAndResampleCircle(clean, num_samples), "left");
    }
    if (right_raw.size() >= 5) {
        std::vector<pcl::PointXYZ> clean = madFilter(right_raw);
        rightWeld = toWeld(fitAndResampleCircle(clean, num_samples), "right");
    }
}