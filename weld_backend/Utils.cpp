#include "Utils.h"

std::vector<double> Utils::top_length = []() {
	std::vector<double> vec;
	vec.reserve(1000); // 分配初始容量
	return vec;
	}();

std::vector<double> Utils::bottom_length = []() {
	std::vector<double> vec;
	vec.reserve(1000); // 分配初始容量
	return vec;
	}();

std::vector<double> Utils::height = []() {
	std::vector<double> vec;
	vec.reserve(1000); // 分配初始容量
	return vec;
	}();

// 计算两点向量（p2-p1）
pcl::PointXYZ Utils::vectorBetween(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2) {
	return pcl::PointXYZ{ p2.x - p1.x, p2.y - p1.y, p2.z - p1.z };
}

// 向量长度
double Utils::vectorLength(const pcl::PointXYZ& v) {
	return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

// 归一化向量
pcl::PointXYZ Utils::normalize(const pcl::PointXYZ& v) {
	float len = vectorLength(v);
	return pcl::PointXYZ{ v.x / len, v.y / len, v.z / len };
}

// 计算两个向量的点积
double Utils::dotProduct(const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

// 判断向量是否平行（cosθ绝对值大于阈值）
bool Utils::isParallel(const pcl::PointXYZ& a, const pcl::PointXYZ& b, double threshold_cos = 0.95f) {
	pcl::PointXYZ na = normalize(a);
	pcl::PointXYZ nb = normalize(b);
	double dp = dotProduct(na, nb);
	return std::fabs(dp) >= threshold_cos;
}

// 计算两点距离
double Utils::distance(const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
	return std::sqrt((a.x - b.x) * (a.x - b.x)
		+ (a.y - b.y) * (a.y - b.y)
		+ (a.z - b.z) * (a.z - b.z));
}

// 计算中点
pcl::PointXYZ Utils::midpoint(const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
	return pcl::PointXYZ{ (a.x + b.x) / 2.f, (a.y + b.y) / 2.f, (a.z + b.z) / 2.f };
}

// 找出平行的两条边并计算上下底和高
bool Utils::computeTrapezoidFromPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr points,
	double& top_length, double& bottom_length, double& height) {
	if (points->size() != 4) {
		std::cerr << "需要4个点" << std::endl;
		return false;
	}

	// 4条边
	struct Edge {
		int idx1, idx2;
		pcl::PointXYZ vec;
		double length;
	};

	std::vector<Edge> edges = {
		{0, 1, vectorBetween((*points)[0], (*points)[1]), distance((*points)[0], (*points)[1])},
		{1, 2, vectorBetween((*points)[1], (*points)[2]), distance((*points)[1], (*points)[2])},
		{2, 3, vectorBetween((*points)[2], (*points)[3]), distance((*points)[2], (*points)[3])},
		{3, 0, vectorBetween((*points)[3], (*points)[0]), distance((*points)[3], (*points)[0])}
	};

	// 找出两条平行边
	int idx_a = -1, idx_b = -1;
	for (int i = 0; i < 4; ++i) {
		for (int j = i + 1; j < 4; ++j) {
			// 不共点（即两条边不共享顶点）
			if (edges[i].idx1 != edges[j].idx1 &&
				edges[i].idx1 != edges[j].idx2 &&
				edges[i].idx2 != edges[j].idx1 &&
				edges[i].idx2 != edges[j].idx2) {
				if (isParallel(edges[i].vec, edges[j].vec)) {
					idx_a = i;
					idx_b = j;
					break;
				}
			}
		}
		if (idx_a != -1) break;
	}

	if (idx_a == -1 || idx_b == -1) {
		/*std::cerr << "未找到平行的两条边" << std::endl;*/
		return false;
	}

	// 这两条平行边为上下底边
	top_length = edges[idx_a].length;
	bottom_length = edges[idx_b].length;

	// 中点
	pcl::PointXYZ mid_top = midpoint((*points)[edges[idx_a].idx1], (*points)[edges[idx_a].idx2]);
	pcl::PointXYZ mid_bottom = midpoint((*points)[edges[idx_b].idx1], (*points)[edges[idx_b].idx2]);


	// 高度
	height = distance(mid_top, mid_bottom);

	//将数据存储
	addTopLength(top_length);
	addBottomLength(bottom_length);
	addHeight(height);

	return true;
}

double Utils::filteredMean(const std::vector<double>& data, double stddevThreshold = 2.0f) {
	if (data.empty()) return 0.0f;

	// 1. 计算均值
	double mean = std::accumulate(data.begin(), data.end(), 0.0f) / data.size();

	// 2. 计算标准差
	double variance = 0.0f;
	for (double v : data) {
		variance += (v - mean) * (v - mean);
	}
	variance /= data.size();
	double stddev = std::sqrt(variance);

	// 3. 过滤数据，留下 stddevThreshold 倍标准差以内的点
	std::vector<double> filtered;
	for (double v : data) {
		if (std::fabs(v - mean) <= stddevThreshold * stddev) {
			filtered.push_back(v);
		}
	}

	// 4. 求过滤后的均值
	if (filtered.empty()) return 0.0f;

	return std::accumulate(filtered.begin(), filtered.end(), 0.0f) / filtered.size();
}

bool Utils::getWeldInfo(const std::vector<TimestampedPoint>& cloudData, std::vector<double>& result) {
	std::vector<TimestampedPointXYZ> clouds = PointCloudProcessor::groupPointsToVector(cloudData);
	int i = 1;
	for (const auto& item : clouds) {
		if (i >= 111) {
			double epsilon = 0.8; // 简化精度
			pcl::PointCloud<pcl::PointXYZ>::Ptr simplified = PointCloudProcessor::simplifyPointCloud(item.cloud, epsilon);

			//计算上下底边和长度
			if (simplified->size() == 4) {
				double top_len, bottom_len, h;
				if (!Utils::computeTrapezoidFromPoints(simplified, top_len, bottom_len, h)) {
					return false;
				}
			}

		}
		i++;
	}

	result.push_back(Utils::filteredMean(Utils::getTopLength(), 2.0));
	result.push_back(Utils::filteredMean(Utils::getBottomLength(), 2.0));
	result.push_back(Utils::filteredMean(Utils::getHeight(), 2.0));

	return true;
}

std::vector<double> Utils::getTopLength() {
	return top_length;
}

std::vector<double> Utils::getBottomLength() {
	return bottom_length;
}

std::vector<double> Utils::getHeight() {
	return height;
}

void Utils::clearData() {
	top_length.clear();
	bottom_length.clear();
	height.clear();
}

void Utils::addTopLength(double data) {
	top_length.push_back(data);
}

void Utils::addBottomLength(double data) {
	bottom_length.push_back(data);
}

void Utils::addHeight(double data) {
	height.push_back(data);
}


std::vector<Pose> Utils::computeHalfCirclePoses(const Pose& startPose, double diameter, double robotOffset) {

	std::vector<Pose> poses;

	double radius = diameter / 2.0 + robotOffset;

	// 定义Z轴朝上
	Eigen::Vector3d z_axis(0, 0, 1);

	// 圆心 = 起点 + z轴 * 半径
	Eigen::Vector3d center = startPose.position + z_axis *
		radius;

	// 返回给定角度的圆弧点
	auto pointOnArc = [&](double theta_deg) {
		double theta = theta_deg * M_PI / 180.0;
		return center + Eigen::Vector3d(
			0,
			std::sin(theta),
			-std::cos(theta)
		) * radius;
		};

	// 计算旋转角度
	auto orientationAt = [&](double theta_deg) {
		double theta = theta_deg * M_PI / 180.0;

		// 圆弧轨迹在 Y-Z 平面 => 切向量如下
		Eigen::Vector3d tangent(0, std::cos(theta), std::sin(theta));

		// 法向量指向圆心（Z轴 = 工具朝向）
		Eigen::Vector3d normal = pointOnArc(theta_deg) - center;
		normal.normalize();

		// Y轴 = Z * X，构建正交基
		Eigen::Vector3d y_axis = normal.cross(tangent);

		Eigen::Matrix3d R;
		R.col(0) = tangent.normalized(); // X: 工具前进方向
		R.col(1) = y_axis.normalized();  // Y: 侧向
		R.col(2) = normal;               // Z: 法向，指向圆心

		Eigen::AngleAxisd aa(R);
		return aa.angle() * aa.axis(); // 输出旋转向量
		};

	// 构造三个路径点
	for (double angle : {0.0, 90.0, 180.0}) {
		Pose p;
		p.position = pointOnArc(angle);
		p.rotation_vector = orientationAt(angle);
		poses.push_back(p);
	}

	return poses;
}
