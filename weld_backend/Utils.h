#pragma once
#ifndef UTILS_H
#define UTILS_H
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include <cmath>
#include <numeric>
#include <iostream>
#include "DataTypeUtil.h"
#include "PointCloudProcess.h"

class Utils
{

public:

	static pcl::PointXYZ vectorBetween(const pcl::PointXYZ& p1, const pcl::PointXYZ& p2);

	// 向量长度
	static double vectorLength(const pcl::PointXYZ& v);

	// 归一化向量
	static pcl::PointXYZ normalize(const pcl::PointXYZ& v);

	// 计算两个向量的点积
	static double dotProduct(const pcl::PointXYZ& a, const pcl::PointXYZ& b);

	// 判断向量是否平行（cosθ绝对值大于阈值）
	static bool isParallel(const pcl::PointXYZ& a, const pcl::PointXYZ& b, double threshold_cos);

	// 计算两点距离
	static double distance(const pcl::PointXYZ& a, const pcl::PointXYZ& b);

	// 计算中点
	static pcl::PointXYZ midpoint(const pcl::PointXYZ& a, const pcl::PointXYZ& b);

	// 找出平行的两条边并计算上下底和高
	static bool computeTrapezoidFromPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr points, double& top_length, double& bottom_length, double& height);

	//计算平均值
	static double filteredMean(const std::vector<double>& data, double stddevThreshold);

	// 计算焊缝上下底边长度、深度信息
	static bool getWeldInfo(const std::vector<TimestampedPoint>& point, std::vector<double>& result);

	// 计算扫描轨迹的中点和终点的坐标以及位姿,单位mm
	static std::vector<Pose> computeHalfCirclePoses(const Pose& startPose, double diameter, double robotOffset);

	static std::vector<double> getTopLength();

	static std::vector<double> getBottomLength();

	static std::vector<double> getHeight();

	static void clearData();

	static void addTopLength(double data);

	static void addBottomLength(double data);

	static void addHeight(double data);

private:
	static std::vector<double> top_length;
	static std::vector<double> bottom_length;
	static std::vector<double> height;
};
#endif