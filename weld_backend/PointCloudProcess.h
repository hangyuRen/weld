#pragma once
#ifndef POINT_CLOUD_PROCESSOR_H
#define POINT_CLOUD_PROCESSOR_H

#include <pcl/kdtree/kdtree_flann.h>
#include "DataTypeUtil.h"

class PointCloudProcessor {
public:
    // 直通滤波
    static std::vector<TimestampedPoint> throughOutlier(std::vector<TimestampedPoint>& data,
        char feature = 'x');

    // 统计滤波
    static std::vector<TimestampedPoint> statisticalOutlier(std::vector<TimestampedPoint>& data,
        int k = 10,
        float std_dev = 1.0f);

    // 求指定轴数据的均值和方差
    static MeanVariance computeMeanVariance(const std::vector<TimestampedPoint>& data, char axis = 'x');

    // 根据属于同一timestamp的点集来获取焊接点
    static TimestampedPoint weldValueByTimestamp(const std::vector<TimestampedPoint>& data, float percentage = 0.05f);

    // 根据timestamp来分组
    static std::vector<std::vector<TimestampedPoint>> groupByTimestamp(
        const std::vector<TimestampedPoint>& data);

    // 去除离群焊缝点
    static std::vector<TimestampedPoint> removeOutliersByDistance(const std::vector<TimestampedPoint>& data, int k = 5, float threshold = 3.0);

    // 按时间戳分组
    static std::vector<TimestampedPointXYZ> groupPointsToVector(const std::vector<TimestampedPoint>& points);

    // 计算点到线段的垂直距离
    static double perpendicularDistance(const pcl::PointXYZ& point, const pcl::PointXYZ& line_start, const pcl::PointXYZ& line_end);

    // RDP 主体算法（输入：点云；输出：简化后的关键点）
    static void RDP(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int start_index, int end_index, double epsilon, std::vector<int>& output_indices);

    // 封装：调用 RDP 并输出简化点云
    static pcl::PointCloud<pcl::PointXYZ>::Ptr simplifyPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double epsilon);
};

#endif