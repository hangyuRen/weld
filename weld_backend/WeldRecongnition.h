#pragma once
#ifndef WELDRECONGNITION_H
#define WELDRECONGNITION_H

#include "PointCloudProcess.h"
#include <open3d/Open3D.h>
#include <pcl/filters/statistical_outlier_removal.h>

class WeldRecongnition {
public:
	static void getWeldSeam(std::vector<TimestampedPoint>& cloud, std::vector<weldStruct>& weld);
	static std::vector<weldStruct> solveWeldByPCA(const std::vector<TimestampedPoint>& points, float target_z, float& width);
	static void loadAndProcess(std::vector<TimestampedPoint>& rawPoints, double targetZ, std::vector<TimestampedPoint>& outCleanRaw, std::vector<weldStruct>& outWeld, const std::string& flag, float& width);
	static bool loadTimestampedDataFromTxt(const std::string& filename, std::vector<TimestampedPoint>& points);
	static std::vector<weldStruct> filterOverlappingPoints(const std::vector<weldStruct>& base_pts, const std::vector<weldStruct>& query_pts, double threshold = 10.0);
	static std::vector<weldStruct> fitAndResamplePath(const std::vector<weldStruct>& points, int num_samples = 30);
	static Eigen::Vector3d fitCircle3D(const std::vector<weldStruct>& points);
	static std::vector<weldStruct> offsetAlongBendRadius(const std::vector<weldStruct>& weld_pts, const Eigen::Vector3d& center, double offset);

private:
	static std::vector<TimestampedPoint> timeCloud;
};
#endif