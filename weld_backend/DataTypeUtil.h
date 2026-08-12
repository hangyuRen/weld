#pragma once
#ifndef DATATYPEUTIL_H
#define DATATYPEUTIL_H

#include <string>
#include <pcl/point_types.h>

typedef std::vector<double> LocData;

struct Point3D {
	double x;
	double y;
	double z;
};

struct Pose {
	Eigen::Vector3d position;        // x, y, z
	Eigen::Vector3d rotation_vector; // rx, ry, rz
};

// 定义带时间戳和位姿信息的数据结构
struct TimestampedPoint {
	std::string timestamp;
	double x, y, z;
	double rx, ry, rz;
};

struct TimestampedPointXYZ {
	std::string timestamp;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
};


struct MeanVariance {
	double mean;
	double variance;
};

struct RobotInfo
{
	LocData robpos = { 0,0,0,0,0,0,0,0,0 };
	uint64_t systime = 0;
};

struct weldStruct {
	std::string timestamp;
	pcl::PointXYZ point;
	double rx, ry, rz;
	double width;
};
#endif