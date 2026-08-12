#pragma once
#ifndef LASERWORKER_H
#define LASERWORKER_H
#include <memory>
#include <atomic>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fv/fvlaser/laser.hpp>
#include "robotconnect.h"
#include "DataTypeUtil.h"
#include <chrono>
#include <thread>
#include <cmath>

class LaserWorker {

public:
	LaserWorker(robotConnect* rbc,
		const std::vector<std::vector<double>>& cmatrix,
		const std::string ip);


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	std::vector<TimestampedPoint> tsCloud;

	void openLaser();
	void closeLaser();

	void start();
	void stop();
	void asyncStart();
	bool isOver = false;

private:
	void imagePosToRobotPos(LocData& pos, const fv::PointCloud& localPerson, uint64_t syst);
	vector<vector<double>> matrixMultiply(const vector<vector<double>>& A, const vector<vector<double>>& B);

	robotConnect* rbc;
	std::vector<std::vector<double>> cmatrix;
	std::atomic<bool> running{ true };
	int bufReadIndex;
	int bufWriteIndex;
	uint64_t dpts;
	vector<RobotInfo> rbif;
	const double pia = 3.141592;
	fv::Laser* laser;

	std::thread workerThread;
};
#endif