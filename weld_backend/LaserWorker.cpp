#include "LaserWorker.h"


LaserWorker::LaserWorker(robotConnect* rbc,
	const std::vector<std::vector<double>>& cmatrix, const std::string ip)
	:
	cloud(new pcl::PointCloud<pcl::PointXYZ>),
	rbc(rbc),
	cmatrix(cmatrix),
	rbif(3000),
	bufReadIndex(0),
	bufWriteIndex(0),
	dpts(0) {
	// 初始化全局资源（只需一次）
	fv::LaserFactory::initialize();

	// 激光A IP：192.168.1.61
	// 激光B IP：192.168.1.62
	this->laser = fv::LaserFactory::create(ip);
}

void LaserWorker::openLaser()
{
	this->laser->laserOn();
}

void LaserWorker::closeLaser()
{
	this->laser->laserOff();
}


void LaserWorker::stop() {
	running = false;
}

void LaserWorker::start() {
	if (!laser) {
		return;
	}

	auto packetProcessor = [this](fv::PointCloudInfoPtr info, void* opaque) {
		double ret = -1;
		uint64_t pts1 = 0;
		uint64_t cloudpts = 0;
		LocData pos = { 0,0,0,0,0,0,0,0,0 };
		rbc->m_pMot->getLocData(0, pos);
		auto pts = rbc->getRobSysTime();
		pts1 = laser->getCurrentPTS() / 1000;
		if (dpts == 0) {
			dpts = pts - pts1;
			bufReadIndex = 0;
			bufWriteIndex = 0;
		}

		rbif[bufWriteIndex].robpos = pos;
		rbif[bufWriteIndex].systime = pts;
		bufWriteIndex = (bufWriteIndex + 1) % rbif.size();
		cloudpts = info->pts / 1000;
		auto const& points = info->points;

		if (!points.empty()) {
			if (bufReadIndex > bufWriteIndex && bufReadIndex > 2900) {
				bufReadIndex = 1;
			}
			for (int i = bufReadIndex; i < bufWriteIndex; i++)
			{
				if (i % 10 != 0) {
					continue;
				}
				int devtime = rbif[i].systime - cloudpts - dpts;
				if (abs(devtime) < 60)
				{
					if (!points.empty()) {
						this->imagePosToRobotPos(rbif[i].robpos, points, rbif[i].systime);
						bufReadIndex = i + 1;
					}
				}
			}
		}
		};

	auto stateProcessor = [](XtrClientState state, void* opaque) {
		std::cout << "激光状态: " << state;
		};

	this->laser->addPointCloudCallback(packetProcessor, nullptr);
	this->laser->addStateCallback(stateProcessor, nullptr);

	this->laser->start();
	this->laser->waitForReady();
	this->laser->setTaskId(0);

	double ret = -1;
	double ret2 = -1;
	std::cout << "start scan" << std::endl;
	rbc->m_pVar->setR(50, 1);
	rbc->m_pVar->setR(51, 0);
	isOver = false;
	while (ret != 0) {
		// 扫描结束标志
		rbc->m_pVar->getR(50, ret);
		std::cout << "R[50]:" << ret << std::endl;

		// 激光开关标志
		rbc->m_pVar->getR(51, ret2);
		if (ret2 == 1) {
			if (laser->isLaserOff()) {
				openLaser();
			}
		}
		else {
			if (laser->isLaserOn()) {
				closeLaser();
			}
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	isOver = true;
	closeLaser();
	this->laser->stop();
}

void LaserWorker::asyncStart() {
	if (workerThread.joinable()) {
		workerThread.join();
	}
	workerThread = std::thread(&LaserWorker::start, this);
}

void LaserWorker::imagePosToRobotPos(LocData& pos, const fv::PointCloud& localPerson, uint64_t syst) {
	vector<vector<double>> dpos = {
		{0},
		{0},
		{0} };
	vector<vector<double>> drpos = {
		{0},
		{0},
		{0} };
	vector<vector<double>> matrix_3d_RR = {
		{1, 2, 3},
		{4, 5, 6},
		{7, 8, 9}
	};
	LocData lrpos = { 0,0,0,0,0,0,0,0,0 };

	double rar[3];

	//转换机器人点位度数为弧度
	rar[0] = (pos[3] * pia) / 180;
	rar[1] = (pos[4] * pia) / 180;
	rar[2] = (pos[5] * pia) / 180;
	//计算机器人旋转矩阵
	matrix_3d_RR[0][0] = cos(rar[0]) * cos(rar[1]);
	matrix_3d_RR[0][1] = cos(rar[0]) * sin(rar[1]) * sin(rar[2]) - (sin(rar[0]) * cos(rar[2]));
	matrix_3d_RR[0][2] = cos(rar[0]) * sin(rar[1]) * cos(rar[2]) + (sin(rar[0]) * sin(rar[2]));
	matrix_3d_RR[1][0] = sin(rar[0]) * cos(rar[1]);
	matrix_3d_RR[1][1] = sin(rar[0]) * sin(rar[1]) * sin(rar[2]) + (cos(rar[0]) * cos(rar[2]));
	matrix_3d_RR[1][2] = sin(rar[0]) * sin(rar[1]) * cos(rar[2]) - (cos(rar[0]) * sin(rar[2]));
	matrix_3d_RR[2][0] = 0 - (sin(rar[1]));
	matrix_3d_RR[2][1] = cos(rar[1]) * sin(rar[2]);
	matrix_3d_RR[2][2] = cos(rar[1]) * cos(rar[2]);

	if (!localPerson.empty())
	{
		for (int i = 0; i < localPerson.size(); i++)
		{
			if (i % 2 == 0) {
				continue;
			}
			dpos[0][0] = localPerson[i].y;
			dpos[1][0] = localPerson[i].z;
			dpos[2][0] = 1;
			drpos = matrixMultiply(cmatrix, dpos);
			drpos = matrixMultiply(matrix_3d_RR, drpos);

			lrpos[0] = pos[0] + drpos[0][0];
			lrpos[1] = pos[1] + drpos[1][0];
			lrpos[2] = pos[2] + drpos[2][0];

			cloud->push_back(pcl::PointXYZ(lrpos[0], lrpos[1], lrpos[2]));
			tsCloud.push_back(TimestampedPoint{
				std::to_string(syst),
				lrpos[0],
				lrpos[1],
				lrpos[2],
				pos[3],
				pos[4],
				pos[5]
				});
		}
	}
}

vector<vector<double>> LaserWorker::matrixMultiply(const vector<vector<double>>& A, const vector<vector<double>>& B) {
	int m = A.size();
	int n = A[0].size();
	int p = B[0].size();

	// 结果矩阵初始化为0.0
	vector<vector<double>> C(m, vector<double>(p, 0.0));

	// 矩阵乘法
	for (int i = 0; i < m; ++i) {
		for (int j = 0; j < p; ++j) {
			for (int k = 0; k < n; ++k) {
				C[i][j] += A[i][k] * B[k][j];
			}
		}
	}

	return C;
}