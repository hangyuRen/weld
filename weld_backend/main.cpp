#include <fstream>
#include <filesystem>
#include "httplib.h"
#include "CurrentServer.h"
#include "DigitalTwinServer.h"
#include "ModbusServer.h"
#include "robotConnect.h"
#include "LaserWorker.h"
#include "WeldRecongnition.h"
#include "Calculate.h"

namespace fs = std::filesystem;

// 对原始焊接点进行均匀采样，保留首尾，去重后返回 sampleCount 个点
std::vector<weldStruct> sampleAverageWeld(const std::vector<weldStruct>& data, size_t sampleCount) {
    std::vector<weldStruct> result;
    size_t total = data.size();
    if (total == 0) return result;

    // 1. 去掉前15%的点
    size_t removeCount = static_cast<size_t>(std::floor(static_cast<double>(total) * 0.15));
    if (removeCount >= total) removeCount = total - 1; // 至少保留一个

    size_t startIdx = removeCount;
    size_t endIdx = total - 1;
    size_t n = endIdx >= startIdx ? (endIdx - startIdx + 1) : 0;

    // 剩余点数不足5个直接返回
    if (n == 0) return result;
    if (n <= 5) {
        result.reserve(n);
        for (size_t i = startIdx; i <= endIdx; ++i) result.push_back(data[i]);
        return result;
    }

    // 2. 等间距步长采样
    double step = static_cast<double>(n - 1) / static_cast<double>(sampleCount - 1);

    // 3. 按步长选取索引
    std::vector<size_t> picks;
    picks.reserve(sampleCount);
    for (size_t i = 0; i < sampleCount; ++i) {
        double pos = i * step; // 范围 [0, n-1]
        size_t relIdx = static_cast<size_t>(std::round(pos)); // 四舍五入到整数
        if (relIdx >= n) relIdx = n - 1;
        picks.push_back(startIdx + relIdx);
    }

    // 4. 强制首尾为 startIdx / endIdx
    picks.front() = startIdx;
    picks.back() = endIdx;

    // 5. 处理重复索引，确保单调递增不重复
    for (size_t i = 1; i < picks.size(); ++i) {
        if (picks[i] <= picks[i - 1]) {
            size_t newIdx = picks[i - 1] + 1;
            if (newIdx > endIdx) {
                // 无法向后移动，则尝试向前调整前面的索引
                for (size_t j = i - 1; j > 0; --j) {
                    if (picks[j] > picks[j - 1] + 1) {
                        picks[j] = picks[j] - 1;
                        break;
                    }
                }
            }
            else {
                picks[i] = newIdx;
            }
        }
    }

    // 6. 去重并按需补齐到 sampleCount
    std::vector<size_t> uniquePicks;
    uniquePicks.reserve(picks.size());
    for (size_t idx : picks) {
        if (uniquePicks.empty() || uniquePicks.back() != idx) uniquePicks.push_back(idx);
    }
    while (uniquePicks.size() < sampleCount) {
        bool inserted = false;
        for (size_t pos = startIdx; pos <= endIdx && uniquePicks.size() < sampleCount; ++pos) {
            if (std::find(uniquePicks.begin(), uniquePicks.end(), pos) == uniquePicks.end()) {
                auto it = std::upper_bound(uniquePicks.begin(), uniquePicks.end(), pos);
                uniquePicks.insert(it, pos);
                inserted = true;
            }
        }
        if (!inserted) break;
        if (uniquePicks.size() > sampleCount) {
            uniquePicks.resize(sampleCount);
            break;
        }
    }

    // 确保首尾
    if (!uniquePicks.empty()) {
        uniquePicks.front() = startIdx;
        uniquePicks.back() = endIdx;
    }

    // 7. 输出结果
    result.reserve(uniquePicks.size());
    for (size_t idx : uniquePicks) {
        result.push_back(data[idx]);
    }

    return result;
}

int main() {
    // 机械臂连接对象
    robotConnect* robotA = new robotConnect();
    robotConnect* robotB = new robotConnect();

    LaserWorker* laserA = nullptr;
    LaserWorker* laserB = nullptr;

    CurrentServerWorker* currWorker = new CurrentServerWorker();
    DigitalTwinServer* dtWorker = new DigitalTwinServer();
    ModbusServerWorker* modbusWorker = new ModbusServerWorker();


    // =====================http==========================
    httplib::Server svr;

    svr.Options(R"(/.*)", [](const httplib::Request&, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        res.set_header("Access-Control-Allow-Methods", "POST, OPTIONS");
        res.set_header("Access-Control-Allow-Headers", "*");
        });

    // 连接机械臂和激光，启动数据推送服务
    bool serversStarted = false;
    svr.Post("/connect", [&](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        try {
            printf("正在尝试连接机械臂...\n");

            // 连接机械臂A
            if (!robotA->m_pCmApi->isConnected()) {
                bool flag = robotA->connectRobot("192.168.1.71", 23234);
                if (!flag) {
                    res.status = 500;
                    res.set_content(u8R"({"status":"fail","message":"机械臂A连接失败，请重试"})", "application/json");
                    return;
                }

                bool enable = true;
                robotA->m_pMot->setGpEn(1, enable);

                std::cout << "机械臂A连接成功!" << std::endl;
            }

            // 连接机械臂A的激光
            if (!laserA) {
                LocPos LRA205;
                vector<vector<double>> cmatrixA = {
                    {1, 2, 3},
                    {4, 5, 6},
                    {7, 8, 9}
                };
                robotA->m_pVar->getLR(0, 205, LRA205);
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        cmatrixA[i][j] = LRA205.vecPos.at(i * 3 + j);
                    }
                }
                laserA = new LaserWorker(robotA, cmatrixA, "192.168.1.61");
            }

            // 连接机械臂B
            if (!robotB->m_pCmApi->isConnected()) {
                bool flag = robotB->connectRobot("192.168.1.72", 23234);
                if (!flag) {
                    res.status = 500;
                    res.set_content(u8R"({"status":"fail","message":"机械臂B连接失败，请重试"})", "application/json");
                    return;
                }

                bool enable = true;
                robotB->m_pMot->setGpEn(1, enable);

                std::cout << "机械臂B连接成功!" << std::endl;;
            }

            // 连接机械臂B的激光
            if (!laserB) {
                LocPos LRB205;
                vector<vector<double>> cmatrixB = {
                    {1, 2, 3},
                    {4, 5, 6},
                    {7, 8, 9}
                };
                robotB->m_pVar->getLR(0, 205, LRB205);
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        cmatrixB[i][j] = LRB205.vecPos.at(i * 3 + j);
                    }
                }
                laserB = new LaserWorker(robotB, cmatrixB, "192.168.1.62");
            }

            if (!serversStarted) {
                // 机械臂关节电流
                std::thread([currWorker]() { currWorker->startServer("127.0.0.1", 8081, 100); }).detach();

                // 机械臂关节角度
                std::thread([dtWorker]() { dtWorker->startServer("127.0.0.1", 8080, 100); }).detach();

                // Modbus PLC 数据推送 (端口 8083)
                modbusWorker->connectModbus("192.168.1.88", 502);
                std::thread([modbusWorker]() { modbusWorker->startServer("127.0.0.1", 8083, 200); }).detach();

                serversStarted = true;
            }

            res.set_content(u8R"({"status":"success","message":"机械臂连接成功"})", "application/json");
        }
        catch (const std::exception& e) {
            res.status = 500;
            res.set_content(u8R"({"status":"fail","message":"机械臂连接失败"})", "application/json");
        }
        });

    // 管径 -> 扫描程序匹配 (mm)，新程序在此处添加对应条目
    std::map<int, std::string> scanPrograms = {
        {300, "SCAN_300.PRG"},
        {500, "SCAN_500.PRG"},
        {800, "SCAN_800.PRG"},
    };
    auto nearestScanProgram = [&](int diameter) -> std::string {
        if (scanPrograms.empty()) return lastScanProgram;
        auto it = scanPrograms.lower_bound(diameter);
        if (it == scanPrograms.begin()) return it->second;
        if (it == scanPrograms.end()) return std::prev(it)->second;
        int dUpper = std::abs(it->first - diameter);
        int dLower = std::abs(std::prev(it)->first - diameter);
        return dLower <= dUpper ? std::prev(it)->second : it->second;
    };
    std::string lastScanProgram = "SCAN_1.PRG";

    // 获取距离
    svr.Post("/detect", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            res.set_header("Access-Control-Allow-Origin", "*");

            std::string diameter = req.get_header_value("X-Pipe-Diameter");
            std::string thickness = req.get_header_value("X-Pipe-Thickness");

            int pipeDiameter = 0;
            try { pipeDiameter = std::stoi(diameter); } catch (...) { pipeDiameter = 0; }
            lastScanProgram = nearestScanProgram(pipeDiameter);
            std::cout << "pipe: " << diameter << " " << thickness
                      << " -> matched: " << lastScanProgram << std::endl;

            // 1. 创建目录
            fs::create_directories("pic");

            // 2. 保存图片
            std::ofstream ofs("pic/001.jpg", std::ios::binary);
            ofs.write(req.body.data(), req.body.size());
            ofs.close();

            // 3. 计算距离
            double dis = Calculate::getDistance("pic/001.jpg");
            std::cout << "距离:" << dis << std::endl;
            if (dis == -1) {
                res.status = 500;
                res.set_content(u8R"({"status":"fail","message":"距离获取失败，请重试"})", "application/json");
                return;
            }

            // 4. 触发激光扫描
            if (robotA->m_pCmApi->isConnected() && laserA && robotB->m_pCmApi->isConnected() && laserB) {
                std::cout << "开始激光扫描" << std::endl;

                bool robotALoaded = false;
                bool robotBLoaded = false;

                // 卸载程序，防止无法加载程序
                robotA->m_pVm->isLoaded("RUN.PRG", robotALoaded);
                if (robotALoaded) {
                    robotA->m_pVm->unload("RUN.PRG");
                }
                robotA->m_pVm->isLoaded(lastScanProgram, robotALoaded);
                if (robotALoaded) {
                    robotA->m_pVm->unload(lastScanProgram);
                }

                robotB->m_pVm->isLoaded("RUN.PRG", robotBLoaded);
                if (robotBLoaded) {
                    robotB->m_pVm->unload("RUN.PRG");
                }
                robotB->m_pVm->isLoaded(lastScanProgram, robotBLoaded);
                if (robotBLoaded) {
                    robotB->m_pVm->unload(lastScanProgram);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                // 加载扫描程序
                robotA->m_pVm->load("/usr/codesys/hsc3_app/script/", lastScanProgram);
                robotB->m_pVm->load("/usr/codesys/hsc3_app/script/", lastScanProgram);

                // 将计算得到的距离写入工件坐标，用于扫描定位
                /*LocData posA;
                robotA->m_pMot->getWorkpiece(0, 1, posA);
                posA[1] += 530 - dis * 1000;
                robotA->m_pMot->setWorkpiece(0, 1, posA);*/
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                robotA->m_pVm->start(lastScanProgram);

                // 机械臂B延时10s，防止碰撞
                /*std::this_thread::sleep_for(std::chrono::milliseconds(10000));

                LocData posB;
                robotB->m_pMot->getWorkpiece(0, 1, posB);
                posB[1] += 530 - dis * 1000;
                robotB->m_pMot->setWorkpiece(0, 1, posB);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                robotB->m_pVm->start(lastScanProgram);*/

                laserA->cloud->clear();
                laserA->tsCloud.clear();
                laserB->cloud->clear();
                laserB->tsCloud.clear();
                laserA->asyncStart();
                //laserB->asyncStart();

                // 恢复原始工件坐标，防止下次焊接出问题
                /*posA[1] -= 530 - dis * 1000;
                robotA->m_pMot->setWorkpiece(0, 1, posA);*/

                res.status = 200;
                res.set_content(u8R"({"status":"success","message":"激光扫描已启动"})", "application/json");
            }
            else {
                res.status = 500;
                res.set_content(u8R"({"status":"fail","message":"请先连接机械臂"})", "application/json");
            }

        }
        catch (...) {
            res.status = 500;
            res.set_content(u8R"({"status":"fail","message":"save failed"})", "application/json");
        }
        });

    // 接收前端手动点的像素坐标，并触发激光扫描
    svr.Post("/detect/manual", [&](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        try {
            // 获取 Header 参数
            std::string diameter = req.get_header_value("X-Pipe-Diameter");
            std::string thickness = req.get_header_value("X-Pipe-Thickness");
            std::string pixelXStr = req.get_header_value("X-Pixel-X");
            std::string pixelYStr = req.get_header_value("X-Pixel-Y");

            int pipeDiameter = 0;
            try { pipeDiameter = std::stoi(diameter); } catch (...) { pipeDiameter = 0; }
            lastScanProgram = nearestScanProgram(pipeDiameter);
            std::cout << "pipe: " << diameter << " " << thickness
                      << " -> matched: " << lastScanProgram << std::endl;

            // 解析像素坐标
            double pixelX = 0, pixelY = 0;
            try { pixelX = std::stod(pixelXStr); } catch (...) { pixelX = 0; }
            try { pixelY = std::stod(pixelYStr); } catch (...) { pixelY = 0; }
            
            std::cout << "像素坐标: (" << pixelX << ", " << pixelY << ")" << std::endl;

            // 1. 创建目录
            fs::create_directories("pic");

            // 2. 保存图片
            std::ofstream ofs("pic/001.jpg", std::ios::binary);
            ofs.write(req.body.data(), req.body.size());
            ofs.close();

            std::cout << "image saved: pic/001.jpg, size: " << req.body.size() << " bytes" << std::endl;

            // Compute pipe distance via the manual depth service (1.py on port 8001)
            double dis = Calculate::getDistanceManual("pic/001.jpg", pixelX, pixelY);
            std::cout << "pipe distance: " << dis << " cm" << std::endl;

            if (dis == -1) {
                res.status = 400;
                res.set_content(R"({"status":"fail","message":"未找到可用图片，无法获取管道距离"})", "application/json");
                return;
            }

            // 触发激光扫描 (与 /detect 一致)
            if (robotA->m_pCmApi->isConnected() && laserA && robotB->m_pCmApi->isConnected() && laserB) {
                std::cout << "开始激光扫描(手动模式)" << std::endl;

                bool robotALoaded = false;
                bool robotBLoaded = false;

                // 卸载程序
                robotA->m_pVm->isLoaded("RUN.PRG", robotALoaded);
                if (robotALoaded) { robotA->m_pVm->unload("RUN.PRG"); }
                robotA->m_pVm->isLoaded(lastScanProgram, robotALoaded);
                if (robotALoaded) { robotA->m_pVm->unload(lastScanProgram); }

                robotB->m_pVm->isLoaded("RUN.PRG", robotBLoaded);
                if (robotBLoaded) { robotB->m_pVm->unload("RUN.PRG"); }
                robotB->m_pVm->isLoaded(lastScanProgram, robotBLoaded);
                if (robotBLoaded) { robotB->m_pVm->unload(lastScanProgram); }

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                // 加载扫描程序
                robotA->m_pVm->load("/usr/codesys/hsc3_app/script/", lastScanProgram);
                robotB->m_pVm->load("/usr/codesys/hsc3_app/script/", lastScanProgram);

                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                robotA->m_pVm->start(lastScanProgram);

                // 启动激光扫描
                laserA->cloud->clear();
                laserA->tsCloud.clear();
                laserB->cloud->clear();
                laserB->tsCloud.clear();
                laserA->asyncStart();

                res.status = 200;
                res.set_content(R"({"status":"success","message":"手动扫描已启动"})", "application/json");
            }
            else {
                res.status = 500;
                res.set_content(R"({"status":"fail","message":"机器人未连接"})", "application/json");
            }
        }
        catch (...) {
            res.status = 500;
            res.set_content(R"({"status":"fail","message":"处理异常"})", "application/json");
        }
        });


    // 检测焊接点并返回
    float widthA = 0;
    float widthB = 0;
    std::vector<weldStruct> weldA_final;
    std::vector<weldStruct> weldB_final;
    svr.Get("/weld", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            res.set_header("Access-Control-Allow-Origin", "*");

            if (/*!laserB->isOver ||*/ !laserA->isOver) {
                res.set_content(u8R"({"status":"fail","message":"请等待激光扫描完成"})", "application/json");
                return;
            }

            double target_z = 0;

            // --- 机械臂 A 侧 ---
            std::vector<TimestampedPoint> cleanA;
            std::vector<weldStruct> weldA;
            WeldRecongnition::loadAndProcess(laserA->tsCloud, target_z, cleanA, weldA, "A", widthA);

            // --- 机械臂 B 侧 ---
            std::vector<TimestampedPoint> cleanB;
            std::vector<weldStruct> weldB;
            WeldRecongnition::loadAndProcess(laserB->tsCloud, target_z, cleanB, weldB, "B", widthB);

            // --- 重叠剔除 (保留A侧焊接点) ---
            double overlap_dist = 10.0;
            std::vector<weldStruct> weldB_filtered = WeldRecongnition::filterOverlappingPoints(weldA, weldB, overlap_dist);

            Eigen::Vector3d centerA = WeldRecongnition::fitCircle3D(weldA);
            Eigen::Vector3d centerB = WeldRecongnition::fitCircle3D(weldB_filtered);

            std::vector<weldStruct> weldA_offset = WeldRecongnition::offsetAlongBendRadius(weldA, centerA, 3.0);
            std::vector<weldStruct> weldB_offset = WeldRecongnition::offsetAlongBendRadius(weldB_filtered, centerB, 3.0);

            int num_samples = 20;
            std::vector<weldStruct> weldA_resampled = WeldRecongnition::fitAndResamplePath(weldA_offset, num_samples);
            std::vector<weldStruct> weldB_resampled = WeldRecongnition::fitAndResamplePath(weldB_offset, num_samples);
            weldA_final = weldA_resampled;
            weldB_final = weldB_resampled;

            // --- 构造 JSON 返回 ---
            std::ostringstream json;
            json << "{\"status\":\"success\",";
            auto writePoints = [&](const std::string& key, const std::vector<TimestampedPoint>& pts, bool end) {
                json << "\"" << key << "\":[";
                for (size_t i = 0; i < pts.size(); ++i) {
                    json << "{\"x\":" << pts[i].x << ",\"y\":" << pts[i].y << ",\"z\":" << pts[i].z << "}";
                    if (i != pts.size() - 1) json << ",";
                }
                json << "]" << (end ? "" : ",");
                };

            auto writeWeld = [&](const std::string& key, const std::vector<weldStruct>& welds, bool end) {
                json << "\"" << key << "\":[";
                for (size_t i = 0; i < welds.size(); ++i) {
                    json << "{\"x\":" << welds[i].point.x << ",\"y\":" << welds[i].point.y << ",\"z\":" << welds[i].point.z << ",\"rx\":" << welds[i].rx << ",\"ry\":" << welds[i].ry << ",\"rz\":" << welds[i].rz << "}";
                    if (i != welds.size() - 1) json << ",";
                }
                json << "]" << (end ? "" : ",");
                };

            auto writeLR = [&]() {
                // 1. 固定寄存器范围
                const int32_t START_REG_INDEX = 11; // 起始寄存器号
                const int32_t POINTS_COUNT = 20;    // 写入点数

                int32_t configA = 0;
                robotA->m_pMot->getConfig(0, configA);

                // 2. 准备 LocPos 数据模板
                LocPos posData;
                posData.ufNum = 1;     // 用户号，默认为 1
                posData.utNum = 0;     // 工具号，默认为 0
                posData.config = configA;     // 配置字，通常为 0，根据具体 CONFIG 取值逻辑

                // 3. 遍历 weldA 并写入寄存器
                for (int i = 0; i < POINTS_COUNT && i < static_cast<int>(weldA_resampled.size()); ++i) {
                    const weldStruct& currentWeld = weldA_resampled[i];

                    // 清空之前的数据防止累积
                    posData.vecPos.clear();

                    // 4. 填充位置到 vecPos
                    // 位置 X, Y, Z
                    posData.vecPos.push_back(currentWeld.point.x);
                    posData.vecPos.push_back(currentWeld.point.y);
                    posData.vecPos.push_back(currentWeld.point.z);

                    // 姿态 Rx, Ry, Rz (对应 W, P, R)
                    posData.vecPos.push_back(currentWeld.rx);
                    posData.vecPos.push_back(currentWeld.ry);
                    posData.vecPos.push_back(currentWeld.rz);

                    // 5. 计算当前寄存器地址
                    int32_t currentIndex = START_REG_INDEX + i;

                    // 6. 调用接口写入
                    // 参数 gpId 固定为 0
                    Hsc3::Comm::HMCErrCode errCode = robotA->m_pVar->setLR(0, currentIndex, posData);

                    if (errCode == 0) { // 返回 0 表示成功，具体参见 HMCErrCode 定义
                        std::cout << "成功写入 LR[" << currentIndex << "]: ("
                            << currentWeld.point.x << ", "
                            << currentWeld.point.y << ", "
                            << currentWeld.point.z << ", "
                            << currentWeld.rx << ", "
                            << currentWeld.ry << ", "
                            << currentWeld.rz << ")" << std::endl;
                    }
                    else {
                        std::cerr << "写入 LR[" << currentIndex << "] 失败，错误码: " << errCode << std::endl;
                    }
                }
                };


            // 1. 写入 CloudA
            if (!cleanA.empty()) {
                writePoints("cloudA", cleanA, weldA_resampled.empty());
            }

            // 2. 写入 WeldA
            if (!weldA_resampled.empty()) {
                writeWeld("weldA", weldA_resampled, cleanB.empty());
            }

            // 3. 写入 CloudB
            if (!cleanB.empty()) {
                writePoints("cloudB", cleanB, weldB_resampled.empty());
            }

            // 4. 写入 WeldB
            if (!weldB_resampled.empty()) {
                writeWeld("weldB", weldB_resampled, true); // 最后一个字段，必须传 true
            }

            json << "}";

            // 将结果写入 LR 寄存器
            writeLR();
            res.set_content(json.str(), "application/json");

        }
        catch (...) {

            res.status = 500;
            res.set_content("{\"status\":\"fail\"}", "application/json");

        }
        });

    // 接收前端修改后的焊接点
    svr.Post("/weld/update", [&](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        try {
            std::string body = req.body;

            // JSON 解析焊接点数组
            auto parseWeldArray = [](const std::string& json, const std::string& key) -> std::vector<weldStruct> {
                std::vector<weldStruct> result;
                std::string searchKey = "\"" + key + "\":[";
                size_t start = json.find(searchKey);
                if (start == std::string::npos) return result;

                size_t pos = start + searchKey.length();
                while (pos < json.length()) {
                    size_t objStart = json.find('{', pos);
                    if (objStart == std::string::npos) break;
                    size_t objEnd = json.find('}', objStart);
                    if (objEnd == std::string::npos) break;

                    std::string obj = json.substr(objStart, objEnd - objStart + 1);

                    auto extractNum = [&](const std::string& field) -> double {
                        size_t f = obj.find("\"" + field + "\":");
                        if (f == std::string::npos) return 0.0;
                        f = obj.find(':', f) + 1;
                        size_t end = obj.find_first_of(",}", f);
                        return std::stod(obj.substr(f, end - f));
                    };

                    weldStruct ws;
                    ws.point.x = static_cast<float>(extractNum("x"));
                    ws.point.y = static_cast<float>(extractNum("y"));
                    ws.point.z = static_cast<float>(extractNum("z"));
                    ws.rx = extractNum("rx");
                    ws.ry = extractNum("ry");
                    ws.rz = extractNum("rz");
                    result.push_back(ws);

                    pos = objEnd + 1;
                    size_t next = json.find_first_not_of(" \t\n\r,", pos);
                    if (next == std::string::npos || json[next] == ']') break;
                    pos = next;
                }
                return result;
            };

            std::vector<weldStruct> newWeldA = parseWeldArray(body, "weldA");
            std::vector<weldStruct> newWeldB = parseWeldArray(body, "weldB");

            if (newWeldA.empty() && newWeldB.empty()) {
                res.status = 400;
                res.set_content(R"({"status":"fail","message":"未收到有效的焊接点数据"})", "application/json");
                return;
            }

            // 更新持久化数据
            if (!newWeldA.empty()) weldA_final = newWeldA;
            if (!newWeldB.empty()) weldB_final = newWeldB;

            // 写入 Robot A LR 寄存器
            if (!weldA_final.empty() && robotA->m_pCmApi->isConnected()) {
                const int32_t START_REG = 11;
                int32_t configA = 0;
                robotA->m_pMot->getConfig(0, configA);

                LocPos posData;
                posData.ufNum = 1;
                posData.utNum = 0;
                posData.config = configA;

                for (int i = 0; i < 20 && i < static_cast<int>(weldA_final.size()); ++i) {
                    const weldStruct& w = weldA_final[i];
                    posData.vecPos.clear();
                    posData.vecPos.push_back(w.point.x);
                    posData.vecPos.push_back(w.point.y);
                    posData.vecPos.push_back(w.point.z);
                    posData.vecPos.push_back(w.rx);
                    posData.vecPos.push_back(w.ry);
                    posData.vecPos.push_back(w.rz);
                    robotA->m_pVar->setLR(0, START_REG + i, posData);
                }
                std::cout << "已更新 Robot A 焊接点: " << weldA_final.size() << " 个" << std::endl;
            }

            // 写入 Robot B LR 寄存器
            if (!weldB_final.empty() && robotB->m_pCmApi->isConnected()) {
                const int32_t START_REG = 11;
                int32_t configB = 0;
                robotB->m_pMot->getConfig(0, configB);

                LocPos posData;
                posData.ufNum = 1;
                posData.utNum = 0;
                posData.config = configB;

                for (int i = 0; i < 20 && i < static_cast<int>(weldB_final.size()); ++i) {
                    const weldStruct& w = weldB_final[i];
                    posData.vecPos.clear();
                    posData.vecPos.push_back(w.point.x);
                    posData.vecPos.push_back(w.point.y);
                    posData.vecPos.push_back(w.point.z);
                    posData.vecPos.push_back(w.rx);
                    posData.vecPos.push_back(w.ry);
                    posData.vecPos.push_back(w.rz);
                    robotB->m_pVar->setLR(0, START_REG + i, posData);
                }
                std::cout << "已更新 Robot B 焊接点: " << weldB_final.size() << " 个" << std::endl;
            }

            res.set_content(R"({"status":"success","message":"焊接点已更新"})", "application/json");
        }
        catch (...) {
            res.status = 500;
            res.set_content(R"({"status":"fail","message":"焊接点更新失败"})", "application/json");
        }
        });


    // 设置焊接参数并启动焊接程序
    svr.Get("/start", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            res.set_header("Access-Control-Allow-Origin", "*");

            std::string current = req.get_header_value("X-Current");
            std::string voltage = req.get_header_value("X-Voltage");
            std::string speed = req.get_header_value("X-Speed");
            std::string Frequency = req.get_header_value("X-Frequency");
            std::string vibrateFrequency = req.get_header_value("X-Vibrate-Frequency");
            std::string stayTime = req.get_header_value("X-Stay-Time");

            // 设置焊接工艺参数，主要修改摆动宽度
            // ====================================robotA====================================
            Hsc3::Comm::CommApi apiA("");
            apiA.connect("192.168.1.71", 23234);
            std::string strCmdA = "arcWaveWeld.get_WaveChannel(1)";
            std::string strRetA;
            apiA.execCmd(strCmdA, strRetA, Hsc3::Comm::PRIORITY_HIGH);

            std::vector<std::string> partsA;
            std::stringstream ssA(strRetA);
            std::string itemA;

            // 1. 分割字符串，使用 getline 按逗号分割
            while (std::getline(ssA, itemA, ',')) {
                partsA.push_back(itemA);
            }

            // 2. 替换摆动宽度值（第4个位置，索引为3）
            if (partsA.size() > 3) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2) << widthA / 2.0;
                partsA[3] = oss.str();
            }

            // 3. 重新拼接字符串
            std::string resultA;
            for (size_t i = 0; i < partsA.size(); ++i) {
                if (i > 0) resultA += ",";
                resultA += partsA[i];
            }
            // 写入焊接工艺参数
            strCmdA = "arcWaveWeld.modify_WaveChannel(1, " + resultA + ")";
            apiA.execCmd(strCmdA, strRetA, Hsc3::Comm::PRIORITY_HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            // ====================================robotB====================================
            Hsc3::Comm::CommApi apiB("");
            apiA.connect("192.168.1.72", 23234);
            std::string strCmdB = "arcWaveWeld.get_WaveChannel(1)";
            std::string strRetB;
            apiA.execCmd(strCmdB, strRetB, Hsc3::Comm::PRIORITY_HIGH);

            std::vector<std::string> partsB;
            std::stringstream ssB(strRetB);
            std::string itemB;

            // 1. 分割字符串，使用 getline 按逗号分割
            while (std::getline(ssB, itemB, ',')) {
                partsB.push_back(itemB);
            }

            // 2. 替换摆动宽度值（第4个位置）
            if (partsB.size() > 3) {
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(2) << widthB / 2.0;
                partsB[3] = oss.str();
            }

            // 3. 重新拼接字符串
            std::string resultB;
            for (size_t i = 0; i < partsB.size(); ++i) {
                if (i > 0) resultB += ",";
                resultB += partsB[i];
            }

            strCmdB = "arcWaveWeld.modify_WaveChannel(1, " + resultB + ")";
            apiB.execCmd(strCmdB, strRetB, Hsc3::Comm::PRIORITY_HIGH);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            bool robotALoaded = false;
            bool robotBLoaded = false;

            // 卸载程序，防止无法加载程序
            robotA->m_pVm->isLoaded("RUN.PRG", robotALoaded);
            if (robotALoaded) {
                robotA->m_pVm->unload("RUN.PRG");
            }
            robotA->m_pVm->isLoaded(lastScanProgram, robotALoaded);
            if (robotALoaded) {
                robotA->m_pVm->unload(lastScanProgram);
            }

            robotB->m_pVm->isLoaded("RUN.PRG", robotBLoaded);
            if (robotBLoaded) {
                robotB->m_pVm->unload("RUN.PRG");
            }
            robotB->m_pVm->isLoaded(lastScanProgram, robotBLoaded);
            if (robotBLoaded) {
                robotB->m_pVm->unload(lastScanProgram);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            // 加载焊接程序
            robotA->m_pVm->load("/usr/codesys/hsc3_app/script/", "RUN.PRG");
            robotB->m_pVm->load("/usr/codesys/hsc3_app/script/", "RUN.PRG");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            // 设置焊接进行中标志 (RUN.PRG 完成后需将此寄存器置 0)
            robotA->m_pVar->setR(52, 1);
            robotA->m_pVm->start("RUN.PRG");

            /*std::this_thread::sleep_for(std::chrono::milliseconds(10000));
            robotB->m_pVm->start("RUN.PRG");*/

            // 等待焊接完成: 轮询 R[52]，RUN.PRG 结束时置 R[52]=0
            double weldStatus = 1;
            int maxWait = 3000;  // 超时 300 秒 (100ms * 3000)
            int waited = 0;
            while (weldStatus != 0 && waited < maxWait) {
                robotA->m_pVar->getR(52, weldStatus);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                waited++;
            }

            if (waited >= maxWait) {
                res.status = 500;
                res.set_content(R"({"status":"fail","message":"焊接超时，请检查机器人状态"})", "application/json");
                return;
            }

            std::cout << "焊接完成，总耗时约 " << (waited * 100 / 1000) << " 秒" << std::endl;
            res.set_content(R"({"status":"success","message":"焊接完成"})", "application/json");
        }
        catch (...) {
            res.status = 500;
            res.set_content("{\"status\":\"fail\"}", "application/json");
        }
        });

    printf("Server start\n");
    svr.listen("0.0.0.0", 8082);
}