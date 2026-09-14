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
#include <mutex>

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

// =====================================================================
// 算法二启动辅助：把“当前保存的点”(可能是 左+右 拼接、且前端增删过) 用 /weld2
// 的初始左右点(initL/initR)作参照，复原每个点的左右属性与“同一条截面”配对序号；
// 只保留左右点都还在的配对，按序号升序最多取 maxPairs 对（必含首尾），
// 输出按 左0,右0,左1,右1,… 交错排列的点列，并给出平均槽宽 avgWidth。
// =====================================================================
static std::vector<weldStruct> pairLeftRightInterleave(
    const std::vector<weldStruct>& cur,
    const std::vector<weldStruct>& initL,
    const std::vector<weldStruct>& initR,
    int maxPairs,
    double& avgWidth)
{
    avgWidth = 0.0;
    if (initL.empty() || initR.empty()) return {};

    // 前端没传回任何点时，退化为初始点对本身
    std::vector<weldStruct> points = cur;
    if (points.empty()) {
        points = initL;
        points.insert(points.end(), initR.begin(), initR.end());
    }

    const int nL = static_cast<int>(initL.size());
    const int nR = static_cast<int>(initR.size());
    const int nIdx = (nL > nR ? nL : nR); // 配对序号范围 [0, nIdx)

    // 参照点：0..nL-1 为左，nL..nL+nR-1 为右
    struct Ref { double x, y, z; bool isLeft; int idx; };
    std::vector<Ref> refs;
    refs.reserve(nL + nR);
    for (int i = 0; i < nL; ++i) refs.push_back({ initL[i].point.x, initL[i].point.y, initL[i].point.z, true, i });
    for (int i = 0; i < nR; ++i) refs.push_back({ initR[i].point.x, initR[i].point.y, initR[i].point.z, false, i });

    // best[i] 记录第 i 个参照点命中最贴近的“当前点”下标（-1 表示无）
    std::vector<int> bestL(nIdx, -1), bestR(nIdx, -1);
    std::vector<double> bestLd(nIdx, 1e30), bestRd(nIdx, 1e30);
    for (size_t pi = 0; pi < points.size(); ++pi) {
        const auto& p = points[pi];
        double bestD = 1e30;
        const Ref* bestRef = nullptr;
        for (const auto& r : refs) {
            double dx = p.point.x - r.x, dy = p.point.y - r.y, dz = p.point.z - r.z;
            double d = dx * dx + dy * dy + dz * dz;
            if (d < bestD) { bestD = d; bestRef = &r; }
        }
        if (!bestRef) continue;
        if (bestRef->isLeft) {
            if (bestD < bestLd[bestRef->idx]) { bestLd[bestRef->idx] = bestD; bestL[bestRef->idx] = static_cast<int>(pi); }
        }
        else {
            if (bestD < bestRd[bestRef->idx]) { bestRd[bestRef->idx] = bestD; bestR[bestRef->idx] = static_cast<int>(pi); }
        }
    }

    // 收集左右都还在的配对（升序）
    std::vector<std::pair<int, int>> pairs; // (idx of left in points, idx of right in points)
    for (int i = 0; i < nIdx; ++i) {
        if (bestL[i] >= 0 && bestR[i] >= 0) pairs.push_back({ bestL[i], bestR[i] });
    }
    if (pairs.empty()) return {};

    // 平均槽宽：配对左右点间距均值
    {
        double sum = 0.0;
        for (const auto& pr : pairs) {
            const weldStruct& a = points[pr.first];
            const weldStruct& b = points[pr.second];
            double dx = a.point.x - b.point.x, dy = a.point.y - b.point.y, dz = a.point.z - b.point.z;
            sum += std::sqrt(dx * dx + dy * dy + dz * dz);
        }
        avgWidth = sum / static_cast<double>(pairs.size());
    }

    // 最多取 maxPairs 对，且必含首、末对
    std::vector<size_t> sel; // 指向 pairs 的下标
    int total = static_cast<int>(pairs.size());
    if (total <= maxPairs) {
        for (int i = 0; i < total; ++i) sel.push_back(static_cast<size_t>(i));
    }
    else {
        if (maxPairs <= 1) {
            sel.push_back(0);
        }
        else {
            for (int k = 0; k < maxPairs; ++k) {
                double pos = static_cast<double>(k) * static_cast<double>(total - 1) / static_cast<double>(maxPairs - 1);
                sel.push_back(static_cast<size_t>(std::lround(pos)));
            }
            // 去重并保证单调
            std::vector<size_t> uni;
            for (size_t v : sel) {
                if (uni.empty() || uni.back() != v) uni.push_back(v);
            }
            // 强制首尾
            uni.front() = 0;
            uni.back() = static_cast<size_t>(total - 1);
            sel = uni;
        }
    }

    // 交错输出：左,右,左,右…
    std::vector<weldStruct> out;
    out.reserve(sel.size() * 2);
    for (size_t s : sel) {
        out.push_back(points[pairs[s].first]);
        out.push_back(points[pairs[s].second]);
    }
    return out;
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
    std::mutex robotCommandMutex;

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

    // 急停：调用 robot SDK 的 setEstop(true) 停止 A、B 两臂
    svr.Post("/estop", [&](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        try {
            bool anyConnected = false;
            bool allOk = true;

            if (robotA->m_pCmApi->isConnected()) {
                anyConnected = true;
                if (robotA->m_pMot->setEstop(true) != 0) allOk = false;
            }
            if (robotB->m_pCmApi->isConnected()) {
                anyConnected = true;
                if (robotB->m_pMot->setEstop(true) != 0) allOk = false;
            }

            if (!anyConnected) {
                res.status = 500;
                res.set_content(u8R"({"status":"fail","message":"请先连接机械臂"})", "application/json");
            }
            else if (!allOk) {
                res.status = 500;
                res.set_content(u8R"({"status":"fail","message":"急停触发失败"})", "application/json");
            }
            else {
                res.set_content(u8R"({"status":"success","message":"急停已触发"})", "application/json");
            }
        }
        catch (...) {
            res.status = 500;
            res.set_content(u8R"({"status":"fail","message":"急停触发异常"})", "application/json");
        }
        });

    // 取消急停：调用 robot SDK 的 setEstop(false) 释放 A、B 两臂
    svr.Post("/estop/cancel", [&](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        try {
            bool anyConnected = false;
            bool allOk = true;

            if (robotA->m_pCmApi->isConnected()) {
                anyConnected = true;
                if (robotA->m_pMot->setEstop(false) != 0) allOk = false;
            }
            if (robotB->m_pCmApi->isConnected()) {
                anyConnected = true;
                if (robotB->m_pMot->setEstop(false) != 0) allOk = false;
            }

            if (!anyConnected) {
                res.status = 500;
                res.set_content(u8R"({"status":"fail","message":"请先连接机械臂"})", "application/json");
            }
            else if (!allOk) {
                res.status = 500;
                res.set_content(u8R"({"status":"fail","message":"取消急停失败"})", "application/json");
            }
            else {
                res.set_content(u8R"({"status":"success","message":"急停已取消"})", "application/json");
            }
        }
        catch (...) {
            res.status = 500;
            res.set_content(u8R"({"status":"fail","message":"取消急停异常"})", "application/json");
        }
        });

    // 将指定机械臂以低速关节运动复位到扫描程序的初始点。
    // moveTo是异步下发运动命令；接口成功表示控制器已经接受命令。
    auto resetRobotToJoint = [&](robotConnect* robot,
                                 const GeneralPos& target,
                                 const char* robotName,
                                 httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");

        std::lock_guard<std::mutex> commandLock(robotCommandMutex);

        if (!robot || !robot->m_pCmApi->isConnected()) {
            res.status = 500;
            res.set_content(
                std::string("{\"status\":\"fail\",\"message\":\"") +
                robotName + "未连接\"}",
                "application/json"
            );
            return;
        }

        bool estopped = false;
        Hsc3::Comm::HMCErrCode ret = robot->m_pMot->getEstop(estopped);
        if (ret != 0 || estopped) {
            res.status = 409;
            res.set_content(
                std::string("{\"status\":\"fail\",\"message\":\"") +
                robotName + "处于急停状态，请先取消急停\"}",
                "application/json"
            );
            return;
        }

        bool moving = false;
        ret = robot->m_pMot->isMovingStatus(0, moving);
        if (ret != 0) {
            res.status = 500;
            res.set_content(
                std::string("{\"status\":\"fail\",\"message\":\"无法读取") +
                robotName + "运动状态\"}",
                "application/json"
            );
            return;
        }
        if (moving) {
            res.status = 409;
            res.set_content(
                std::string("{\"status\":\"fail\",\"message\":\"") +
                robotName + "正在运动，不能执行复位\"}",
                "application/json"
            );
            return;
        }

        // 参考SDK Training_Move示例：T1、关节坐标系、低倍率、组使能。
        if (robot->m_pMot->setOpMode(OP_T1) != 0 ||
            robot->m_pMot->setWorkFrame(0, FRAME_JOINT) != 0 ||
            robot->m_pMot->setJogVord(5) != 0 ||
            robot->m_pMot->setGpEn(0, true) != 0) {
            res.status = 500;
            res.set_content(
                std::string("{\"status\":\"fail\",\"message\":\"") +
                robotName + "复位前置设置失败\"}",
                "application/json"
            );
            return;
        }

        ret = robot->m_pMot->moveTo(0, target, false);
        if (ret != 0) {
            std::cout << robotName << "复位命令失败，错误码: " << ret << std::endl;
            res.status = 500;
            res.set_content(
                std::string("{\"status\":\"fail\",\"message\":\"") +
                robotName + "复位命令下发失败，错误码: " +
                std::to_string(ret) + "\"}",
                "application/json"
            );
            return;
        }

        std::cout << robotName << "复位命令已下发" << std::endl;
        res.set_content(
            std::string("{\"status\":\"success\",\"message\":\"") +
            robotName + "正在复位\"}",
            "application/json"
        );
    };

    // 机械臂A复位到扫描程序P[9]。
    svr.Post("/robot/a/reset", [&](const httplib::Request&, httplib::Response& res) {
        GeneralPos target{};
        target.isJoint = true;
        target.ufNum = -1;
        target.utNum = 0;
        target.config = 0;
        target.vecPos = {
            0.0026332, -178.568808, 229.6947944,
            0.0055652, 89.9972763, 0.0028015,
            0.0, 0.0, 0.0
        };
        resetRobotToJoint(robotA, target, "机械臂A", res);
    });

    // 机械臂B复位到扫描程序P[7]。
    svr.Post("/robot/b/reset", [&](const httplib::Request&, httplib::Response& res) {
        GeneralPos target{};
        target.isJoint = true;
        target.ufNum = 1;
        target.utNum = 0;
        target.config = 0;
        target.vecPos = {
            0.0015435, -179.9989531, 229.2422257,
            0.0053558, 89.9975322, -0.0073609,
            0.0, 0.0, 0.0
        };
        resetRobotToJoint(robotB, target, "机械臂B", res);
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
                LocData posA;
                robotA->m_pMot->getWorkpiece(0, 1, posA);
                posA[1] += 530 - dis * 1000;
                robotA->m_pMot->setWorkpiece(0, 1, posA);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                robotA->m_pVm->start(lastScanProgram);

                // 机械臂B延时10s，防止碰撞
                std::this_thread::sleep_for(std::chrono::milliseconds(10000));

                LocData posB;
                robotB->m_pMot->getWorkpiece(0, 1, posB);
                posB[1] += 530 - dis * 1000;
                robotB->m_pMot->setWorkpiece(0, 1, posB);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                robotB->m_pVm->start(lastScanProgram);

                laserA->cloud->clear();
                laserA->tsCloud.clear();
                laserB->cloud->clear();
                laserB->tsCloud.clear();
                laserA->asyncStart();
                laserB->asyncStart();

                // 恢复原始工件坐标，防止下次焊接出问题
                posA[1] -= 530 - dis * 1000;
                robotA->m_pMot->setWorkpiece(0, 1, posA);
                posB[1] -= 530 - dis * 1000;
                robotB->m_pMot->setWorkpiece(0, 1, posB);

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

            // Compute pipe distance via the manual depth service (2.py on port 8000, path /detect-distance/manual)
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

                // 将计算得到的距离写入工件坐标，用于扫描定位
                LocData posA;
                robotA->m_pMot->getWorkpiece(0, 1, posA);
                posA[1] += 530 - dis * 1000;
                robotA->m_pMot->setWorkpiece(0, 1, posA);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                robotA->m_pVm->start(lastScanProgram);

                // 机械臂B延时10s，防止碰撞
                std::this_thread::sleep_for(std::chrono::milliseconds(10000));

                LocData posB;
                robotB->m_pMot->getWorkpiece(0, 1, posB);
                posB[1] += 530 - dis * 1000;
                robotB->m_pMot->setWorkpiece(0, 1, posB);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
                robotB->m_pVm->start(lastScanProgram);

                laserA->cloud->clear();
                laserA->tsCloud.clear();
                laserB->cloud->clear();
                laserB->tsCloud.clear();
                laserA->asyncStart();
                laserB->asyncStart();

                // 恢复原始工件坐标，防止下次焊接出问题
                posA[1] -= 530 - dis * 1000;
                robotA->m_pMot->setWorkpiece(0, 1, posA);
                posB[1] -= 530 - dis * 1000;
                robotB->m_pMot->setWorkpiece(0, 1, posB);

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

    // 当前筛选算法：1 = /weld（算法一，焊缝最低点单线），2 = /weld2（算法二，左右两边点）
    int weldMode = 1;
    // 算法二初始左右点：作为配对参照（“同一截面”的左右必须同属一条焊缝）
    std::vector<weldStruct> initLeftA, initRightA;
    std::vector<weldStruct> initLeftB, initRightB;
    svr.Get("/weld", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            res.set_header("Access-Control-Allow-Origin", "*");

            if (!laserB->isOver || !laserA->isOver) {
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
            weldMode = 1;   // 算法一

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

            auto writeLR = [&](robotConnect* robot, const std::vector<weldStruct>& welds) {
                // 1. 固定寄存器范围
                const int32_t START_REG_INDEX = 11; // 起始寄存器号
                const int32_t POINTS_COUNT = 20;    // 写入点数

                int32_t config = 0;
                robot->m_pMot->getConfig(0, config);

                // 2. 准备 LocPos 数据模板
                LocPos posData;
                posData.ufNum = 1;     // 用户号，默认为 1
                posData.utNum = 0;     // 工具号，默认为 0
                posData.config = config;     // 配置字，通常为 0，根据具体 CONFIG 取值逻辑

                // 3. 遍历焊接点并写入寄存器
                for (int i = 0; i < POINTS_COUNT && i < static_cast<int>(welds.size()); ++i) {
                    const weldStruct& currentWeld = welds[i];

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
                    Hsc3::Comm::HMCErrCode errCode = robot->m_pVar->setLR(0, currentIndex, posData);

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

            // 将结果写入 LR 寄存器（机械臂 A、B 分别写入）
            writeLR(robotA, weldA_resampled);
            writeLR(robotB, weldB_resampled);
            res.set_content(json.str(), "application/json");

        }
        catch (...) {

            res.status = 500;
            res.set_content("{\"status\":\"fail\"}", "application/json");

        }
        });

    // 点云扫描状态轮询接口（轻量，只判断两侧激光是否扫描完成，不计算焊缝、不写寄存器）
    svr.Get("/weld/status", [&](const httplib::Request& req, httplib::Response& res) {
        res.set_header("Access-Control-Allow-Origin", "*");
        try {
            if (laserB && laserA && laserB->isOver && laserA->isOver) {
                res.set_content(u8R"({"status":"success","message":"点云扫描完成"})", "application/json");
            }
            else {
                res.set_content(u8R"({"status":"pending","message":"激光扫描中，请稍候"})", "application/json");
            }
        }
        catch (...) {
            res.status = 500;
            res.set_content("{\"status\":\"fail\"}", "application/json");
        }
        });

    // 算法二：detectWeldLeftRight（按扫描线检测坡口左右拐角）
    svr.Get("/weld2", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            res.set_header("Access-Control-Allow-Origin", "*");

            if (!laserB->isOver || !laserA->isOver) {
                res.set_content(u8R"({"status":"fail","message":"请等待激光扫描完成"})", "application/json");
                return;
            }

            // 算法二：按扫描线检测坡口左右拐角
            const int num_samples = 20;
            std::vector<weldStruct> leftA, rightA, leftB, rightB;
            WeldRecongnition::detectWeldLeftRight(laserA->tsCloud, leftA, rightA, num_samples);
            WeldRecongnition::detectWeldLeftRight(laserB->tsCloud, leftB, rightB, num_samples);

            // 记录初始左右点对（前端可能随后编辑保存，用于 /start2 复原配对）
            weldMode = 2;   // 算法二
            initLeftA = leftA;  initRightA = rightA;
            initLeftB = leftB;  initRightB = rightB;

            // 合并左右焊缝点到 weldA / weldB，保持与 /weld 相同的四字段结构
            std::vector<weldStruct> weldA, weldB;
            weldA.insert(weldA.end(), leftA.begin(), leftA.end());
            weldA.insert(weldA.end(), rightA.begin(), rightA.end());
            weldB.insert(weldB.end(), leftB.begin(), leftB.end());
            weldB.insert(weldB.end(), rightB.begin(), rightB.end());

            // 构造 JSON 返回（与 /weld 结构一致；此处不写机器人 LR 寄存器）
            std::ostringstream json;
            json << "{\"status\":\"success\",";

            auto writePoints = [&](const std::string& key, const std::vector<TimestampedPoint>& pts) {
                json << "\"" << key << "\":[";
                for (size_t i = 0; i < pts.size(); ++i) {
                    json << "{\"x\":" << pts[i].x << ",\"y\":" << pts[i].y << ",\"z\":" << pts[i].z << "}";
                    if (i != pts.size() - 1) json << ",";
                }
                json << "],";
            };

            auto writeWeld = [&](const std::string& key, const std::vector<weldStruct>& welds) {
                json << "\"" << key << "\":[";
                for (size_t i = 0; i < welds.size(); ++i) {
                    json << "{\"x\":" << welds[i].point.x << ",\"y\":" << welds[i].point.y << ",\"z\":" << welds[i].point.z
                         << ",\"rx\":" << welds[i].rx << ",\"ry\":" << welds[i].ry << ",\"rz\":" << welds[i].rz << "}";
                    if (i != welds.size() - 1) json << ",";
                }
                json << "]";
            };

            writePoints("cloudA", laserA->tsCloud);
            writePoints("cloudB", laserB->tsCloud);
            writeWeld("weldA", weldA);
            json << ",";
            writeWeld("weldB", weldB);

            json << "}";
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
            apiB.connect("192.168.1.72", 23234);
            std::string strCmdB = "arcWaveWeld.get_WaveChannel(1)";
            std::string strRetB;
            apiB.execCmd(strCmdB, strRetB, Hsc3::Comm::PRIORITY_HIGH);

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

            // 机械臂B延时10s，防止碰撞
            std::this_thread::sleep_for(std::chrono::milliseconds(10000));
            robotB->m_pVar->setR(52, 1);
            robotB->m_pVm->start("RUN.PRG");

            // 等待焊接完成: 轮询 R[52]，RUN.PRG 结束时置 R[52]=0
            double weldStatusA = 1;
            double weldStatusB = 1;
            int maxWait = 6000;  // 超时 600 秒 (100ms * 6000)
            int waited = 0;
            while ((weldStatusA != 0 || weldStatusB != 0) && waited < maxWait) {
                robotA->m_pVar->getR(52, weldStatusA);
                robotB->m_pVar->getR(52, weldStatusB);
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

    // =====================================================================
    // 算法二启动：把 /weld/update 保存的“左+右”点按同一截面配对、交错写成焊接
    // 轨迹（最多 20 对、必含首尾对）写入 LR，再设置工艺参数并启动焊接程序。
    // 与 /start(算法一) 的区别：算法一保存的就是单线最低点可直接进 LR；
    // 算法二保存的是左右两边点，需先配对成交错轨迹再执行。
    // =====================================================================
    svr.Get("/start2", [&](const httplib::Request& req, httplib::Response& res) {
        try {
            res.set_header("Access-Control-Allow-Origin", "*");

            if (weldMode != 2) {
                res.status = 400;
                res.set_content(u8R"({"status":"fail","message":"当前数据不是算法二（左右点），请先在焊缝特征点筛选中选择算法二"})", "application/json");
                return;
            }

            // 1) 左右配对 + 交错（每臂最多 20 对 -> <=40 点），返回平均槽宽
            double widthA2 = 0.0, widthB2 = 0.0;
            std::vector<weldStruct> pathA = pairLeftRightInterleave(weldA_final, initLeftA, initRightA, 20, widthA2);
            std::vector<weldStruct> pathB = pairLeftRightInterleave(weldB_final, initLeftB, initRightB, 20, widthB2);

            if (pathA.empty() && pathB.empty()) {
                res.status = 400;
                res.set_content(u8R"({"status":"fail","message":"没有可用的左右配对点，请先进行算法二焊缝特征点筛选"})", "application/json");
                return;
            }

            // 2) 将交错轨迹写入 LR 寄存器（LR[11] 起，每臂最多 40 点）
            auto writePath = [&](robotConnect* robot, const std::vector<weldStruct>& path, const char* tag) {
                if (path.empty() || !robot || !robot->m_pCmApi->isConnected()) return;
                const int32_t START_REG = 11;
                int32_t config = 0;
                robot->m_pMot->getConfig(0, config);

                LocPos posData;
                posData.ufNum = 1;
                posData.utNum = 0;
                posData.config = config;

                for (int i = 0; i < static_cast<int>(path.size()); ++i) {
                    const weldStruct& w = path[i];
                    posData.vecPos.clear();
                    posData.vecPos.push_back(w.point.x);
                    posData.vecPos.push_back(w.point.y);
                    posData.vecPos.push_back(w.point.z);
                    posData.vecPos.push_back(w.rx);
                    posData.vecPos.push_back(w.ry);
                    posData.vecPos.push_back(w.rz);
                    robot->m_pVar->setLR(0, START_REG + i, posData);
                }
                std::cout << "算法二写入 " << tag << " LR 焊接点: " << path.size() << " 个" << std::endl;
            };
            writePath(robotA, pathA, "RobotA");
            writePath(robotB, pathB, "RobotB");
            
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

            // 机械臂B延时10s，防止碰撞
            std::this_thread::sleep_for(std::chrono::milliseconds(10000));
            robotB->m_pVar->setR(52, 1);
            robotB->m_pVm->start("RUN.PRG");

            // 等待焊接完成: 轮询 R[52]，RUN.PRG 结束时置 R[52]=0
            double weldStatusA = 1;
            double weldStatusB = 1;
            int maxWait = 6000;  // 超时 600 秒 (100ms * 6000)
            int waited = 0;
            while ((weldStatusA != 0 || weldStatusB != 0) && waited < maxWait) {
                robotA->m_pVar->getR(52, weldStatusA);
                robotB->m_pVar->getR(52, weldStatusB);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                waited++;
            }

            if (waited >= maxWait) {
                res.status = 500;
                res.set_content(R"({"status":"fail","message":"焊接超时，请检查机器人状态"})", "application/json");
                return;
            }

            std::cout << "算法二焊接完成，总耗时约 " << (waited * 100 / 1000) << " 秒" << std::endl;
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
