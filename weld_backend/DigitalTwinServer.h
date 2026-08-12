#pragma once
#ifndef WSSERVER_H
#define WSSERVER_H
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <atomic>
#include <memory>
#include <mutex>
#include <iostream>
#include "process.h"
#include "CommApi.h"
#include "proxy/ProxyMotion.h"
#include <algorithm>
#include <sstream>
#include <vector>
#include <string>
#include <thread>
#include <iomanip>
#include <chrono>
#include <set>
#include <deque>
#include <cmath>
#include <Windows.h>

// 你的 JointInfo 定义（与原来一致）
struct JointInfo {
    std::string name;
    double lower;
    double upper;
    double degree;
};

class DigitalTwinServer {
public:
    explicit DigitalTwinServer();
    ~DigitalTwinServer();

    void startServer(const std::string& ip, int port, int intervalMs = 200);
    void stopServer();
    // 立即发送一次自定义 joint json
    void sendOnce(const std::string& jointsJson);


private:
    // 禁止复制
    DigitalTwinServer(const DigitalTwinServer&) = delete;
    DigitalTwinServer& operator=(const DigitalTwinServer&) = delete;

    struct Impl;
    Impl* impl_;
};
#endif // WSSERVER_H