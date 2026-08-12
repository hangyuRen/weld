#pragma once
#ifndef CURRENTSERVER_H
#define CURRENTSERVER_H

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

class CurrentServerWorker {

public:
    explicit CurrentServerWorker();
    ~CurrentServerWorker();

    // 在 UI 线程调用 -> 通过 QueuedConnection 发到 worker 线程执行
    void startServer(const std::string& ip, int port, int intervalMs = 200);
    void stopServer();
    // 立即发送一次自定义 joint json
    void sendOnce(const std::string& currentJson);

private:
    // 禁止复制
    CurrentServerWorker(const CurrentServerWorker&) = delete;
    CurrentServerWorker& operator=(const CurrentServerWorker&) = delete;

    // PIMPL-like 简化：把原有的 asio / session / broadcaster 放在这里实现（cpp）
    struct Impl;
    Impl* impl_;
};

#endif // CURRENTSERVER_H