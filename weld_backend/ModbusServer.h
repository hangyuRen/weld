#pragma once
#ifndef MODBUS_SERVER_H
#define MODBUS_SERVER_H

#include <atomic>
#include <memory>
#include <string>

class ModbusServerWorker {
public:
    explicit ModbusServerWorker();
    ~ModbusServerWorker();

    void startServer(const std::string& ip, int port, int intervalMs = 200);
    void stopServer();

    // Modbus device connection (PLC at 192.168.1.88:502 by default)
    void connectModbus(const std::string& ip, int port);
    bool isModbusConnected() const;

private:
    ModbusServerWorker(const ModbusServerWorker&) = delete;
    ModbusServerWorker& operator=(const ModbusServerWorker&) = delete;

    struct Impl;
    Impl* impl_;
};

#endif // MODBUS_SERVER_H
