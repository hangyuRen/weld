#pragma once
#ifndef MODBUS_TOOLS_H
#define MODBUS_TOOLS_H

#include "modbus.h"

class Modbus {
private:
    //// D300~D399
    //const static int START_D_ADDR = 300;
    //const static int END_D_ADDR = 399;
    //const static int READ_D_COUNT = END_D_ADDR - START_D_ADDR + 1;
    //uint16_t d_regs[READ_D_COUNT];

    //// M300~M399
    //const static int START_M_ADDR = 300;
    //const static int END_M_ADDR = 399;
    //const static int READ_M_COUNT = END_M_ADDR - START_M_ADDR + 1;
    //uint8_t m_regs[READ_M_COUNT];

    modbus_t* ctx = nullptr;

public:
    Modbus();

    // 连接设备
    int connect(const char* ip, int port);

    // 批量读取D寄存器
    void read_registers(int start_addr, int count, uint16_t* buf);

    // 批量写D寄存器
    void write_registers(int start_addr, int count, uint16_t* buf);

    // 读取单个D寄存器
    void read_register(int addr, uint16_t* value);

    // 写单个D寄存器
    void write_register(int addr, uint16_t value);

    // 批量读取M寄存器
    void read_bits(int start_addr, int count, uint8_t* buf);

    // 批量写M寄存器
    void write_bits(int start_addr, int count, uint8_t* buf);

    // 读取单个M寄存器
    void read_bit(int addr, uint8_t* value);

    // 写单个M寄存器
    void write_bit(int addr, uint8_t value);
};

#endif