#include "ModbusTools.h"

Modbus::Modbus()
{
}

int Modbus::connect(const char* ip, int port)
{
    this->ctx = modbus_new_tcp(ip, port);

    if (ctx == nullptr) {
        return -1;
    }

    if (modbus_connect(ctx) == -1) {
        modbus_free(ctx);
        return -1;
    }
	return 0;
}

void Modbus::read_registers(int start_addr, int count, uint16_t* buf)
{
    modbus_read_registers(this->ctx, start_addr, count, buf);
}

void Modbus::write_registers(int start_addr, int count, uint16_t* buf)
{
    modbus_write_registers(this->ctx, start_addr, count, buf);
}

void Modbus::read_register(int addr, uint16_t* value)
{
    modbus_read_registers(this->ctx, addr, 1, value);
}

void Modbus::write_register(int addr, uint16_t value)
{
    modbus_write_register(this->ctx, addr, value);
}

void Modbus::read_bits(int start_addr, int count, uint8_t* buf)
{
    modbus_read_bits(this->ctx, start_addr, count, buf);
}

void Modbus::write_bits(int start_addr, int count, uint8_t* buf)
{
    modbus_write_bits(this->ctx, start_addr, count, buf);
}

void Modbus::read_bit(int addr, uint8_t* value)
{
    modbus_read_bits(this->ctx, addr, 1, value);
}

void Modbus::write_bit(int addr, uint8_t value)
{
    modbus_write_bit(this->ctx, addr, value);
}