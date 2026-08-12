#include "ModbusServer.h"
#include "ModbusTools.h"

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <set>
#include <sstream>
#include <thread>
#include <vector>

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;

// ---------------------------------------------------------------------------
//  Data model — derived from modbus.doc (ControlBID4 ~ BID7 register layout)
//  The PLC exposes sensor values across holding registers. We read a batch
//  from register 0 and parse meaningful fields according to the doc spec.
// ---------------------------------------------------------------------------

// Number of holding registers (D registers) read in a single Modbus poll.
static const int MODBUS_REG_COUNT = 40;

// Parsed Modbus payload that gets serialized to JSON and pushed to the frontend.
struct ModbusPayload {
    // ControlBID4
    double angle = 0;          // angle, show = val/10 - 50
    double length = 0;         // length, show = val/10
    double slewingAngle = 0;   // slewing angle, show = val/10 - 360
    double height = 0;         // height, show = val/100
    // ControlBID5
    double actualRadius = 0;   // actual amplitude, show = val/100
    double ratedRadius = 0;    // rated amplitude, show = val/100
    int    amplitudeRatio = 0; // amplitude ratio (integer)
    double actualWeight = 0;   // actual load, show = val/10
    // ControlBID6
    double ratedWeight = 0;    // rated load, show = val/10
    int    torqueRatio = 0;    // torque ratio (integer)
    double workTime = 0;       // work time, show = val/10
    // ControlBID7
    double workRadius = 0;     // working amplitude, show = val/100
    int    engineRpm = 0;      // engine RPM
    int    oilLevel = 0;       // oil level
    int    lockFlag = 0;       // lock flag (>=1 means locked)
    // Raw alarm / status bits from ControlBID1 (coils M300~M307)
    int    alarmBits = 0;      // packed alarm byte0
    int    statusBits = 0;     // packed status byte4
    // Timestamp
    std::string timestamp;
};

// ---------------------------------------------------------------------------
//  JSON serialization
// ---------------------------------------------------------------------------
static std::string modbus_to_json(const ModbusPayload& p) {
    std::ostringstream o;
    o << std::fixed << std::setprecision(2);
    o << R"({"type":"MODBUS_DATA",)";
    o << "\"angle\":" << p.angle << ",";
    o << "\"length\":" << p.length << ",";
    o << "\"slewingAngle\":" << p.slewingAngle << ",";
    o << "\"height\":" << p.height << ",";
    o << "\"actualRadius\":" << p.actualRadius << ",";
    o << "\"ratedRadius\":" << p.ratedRadius << ",";
    o << "\"amplitudeRatio\":" << p.amplitudeRatio << ",";
    o << "\"actualWeight\":" << p.actualWeight << ",";
    o << "\"ratedWeight\":" << p.ratedWeight << ",";
    o << "\"torqueRatio\":" << p.torqueRatio << ",";
    o << "\"workTime\":" << p.workTime << ",";
    o << "\"workRadius\":" << p.workRadius << ",";
    o << "\"engineRpm\":" << p.engineRpm << ",";
    o << "\"oilLevel\":" << p.oilLevel << ",";
    o << "\"lockFlag\":" << p.lockFlag << ",";
    o << "\"alarmBits\":" << p.alarmBits << ",";
    o << "\"statusBits\":" << p.statusBits << ",";
    o << "\"timestamp\":\"" << p.timestamp << "\"";
    o << "}";
    return o.str();
}

// ---------------------------------------------------------------------------
//  Session & Manager (mirrors CurrentServer architecture)
// ---------------------------------------------------------------------------
struct ModbusSession;
class ModbusSessionManager {
    std::mutex mtx_;
    std::set<std::shared_ptr<ModbusSession>> sessions_;
public:
    void add(std::shared_ptr<ModbusSession> s) {
        std::lock_guard<std::mutex> lk(mtx_);
        sessions_.insert(s);
    }
    void remove(std::shared_ptr<ModbusSession> s) {
        std::lock_guard<std::mutex> lk(mtx_);
        sessions_.erase(s);
    }
    std::vector<std::shared_ptr<ModbusSession>> snapshot() {
        std::lock_guard<std::mutex> lk(mtx_);
        return std::vector<std::shared_ptr<ModbusSession>>(sessions_.begin(), sessions_.end());
    }
} g_modbus_sessions;

struct ModbusSession : std::enable_shared_from_this<ModbusSession> {
    websocket::stream<tcp::socket> ws_;
    asio::strand<asio::io_context::executor_type> strand_;
    beast::flat_buffer buffer_;
    std::deque<std::string> outbox_;
    std::atomic<bool> open_{ true };

    ModbusSession(tcp::socket socket, asio::io_context& ioc)
        : ws_(std::move(socket))
#if BOOST_VERSION >= 107700
        , strand_(asio::make_strand(ioc.get_executor()))
#else
        , strand_(ioc.get_executor())
#endif
    {
        ws_.set_option(websocket::stream_base::timeout::suggested(beast::role_type::server));
    }

    void start() {
        auto self = shared_from_this();
        ws_.async_accept(asio::bind_executor(strand_, [this, self](beast::error_code ec) {
            if (ec) {
                try { std::cerr << "[modbus-session] accept failed: " << ec.message() << std::endl; }
                catch (...) {}
                return;
            }
            try {
                auto remote = ws_.next_layer().remote_endpoint().address().to_string();
                std::cout << "[modbus-session] accepted from " << remote << std::endl;
            }
            catch (...) {}
            g_modbus_sessions.add(self);
            do_read();
        }));
    }

    void do_read() {
        auto self = shared_from_this();
        ws_.async_read(buffer_, asio::bind_executor(strand_, [this, self](beast::error_code ec, std::size_t) {
            if (ec) {
                close_and_cleanup();
                return;
            }
            buffer_.consume(buffer_.size());
            do_read();
        }));
    }

    void send_text(const std::string& msg) {
        auto self = shared_from_this();
        asio::post(strand_, [this, self, msg]() {
            bool writing = !outbox_.empty();
            outbox_.push_back(msg);
            if (!writing) do_write();
        });
    }

    void do_write() {
        if (outbox_.empty() || !open_) return;
        auto self = shared_from_this();
        ws_.text(true);
        ws_.async_write(asio::buffer(outbox_.front()),
            asio::bind_executor(strand_, [this, self](beast::error_code ec, std::size_t) {
                if (ec) { close_and_cleanup(); return; }
                outbox_.pop_front();
                if (!outbox_.empty()) do_write();
            }));
    }

    void close_and_cleanup() {
        if (!open_.exchange(false)) return;
        beast::error_code ec;
        try { ws_.next_layer().shutdown(tcp::socket::shutdown_both, ec); }
        catch (...) {}
        try { ws_.next_layer().close(ec); }
        catch (...) {}
        g_modbus_sessions.remove(shared_from_this());
    }
};

static void do_modbus_accept(tcp::acceptor& acceptor, asio::io_context& ioc) {
    acceptor.async_accept([&acceptor, &ioc](beast::error_code ec, tcp::socket socket) {
        if (!ec) {
            auto s = std::make_shared<ModbusSession>(std::move(socket), ioc);
            s->start();
        }
        do_modbus_accept(acceptor, ioc);
    });
}

// ---------------------------------------------------------------------------
//  Impl
// ---------------------------------------------------------------------------
struct ModbusServerWorker::Impl {
    std::atomic<bool> running_{ false };
    std::atomic<int> interval_ms_{ 200 };

    std::unique_ptr<asio::io_context> ioc_;
    std::unique_ptr<tcp::acceptor> acceptor_;

    std::thread io_thread_;
    std::thread broadcaster_thread_;

    std::string local_ip_ = "127.0.0.1";
    unsigned short port_ = 8083;

    // Modbus device
    std::mutex modbus_mtx_;
    std::unique_ptr<Modbus> modbus_;
    std::string modbus_ip_ = "192.168.1.88";
    int modbus_port_ = 502;
    std::atomic<bool> modbus_connected_{ false };

    Impl() = default;
    ~Impl() { stop(); }

    // --- Modbus connection ---
    void connectModbus(const std::string& ip, int port) {
        std::lock_guard<std::mutex> lk(modbus_mtx_);
        modbus_ip_ = ip;
        modbus_port_ = port;
        modbus_ = std::make_unique<Modbus>();
        int ret = modbus_->connect(ip.c_str(), port);
        modbus_connected_.store(ret == 0);
        if (ret == 0) {
            std::cout << "[modbus] connected to " << ip << ":" << port << std::endl;
        } else {
            std::cerr << "[modbus] connection failed to " << ip << ":" << port << std::endl;
        }
    }

    bool isModbusConnected() const {
        return modbus_connected_.load();
    }

    // --- Read a batch of registers and build a payload ---
    ModbusPayload readPayload() {
        ModbusPayload p;

        std::lock_guard<std::mutex> lk(modbus_mtx_);
        if (!modbus_ || !modbus_connected_.load()) {
            return p;
        }

        // Read holding registers D0 ~ D(MODBUS_REG_COUNT-1)
        uint16_t regs[MODBUS_REG_COUNT] = { 0 };
        modbus_->read_registers(0, MODBUS_REG_COUNT, regs);

        // Read a few coil bits (M300~M307) for alarm/status flags
        uint8_t bits[8] = { 0 };
        modbus_->read_bits(300, 8, bits);

        // --- Parse according to modbus.doc conventions ---
        // Values use little-endian: val = Byte1*256 + Byte0 => high*256 + low
        // Two-register (32-bit) values: val = regs[high]*65536 + regs[low]

        auto twoReg = [](uint16_t lo, uint16_t hi) -> int32_t {
            return static_cast<int32_t>(static_cast<uint32_t>(hi) << 16 | lo);
        };

        // ControlBID4 layout (adjust register indices to match actual PLC mapping)
        // angle: (actual + 50) * 10, show = val/10 - 50
        int32_t rawAngle = twoReg(regs[0], regs[1]);
        p.angle = rawAngle / 10.0 - 50.0;

        // length: val/10
        int32_t rawLen = twoReg(regs[2], regs[3]);
        p.length = rawLen / 10.0;

        // slewing angle: val/10 - 360
        int32_t rawSlew = twoReg(regs[4], regs[5]);
        p.slewingAngle = rawSlew / 10.0 - 360.0;

        // height: val/100
        int32_t rawHeight = twoReg(regs[6], regs[7]);
        p.height = rawHeight / 100.0;

        // ControlBID5
        int32_t rawActRadius = twoReg(regs[8], regs[9]);
        p.actualRadius = rawActRadius / 100.0;

        int32_t rawRatedRadius = twoReg(regs[10], regs[11]);
        p.ratedRadius = rawRatedRadius / 100.0;

        p.amplitudeRatio = static_cast<int>(regs[12]);

        int32_t rawActWeight = twoReg(regs[13], regs[14]);
        p.actualWeight = rawActWeight / 10.0;

        // ControlBID6
        int32_t rawRatedWeight = twoReg(regs[15], regs[16]);
        p.ratedWeight = rawRatedWeight / 10.0;

        p.torqueRatio = static_cast<int>(regs[17]);

        int32_t rawWorkTime = twoReg(regs[18], regs[19]);
        p.workTime = rawWorkTime / 10.0;

        // ControlBID7
        int32_t rawWorkRadius = twoReg(regs[20], regs[21]);
        p.workRadius = rawWorkRadius / 100.0;

        p.engineRpm = static_cast<int>(regs[22]);
        p.oilLevel = static_cast<int>(regs[23]);
        p.lockFlag = static_cast<int>(regs[24]);

        // Alarm/status bits
        int packed = 0;
        for (int i = 0; i < 8; ++i) {
            if (bits[i]) packed |= (1 << i);
        }
        p.alarmBits = packed;

        // Timestamp
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
#ifdef _WIN32
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif
        std::ostringstream ts;
        ts << std::put_time(&tm, "%H:%M:%S");
        p.timestamp = ts.str();

        return p;
    }

    // --- Broadcaster loop ---
    void broadcastLoop() {
        std::cout << "[modbus-broadcaster] started" << std::endl;

        while (running_.load()) {
            ModbusPayload p;

            if (modbus_connected_.load()) {
                try {
                    p = readPayload();
                } catch (...) {
                    // Modbus read failure — keep last values, try reconnect next cycle
                    std::cerr << "[modbus-broadcaster] read error, attempting reconnect" << std::endl;
                    modbus_connected_.store(false);
                    // Attempt reconnect
                    {
                        std::lock_guard<std::mutex> lk(modbus_mtx_);
                        modbus_ = std::make_unique<Modbus>();
                        int ret = modbus_->connect(modbus_ip_.c_str(), modbus_port_);
                        modbus_connected_.store(ret == 0);
                    }
                }
            }

            std::string msg = modbus_to_json(p);
            auto snaps = g_modbus_sessions.snapshot();
            for (auto& s : snaps) {
                if (s) s->send_text(msg);
            }

            int ms = interval_ms_.load();
            std::this_thread::sleep_for(std::chrono::milliseconds(ms));
        }

        std::cout << "[modbus-broadcaster] exiting" << std::endl;
    }

    void start(const std::string& ip, unsigned short port, int interval_ms) {
        if (running_.load()) return;
        local_ip_ = ip;
        port_ = port;
        interval_ms_.store(interval_ms);
        running_.store(true);

        ioc_ = std::make_unique<asio::io_context>(1);

        try {
            boost::system::error_code ec;
            tcp::endpoint endpoint(asio::ip::make_address(local_ip_, ec), port_);
            if (ec) { throw std::runtime_error(std::string("Invalid IP: ") + ec.message()); }

            acceptor_ = std::make_unique<tcp::acceptor>(*ioc_);
            acceptor_->open(endpoint.protocol(), ec);
            if (ec) throw std::runtime_error(std::string("open failed: ") + ec.message());
            acceptor_->set_option(asio::socket_base::reuse_address(true), ec);
            acceptor_->bind(endpoint, ec);
            if (ec) throw std::runtime_error(std::string("bind failed: ") + ec.message());
            acceptor_->listen(asio::socket_base::max_listen_connections, ec);
            if (ec) throw std::runtime_error(std::string("listen failed: ") + ec.message());

            do_modbus_accept(*acceptor_, *ioc_);
        } catch (const std::exception& e) {
            running_.store(false);
            std::cerr << "[modbus-server] start failed: " << e.what() << std::endl;
            return;
        }

        io_thread_ = std::thread([this]() {
            try { ioc_->run(); }
            catch (const std::exception& e) {
                std::cerr << "[modbus-server] io_context exception: " << e.what() << std::endl;
            }
        });

        broadcaster_thread_ = std::thread([this]() { broadcastLoop(); });

        std::cout << "[modbus-server] started on " << local_ip_ << ":" << port_ << std::endl;
    }

    void stop() {
        if (!running_.exchange(false)) return;

        try {
            if (acceptor_) {
                beast::error_code ec;
                acceptor_->cancel(ec);
                acceptor_->close(ec);
            }
        } catch (...) {}

        auto snaps = g_modbus_sessions.snapshot();
        for (auto& s : snaps) {
            if (s) s->close_and_cleanup();
        }

        try { if (ioc_) ioc_->stop(); }
        catch (...) {}

        if (broadcaster_thread_.joinable()) broadcaster_thread_.join();
        if (io_thread_.joinable()) io_thread_.join();

        acceptor_.reset();
        ioc_.reset();

        std::cout << "[modbus-server] stopped" << std::endl;
    }
};

// ---------------------------------------------------------------------------
//  ModbusServerWorker methods
// ---------------------------------------------------------------------------
ModbusServerWorker::ModbusServerWorker() : impl_(new Impl()) {}

ModbusServerWorker::~ModbusServerWorker() {
    if (impl_) {
        impl_->stop();
        delete impl_;
        impl_ = nullptr;
    }
}

void ModbusServerWorker::startServer(const std::string& ip, int port, int intervalMs) {
    if (!impl_) return;
    impl_->start(ip, static_cast<unsigned short>(port), intervalMs);
}

void ModbusServerWorker::stopServer() {
    if (!impl_) return;
    impl_->stop();
}

void ModbusServerWorker::connectModbus(const std::string& ip, int port) {
    if (!impl_) return;
    impl_->connectModbus(ip, port);
}

bool ModbusServerWorker::isModbusConnected() const {
    if (!impl_) return false;
    return impl_->isModbusConnected();
}
