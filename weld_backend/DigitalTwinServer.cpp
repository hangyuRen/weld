#include "DigitalTwinServer.h"


namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;


// --- JSON helper ---
static std::string json_escape(const std::string& s) {
    std::ostringstream o;
    for (unsigned char c : s) {
        switch (c) {
        case '\"': o << "\\\""; break;
        case '\\': o << "\\\\"; break;
        case '\b': o << "\\b";  break;
        case '\f': o << "\\f";  break;
        case '\n': o << "\\n";  break;
        case '\r': o << "\\r";  break;
        case '\t': o << "\\t";  break;
        default:
            if (c < 0x20) {
                o << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
            }
            else {
                o << c;
            }
        }
    }
    return o.str();
}
static std::string jointInfos_to_json(const std::vector<JointInfo>& joints) {
    std::ostringstream o;
    o << R"({"type":"jointUpdate","data":[)";
    for (size_t i = 0; i < joints.size(); ++i) {
        const auto& j = joints[i];
        o << "{\"name\":\"" << json_escape(j.name) << "\""
            << ",\"lower\":" << std::fixed << std::setprecision(6) << j.lower
            << ",\"upper\":" << std::fixed << std::setprecision(6) << j.upper
            << ",\"degree\":" << std::fixed << std::setprecision(6) << j.degree
            << "}";
        if (i + 1 < joints.size()) o << ",";
    }
    o << "]}";
    return o.str();
}

// ----------------- Session & Manager -----------------
struct Session;
class SessionManager {
    std::mutex mtx_;
    std::set<std::shared_ptr<Session>> sessions_;
public:
    void add(std::shared_ptr<Session> s) {
        std::lock_guard<std::mutex> lk(mtx_);
        sessions_.insert(s);
    }
    void remove(std::shared_ptr<Session> s) {
        std::lock_guard<std::mutex> lk(mtx_);
        sessions_.erase(s);
    }
    std::vector<std::shared_ptr<Session>> snapshot() {
        std::lock_guard<std::mutex> lk(mtx_);
        return std::vector<std::shared_ptr<Session>>(sessions_.begin(), sessions_.end());
    }
} g_sessions; // 单例管理（可改为每 Impl 一份）

struct Session : std::enable_shared_from_this<Session> {
    websocket::stream<tcp::socket> ws_;
    asio::strand<asio::io_context::executor_type> strand_;
    beast::flat_buffer buffer_;
    std::deque<std::string> outbox_;
    std::atomic<bool> open_{ true };

    Session(tcp::socket socket, asio::io_context& ioc)
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
                // 握手失败
                try { std::cerr << "[session] digital twin server accept failed: " << ec.message() << std::endl; }
                catch (...) {}
                return;
            }
            try {
                auto remote = ws_.next_layer().remote_endpoint().address().to_string();
                std::cout << "[session] digital twin server accepted from " << remote << std::endl;
            }
            catch (...) {}
            g_sessions.add(self);
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
        ws_.async_write(asio::buffer(outbox_.front()), asio::bind_executor(strand_, [this, self](beast::error_code ec, std::size_t) {
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
        g_sessions.remove(shared_from_this());
    }
};

// accept helper
static void do_accept(tcp::acceptor& acceptor, asio::io_context& ioc) {
    acceptor.async_accept([&acceptor, &ioc](beast::error_code ec, tcp::socket socket) {
        if (!ec) {
            auto s = std::make_shared<Session>(std::move(socket), ioc);
            s->start();
        }
        // 继续接受
        do_accept(acceptor, ioc);
        });
}

// ----------------- Impl: 包含 io_context / acceptor / threads -----------------
struct DigitalTwinServer::Impl {
    // 控制变量
    std::atomic<bool> running_{ false };
    std::atomic<int> interval_ms_{ 200 };

    // Asio
    std::unique_ptr<asio::io_context> ioc_;
    std::unique_ptr<tcp::acceptor> acceptor_;

    // 线程
    std::thread io_thread_;
    std::thread broadcaster_thread_;

    // 参数
    std::string local_ip_ = "127.0.0.1";
    unsigned short port_ = 8080;

    // 连接机器人
    std::mutex m_mutex;

    // 日志回调到外部（由 DigitalTwinServer 转发到 Qt）
    std::function<void(const std::string&)> logCb_;
    std::function<void(const std::string&)> errorCb_;

    Impl() {
    }
    ~Impl() { stop(); }

    void start(const std::string& ip, unsigned short port, int interval_ms) {
        if (running_.load()) return;
        local_ip_ = ip; port_ = port; interval_ms_.store(interval_ms);
        running_.store(true);

        // 创建 io_context
        ioc_ = std::make_unique<asio::io_context>(1);

        // acceptor setup (可能抛异常，需要捕获并回调)
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

            // async accept loop
            do_accept(*acceptor_, *ioc_);
        }
        catch (const std::exception& e) {
            running_.store(false);
            if (errorCb_) errorCb_(std::string("start failed: ") + e.what());
            return;
        }

        // run io_context in its own thread
        io_thread_ = std::thread([this]() {
            if (logCb_) logCb_(std::string("io_context running"));
            try {
                ioc_->run();
            }
            catch (const std::exception& e) {
                if (errorCb_) errorCb_(std::string("io_context exception: ") + e.what());
            }
            if (logCb_) logCb_(std::string("io_context stopped"));
            });

        // start broadcaster thread
        broadcaster_thread_ = std::thread([this]() {
            if (logCb_) logCb_(std::string("[broadcaster] started"));
            double t = 0;

            // robotA
            Hsc3::Comm::CommApi cmApi("");
            Hsc3::Proxy::ProxyMotion pMot(&cmApi);
            cmApi.setAutoConn(false);
            cmApi.connect("192.168.1.71", 23234);

            // robotB
            Hsc3::Comm::CommApi cmApi2("");
            Hsc3::Proxy::ProxyMotion pMot2(&cmApi2);
            cmApi2.setAutoConn(false);
            cmApi2.connect("192.168.1.72", 23234);

            while (running_.load()) {
                // 读取关节角度
                std::vector<JointInfo> joints = {
                   {"arm1-wrist1_link_joint", -240, 120.0, -60},
                   {"arm1-lower-arm_link_joint", -30.0, 190.0, 150},
                   {"arm1-wrist2_link_joint", -110.0, 190.0, -50},
                   {"arm1-upper-arm_link_joint", -280.0, 80.0, -100},
                   {"arm1-wrist3_link_joint", -450.0, -90.0, -270},
                   {"arm1-tool_link_joint", -470.0, 250.0, -110 },
                   {"arm2-wrist1_link_joint", -60.0, 300.0, 120},
                   {"arm2-lower-arm_link_joint", -150.0, 70.0, 30},
                   {"arm2-wrist2_link_joint", -210.0, 90.0, -150},
                   {"arm2-upper-arm_link_joint", -110.0, 250.0, 70},
                   {"arm2-wrist3_link_joint", -280.0, 80.0, -100},
                   {"arm2-tool_link_joint", -430.0, 290.0, -70}
                };

                // robotA
                std::vector<double> jointAnglesA = getJointData(cmApi);
                if (!jointAnglesA.empty()) {
                    for (size_t i = 0; i < 6; ++i) {
                        if (i == 0 || i == 3 || i == 5) {
                            joints[i].degree -= jointAnglesA[i];
                        }
                        else {
                            joints[i].degree += jointAnglesA[i];
                        }

                    }
                }

                // robotB
                std::vector<double> jointAnglesB = getJointData(cmApi2);
                if (!jointAnglesB.empty()) {
                    for (size_t i = 6; i < 12; ++i) {
                        if (i == 9 || i == 11) {
                            joints[i].degree -= jointAnglesB[i - 6];
                        }
                        else {
                            joints[i].degree += jointAnglesB[i - 6];
                        }

                    }
                }

                std::string msg = jointInfos_to_json(joints);
                auto snaps = g_sessions.snapshot();
                for (auto& s : snaps) {
                    if (s) s->send_text(msg);
                }
                int ms = interval_ms_.load();
                std::this_thread::sleep_for(std::chrono::milliseconds(ms));
            }
            if (logCb_) logCb_(std::string("[broadcaster] exiting"));
            });
        if (logCb_) logCb_(std::string("SERVER_STARTED"));
    }

    void stop() {
        if (!running_.exchange(false)) return;

        // 关闭 acceptor
        try {
            if (acceptor_) {
                beast::error_code ec;
                acceptor_->cancel(ec);
                acceptor_->close(ec);
            }
        }
        catch (...) {}

        // 关闭所有 session（遍历 snapshot 并 cleanup）
        auto snaps = g_sessions.snapshot();
        for (auto& s : snaps) {
            if (s) s->close_and_cleanup();
        }

        // 停止 io_context
        try {
            if (ioc_) {
                ioc_->stop();
            }
        }
        catch (...) {}

        // join threads
        if (broadcaster_thread_.joinable()) broadcaster_thread_.join();
        if (io_thread_.joinable()) io_thread_.join();

        // reset
        acceptor_.reset();
        ioc_.reset();

        if (logCb_) logCb_(std::string("Server stopped"));
    }

    void sendOnce(const std::string& jsonMsg) {
        auto snaps = g_sessions.snapshot();
        for (auto& s : snaps) {
            if (s) s->send_text(jsonMsg);
        }
    }

    std::string processJointData(const std::string& raw) {
        std::string processed = raw;

        // 移除双引号
        if (!processed.empty() && processed.front() == '"' && processed.back() == '"') {
            processed = processed.substr(1, processed.size() - 2);
        }

        // 移除花括号
        processed.erase(std::remove(processed.begin(), processed.end(), '{'), processed.end());
        processed.erase(std::remove(processed.begin(), processed.end(), '}'), processed.end());

        // 移除末尾逗号
        if (!processed.empty() && processed.back() == ',') {
            processed.pop_back();
        }

        return processed;
    }

    std::vector<double> parseJointAngles(const std::string& data) {
        std::vector<double> values;
        std::stringstream ss(data);
        std::string item;

        while (std::getline(ss, item, ',')) {
            if (!item.empty()) {
                try {
                    values.push_back(std::stod(item));
                }
                catch (...) {
                    std::cerr << "数值转换失败:" << item;
                }
            }
        }

        return values;
    }

    std::vector<double> getJointData(Hsc3::Comm::CommApi& cmApi) {
        std::lock_guard<std::mutex> lock(m_mutex);

        std::string strCmd = "mot.getJntData(0)";
        std::string strRet;
        cmApi.execCmd(strCmd, strRet, Hsc3::Comm::PRIORITY_HIGH);

        // 处理返回数据
        strRet = processJointData(strRet);
        return parseJointAngles(strRet);
    }
};

// ----------------- DigitalTwinServer methods -----------------
DigitalTwinServer::DigitalTwinServer() : impl_(new Impl()) {}
DigitalTwinServer::~DigitalTwinServer() {
    if (impl_) {
        impl_->stop();
        delete impl_;
        impl_ = nullptr;
    }
}

void DigitalTwinServer::startServer(const std::string& ip, int port, int intervalMs) {
    if (!impl_) return;
    // 设置回调，把 impl 的日志/error 回传到 Qt 主线程
    impl_->logCb_ = [this](const std::string& s) {
        if (s == "SERVER_STARTED") {
            std::cout << "DigitalTwinServer服务已启动" << std::endl;
        }
        };
    // 启动（在 worker 线程被调用）
    impl_->start(ip, static_cast<unsigned short>(port), intervalMs);
}

void DigitalTwinServer::stopServer() {
    if (!impl_) return;
    impl_->stop();
}

void DigitalTwinServer::sendOnce(const std::string& jointsJson) {
    if (!impl_) return;
    impl_->sendOnce(jointsJson);
}