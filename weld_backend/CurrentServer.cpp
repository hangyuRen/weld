#include "CurrentServer.h"

namespace asio = boost::asio;
namespace beast = boost::beast;
namespace websocket = beast::websocket;
using tcp = asio::ip::tcp;


// --- JSON helper ---
static std::string currents_to_json(const std::vector<double>& armA, const std::vector<double>& armB) {
    std::ostringstream o;
    o << std::fixed << std::setprecision(3);
    o << R"({"type":"CURRENT_DATA",)";

    o << R"("armA":[)";
    for (size_t i = 0; i < armA.size(); ++i) {
        o << armA[i] << (i + 1 < armA.size() ? "," : "");
    }
    o << R"(],"armB":[)";
    for (size_t i = 0; i < armB.size(); ++i) {
        o << armB[i] << (i + 1 < armB.size() ? "," : "");
    }
    o << "]}";
    return o.str();
}

// ----------------- Session & Manager -----------------
struct CurrentSession;
class CurrentSessionManager {
    std::mutex mtx_;
    std::set<std::shared_ptr<CurrentSession>> sessions_;
public:
    void add(std::shared_ptr<CurrentSession> s) {
        std::lock_guard<std::mutex> lk(mtx_);
        sessions_.insert(s);
    }
    void remove(std::shared_ptr<CurrentSession> s) {
        std::lock_guard<std::mutex> lk(mtx_);
        sessions_.erase(s);
    }
    std::vector<std::shared_ptr<CurrentSession>> snapshot() {
        std::lock_guard<std::mutex> lk(mtx_);
        return std::vector<std::shared_ptr<CurrentSession>>(sessions_.begin(), sessions_.end());
    }
} g_sessions; // 单例管理（可改为每 Impl 一份）

struct CurrentSession : std::enable_shared_from_this<CurrentSession> {
    websocket::stream<tcp::socket> ws_;
    asio::strand<asio::io_context::executor_type> strand_;
    beast::flat_buffer buffer_;
    std::deque<std::string> outbox_;
    std::atomic<bool> open_{ true };

    CurrentSession(tcp::socket socket, asio::io_context& ioc)
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
                try { std::cerr << "[session] current server accept failed: " << ec.message() << std::endl; }
                catch (...) {}
                return;
            }
            try {
                auto remote = ws_.next_layer().remote_endpoint().address().to_string();
                std::cout << "[session] current server accepted from " << remote << std::endl;
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
            auto s = std::make_shared<CurrentSession>(std::move(socket), ioc);
            s->start();
        }
        // 继续接受
        do_accept(acceptor, ioc);
        });
}

// ----------------- Impl: 包含 io_context / acceptor / threads -----------------
struct CurrentServerWorker::Impl {
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
    unsigned short port_ = 8081;

    // 连接机器人
    std::mutex m_mutex;

    // 日志回调到外部（由 WsServerWorker 转发到 Qt）
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

            Hsc3::Comm::CommApi apiA("");
            Hsc3::Comm::CommApi apiB("");
            apiA.connect("192.168.1.71", 23234);
            apiB.connect("192.168.1.72", 23234);

            while (running_.load()) {
                std::vector<double> armA = getSensorData(apiA);
                std::vector<double> armB = getSensorData(apiB);

                std::string msg = currents_to_json(armA, armB);
                auto snaps = g_sessions.snapshot();
                for (auto& s : snaps) {
                    if (s) s->send_text(msg);
                }
                int ms = interval_ms_.load();
                std::this_thread::sleep_for(std::chrono::milliseconds(ms));
            }

            //// 在采样线程循环中
            //std::vector<double> mockArmA(6);
            //std::vector<double> mockArmB(6);

            //// 生成 12 个轴的模拟数据
            //for (int i = 0; i < 6; ++i) {
            //    mockArmA[i] =  rand();
            //    mockArmB[i] = rand();
            //}

            //while (running_.load()) {
            //    std::string msg = currents_to_json(mockArmA, mockArmB);
            //    auto snaps = g_sessions.snapshot();
            //    for (auto& s : snaps) {
            //        if (s) s->send_text(msg);
            //    }
            //    int ms = interval_ms_.load();
            //    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
            //}

            if (logCb_) logCb_(std::string("[broadcaster] exiting"));
            });
        if (logCb_) logCb_(std::string("SERVER_STARTED"));
    }

    std::vector<double> getSensorData(Hsc3::Comm::CommApi& api) {
        std::string ret;
        // 假设 mot.getJntEData(0) 返回的是电流/力矩数据
        api.execCmd("mot.getJntEData(0)", ret, Hsc3::Comm::PRIORITY_HIGH);
        return parseString(ret);
    }

    std::vector<double> parseString(std::string raw) {
        // 复用你 Worker.cpp 中的数据清洗逻辑
        raw.erase(std::remove(raw.begin(), raw.end(), '\"'), raw.end());
        raw.erase(std::remove(raw.begin(), raw.end(), '{'), raw.end());
        raw.erase(std::remove(raw.begin(), raw.end(), '}'), raw.end());

        std::vector<double> vals;
        std::stringstream ss(raw);
        std::string item;
        while (std::getline(ss, item, ',')) {
            try { if (!item.empty()) vals.push_back(std::stod(item)); }
            catch (...) {}
        }
        return vals;
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
                    std::cout << "数值转换失败:";
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

// ----------------- WsServerWorker methods -----------------
CurrentServerWorker::CurrentServerWorker() : impl_(new Impl()) {}
CurrentServerWorker::~CurrentServerWorker() {
    if (impl_) {
        impl_->stop();
        delete impl_;
        impl_ = nullptr;
    }
}

void CurrentServerWorker::startServer(const std::string& ip, int port, int intervalMs) {
    if (!impl_) return;
    // 设置回调，把 impl 的日志/error 回传到 Qt 主线程
    impl_->logCb_ = [this](const std::string& s) {
        if (s == "SERVER_STARTED") {
            std::cout << "CurrentServer服务已启动" << std::endl;
        }
        };
    // 启动（在 worker 线程被调用）
    impl_->start(ip.c_str(), static_cast<unsigned short>(port), intervalMs);
}

void CurrentServerWorker::stopServer() {
    if (!impl_) return;
    impl_->stop();
}

void CurrentServerWorker::sendOnce(const std::string& currentJson) {
    if (!impl_) return;
    impl_->sendOnce(currentJson.c_str());
}