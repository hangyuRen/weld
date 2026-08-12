#include "calculate.h"



namespace beast = boost::beast;
namespace http = beast::http;
namespace asio = boost::asio;
using tcp = asio::ip::tcp;

Calculate::Calculate() {}

double Calculate::getDistance(const std::string& imagePath)
{
    try
    {
        // 读取文件
        std::ifstream file(imagePath, std::ios::binary);
        if (!file.is_open())
        {
            std::cout << "open file failed\n";
            return -1;
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string fileContent = buffer.str();

        // ⭐ 构造 multipart
        std::string boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";

        std::ostringstream bodyStream;
        bodyStream
            << "--" << boundary << "\r\n"
            << "Content-Disposition: form-data; name=\"file\"; filename=\"image.jpg\"\r\n"
            << "Content-Type: image/jpeg\r\n\r\n"
            << fileContent << "\r\n"
            << "--" << boundary << "--\r\n";

        std::string body = bodyStream.str();

        // ⭐ 建立连接
        asio::io_context ioc;
        tcp::resolver resolver(ioc);
        beast::tcp_stream stream(ioc);

        auto const results =
            resolver.resolve("127.0.0.1", "8000");

        stream.connect(results);

        // ⭐ 构造 HTTP 请求
        http::request<http::string_body> req{
            http::verb::post,
            "/detect-distance",
            11
        };

        req.set(http::field::host, "127.0.0.1");
        req.set(http::field::user_agent,
            BOOST_BEAST_VERSION_STRING);
        req.set(http::field::content_type,
            "multipart/form-data; boundary=" + boundary);

        req.body() = body;
        req.prepare_payload();

        // 发送
        http::write(stream, req);

        // 接收响应
        beast::flat_buffer bufferResp;
        http::response<http::string_body> res;
        http::read(stream, bufferResp, res);

        std::string responseBody = res.body();

        //std::cout << "raw response: "
        //    << responseBody << std::endl;

        double dis = parseDistance(responseBody);

        beast::error_code ec;
        stream.socket().shutdown(
            tcp::socket::shutdown_both, ec);

        return dis;
    }
    catch (std::exception& e)
    {
        std::cout << "Exception: "
            << e.what() << std::endl;
        return -1;
    }
}

double Calculate::parseDistance(const std::string& response)
{
    auto pos = response.find("distance_m");
    if (pos == std::string::npos)
        return 0.0;

    auto colon = response.find(":", pos);
    auto end = response.find("}", colon);

    std::string num =
        response.substr(colon + 1,
            end - colon - 1);

    return std::stod(num);
}

double Calculate::getDistanceManual(const std::string& imagePath, double pixelX, double pixelY)
{
    try
    {
        // read image file
        std::ifstream file(imagePath, std::ios::binary);
        if (!file.is_open())
        {
            std::cout << "open file failed\n";
            return -1;
        }

        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string fileContent = buffer.str();

        // build multipart body: image + pixelX + pixelY
        std::string boundary = "----WebKitFormBoundary7MA4YWxkTrZu0gW";

        std::ostringstream bodyStream;
        bodyStream
            << "--" << boundary << "\r\n"
            << "Content-Disposition: form-data; name=\"file\"; filename=\"image.jpg\"\r\n"
            << "Content-Type: image/jpeg\r\n\r\n"
            << fileContent << "\r\n"
            << "--" << boundary << "\r\n"
            << "Content-Disposition: form-data; name=\"pixelX\"\r\n\r\n"
            << pixelX << "\r\n"
            << "--" << boundary << "\r\n"
            << "Content-Disposition: form-data; name=\"pixelY\"\r\n\r\n"
            << pixelY << "\r\n"
            << "--" << boundary << "--\r\n";

        std::string body = bodyStream.str();

        // connect to the manual depth service (1.py on port 8001)
        asio::io_context ioc;
        tcp::resolver resolver(ioc);
        beast::tcp_stream stream(ioc);

        auto const results =
            resolver.resolve("127.0.0.1", "8001");

        stream.connect(results);

        // build HTTP request
        http::request<http::string_body> req{
            http::verb::post,
            "/detect-distance",
            11
        };

        req.set(http::field::host, "127.0.0.1");
        req.set(http::field::user_agent,
            BOOST_BEAST_VERSION_STRING);
        req.set(http::field::content_type,
            "multipart/form-data; boundary=" + boundary);

        req.body() = body;
        req.prepare_payload();

        // send
        http::write(stream, req);

        // receive response
        beast::flat_buffer bufferResp;
        http::response<http::string_body> res;
        http::read(stream, bufferResp, res);

        std::string responseBody = res.body();

        double dis = parseDistanceManual(responseBody);

        beast::error_code ec;
        stream.socket().shutdown(
            tcp::socket::shutdown_both, ec);

        return dis;
    }
    catch (std::exception& e)
    {
        std::cout << "Exception: "
            << e.what() << std::endl;
        return -1;
    }
}

double Calculate::parseDistanceManual(const std::string& response)
{
    auto pos = response.find("\"distance\"");
    if (pos == std::string::npos)
        return -1.0;

    auto colon = response.find(":", pos);
    auto end = response.find("}", colon);

    std::string num =
        response.substr(colon + 1,
            end - colon - 1);

    try {
        return std::stod(num);
    }
    catch (...) {
        return -1.0;
    }
}