#include "robotConnect.h"
#include <thread>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>
robotConnect::robotConnect()
{

    m_pCmApi = new Hsc3::Comm::CommApi();
    m_pMot = new Hsc3::Proxy::ProxyMotion(m_pCmApi);
    m_pVar = new Hsc3::Proxy::ProxyVar(m_pCmApi);
    m_pExt = new Hsc3::Proxy::ProxyExt(m_pCmApi);
    m_pSys = new Hsc3::Proxy::ProxySys(m_pCmApi);
    m_pVm = new  Hsc3::Proxy::ProxyVm(m_pCmApi);
}

robotConnect::~robotConnect()
{

}
void robotConnect::setCommMessage(RobCommInfo Robf)
{
    robotIp = Robf.robIp;

}

bool robotConnect::connectRobot(string strIp, int uPort)
{
    m_pCmApi->setAutoConn(false);
    Hsc3::Comm::HMCErrCode ret = m_pCmApi->connect(strIp, uPort);
    if (ret != 0)
    {
        printf("CommApi::connect() : ret = %lld\n", ret);
    }
    Sleep(500);
    if (m_pCmApi->isConnected())
    {

        std::cout << "连接成功" << std::endl;
        Sleep(10);
        return true;
    }
    else
    {
        std::cout << "连接失败" << std::endl;
        return false;
    }
}
bool robotConnect::disconnectRobot()//Hsc3::Comm::CommApi& cmApi
{
    Hsc3::Comm::HMCErrCode ret = m_pCmApi->disconnect();
    Sleep(500);
    if (m_pCmApi->isConnected())
    {
        return false;
    }
    else
    {
        return true;
    }
}







bool robotConnect::GetWeldError(string& getError)//
{
    ErrLevel a1;
    uint64_t b1;
    //string c1;

    int retError = m_pSys->getMessage(a1, b1, getError, 500);//获取错误

    Sleep(500);
    if (retError == 0)//m_pCmApi->isConnected()
    {
        return true;
    }
    else
    {
        return false;
    }
}







int robotConnect::getRobPosData(LocData& LocData)
{
    int retJnt = m_pMot->getLocData(0, LocData);
    if (retJnt == 0)
    {
        return 0;
    }
    else
    {
        return -1;
    }

}

uint64_t robotConnect::getRobSysTime()
{
    std::string time_str = "2025-02-08 10:05:15.885";
    string cmdStr = "sys.getDateTime()";
    //调用系统时间获取指令
    auto rstr = this->m_pCmApi->execCmd(cmdStr, time_str, 3);
    if (rstr == 0)
    {
        std::cout << "当前系统时间:" << time_str << std::endl;
    }

    // 解析日期和时间部分
    std::tm tm = {};
    time_str = time_str.substr(1, time_str.length() - 2);
    std::cout << "当前系统时间:" << time_str << std::endl;
    std::stringstream ss(time_str);
    ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

    // 解析毫秒部分
    double milliseconds = 0.0;
    char dot;
    ss >> dot >> milliseconds; // 读取小数点后的毫秒部分

    // 将 std::tm 结构转换为 time_t
    std::time_t time = std::mktime(&tm);

    // 将 time_t 转换为毫秒时间戳
    auto duration = std::chrono::system_clock::from_time_t(time).time_since_epoch();
    auto totalMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() + static_cast<long long>(milliseconds);

    // 输出毫秒时间戳
    std::cout << "Milliseconds since epoch: " << totalMilliseconds << std::endl;
    return totalMilliseconds;
}
