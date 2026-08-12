#pragma once
#ifndef ROBOTCONNECT_H
#define ROBOTCONNECT_H
#include <Windows.h>
#include <iostream>
#include "process.h"
#include "CommApi.h"
#include "proxy/ProxyMotion.h"
#include "proxy/ProxySys.h"
#include "proxy/ProxyVar.h"
#include "proxy/ProxyExt.h"
#include"proxy/ProxyVm.h"
#include "ErrDef.h"
#include <vector>
using namespace std;
/**
 * 功能描述：存储机器人通信相关数据
 * @brief RobCommInfo   存储机器人通信相关数据
 * @param robPortNun    机器人通信端口号
 * @param robGroupId    机器人组号，如果是单个机器人，则为0
 * @param robIp         机器人通信IP地址
 */
struct RobCommInfo
{
    uint16_t robPort;
    int8_t robGroupId;
    string robIp;
};

class robotConnect
{
public:
    robotConnect();
    ~robotConnect();

    void setCommMessage(RobCommInfo Robf);                                          //设置通讯信息 
    bool connectRobot(string strIp, int uPort);
    bool disconnectRobot();
    //bool GetWeldEleVol(string& getEle, string& getVol, string& getSpeed);
    //bool GetSNCode(string& getSn);//
   // bool GetSnTime(string& getSnTime);//
//bool SetSnEnroll(string SnPstr, string SnCode);//
    int getRobPosData(LocData& LocData);
    uint64_t getRobSysTime();
    bool GetWeldError(string& getError);//

    Hsc3::Comm::CommApi* m_pCmApi;    //机器人通信指针
    Hsc3::Proxy::ProxyMotion* m_pMot;    //机器人运动指针
    Hsc3::Proxy::ProxyVar* m_pVar;    //变量指针，todo:
    Hsc3::Proxy::ProxyExt* m_pExt;    //变量指针，todo:
    Hsc3::Proxy::ProxySys* m_pSys;    //变量指针，todo:可能无用
    Hsc3::Proxy::ProxyVm* m_pVm;
private:
    string robotIp;

};
#endif
