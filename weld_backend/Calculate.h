#pragma once
#ifndef CALCULATE_H
#define CALCULATE_H

#include <string>
#include <functional>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <fstream>
#include <iostream>
#include <sstream>

class Calculate
{
public:
    Calculate();

    // ����ͼƬ
    static double getDistance(const std::string& imagePath);
    static double getDistanceManual(const std::string& imagePath, double pixelX, double pixelY);

private:
    static double parseDistance(const std::string& body);
    static double parseDistanceManual(const std::string& body);
};

#endif