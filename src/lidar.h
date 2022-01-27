#pragma once

#include <string>

#include "include/rplidar/sl_lidar.h" 
#include "include/rplidar/sl_lidar_driver.h"

struct SensorData {
    double      angle_degree;
    double      distance_mm;
    int32_t     quality;
};

class Lidar {
public:
    Lidar(std::string port, int baudrate);
    ~Lidar();
    
    std::vector<SensorData> scanData();

private:
    void init();
    sl::IChannel* createChannel();

public:
    static const uint32_t MAX_DISTANCE { 12000 };

private:
    std::string m_port{};
    int m_baudrate{};

    sl::ILidarDriver* m_driver;
    sl::IChannel* m_channel;
};