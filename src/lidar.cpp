#include "lidar.h"

#include <string>

#include <spdlog/spdlog.h>

#include "include/rplidar/sl_lidar.h" 
#include "include/rplidar/sl_lidar_driver.h"

using namespace sl;

Lidar::Lidar(std::string port, int baudrate) : m_port(port), m_baudrate(baudrate) {
    init();
};

Lidar::~Lidar() {
    m_driver->stop();
    m_driver->setMotorSpeed(0);

    delete m_driver;
    delete m_channel;
}

void Lidar::init() {
    auto retcode = createLidarDriver();
    if (!retcode) {
        spdlog::error("Failed to initialize driver.");
        throw std::runtime_error("Failed to initialize driver.");
    }

    m_driver = *retcode;
    m_channel = this->createChannel();

    sl_result result;
    result = m_driver->connect(m_channel);
    if (!SL_IS_OK(result)) {
        spdlog::error("Failed to connect to sensor.");
        throw std::runtime_error("Failed to connect to sensor.");
    }

    sl_lidar_response_device_info_t devinfo{};
    result = m_driver->getDeviceInfo(devinfo);
    if (!SL_IS_OK(result)) {
        spdlog::error("Failed to retrive device info data.");
        throw std::runtime_error("Failed to retrive device info data.");
    }

    spdlog::info("SLAMTEC LIDAR");

    std::string serial_num{};
    constexpr size_t count = sizeof(devinfo.serialnum) / sizeof(devinfo.serialnum[0]);
    for (size_t pos = 0; pos < count; pos++) {
        serial_num.append(fmt::format("{0:X}", devinfo.serialnum[pos]));
    }

    spdlog::info("S/N: {}", serial_num);

    spdlog::info(
            "Firmware Ver: {}.{} Hardware Rev: {}"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , devinfo.hardware_version);

    sl_lidar_response_device_health_t health_info;
    result = m_driver->getHealth(health_info);
    if (!SL_IS_OK(result)) {
        spdlog::error("Failed to retrive device health.");
        throw std::runtime_error("Failed to retrive device health.");
    }

    spdlog::info("Health status: {}", health_info.status);
    if (health_info.status == SL_LIDAR_STATUS_ERROR) {
        spdlog::error("Internal error detected. Reboot the device and retry.");
        throw std::runtime_error("Internal error detected. Reboot the device and retry.");
    }

    m_driver->setMotorSpeed();
    m_driver->startScan(false, true);
}

std::vector<SensorData> Lidar::scanData() {
    std::array<sl_lidar_response_measurement_node_hq_t, 8192> nodes{};
    auto size = nodes.size();

    sl_result result;
    result = m_driver->grabScanDataHq(nodes.data(), size);
    if (!SL_IS_OK(result)) {
        spdlog::error("Error on grabScanDataHq().");
        throw std::runtime_error("Error on grabScanDataHq().");
    }

    std::vector<SensorData> vData{};
    for (const auto& [a, d, q, f] : nodes) {
        vData.push_back(
            SensorData{
                static_cast<double>(a * 90.f / (1 << 14)),
                static_cast<double>(d / (1 << 2)),
                q >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT
            }
        );
    }

    return vData;
}

sl::IChannel* Lidar::createChannel() {
    auto result = createSerialPortChannel(m_port, m_baudrate);
    if(!result) {
        spdlog::error("Failed to initialize channel.");
        throw std::runtime_error("Failed to initialize channel.");
    }
    return *result;
}