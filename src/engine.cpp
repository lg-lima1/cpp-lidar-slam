#include "engine.h"

#include <string>
#include <fstream>

#include <spdlog/spdlog.h>

#include "configuration.h"
#include "lidar.h"
#include "olcPixelGameEngine.h"

#define PI 3.14159265

GUI::GUI(Configuration configuration) {
    sAppName = "LIDAR Prototyping";
    m_lidar = new Lidar(configuration.port, configuration.baudrate);
}

GUI::~GUI() {}

bool GUI::OnUserCreate() {
    m_screen_size = GetWindowSize() / GetPixelSize();
    return true;
}

bool GUI::OnUserUpdate(float fElapsedTime) {    
    Clear(olc::BLACK);
    FillCircle(m_screen_size / 2, 1, olc::RED);

    for (double size = 0; size <= m_lidar->MAX_DISTANCE; size += 1000)
        DrawCircle(m_screen_size / 2, ScaleDistance(size), olc::VERY_DARK_GREY);

    const auto scan_data = m_lidar->scanData();
    for (const auto& [angle, distance, quality] : scan_data) {
        spdlog::debug("Angle [deg]: {:<20} Distance [mm]: {:<12} Quality [%]: {:<3}", 
            angle, distance, quality);
        
        if (quality > 0) {
            const auto rad = Degree2Rad(angle);
            const auto scaled_distance = ScaleDistance(distance);
            const auto transformed = Transform(rad, scaled_distance);
            const auto coord = Offset(transformed);
            Draw(coord, olc::GREEN);
        }
    }

    if (GetKey(olc::Key::SPACE).bReleased)
        SaveMap();

    return true;
}

bool GUI::OnUserDestroy() {
    return true;
}

double GUI::Degree2Rad(double degree) const {
    return (degree * PI / 180.0);
}

double GUI::ScaleDistance(double distance) const {
    return distance * ((static_cast<double>(m_screen_size.x) / 2) / m_lidar->MAX_DISTANCE);
}

olc::vd2d GUI::Transform(double angle, double distance) const {
    const olc::vd2d coord(
            sin(angle) * distance * -1, 
            cos(angle) * distance);

    return coord;
}

olc::vd2d GUI::Offset(olc::vd2d v) const {
    return (m_screen_size / 2) - v;
}

void GUI::SaveMap() {
    const auto filename = "/home/batman/projects/python/slam/test.csv";
    spdlog::info("Saving CSV coordinates to {} file.", filename);

    using namespace std;

    fstream file;
    file.open(filename, ios::out | ios::trunc);
    file << "x;y;angle;distance" << '\n';
    
    const auto scan_data = m_lidar->scanData();
    for (const auto& [angle, distance, quality] : scan_data) {
        if (quality > 0) {
            const auto rad = Degree2Rad(angle);
            const auto coord = Transform(rad, distance);
            const auto str = fmt::format("{};{};{};{}\n",
                coord.x, coord.y, angle, distance);

            file << str;
        }
    }

    file.close();
}
