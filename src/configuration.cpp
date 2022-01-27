#include "configuration.h"

#include <spdlog/spdlog.h>

#include <string>
#include <vector>

Configuration::Configuration(std::vector<std::string> args) : port(args.at(0)) {
    const int value = std::stoi(args.at(1));
    this->baudrate = this->baudrate_from_args(value);
}

Configuration::~Configuration() {}

constexpr int Configuration::baudrate_from_args(int value) {
    if (value == 256000) {
        return 256000;
    } else if (value == 115200) {
        return 115200;
    } else {
        spdlog::info("Unknown baudrate [{}]. Default to 115200.", value);
        return 115200;
    }
}