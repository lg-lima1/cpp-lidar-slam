#pragma once

#include <string>
#include <vector>

class Configuration {
public:
    Configuration(std::vector<std::string> args);
    ~Configuration();

private:
    constexpr int baudrate_from_args(int value);

public:
    std::string port;
    int baudrate;
};
