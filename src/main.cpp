#include <memory>
#include <vector>

#include <spdlog/spdlog.h>
#include <fmt/ostream.h>
#include <argparse/argparse.hpp>

#include "configuration.h"
#include "engine.h"

int main(int argc, const char * argv[]) {
    argparse::ArgumentParser program("lidar-driver", "0.1");

    program.add_description("LIDAR Driver for prototyping purposes.");

    program.add_argument("-s", "--serial")
        .nargs(2)
        .help("Use serial interface [port] [115200, 256000]");

    program.add_argument("-d", "--debug")
        .help("Enable verbosity.")
        .default_value(false)
        .implicit_value(true);

    try 
    {
        program.parse_args(argc, argv);
    }
    catch (const std::runtime_error& err) 
    {
        spdlog::error("{}", err.what());
        spdlog::error("{}", program);
        std::exit(1);
    }

    if (program.is_used("--debug")) {
        spdlog::set_level(spdlog::level::debug);
    } else {
        spdlog::set_level(spdlog::level::info);
    }

    if (!program.is_used("--serial")) {
        spdlog::error("{}", program);
        std::exit(1);
    }

    auto configuration = Configuration(
        program.get<std::vector<std::string>>("--serial"));
        
	auto demo = GUI(configuration);
	if (demo.Construct(1024, 960, 1, 1)) {
        try {
            demo.Start();
        } catch (const std::runtime_error& err) {
            spdlog::error("Application error. Handling exception.");
            spdlog::error("{}", err.what());
            std::exit(1);
        }
    }

    spdlog::info("Closing application.");
    return 0;
}
