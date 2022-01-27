add_executable(
  lidar_driver 
    main.cpp 
    configuration.cpp
    engine.cpp
    lidar.cpp
    olcPixelGameEngine.cpp)

target_link_libraries(
  lidar_driver
  PRIVATE project_options
          project_warnings
          CONAN_PKG::fmt
          CONAN_PKG::spdlog
          CONAN_PKG::argparse
          ${CMAKE_CURRENT_SOURCE_DIR}/include/rplidar/objects/libsl_lidar_sdk.a
          X11 
          GL 
          pthread 
          png)
