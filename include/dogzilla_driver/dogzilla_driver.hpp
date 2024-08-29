#pragma once

#include <array>
#include <bitset>
#include <chrono>
#include <cstddef>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include <boost/asio/io_service.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/write.hpp>

#if defined(__linux__)
#include <linux/serial.h>
#include <sys/ioctl.h>
#endif


struct ParamLimits
{
  double translation_x = 35;
  double translation_y = 19.5;
  std::array<double, 2> translation_z = { 75, 115 };

  std::array<double, 3> attitude = { 20, 22, 16 };
  double leg_x = 35;
  double leg_y = 19.5;
  std::array<double, 2> leg_z = { 75, 115 };

  std::array<std::array<double, 2>, 3> motor = { { { -73, 57 }, { -66, 93 }, { -31, 31 } } };
  std::array<double, 2> period = { 1.5, 8 };
  std::array<double, 2> mark_time = { 10, 35 };
  double vx = 25;
  double vy = 18;
  double vyaw = 100;
};

class DogzillaDriver
{
public:
  DogzillaDriver(const std::string &port = "/dev/ttyAMA0", int baudrate = 115200);
  ~DogzillaDriver();

  DogzillaDriver(const DogzillaDriver &) = delete;
  DogzillaDriver &operator=(const DogzillaDriver &) = delete;

  DogzillaDriver(DogzillaDriver &&) noexcept = default;
  DogzillaDriver &operator=(DogzillaDriver &&) noexcept = default;

  auto moveX(int step) -> void;
  auto moveY(int step) -> void;
  auto turn(int step) -> void;
  auto markTime(int data) -> void;
  auto stop() -> void;
  auto forward(int step) -> void;
  auto backwards(int step) -> void;
  auto left(int step) -> void;
  auto right(int step) -> void;
  auto turnLeft(int step) -> void;
  auto turnRight(int step) -> void;
  auto action(int action_id) -> void;
  auto reset() -> void;
  auto motor(const auto &motor_id, int data) -> void;

  [[nodiscard]] auto readBattery() -> int;


  static auto conver2u8(const double data, const auto &limit, const int mode = 0) -> uint8_t;

private:
  auto read(uint8_t addr, uint8_t read_len = 1) -> void;
  [[nodiscard]] auto unpack(int timeout = 1) -> bool;
  auto resetState() -> void;
  auto send(const std::string &key, uint8_t index = 1, uint8_t len = 1) -> void;
  auto motorSend(int index, int data) -> void;

  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;

  std::array<uint8_t, 50> rx_data_;

  int rx_flag_;
  uint8_t rx_len_;
  uint8_t rx_type_;
  uint8_t rx_addr_;
  uint8_t rx_count_;

  // clang-format off
  std::unordered_map<std::string, std::vector<uint8_t>> commands_ = { { "BATTERY", { 0x01, 100 } },
    { "PERFORM", { 0x03, 0 } },
    { "CALIBRATION", { 0x04, 0 } },
    { "UPGRADE", { 0x05, 0 } },
    { "MOVE_TEST", { 0x06, 1 } },
    { "FIRMWARE_VERSION", { 0x07 } },
    { "GAIT_TYPE", { 0x09, 0x00 } },
    { "BT_NAME", { 0x13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    { "UNLOAD_MOTOR", { 0x20, 0 } },
    { "LOAD_MOTOR", { 0x20, 0 } },
    { "VX", { 0x30, 128 } },
    { "VY", { 0x31, 128 } },
    { "VYAW", { 0x32, 128 } },
    { "TRANSLATION", { 0x33, 0, 0, 0 } },
    { "ATTITUDE", { 0x36, 0, 0, 0 } },
    { "PERIODIC_ROT", { 0x39, 0, 0, 0 } },
    { "MarkTime", { 0x3C, 0 } },
    { "MOVE_MODE", { 0x3D, 0 } },
    { "ACTION", { 0x3E, 0 } },
    { "PERIODIC_TRAN", { 0x80, 0, 0, 0 } },
    { "MOTOR_ANGLE", { 0x50, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128 } },
    { "MOTOR_SPEED", { 0x5C, 1 } },
    { "LEG_POS", { 0x40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } },
    { "IMU", { 0x61, 0 } },
    { "ROLL", { 0x62, 0 } },
    { "PITCH", { 0x63, 0 } },
    { "YAW", { 0x64, 0 } },
    { "IMU_RAW",
      { 0x65, 128, 128, 128, 128, 128, 
      128, 128, 128, 128, 128, 128, 
      128, 128, 128, 128, 128, 128, 
      128, 128, 128, 128, 128, 128, 128 } } 
  };
  // clang-format on
  ParamLimits param_limits_;
};
