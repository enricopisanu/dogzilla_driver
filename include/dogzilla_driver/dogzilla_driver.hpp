#pragma once

#include <array>
#include <bitset>
#include <boost/asio/io_service.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/write.hpp>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#if defined(__linux__)
#include <linux/serial.h>
#include <sys/ioctl.h>
#endif

class DogzillaDriver
{
public:
  DogzillaDriver(const std::string &port = "/dev/ttyAMA0", int baudrate = 115200);
  ~DogzillaDriver();

  DogzillaDriver(const DogzillaDriver &) = delete;
  DogzillaDriver &operator=(const DogzillaDriver &) = delete;

  DogzillaDriver(DogzillaDriver &&) noexcept = default;
  DogzillaDriver &operator=(DogzillaDriver &&) noexcept = default;

private:
  auto read(uint8_t addr, uint8_t read_len = 1) -> void;
  auto readBattery() -> int;
  auto unpack(int timeout = 1) -> bool;
  auto resetState() -> void;


  boost::asio::serial_port serial_port_;
  boost::asio::io_service io_service_;

  std::array<uint8_t, 50> rx_data_;

  int rx_flag_;
  uint8_t rx_len_;
  uint8_t rx_type_;
  uint8_t rx_addr_;
  uint8_t rx_count_;

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
      { 0x65,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128,
        128 } } };
};
