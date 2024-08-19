#include "dogzilla_driver/dogzilla_driver.hpp"

#include <iostream>
#include <vector>

DogzillaDriver::DogzillaDriver(const std::string &port, int baudrate)
  : serial_port_{ io_service_ }, rx_data_{ 0 }, rx_flag_{ 0 }
{
  using serial = boost::asio::serial_port_base;
  try {
    serial_port_.open(port);
    serial_port_.set_option(serial::baud_rate(baudrate));
    serial_port_.set_option(serial::character_size(8));
    serial_port_.set_option(serial::stop_bits(serial::stop_bits::one));
    serial_port_.set_option(serial::parity(serial::parity::none));
    serial_port_.set_option(serial::flow_control(serial::flow_control::none));
#if defined(__linux__)
    // Enable low latency mode on Linux
    {
      int fd = serial_port_.native_handle();
      struct serial_struct serial_info;
      ioctl(fd, TIOCGSERIAL, &serial_info);
      serial_info.flags |= ASYNC_LOW_LATENCY;
      ioctl(fd, TIOCSSERIAL, &serial_info);
    }
#endif
  } catch (const std::exception &e) {
    throw std::runtime_error("Failed to initialize serial port: " + std::string(e.what()));
  }
}

DogzillaDriver::~DogzillaDriver()
{
  io_service_.stop();
  serial_port_.close();
}

// DogzillaDriver::read(){}

auto DogzillaDriver::read(uint8_t addr, uint8_t read_len) -> void
{
  std::array<uint8_t, 50> rx_data_ = { 0 };
  uint8_t mode = 0x02;
  uint8_t sum_data = (0x09 + mode + addr + read_len) % 256;
  sum_data = 255 - sum_data;

  std::array<uint8_t, 9> tx = { 0x55, 0x00, 0x09, mode, addr, read_len, sum_data, 0x00, 0xAA };


#ifdef __linux__
  {
    int fd = serial_port_.native_handle();
    ::tcflush(fd, TCIOFLUSH);
  }
#endif

  boost::asio::write(serial_port_, boost::asio::buffer(tx, tx.size()));
}


auto DogzillaDriver::readBattery() -> int
{
  uint8_t read_len = 1;
  this->read(commands_["BATTERY"][0], read_len);
  boost::asio::read(serial_port_, boost::asio::buffer(rx_data_, read_len));

  if (this->unpack()) { std::cout << static_cast<int>(rx_data_[0]); }
  return 0;
}


auto DogzillaDriver::unpack(int timeout) -> bool
{
  auto start_time = std::chrono::steady_clock::now();
  std::vector<uint8_t> rx_msg;
  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(timeout)) {
    int bytes = 0;

    // TODO: make it portable
    // System call Linux
    ioctl(serial_port_.native_handle(), FIONREAD, &bytes);

    if (bytes) {
      std::vector<uint8_t> data(bytes);
      boost::asio::read(serial_port_, boost::asio::buffer(data.data(), bytes));

      for (uint8_t num : data) {
        rx_msg.push_back(num);

        switch (rx_flag_) {
        case 0: {
          rx_flag_ = (num == 0x55) ? 1 : 0;
          break;
        }
        case 1: {
          rx_flag_ = (num == 0x00) ? 2 : 0;
          break;
        }

        case 2: {
          rx_len_ = num;
          rx_flag_ = 3;
          break;
        }

        case 3: {
          rx_type_ = num;
          rx_flag_ = 4;
          break;
        }

        case 4: {
          rx_addr_ = num;
          rx_flag_ = 5;
          rx_count_ = 0;
          break;
        }
        case 5: {
          if (rx_count_ == rx_len_ - 9) {
            rx_data_[rx_count_] = num;
            rx_flag_ = 6;
          } else if (rx_count_ < rx_len_ - 9) {
            rx_data_[rx_count_] = num;
            ++rx_count_;
          }
          break;
        }

        case 6: {
          uint8_t rx_check = 0;
          for (size_t i = 0; i < rx_len_ - 9; ++i) { rx_check += rx_data_[i]; }

          rx_check = 255 - (rx_len_ + rx_type_ + rx_addr_ + rx_check) % 256;
          if (num == rx_check) {
            rx_flag_ = 7;
          } else {
            resetState();
          }
          break;
        }

        case 7: {
          if (num == 0x00) {
            rx_flag_ = 8;
          } else {
            resetState();
          }
          break;
        }

        case 8: {
          if (num == 0xAA) {
            rx_flag_ = 0;
            return true;
          } else {
            resetState();
          }
          break;
        }
        }
      }
    }
  }
  return false;
}

auto DogzillaDriver::resetState() -> void
{
  rx_flag_ = 0;
  rx_addr_ = 0;
  rx_count_ = 0;
  rx_len_ = 0;
}
