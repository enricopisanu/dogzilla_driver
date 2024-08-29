#include "dogzilla_driver/dogzilla_driver.hpp"

DogzillaDriver::DogzillaDriver(const std::string &port, int baudrate)
  : serial_port_{ io_service_ }, rx_data_{ 0 }, rx_flag_{ 0 }, param_limits_{}
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

auto DogzillaDriver::send(const std::string &key, uint8_t index, uint8_t len) -> void
{
  uint8_t mode = 0x01;
  uint8_t order = commands_[key][0] + index - 1;
  uint8_t value_sum = 0;
  std::vector<uint8_t> value;

  for (uint8_t i = 0; i < len; ++i) {
    value.push_back(commands_[key][index + i]);
    value_sum += commands_[key][index + i];
  }

  uint8_t sum_data = (static_cast<uint8_t>(len + 0x08) + mode + order + value_sum) % 256;
  sum_data = 255 - sum_data;

  std::vector<uint8_t> tx = { 0x55, 0x00, static_cast<uint8_t>(len + 0x08), mode, order };
  tx.insert(tx.end(), value.begin(), value.end());
  tx.push_back(sum_data);
  tx.push_back(0x00);
  tx.push_back(0xAA);

  boost::asio::write(serial_port_, boost::asio::buffer(tx, tx.size()));
}


auto DogzillaDriver::read(uint8_t addr, uint8_t read_len) -> void
{
  std::array<uint8_t, 50> rx_data_ = { 0 };
  uint8_t mode = 0x02;
  uint8_t sum_data = (0x09 + mode + addr + read_len) % 256;
  sum_data = 255 - sum_data;

  std::array<uint8_t, 9> tx = { 0x55, 0x00, 0x09, mode, addr, read_len, sum_data, 0x00, 0xAA };


#ifdef __linux__
  {
    int fd = serial_port_.lowest_layer().native_handle();
    ::tcflush(fd, TCIOFLUSH);
  }
#endif

  boost::asio::write(serial_port_, boost::asio::buffer(tx, tx.size()));
}

auto DogzillaDriver::stop() -> void
{
  this->moveX(0);
  this->moveY(0);
  this->markTime(0);
  this->turn(0);
}

auto DogzillaDriver::moveX(int step) -> void
{
  step = std::clamp(step, -20, 20);
  commands_["VX"][1] = conver2u8(step, param_limits_.vx);
  this->send("VX");
}

auto DogzillaDriver::moveY(int step) -> void
{
  step = std::clamp(step, -18, 18);
  commands_["VY"][1] = conver2u8(step, param_limits_.vy);
  this->send("VY");
}

auto DogzillaDriver::turn(int step) -> void
{
  step = std::clamp(step, -70, 70);
  if (step > 0 && step < 30) { step = 30; }
  if (step > -30 && step < 0) { step = -30; }

  commands_["VYAW"][1] = conver2u8(step, param_limits_.vyaw);
  this->send("VY");
}

auto DogzillaDriver::forward(int step) -> void { this->moveX(std::abs(step)); }

auto DogzillaDriver::backwards(int step) -> void { this->moveX(-std::abs(step)); }

auto DogzillaDriver::left(int step) -> void { this->moveY(std::abs(step)); }

auto DogzillaDriver::right(int step) -> void { this->moveY(-std::abs(step)); }

auto DogzillaDriver::turnLeft(int step) -> void { this->turn(std::abs(step)); }

auto DogzillaDriver::turnRight(int step) -> void { this->turn(-std::abs(step)); }

auto DogzillaDriver::markTime(int data) -> void
{
  commands_["MarkTime"][1] = data ? conver2u8(data, param_limits_.mark_time, 1) : 0;
  this->send("MarkTime");
}

auto DogzillaDriver::action(int action_id) -> void
{
  if (action_id <= 0 || action_id > 255) { throw std::out_of_range("action_id must be between 1 and 255"); }

  commands_["ACTION"][1] = action_id;
  this->send("ACTION");
}

auto DogzillaDriver::reset() -> void
{
  this->action(255);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

auto DogzillaDriver::motorSend(int index, int data) -> void
{
  commands_["MOTOR_ANGLE"][index] = conver2u8(data, param_limits_.motor[index % 3 - 1]);
  this->send("MOTOR_ANGLE", index);
}

auto DogzillaDriver::motor(const auto &motor_id, int data) -> void
{
  std::array<int, 12> set_motor_id{ 11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43 };

  if constexpr (std::is_same_v<std::decay_t<decltype(motor_id)>, std::array<double, 12>>) {}
}


auto DogzillaDriver::readBattery() -> int
{
  uint8_t read_len = 1;
  this->read(commands_["BATTERY"][0], read_len);

  int battery = 0;
  if (this->unpack()) { battery = static_cast<int>(rx_data_[0]); }
  return battery;
}


auto DogzillaDriver::unpack(int timeout) -> bool
{
  auto start_time = std::chrono::steady_clock::now();
  std::vector<uint8_t> rx_msg;
  while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(timeout)) {
    int bytes = 0;

    // TODO: make it portable
    // System call Linux
    ioctl(serial_port_.lowest_layer().native_handle(), FIONREAD, &bytes);

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
          rx_check = std::accumulate(rx_data_.begin(), rx_data_.begin() + (rx_len_ - 8), 0);

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

auto DogzillaDriver::conver2u8(const double data, const auto &limit, const int mode) -> uint8_t
{
  const uint8_t max = 0xFF;
  const uint8_t min = mode ? 0x01 : 0x00;

  if constexpr (std::is_same_v<std::decay_t<decltype(limit)>, std::array<double, 2>>) {
    double limit_min = limit[0];
    double limit_max = limit[1];
    if (data >= limit_max) {
      return max;
    } else if (data <= limit_min) {
      return min;
    } else {
      double scaled_value = 255.0 / (limit_max - limit_min) * (data - limit_min);
      return static_cast<uint8_t>(scaled_value);
    }
  }

  else {
    if (data >= limit) {
      return max;
    } else if (data <= -limit) {
      return min;
    } else {
      return static_cast<uint8_t>(128 + 128 * data / limit);
    }
  }
}
