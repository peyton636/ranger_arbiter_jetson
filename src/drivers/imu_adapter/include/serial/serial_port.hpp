#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace imu_adapter
{

/// @brief Linux termios 串口封装（只读轮询）
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  bool open(const std::string & device, int baudrate);
  void close();
  [[nodiscard]] bool isOpen() const;

  /// 读取当前可读字节，非阻塞
  std::vector<uint8_t> readAvailable();

private:
  int fd_{-1};
};

}  // namespace imu_adapter
