#include "serial/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <system_error>

namespace imu_adapter
{

namespace
{

speed_t baudToConstant(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B9600;
  }
}

}  // namespace

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, const int baudrate)
{
  close();

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios options{};
  if (tcgetattr(fd_, &options) != 0) {
    close();
    return false;
  }

  cfmakeraw(&options);
  cfsetispeed(&options, baudToConstant(baudrate));
  cfsetospeed(&options, baudToConstant(baudrate));
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CRTSCTS;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &options) != 0) {
    close();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::isOpen() const
{
  return fd_ >= 0;
}

std::vector<uint8_t> SerialPort::readAvailable()
{
  std::vector<uint8_t> out;
  if (fd_ < 0) {
    return out;
  }

  uint8_t buf[256];
  while (true) {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      out.insert(out.end(), buf, buf + n);
      continue;
    }
    if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    break;
  }
  return out;
}

}  // namespace imu_adapter
