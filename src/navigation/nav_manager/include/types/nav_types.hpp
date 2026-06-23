#pragma once

#include <cstdint>
#include <string>

namespace navigation
{

enum class NavState : uint8_t
{
  IDLE = 0,
  NAVIGATING,
  ARRIVED,
  FAILED,
  BLOCKED,
};

enum class NavActionStatus : uint8_t
{
  IDLE = 0,
  PENDING,
  SUCCEEDED,
  FAILED,
  CANCELED,
};

std::string navStateToString(NavState state);
std::string navActionStatusToString(NavActionStatus status);

}  // namespace navigation
