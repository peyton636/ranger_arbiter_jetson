#include "types/nav_types.hpp"

namespace navigation
{

std::string navStateToString(NavState state)
{
  switch (state) {
    case NavState::IDLE: return "IDLE";
    case NavState::NAVIGATING: return "NAVIGATING";
    case NavState::ARRIVED: return "ARRIVED";
    case NavState::FAILED: return "FAILED";
    case NavState::BLOCKED: return "BLOCKED";
    default: return "UNKNOWN";
  }
}

std::string navActionStatusToString(NavActionStatus status)
{
  switch (status) {
    case NavActionStatus::IDLE: return "IDLE";
    case NavActionStatus::PENDING: return "PENDING";
    case NavActionStatus::SUCCEEDED: return "SUCCEEDED";
    case NavActionStatus::FAILED: return "FAILED";
    case NavActionStatus::CANCELED: return "CANCELED";
    default: return "UNKNOWN";
  }
}

}  // namespace navigation
