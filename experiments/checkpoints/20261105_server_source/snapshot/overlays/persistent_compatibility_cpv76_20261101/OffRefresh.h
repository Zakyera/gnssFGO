#pragma once
#include "NativePriorRefresh.h"

namespace cpv38 {
inline bool enabled() {
  const char* value = std::getenv("CPV38_OFF_REFRESH");
  if (!value || std::string(value) == "0") return false;
  if (std::string(value) != "1" || !cpk9::enabled())
    throw std::runtime_error("CPV38 invalid selector/CPK9 prerequisite");
  return true;
}
}  // namespace cpv38
