#pragma once
#include "OffRefresh.h"
#include <sstream>
#include <iomanip>
namespace cpv40 {
inline bool enabled() {
  const char* value=std::getenv("CPV40_REFRESH");
  if (!value || std::string(value)=="0") return false;
  if (std::string(value)!="1" || !cpk9::enabled() || cpv38::enabled())
    throw std::runtime_error("CPV40 invalid selector/prerequisite or ambiguous CPV38");
  return true;
}
}
