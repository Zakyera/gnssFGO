#pragma once
#include <cstdlib>
#include <cstring>
#include <stdexcept>

namespace cpv82 {
// Private opt-in. It admits the existing factor path, not new factor semantics.
inline bool persistentCompatibility(const char* value) {
  if (!value || !*value || std::strcmp(value, "0") == 0) return false;
  if (std::strcmp(value, "1") == 0) return true;
  throw std::runtime_error("unsupported CPV82_GLIM_PERSISTENT_COMPAT");
}
inline bool persistentCompatibility() {
  return persistentCompatibility(std::getenv("CPV82_GLIM_PERSISTENT_COMPAT"));
}
}  // namespace cpv82
