#pragma once
#include <cstdlib>
#include <string>
#include <stdexcept>
namespace cpv69 {
// Private compatibility opt-in, NOT a new factor/admission mode.
inline bool retain() {
  const char* e=std::getenv("CPV69_RETAIN_COMPAT");
  if(!e || std::string(e)=="0") return false;
  if(std::string(e)!="1") throw std::runtime_error("CPV69 invalid compatibility selector");
  const auto is=[](const char* key,const char* want){const char* p=std::getenv(key);return p&&std::string(p)==want;};
  if(!is("CPK9_PRIOR_REFRESH","1") || !is("FL4Q_ENABLE","0") ||
     !is("CPV40_REFRESH","1") || !is("CPV38_OFF_REFRESH","0") ||
     !is("FL4M_RETIREMENT","retain"))
    throw std::runtime_error("CPV69 requires corrected CP40/K9 retain with FL4Q and CP38 disabled");
  return true;
}
}
