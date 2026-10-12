#pragma once
#include "LocalUpdateCheck.h"
#include <fstream>
#include <iomanip>
#include <thread>
#include <cstdlib>

namespace e1b4real {
inline void differences(std::ostream& o,const e1b4check::Differences& d) {
  o << "{\"position\":" << d.position << ",\"orientation\":" << d.orientation
    << ",\"velocity\":" << d.velocity << ",\"bias\":" << d.bias
    << ",\"gaussian\":" << d.gaussian << ",\"covariance\":" << d.covariance
    << ",\"relative\":" << d.relative << ",\"states\":" << d.states
    << ",\"slots\":" << d.slots << "}";
}
inline void update(e1b4check::Smoother& solver,
    const gtsam::NonlinearFactorGraph& factors,const gtsam::Values& values,
    const gtsam::FixedLagSmoother::KeyTimestampMap& stamps) {
  static const auto thread=std::this_thread::get_id();
  static size_t call=0;
  const char* root=std::getenv("CBS_E1B_REAL_INPUT_ROOT");
  try {
    e1b4check::require(root && *root,"missing observer output directory");
    e1b4check::require(thread==std::this_thread::get_id(),"LOCAL worker changed");
    e1b4check::require(!glim::e1b4a::enabled(),"authoritative batching must stay OFF");
    ++call;
    const auto before=solver.timestamps();
    bool selected=call==10||call==30||call==60||call==100||call==150||call==180;
    std::unique_ptr<e1b4check::LocalUpdateCheck> check;
    if(selected) {
      std::ofstream(std::string(root)+"/progress.txt") << "starting call " << call << std::endl;
      check=std::make_unique<e1b4check::LocalUpdateCheck>(solver,factors,values,stamps);
    }
    // Authoritative path is exactly one ordinary LOCAL update. No candidate
    // scope surrounds it, and its input factors are never used for diagnostics.
    solver.update(factors,values,stamps);
    if(check) {
      auto fidelity=check->finish(solver);
      std::ofstream out(std::string(root)+"/sample_"+std::to_string(call)+".json");
      out << std::setprecision(17) << "{\"passed\":true,\"call\":" << call
          << ",\"stamp\":" << (stamps.empty()?0:stamps.rbegin()->second)
          << ",\"paired\":";
      differences(out,check->paired());out << ",\"fidelity\":";differences(out,fidelity);
      const auto& s=check->stats();
      out << ",\"requests\":" << s.requests << ",\"batched_graphs\":" << s.batched_graphs
          << ",\"prepared_factors\":" << s.prepared_factors << ",\"fallback\":[";
      for(size_t i=0;i<s.fallback.size();++i)out << (i?",":"") << s.fallback[i];
      out << "]}\n";out.flush();e1b4check::require(bool(out),"cannot write sample");
    }
    size_t retired=0;for(const auto& x:before)if(!solver.timestamps().count(x.first))++retired;
    std::ofstream events(std::string(root)+"/updates.jsonl",std::ios::app);
    events << std::setprecision(17) << "{\"call\":" << call << ",\"stamp\":"
      << (stamps.empty()?0:stamps.rbegin()->second) << ",\"keys\":" << solver.timestamps().size()
      << ",\"slots\":" << solver.getFactors().size() << ",\"retired_keys\":" << retired << "}\n";
    events.flush();e1b4check::require(bool(events),"cannot write update journal");
  } catch(const std::exception& e) {
    if(root)std::ofstream(std::string(root)+"/FAILED.txt") << "call=" << call << ": " << e.what() << std::endl;
    // Never fall through the bridge's warning-only catch or silently proceed.
    std::_Exit(86);
  } catch(...) {
    if(root)std::ofstream(std::string(root)+"/FAILED.txt") << "unknown exception call=" << call << std::endl;
    std::_Exit(87);
  }
}
}
