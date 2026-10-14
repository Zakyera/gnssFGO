// Read-only FL4e diagnostic of the saved ON journal; never used as a receiver.
#define main FL4E_UNUSED_MAIN
#include "../receiver_endpoint_reuse_e2b_20261013/DiagnosticReplay.cpp"
#undef main
#include "../fixed_lag_slots_fl3i_20261014/Receiver.h"

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  try {
    EventStream input(argv[1]);
    fl3i::Receiver receiver(argv[2]);
    const auto dumpX71 = [&](const char* stage) {
      const auto& isam = receiver.state().solver.isam();
      const auto key = X(71);
      std::set<const void*> seen;
      std::cerr << stage << " node_count=" << isam.nodes().size()
                << " x71_node=" << (isam.nodes().find(key) != isam.nodes().end()) << '\n';
      for (const auto& entry : isam.nodes()) {
        const auto clique = entry.second;
        if (!seen.insert(clique.get()).second) continue;
        const auto& keys = clique->conditional()->keys();
        if (std::find(keys.begin(), keys.end(), key) == keys.end()) continue;
        std::cerr << "  frontals=";
        for (auto k : clique->conditional()->frontals()) std::cerr << gtsam::Symbol(k) << ' ';
        std::cerr << " parents=";
        for (auto k : clique->conditional()->parents()) std::cerr << gtsam::Symbol(k) << ' ';
        std::cerr << '\n';
      }
    };
    for (size_t eventno = 1; eventno <= 8129; ++eventno) {
      const auto event = input.Next();
      if (eventno == 8128) dumpX71("before_retire");
      if (eventno == 8129) {
        const auto& state = receiver.state();
        const auto& isam = state.solver.isam();
        const auto& graph = state.solver.getFactors();
        const auto& vi = isam.getVariableIndex();
        const auto key = X(71);
        size_t graphReferences = 0;
        for (const auto& factor : graph)
          if (factor && std::find(factor->keys().begin(), factor->keys().end(), key) != factor->keys().end())
            ++graphReferences;
        size_t cliqueReferences = 0;
        for (const auto& entry : isam.nodes()) {
          const auto& keys = entry.second->conditional()->keys();
          if (std::find(keys.begin(), keys.end(), key) != keys.end()) ++cliqueReferences;
        }
        std::cerr << "before_event=" << eventno << " kind=" << event.kind
                  << " origin=" << state.origin << " oldest=" << state.oldest
                  << " newest=" << state.newest << " x71_in_index="
                  << (vi.find(key) != vi.end()) << " x71_graph_factors="
                  << graphReferences << " x71_in_theta="
                  << isam.getLinearizationPoint().exists(key)
                  << " x71_in_nodes=" << (isam.nodes().find(key) != isam.nodes().end())
                  << " x71_clique_refs=" << cliqueReferences
                  << " x71_fixed=" << isam.getFixedVariables().exists(key) << '\n';
        const auto& r = event.belief;
        fl3i::Id id{r.source_agent == "g" ? "glim" : r.source_agent == "k" ? "kimera" : r.source_agent,
                    r.session_id, r.from_index, r.to_index};
        std::cerr << "incoming=" << fl3i::Name(id) << " revision=" << r.revision
                  << " start=" << r.from_timestamp_ns << " end=" << r.to_timestamp_ns << '\n';
        const auto found = state.active.find(id);
        if (found != state.active.end()) {
          std::cerr << "old_slot=" << found->second.slot << " old_factor_keys=";
          for (auto k : found->second.owner->keys()) std::cerr << gtsam::Symbol(k) << ' ';
          std::cerr << '\n';
        }
      }
      if (event.kind == 'I') receiver.imuClock(event.imu.timestamp_ns, event.arrival_ns);
      else if (event.kind == 'B') {
        const auto& r = event.belief;
        fl3i::Belief b;
        b.source = r.source_agent == "g" ? "glim" : r.source_agent == "k" ? "kimera" : r.source_agent;
        b.session = r.session_id; b.from = r.from_index; b.to = r.to_index;
        b.ordinal = r.ordinal; b.revision = r.revision;
        b.start = r.from_timestamp_ns; b.end = r.to_timestamp_ns;
        b.arrival = r.record_timestamp_ns; b.mean = r.measurement; b.covariance = r.covariance;
        receiver.belief(b);
      } else throw std::runtime_error("unexpected end event");
      if (eventno == 8128) dumpX71("after_retire");
    }
  } catch (const std::exception& e) {
    std::cerr << "probe_error=" << e.what() << '\n';
    return 1;
  }
  return 0;
}
