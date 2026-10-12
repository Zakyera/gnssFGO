#include "RuntimeProfile.h"
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>

namespace e1b {
thread_local ThreadState* current=nullptr;
struct Collector { Record record, workers; std::mutex mutex; };
namespace {
void merge(Record& target, const Record& source) noexcept {
  for (unsigned i=0; i<static_cast<unsigned>(Stage::Count); ++i) {
    auto& t=target.worker[i]; const auto& s=source.worker[i];
    t.calls+=s.calls; t.inclusive_ns+=s.inclusive_ns;
    t.exclusive_ns+=s.exclusive_ns; t.exceptions+=s.exceptions;
  }
  for (unsigned i=0; i<static_cast<unsigned>(Metric::Count); ++i)
    target.metrics[i]+=source.metrics[i];
}
struct Sink {
  FILE* file=nullptr;
  std::mutex mutex;
  Sink() noexcept {
    const char* path=std::getenv("CBS_E1B_PROFILE_PATH");
    if (!path || !*path) return;
    file=std::fopen(path,"wx"); // never overwrite an earlier profile
    if (!file) std::fprintf(stderr,"E1B_PROFILE_SINK_ERROR: refusing unavailable/existing output %s\n",path);
  }
  ~Sink() { if(file) std::fclose(file); }
};
Sink& sink() { static Sink value; return value; }
void emit(const Record& r) {
  std::ostringstream out;
  out<<std::setprecision(17)<<"{\"schema\":1,\"timestamp\":"<<r.timestamp
     <<",\"context\":"<<static_cast<int>(r.context)<<",\"aborted\":"<<(r.aborted?"true":"false");
  for (bool worker : {false,true}) {
    out<<(worker?",\"worker_sum\":{":",\"owner_wall\":{"); bool first=true;
    const auto& cells=worker?r.worker:r.owner;
    for (unsigned i=0;i<cells.size();++i) {
      const auto& c=cells[i]; if(!c.calls)continue;
      if(!first)out<<','; first=false;
      out<<'"'<<stageName(static_cast<Stage>(i))<<"\":["<<c.calls<<','
         <<c.inclusive_ns<<','<<c.exclusive_ns<<','<<c.exceptions<<']';
    }
    out<<'}';
  }
  out<<",\"metrics\":[";
  for(unsigned i=0;i<r.metrics.size();++i){if(i)out<<',';out<<r.metrics[i];}
  out<<"]}\n";
  const std::string line=out.str();
  std::lock_guard<std::mutex> lock(sink().mutex);
  if(std::fwrite(line.data(),1,line.size(),sink().file)!=line.size() || std::fflush(sink().file))
    std::fprintf(stderr,"E1B_PROFILE_SINK_ERROR: write failed\n");
}
}  // namespace
const char* stageName(Stage stage) noexcept {
  static constexpr const char* names[]={"root","worker_root","local_total","bpsam","prepare","backup","base_update",
    "post_update","estimate","marginalize","cleanup","metadata","gpu_linearize","gpu_error","gpu_sync_linearize",
    "gpu_sync_error","gpu_warning","factor_graph_linearize","isam_update","isam_delta","isam_recalculate",
    "isam_batch","isam_incremental","isam_affected","isam_marginalize"};
  static_assert(sizeof(names)/sizeof(*names)==static_cast<unsigned>(Stage::Count));
  return names[static_cast<unsigned>(stage)];
}
struct Session::Impl {
  Collector collector;
  ThreadState state{&collector.record,&collector,nullptr,false};
  ThreadState* previous=nullptr;
  Record* result;
  int exceptions;
  std::optional<Scope> root;
  Impl(double stamp,Context context,Record* output):result(output),exceptions(std::uncaught_exceptions()) {
    collector.record.timestamp=stamp; collector.record.context=context;
    previous=current; current=&state; root.emplace(Stage::Root);
  }
  ~Impl() noexcept {
    root.reset(); current=previous;
    merge(collector.record,collector.workers);
    collector.record.aborted=std::uncaught_exceptions()>exceptions;
    // All TBB ranges have joined before session teardown. No background use.
    if(result) *result=collector.record;
    else try { emit(collector.record); } catch (...) {
      std::fprintf(stderr,"E1B_PROFILE_SINK_ERROR: serialization failed\n");
    }
  }
};
Session::Session(double stamp,Context context,Record* result) {
  if(result || sink().file) impl_=std::make_unique<Impl>(stamp,context,result);
}
Session::~Session() noexcept=default;
struct Attach::Impl {
  Record record;
  ThreadState state;
  ThreadState* previous;
  std::optional<Scope> root;
  explicit Impl(Collector* collector):state{&record,collector,nullptr,true},previous(current) {
    current=&state; root.emplace(Stage::WorkerRoot);
  }
  ~Impl() noexcept {
    root.reset(); current=previous;
    std::lock_guard<std::mutex> lock(state.collector->mutex);
    merge(state.collector->workers,record);
  }
};
Attach::Attach(Token token) {
  if(token.collector && (!current || current->collector!=token.collector))
    impl_=std::make_unique<Impl>(token.collector);
}
Attach::~Attach() noexcept=default;
}  // namespace e1b
