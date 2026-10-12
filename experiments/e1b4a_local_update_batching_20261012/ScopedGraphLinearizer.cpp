#include "ScopedGraphLinearizer.h"
#include <stdexcept>

namespace gtsam { namespace e1b4a {
namespace {
thread_local Callback callback = nullptr;
thread_local void* context = nullptr;
thread_local unsigned depth = 0;
}
ScopedLinearizer::ScopedLinearizer(Callback cb, void* ctx) : installed_(false) {
  if (!cb) return;
  if (callback || depth) throw std::logic_error("nested graph linearizer scope");
  callback = cb;
  context = ctx;
  installed_ = true;
}
ScopedLinearizer::~ScopedLinearizer() {
  if (installed_) { callback = nullptr; context = nullptr; }
}
GraphCall::GraphCall() : outermost_(depth++ == 0) {}
GraphCall::~GraphCall() { --depth; }
GaussianFactorGraph::shared_ptr GraphCall::dispatch(
    const NonlinearFactorGraph& graph, const Values& values) const {
  if (!outermost_ || !callback) return {};
  auto result = callback(graph, values, context);
  if (result && result->size() != graph.size())
    throw std::invalid_argument("graph linearizer changed ordered slot count");
  return result;
}
}}  // namespace gtsam::e1b4a
