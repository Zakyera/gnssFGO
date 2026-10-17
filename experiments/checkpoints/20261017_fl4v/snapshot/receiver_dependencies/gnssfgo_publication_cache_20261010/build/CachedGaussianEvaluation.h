#pragma once
#include "offline_process/StableGaussianCost.h"
#include <memory>
#include <optional>

namespace publication_cache {
using fgo::offline::StableCostChange;
using fgo::offline::StableSum;

// Local to one refinement call with an immutable graph. Values are owned copies;
// factor slots are copied (shared immutable factors), never modified. No global
// cache and no reuse across graph updates, revisions or solver commits.
class Evaluation {
 public:
  Evaluation(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& values)
      : graph_(graph), values_(values), residuals_(graph.size()) {}
  const gtsam::Values& values() const { return values_; }
  const gtsam::NonlinearFactorGraph& graph() const { return graph_; }
  const gtsam::GaussianFactorGraph& linear() const {
    if (!linear_) { linear_ = graph_.linearize(values_); ++linearizations; }
    return *linear_;
  }
  const gtsam::Vector& residual(size_t slot) const {
    if (!residuals_.at(slot)) {
      auto f = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph_.at(slot));
      if (!f) throw std::runtime_error("expected noise model factor");
      residuals_[slot] = f->whitenedError(values_); ++residual_evaluations;
    }
    return *residuals_[slot];
  }
  double gradient() const {
    const auto g = linear().gradientAtZero();
    const auto d = linear().hessianDiagonal();
    double maximum = 0;
    for (const auto& [key, v] : g) {
      const auto& diagonal = d.at(key);
      if (!v.allFinite() || !diagonal.allFinite() || (diagonal.array() < 0).any())
        throw std::runtime_error("nonfinite/negative linear system entry");
      for (int i = 0; i < v.size(); ++i) {
        if (diagonal[i] == 0 && v[i] != 0)
          throw std::runtime_error("nonzero gradient in a zero-information coordinate");
        if (diagonal[i] > 0) maximum = std::max(maximum, std::abs(v[i]) / std::sqrt(diagonal[i]));
      }
    }
    return maximum;
  }
  mutable size_t linearizations{0}, residual_evaluations{0};
 private:
  const gtsam::NonlinearFactorGraph graph_;
  const gtsam::Values values_;
  mutable gtsam::GaussianFactorGraph::shared_ptr linear_;
  mutable std::vector<std::optional<gtsam::Vector>> residuals_;
};

inline void ValidatePair(const Evaluation& a, const Evaluation& b) {
  if (a.graph().size() != b.graph().size()) throw std::runtime_error("cache graph changed");
  for (size_t slot = 0; slot < a.graph().size(); ++slot) {
    if (a.graph()[slot] != b.graph()[slot]) throw std::runtime_error("cache factor identity changed");
    if (!a.graph()[slot]) continue;
    const auto f = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(a.graph()[slot]);
    if (!f || !boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(f->noiseModel()) ||
        boost::dynamic_pointer_cast<gtsam::noiseModel::Constrained>(f->noiseModel()))
      throw std::runtime_error("stable comparison requires unconstrained Gaussian factors");
    if (f->active(a.values()) != f->active(b.values())) throw std::runtime_error("factor activation changed");
  }
}

inline StableCostChange ActualChange(const Evaluation& a, const Evaluation& b) {
  ValidatePair(a, b);
  StableSum actual;
  for (size_t slot = 0; slot < a.graph().size(); ++slot)
    if (a.graph()[slot] && a.graph()[slot]->active(a.values()))
      fgo::offline::AccumulateResidualChange(a.residual(slot), b.residual(slot), actual);
  return {actual.value(), actual.arithmeticBound(), 0, 0, actual.terms};
}

inline StableCostChange PredictedChange(const Evaluation& a, const Evaluation& b) {
  ValidatePair(a, b);
  const auto displacement = a.values().localCoordinates(b.values());
  const auto& linear = a.linear();
  StableSum predicted;
  for (size_t slot = 0; slot < a.graph().size(); ++slot) if (a.graph()[slot]) {
    if (!a.graph()[slot]->active(a.values())) continue;
    const auto jf = boost::dynamic_pointer_cast<gtsam::JacobianFactor>(linear[slot]);
    if (!jf) throw std::runtime_error("expected Gaussian Jacobian factor");
    gtsam::Vector ad = gtsam::Vector::Zero(jf->rows());
    for (auto it = jf->begin(); it != jf->end(); ++it) ad.noalias() += jf->getA(it) * displacement.at(*it);
    const auto rhs = jf->getb();
    for (int j = 0; j < ad.size(); ++j)
      predicted.add((long double)ad[j] * ((long double)ad[j] / 2.L - (long double)rhs[j]));
  }
  return {0, 0, predicted.value(), predicted.arithmeticBound(), 0};
}

inline StableCostChange CumulativeChange(const Evaluation& anchor, const Evaluation& seed,
                                       const Evaluation& candidate) {
  auto result = ActualChange(anchor, candidate);
  const auto local = PredictedChange(seed, candidate);
  result.predicted_delta = local.predicted_delta;
  result.predicted_bound = local.predicted_bound;
  return result;
}
}  // namespace publication_cache
