#pragma once
#include "NativePriorRefresh.h"

// Private CP102 backend integration. No solver members or settings are added.
namespace cpv102 {
cpk9::Promotion promoteWithFailureArchive(
    gtsam::NonlinearFactorGraph& graph, const gtsam::Values& theta,
    const gtsam::GaussianFactorGraph& cache, gtsam::KeySet& fixed,
    int isam_update_count, int agent_id);
}
