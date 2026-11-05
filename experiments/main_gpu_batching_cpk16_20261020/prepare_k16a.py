"""Create a NEW private overlay; exact mechanical replacements, no baseline edits."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

H = Path(__file__).resolve().parent
W = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
R = W / "runs/main_gpu_batching_cpk16_20261020"
P = W / "runs/gnssfgo_eth1_full_rate020_cpk14_20261020/prepared"
BASE = W / "runs/shared_linearization_s4_provider_20261012/release_2"

def sha(p):
    with Path(p).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()

def replace_once(text, old, new):
    assert text.count(old) == 1, old[:100]
    return text.replace(old, new, 1)

def main():
    assert not R.exists(), "refuse existing private root"
    protected = json.loads((P / "protected_manifest.json").read_text())
    assert all(sha(p) == h for p, h in protected.items()), "baseline changed"
    inputs = [
        W/"src/gtsam_points/src/gtsam_points/optimizers/isam2_ext.cpp",
        BASE/"overlay/incremental_fixed_lag_smoother_ext_with_fallback.cpp",
        W/"runs/native_covariance_cache_cpi3_20261019/release_v2/commands.json",
        W/"runs/e1b4a_local_update_batching_20261012/authoring/LocalUpdateBatching.h",
        W/"src/glim_ros1/include/glim_ros/cbs_local_graph_linearizer.hpp",
        W/"runs/e1b4a_snapshot_checker_20261012/authoring/ExactGpuSnapshot.cpp",
        W/"runs/e1b4a_snapshot_checker_20261012/authoring/ExactGpuSnapshot.h",
    ]
    R.mkdir()
    (R/"source").mkdir()
    (R/"protected.json").write_text(json.dumps(protected, indent=2)+"\n")
    (R/"input_hashes.json").write_text(json.dumps({str(p):sha(p) for p in inputs}, indent=2)+"\n")
    states = {}
    for name in ["glim_ros1", "gtsam_points", "glim", "cbs", "Kimera-VIO"]:
        repo = W/"src"/name
        states[name] = dict(head=subprocess.check_output(["git","-C",str(repo),"rev-parse","HEAD"],text=True).strip(),
                           status=subprocess.check_output(["git","-C",str(repo),"status","--porcelain"],text=True),
                           diff_sha256=hashlib.sha256(subprocess.check_output(["git","-C",str(repo),"diff","HEAD","--binary"])).hexdigest())
    (R/"source_git.json").write_text(json.dumps(states, indent=2)+"\n")
    for name in ["MainGpuBatch.h","MainGpuBatch.cpp","test_main_batch.cpp","build_k16a.py"]:
        shutil.copy2(H/name, R/"source"/name)
    for name in ["ExactGpuSnapshot.h","ExactGpuSnapshot.cpp"]:
        shutil.copy2(W/"runs/e1b4a_snapshot_checker_20261012/authoring"/name,R/"source"/name)
    shutil.copytree(W/"runs/e1b4a_snapshot_checker_20261012/include", R/"snapshot_include")
    src = inputs[0].read_text()
    src = '#include "MainGpuBatch.h"\n' + src
    # Prepare only the exact uncached subset. Reassemble original synchronous
    # order, interleaving untouched cached factors at their original positions.
    anchor = "  linearization_hook->clear();\n  std::vector<FactorIndex> gpu_factor_indices;"
    addition = """  if (cpk16::active(this)) {
    NonlinearFactorGraph fresh;
    FactorIndices selected;
    std::vector<bool> reuse;
    for (const FactorIndex idx : candidates) {
      bool inside = true, cached = params_.cacheLinearizedFactors;
      for (Key key : nonlinearFactors_[idx]->keys()) {
        if (affectedKeysSet.find(key) == affectedKeysSet.end()) { inside = false; break; }
        if (cached && relinKeys.find(key) != relinKeys.end()) cached = false;
      }
      if (!inside) continue;
      selected.push_back(idx); reuse.push_back(cached);
      if (!cached) fresh.push_back(nonlinearFactors_[idx]);
    }
    auto prepared = cpk16::tryLinearize(this, fresh, theta_, cpk16::Site::Affected);
    if (prepared) {
      size_t j = 0;
      for (size_t i = 0; i < selected.size(); ++i) {
        const auto idx = selected[i];
        if (reuse[i]) linearized.push_back(linearFactors_[idx]);
        else {
          const auto& factor = (*prepared)[j++];
          linearized.push_back(factor);
          if (params_.cacheLinearizedFactors) linearFactors_[idx] = factor;
        }
      }
      gttoc(check_candidates_and_linearize);
      return linearized;
    }
  }

"""
    src = replace_once(src, anchor, addition+anchor)
    old = """  // issue linearization of GPU factors
  linearization_hook->clear();
  linearization_hook->add(nonlinearFactors_);
  linearization_hook->linearize(theta_);

  auto linearized = nonlinearFactors_.linearize(theta_);"""
    new = """  auto linearized = cpk16::tryLinearize(this, nonlinearFactors_, theta_, cpk16::Site::FullGraph);
  if (!linearized) {
    // Unchanged native/default route, including any existing native hooks.
    linearization_hook->clear();
    linearization_hook->add(nonlinearFactors_);
    linearization_hook->linearize(theta_);
    linearized = nonlinearFactors_.linearize(theta_);
  }"""
    src = replace_once(src, old, new)
    old = """  linearization_hook->clear();
  linearization_hook->add(newFactors);
  linearization_hook->linearize(theta_);

  update.linearizeNewFactors(newFactors, theta_, nonlinearFactors_.size(), result.newFactorsIndices, &linearFactors_);"""
    new = """  auto prepared_new = cpk16::tryLinearize(this, newFactors, theta_, cpk16::Site::NewFactors);
  if (prepared_new) {
    // Same slot assignment as UpdateImpl::linearizeNewFactors.
    if (params_.findUnusedFactorSlots) {
      linearFactors_.resize(nonlinearFactors_.size());
      for (size_t i = 0; i < newFactors.size(); ++i)
        linearFactors_[result.newFactorsIndices[i]] = (*prepared_new)[i];
    } else linearFactors_.push_back(*prepared_new);
    assert(linearFactors_.size() == nonlinearFactors_.size());
  } else {
    linearization_hook->clear();
    linearization_hook->add(newFactors);
    linearization_hook->linearize(theta_);
    update.linearizeNewFactors(newFactors, theta_, nonlinearFactors_.size(), result.newFactorsIndices, &linearFactors_);
  }"""
    src = replace_once(src, old, new)
    start = src.index("        // Create factor graph from factor indices")
    end = src.index("        // Reeliminate the linear graph",start)
    original = src[start:end]
    replacement = """        GaussianFactorGraph::shared_ptr prepared_marginal;
        if (cpk16::active(this)) {
          NonlinearFactorGraph selected;
          for (const auto index : factorsFromMarginalizedInClique_step1)
            selected.push_back(nonlinearFactors_[index]);
          prepared_marginal = cpk16::tryLinearize(this, selected, theta_, cpk16::Site::Marginalize);
        }
        if (prepared_marginal) graph.push_back(*prepared_marginal);
        else {
""" + original + "        }\n\n"
    src = src[:start]+replacement+src[end:]
    (R/"source/isam2_ext.cpp").write_text(src)
    wrapper = inputs[1].read_text()
    wrapper = '#include "MainGpuBatch.h"\n' + wrapper
    wrapper = replace_once(wrapper,
        "  shared_native::UpdateScope shadow(this, newFactors, factorsToRemove);",
        """  // Only the explicitly attached MAIN wrapper may enable the candidate.
  cpk16::Scope main_batch(&smoother->getISAM2(),
      cpk16::requested() && bool(shared_native::lookup(this)));
  shared_native::UpdateScope shadow(this, newFactors, factorsToRemove);""")
    (R/"source/incremental_fixed_lag_smoother_ext_with_fallback.cpp").write_text(wrapper)
    for original,generated in [(inputs[0],R/"source/isam2_ext.cpp"),(inputs[1],R/"source/incremental_fixed_lag_smoother_ext_with_fallback.cpp")]:
        diff=subprocess.run(["diff","-u",str(original),str(generated)],capture_output=True,text=True)
        assert diff.returncode==1
        (R/(generated.name+".patch")).write_text(diff.stdout)
    print(json.dumps(dict(prepared=str(R),protected=len(protected),source_edits=0)))

if __name__ == "__main__": main()
