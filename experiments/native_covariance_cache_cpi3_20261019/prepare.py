"""CP-I3: isolated mechanical overlay, never edit prior builds or captures."""
from pathlib import Path
import hashlib
import json
import shutil
import difflib

HERE = Path(__file__).resolve().parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
BASE = WS/'runs/shared_linearization_s4_provider_20261012/release_2'
PREV = WS/'runs/local_revision_covariance_cpi1_20261019'
OUT = WS/'runs/native_covariance_cache_cpi3_20261019'

def swap(s, old, new):
    assert s.count(old) == 1, (old, s.count(old))
    return s.replace(old, new)

def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    assert not OUT.exists(), 'refuse overwrite'
    files = [p for p in (BASE/'source').rglob('*') if p.is_file()]
    files += [BASE/'libgtsam_points.so.1.2.1', BASE/'libglim_ros.so',
              BASE/'commands.json', PREV/'libglim_ros.so', PREV/'cbs_bridge.cpp',
              PREV/'RevisionCovariance.h', PREV/'test_revision.cpp']
    original = (BASE/'source/NativeGaussianHistory.cpp').read_text()
    header = (BASE/'source/NativeGaussianHistory.h').read_text()
    # An independently compiled, literal original provides the numerical oracle.
    oracle_h = header.replace('namespace shared_native {', 'namespace uncached_native {')
    oracle_c = original.replace('namespace shared_native {', 'namespace uncached_native {')
    oracle_c = oracle_c.replace('#include "NativeGaussianHistory.h"', '#include "OracleHistory.h"')
    header = swap(header, '  Id epoch() const;',
        '  Id epoch() const;\n  // Diagnostic count for the current committed epoch, not lifetime.\n'
        '  std::size_t queryPreparations() const;')
    candidate = swap(original, '  gtsam::Values theta;\n  gtsam::GaussianFactorGraph graph() const {',
        '''  gtsam::Values theta;
  // Query-only memoization owned by this immutable committed epoch. The tree
  // contains native equations only. Access is single-threaded, like history.
  mutable gtsam::GaussianBayesTree::shared_ptr query_tree;
  mutable std::size_t query_preparations = 0;
  gtsam::GaussianFactorGraph graph() const {''')
    candidate = swap(candidate,
        'Id NativeGaussianHistory::epoch() const { return committed_->epoch; }',
        'Id NativeGaussianHistory::epoch() const { return committed_->epoch; }\n'
        'std::size_t NativeGaussianHistory::queryPreparations() const { return committed_->query_preparations; }')
    candidate = swap(candidate, '    auto next = std::make_unique<State>(*committed_);',
        '''    auto next = std::make_unique<State>(*committed_);
    // Never inherit query preparation into a proposed epoch, even if its
    // numerical equations happen to be identical. Abort leaves committed intact.
    next->query_tree.reset();
    next->query_preparations = 0;''')
    candidate = swap(candidate, '  (void)graph(expected);',
        '''  require(mode_ == Mode::Shadow && expected != 0 && expected == committed_->epoch,
          "disabled, uninitialized or stale query");''')
    candidate = swap(candidate, '''  auto native = graph(expected);
  require(committed_->theta.exists(from) && committed_->theta.exists(to),
          "endpoint absent from committed native history");
  auto h = native.hessian().first;
  certifiedRankCheck(h);
  auto tree = native.eliminateMultifrontal(gtsam::Ordering::COLAMD, gtsam::EliminateCholesky);
  auto joint = tree->jointMarginalCovariance({from, to});''',
        '''  require(mode_ == Mode::Shadow && expected != 0 && expected == committed_->epoch,
          "disabled, uninitialized or stale query");
  require(committed_->theta.exists(from) && committed_->theta.exists(to),
          "endpoint absent from committed native history");
  if (!committed_->query_tree) {
    auto native = committed_->graph();
    auto h = native.hessian().first;
    certifiedRankCheck(h);
    // Install only after successful rank validation and elimination. No failed
    // preparation can leave a usable partial cache or bypass the next check.
    auto tree = native.eliminateMultifrontal(gtsam::Ordering::COLAMD, gtsam::EliminateCholesky);
    committed_->query_tree = std::move(tree);
    ++committed_->query_preparations;
  }
  auto joint = committed_->query_tree->jointMarginalCovariance({from, to});''')
    # Chart transport, complete endpoint matrix and relative helper are untouched.
    marker = '  EndpointCovariance out;'
    assert candidate[candidate.index(marker):] == original[original.index(marker):]
    OUT.mkdir()
    shutil.copytree(BASE/'source', OUT/'source')
    (OUT/'source/NativeGaussianHistory.h').write_text(header)
    (OUT/'source/NativeGaussianHistory.cpp').write_text(candidate)
    (OUT/'source/OracleHistory.h').write_text(oracle_h)
    (OUT/'source/OracleHistory.cpp').write_text(oracle_c)
    for name in ['RevisionCovariance.h', 'test_revision.cpp']:
        shutil.copy2(PREV/name, OUT/'source'/name)
    for name in ['test_cache.cpp', 'benchmark.cpp', 'build.py']:
        shutil.copy2(HERE/name, OUT/name)
    reader = Path('/home/yeranis/repos/gnssFGO/experiments/shared_linearization_s4_timing_20261012/replay.cpp')
    (OUT/'source/JournalReader.h').write_text(reader.read_text().split('int main(int argc,char** argv){')[0])
    files.append(reader)
    (OUT/'protected.json').write_text(json.dumps({str(p):sha(p) for p in files}, indent=2)+'\n')
    for name, old, new in [('NativeGaussianHistory.h', (BASE/'source/NativeGaussianHistory.h').read_text(), header),
                           ('NativeGaussianHistory.cpp', original, candidate)]:
        (OUT/(name+'.patch')).write_text(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='baseline/'+name,tofile='candidate/'+name)))
    print(OUT)

if __name__ == '__main__':
    main()
