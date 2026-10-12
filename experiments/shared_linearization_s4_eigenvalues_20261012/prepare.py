"""Mechanically stage immutable inputs and one-line private candidate."""
from pathlib import Path
import difflib
import hashlib
import json
import shutil

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT = WS / 'runs/shared_linearization_s4_eigenvalues_20261012'
S2 = REPO / 'experiments/shared_linearization_s2_20261012'
S4 = REPO / 'experiments/shared_linearization_s4_timing_20261012'

def main():
    ROOT.mkdir(exist_ok=False)
    driver = ROOT / 'driver'; driver.mkdir()
    for name in ['NativeGaussianHistory.cpp', 'NativeGaussianHistory.h', 'test_native_gaussian.cpp']:
        shutil.copy2(S2 / name, driver / name)
    for name in ['replay.cpp', 'Timing.h']:
        shutil.copy2(S4 / name, driver / name)
    shutil.copy2(S4 / 'build.py', driver / 'timing_transform.py')
    for name in ['build.py', 'run.py', 'rank_tests.cpp', 'environment.py']:
        shutil.copy2(HERE / name, driver / name)
    original = (driver / 'NativeGaussianHistory.cpp').read_text()
    old = 'Eigen::SelfAdjointEigenSolver<gtsam::Matrix> eigen(h);'
    new = 'Eigen::SelfAdjointEigenSolver<gtsam::Matrix> eigen(h, Eigen::EigenvaluesOnly);'
    assert original.count(old) == 1
    candidate = original.replace(old, new)
    (driver / 'candidate.cpp').write_text(candidate)
    (HERE / 'candidate.patch').write_text(''.join(difflib.unified_diff(
        original.splitlines(True), candidate.splitlines(True),
        fromfile='S2/NativeGaussianHistory.cpp', tofile='S4b/candidate.cpp')))
    # Expose the actual helper in a test-only TU, not in the production API.
    # Reference helpers are copied verbatim from original S2, not rewritten.
    start = original.index('void require(')
    end = original.index('bool sameChart(')
    access = '#include "candidate.cpp"\nnamespace shared_native {\n'
    access += 'void probeRank(const gtsam::Matrix& h) { positiveDefinite(h); }\n}\n'
    access += 'namespace original_rank {\n' + original[start:end] + '}\n'
    (driver / 'rank_access.cpp').write_text(access)
    pins = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in driver.iterdir()}
    (ROOT / 'input_hashes.json').write_text(json.dumps(pins, indent=2))
    print(ROOT)

if __name__ == '__main__': main()
