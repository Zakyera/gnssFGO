"""Private diagnostic overlays only; production headers/DSOs are not edited."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT = WS/'runs/e1b4a_snapshot_checker_20261012'
OLD = WS/'runs/e1b_local_profile_20261012'

def sha(p):
    # Private runtime symlinks contain container-absolute /workspace paths.
    if p.is_symlink():
        target=p.readlink()
        if str(target).startswith('/workspace/cbs_gtsam4.3/'):
            p=WS/target.relative_to('/workspace/cbs_gtsam4.3')
        else:
            p=(p.parent/target).resolve()
        return sha(p)
    return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    if sys.argv[1:]==['--refresh-authoring']:
        assert (ROOT/'protected.json').exists()
        shutil.copytree(HERE,ROOT/'authoring',dirs_exist_ok=True)
        return
    # The first preparation stopped only on container-absolute symlink hashing.
    # Resume that incomplete preparation without overwriting its copied inputs.
    if ROOT.exists():
        assert not (ROOT/'protected.json').exists()
        assert not list(ROOT.glob('release_*'))
    else:
        ROOT.mkdir(exist_ok=False)
        shutil.copytree(HERE, ROOT/'authoring')
    inputs = [
        (WS/'src/gtsam_points/include', 'gtsam_points/factors/integrated_vgicp_factor_gpu.hpp',
         'class IntegratedVGICPFactorGPU : public gtsam_points::NonlinearFactorGPU {',
         '  friend class ExactGpuSnapshot;'),
        (WS/'src/gtsam_points/include', 'gtsam_points/factors/integrated_vgicp_derivatives.cuh',
         'class IntegratedVGICPDerivatives {', '  friend class ExactGpuSnapshot;'),
        (WS/'src/gtsam', 'gtsam/nonlinear/ISAM2.h',
         'class GTSAM_EXPORT ISAM2 : public BayesTree<ISAM2Clique> {',
         '  friend class LocalSnapshotAccess;'),
        (WS/'src/cbs/include', 'cbs/bpsam/bpsam.h',
         'class BPSAM : public BaseSAM {', '  friend class LocalSnapshotAccess;'),
        (OLD/'overlay/include', 'cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h',
         '    : public gtsam::FixedLagSmoother {', '  friend class LocalSnapshotAccess;'),
    ]
    manifest = {}
    for base, name, needle, friend in inputs:
        src = base/name
        content = src.read_text()
        assert content.count(needle) == 1, (name, needle)
        dest = ROOT/'include'/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        updated=content.replace(needle, needle+'\n'+friend)
        if dest.exists(): assert dest.read_text()==updated
        else: dest.write_text(updated)
        manifest[str(src)] = sha(src)
    prior = WS/'runs/e1b4a_local_update_batching_20261012/release_2'
    for p in prior.glob('lib*.so*'):
        manifest[str(p)] = sha(p)
    for p in prior.glob('test_*_3'):
        if p.is_file(): manifest[str(p)] = sha(p)
    (ROOT/'protected.json').write_text(json.dumps(manifest, indent=2)+'\n')
    sources = json.loads((OLD/'source_git.json').read_text())
    for repo, info in sources.items():
        assert subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip()==info['head']
        assert not subprocess.check_output(['git','-C',repo,'status','--porcelain'],text=True).strip()
    (ROOT/'source_git.json').write_text(json.dumps(sources,indent=2)+'\n')
    print(ROOT)

if __name__=='__main__': main()
