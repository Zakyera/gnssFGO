"""Prepare only an isolated numerical test; no source build or dataset launch."""
import hashlib
import json
from pathlib import Path
import shutil

H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
R = W / 'runs/kimera_prior_refresh_cpk6_20261019'

def digest(p):
    h = hashlib.sha256()
    with open(p, 'rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()

def main():
    old = W / 'runs/kimera_incremental_on_cpk5_20261019/protected_before.json'
    protected = json.loads(old.read_text())
    bad = [p for p, h in protected.items() if not Path(p).is_file() or digest(p) != h]
    if bad:
        raise RuntimeError('Pre-existing protected changes: ' + repr(bad))
    assert not R.exists(), 'Refuse overwrite of existing test root'
    R.mkdir()
    helper = W / 'runs/fixed_lag_direction_fl4p_20261016/OriginalChartPrior.h'
    extra = [helper, W / 'src/cbs/include/cbs/bpsam/fixed_lag_objective_audit.h',
             W / 'devel/lib/libgtsam.so.4.3a1']
    for p in extra:
        protected[str(p)] = digest(p)
    for name in ('test.cpp', 'build.py', 'PROTOCOL.txt'):
        shutil.copy2(H / name, R / name)
    shutil.copy2(helper, R / helper.name)
    (R / 'protected_before.json').write_text(json.dumps(protected, indent=2)+'\n')
    inputs = {str(p): digest(p) for p in sorted(R.iterdir()) if p.is_file()}
    (R / 'inputs.json').write_text(json.dumps(inputs, indent=2)+'\n')
    print(json.dumps({'prepared':str(R),'protected_files':len(protected),'inputs':len(inputs)}))

if __name__ == '__main__':
    main()
