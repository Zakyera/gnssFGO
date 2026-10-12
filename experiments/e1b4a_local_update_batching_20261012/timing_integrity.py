"""Read-only frozen-input checks and exclusive timing audit evidence (host)."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
from datetime import datetime, timezone

WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT = WS / 'runs/e1b4a_local_update_batching_20261012'

def local(path):
    return Path(str(path).replace('/workspace/cbs_gtsam4.3', str(WS)))

def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1048576), b''):
            digest.update(chunk)
    return digest.hexdigest()

def command(args):
    return subprocess.check_output(args, text=True)

def main():
    p = argparse.ArgumentParser()
    p.add_argument('phase', choices=['pre', 'post'])
    a = p.parse_args()
    output = ROOT / 'timing_audit_v3'
    if a.phase == 'pre':
        assert not (ROOT / 'release_2/timing1').exists(), 'refuse duplicate timing'
        output.mkdir(exist_ok=False)
    else:
        assert (output / 'pre.json').exists()
    protected = json.loads((WS / 'runs/gnssfgo_e1b_profile40_mapguard_20261012/prepared/protected_manifest.json').read_text())
    baseline = len(protected)
    protected.update(json.loads((ROOT / 'protected_preparation.json').read_text()))
    host_count = len(protected)
    build = {}
    for file in ROOT.glob('*/*_protected.json'):
        build.update(json.loads(file.read_text()))
    for path, digest in protected.items():
        assert sha(local(path)) == digest, path
    # Some baseline inputs live outside the workspace mount, whereas /usr/lib
    # dependencies belong to the container. Hash each in its owning filesystem.
    check = ('import hashlib,json,sys;from pathlib import Path;'
             'items=json.loads(sys.argv[1]);'
             'bad=[p for p,h in items.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h];'
             'assert not bad,bad;print(len(items))')
    command(['docker', 'exec', 'cbsms_ws_gpu', 'python3', '-c', check, json.dumps(build)])
    repos = json.loads((ROOT / 'source_git.json').read_text())
    for path, expected in repos.items():
        repo = str(local(path))
        assert command(['git', '-C', repo, 'rev-parse', 'HEAD']).strip() == expected['head']
        assert not command(['git', '-C', repo, 'status', '--porcelain']).strip(), repo
    expected = {
        'test_solver_3': '223232f359c3580886a2ff75f91daf26548289c51b0115366678b0878babfa5f',
        'libgtsam.so.4.3a1': '369f45866a82c40170ad7cfa83b4bc6b7f8a6f4ae22db34115daee926529bb37',
        'libglim_ros.so': '83592aba553adbc2879f27b5f2cf3551826ab497a58201ba0101543934f49dec',
    }
    for name, digest in expected.items():
        assert sha(ROOT / 'release_2' / name) == digest, name
    for name in ['test_solver.cpp', 'test_dispatch.cpp', 'LocalUpdateBatching.h',
                 'ScopedGraphLinearizer.h', 'ScopedGraphLinearizer.cpp']:
        assert sha(ROOT / 'authoring' / name) == sha(ROOT / 'release_2/tests_3_source_snapshot/authoring' / name), name
    # The already committed runner was finalized after compilation: checks3
    # admission and a CPU dispatch-only sanitizer option. Pin that actual runner
    # instead of incorrectly requiring the older build-time script snapshot.
    assert sha(ROOT / 'authoring/run_tests.py') == 'dbabd2c638f3cc8d7ff361304c7d9f9ba321d19690659190ee1ab8ff2a2068ff'
    assert json.loads((ROOT / 'release_2/checks3/passed.json').read_text())['passed']
    data = dict(phase=a.phase, timestamp=datetime.now(timezone.utc).isoformat(),
                passed=True, baseline_hashes=baseline, host_protected_hashes=host_count,
                build_protected_hashes=len(build), source_repos=repos, artifacts=expected,
                cpu=command(['lscpu']), processes=command(['ps', '-eo', 'pid,pcpu,etime,args']),
                gpu=command(['nvidia-smi', '--query-gpu=name,driver_version,pstate,temperature.gpu,utilization.gpu,memory.used,clocks.sm,clocks.mem,power.draw', '--format=csv']),
                cpu2_governor=Path('/sys/devices/system/cpu/cpu2/cpufreq/scaling_governor').read_text())
    with (output / (a.phase + '.json')).open('x') as stream:
        json.dump(data, stream, indent=2)
        stream.write('\n')
    print(json.dumps({k: data[k] for k in ['phase', 'passed', 'baseline_hashes', 'host_protected_hashes', 'build_protected_hashes', 'gpu', 'cpu2_governor']}, indent=2))

if __name__ == '__main__':
    main()
