"""Derive a fresh E1b.3c harness; preserve all earlier capture outputs."""

import json
from pathlib import Path
import shutil


HERE = Path(__file__).resolve().parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD = WS / 'runs/gnssfgo_e1b_profile40_fixed_20261012'
BASE = OLD / 'prepared'
RUN = WS / 'runs/gnssfgo_e1b_profile40_mapguard_20261012'
ROOT = RUN / 'prepared'


def once(text, old, new):
    assert text.count(old) == 1, (old, text.count(old))
    return text.replace(old, new)


def transform(text):
    return (text.replace(OLD.name, RUN.name)
            .replace('11408', '11410')
            .replace('eth1_e1b_profile40_fixed_on_20261012',
                     'eth1_e1b_profile40_mapguard_on_20261012'))


def write_new(path, contents):
    with path.open('x') as stream:
        stream.write(contents)


def main():
    assert BASE.is_dir() and (BASE / 'preflight/validation.json').exists()
    assert (BASE / 'profile40_on_control/failed.txt').read_text().strip() == 'source failed'
    assert not RUN.exists(), 'Fresh root required'
    ROOT.mkdir(parents=True)
    shutil.copy2(HERE / 'PROTOCOL.txt', RUN / 'PROTOCOL.txt')
    names = [
        'experiment.py', 'run_case.py', 'source.env', 'source_runner.sh',
        'active_window.launch', 'glim_profile.launch',
        'grandtour_eth1_kimera_stereo.launch', 'kimera_vio_ros.launch',
        'kimera_vio_ros_newer_college_2021_stereo.launch',
        'receiver_e2.sh', 'coverage_checks.py', 'validate_numerical_case.py',
        'numerical_equivalence.py', 'profile40_on.json',
    ]
    for name in names:
        write_new(ROOT / name, transform((BASE / name).read_text()))
    write_new(ROOT / 'map_guard.py', (HERE / 'map_guard.py').read_text())
    gate = transform((BASE / 'verify_effective.py').read_text())
    gate = once(gate,
        "for name in ['libglim_ros.so','libglim.so','libcbs.so','libe1b_profile.so',\n"
        "             'libgtsam.so.4.3a1','libgtsam_points_cuda.so.1.2.1']:\n"
        "    assert str(candidate/name) in glim_loaded, name",
        "from map_guard import require_glim_libraries\n"
        "verified_dso_paths = require_glim_libraries(glim_loaded, candidate)")
    write_new(ROOT / 'verify_effective.py', gate)
    (ROOT / 'receiver_e2.sh').chmod(0o755)
    shutil.copytree(BASE / 'private_build', ROOT / 'private_build', symlinks=True)
    for name in ['validate_native.py', 'validate_lifecycle.py',
                 'validate_diagnostics.py', 'admission_contract.py',
                 'lifecycle_contract.py']:
        write_new(RUN / name, transform((OLD / name).read_text()))
    preflight = transform((HERE.parent / 'e1b_profile40_fixed_20261012/preflight.py').read_text())
    addition = (
        "docker('python3 '+inside(ROOT/'map_guard.py')+' '+inside(LIB)+' '+"
        "inside(WS/'runs/e1b_startup_fix_20261012/fixed_on/maps.txt'),"
        "'map_guard.log')\n"
    )
    preflight = once(preflight,
        "for name in ['libcbs.so','libgtsam_points_cuda.so.1']:assert inside(LIB/name) in ldd\n",
        "for name in ['libcbs.so','libgtsam_points_cuda.so.1']:assert inside(LIB/name) in ldd\n"
        + addition)
    write_new(HERE / 'preflight.py', preflight)
    cfg = json.loads((ROOT / 'profile40_on.json').read_text())
    assert cfg['receiver_model'] == 'source_wnoa'
    assert cfg['sources'][0]['session'] == cfg['sources'][1]['session']
    assert 'require_glim_libraries' in gate
    assert all(not (ROOT / name).exists() for name in
               ['profile40_on_source', 'profile40_on_receiver', 'preflight'])
    print(ROOT)


if __name__ == '__main__':
    main()
