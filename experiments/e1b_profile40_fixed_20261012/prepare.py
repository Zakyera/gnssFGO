"""Derive one fresh, otherwise frozen E1b.3 capture from retained inputs."""
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD = WS/'runs/gnssfgo_e1b_profile40_20261012'
BASE = OLD/'prepared'
RUN = WS/'runs/gnssfgo_e1b_profile40_fixed_20261012'
ROOT = RUN/'prepared'
OLD_LIB = '/workspace/cbs_gtsam4.3/runs/e1b_local_profile_20261012/release_2'
FIXED_LIB = '/workspace/cbs_gtsam4.3/runs/e1b_startup_fix_20261012/fixed_release'


def once(text, a, b):
    assert text.count(a) == 1, (a, text.count(a))
    return text.replace(a, b)


def transform(text):
    return (text.replace(OLD.name, RUN.name)
            .replace(OLD_LIB, FIXED_LIB)
            .replace('e1b_local_profile_20261012/release_2',
                     'e1b_startup_fix_20261012/fixed_release')
            .replace('11406', '11408')
            .replace('eth1_e1b_profile40_on_20261012',
                     'eth1_e1b_profile40_fixed_on_20261012'))


def write_new(path, contents):
    with path.open('x') as f:
        f.write(contents)


def main():
    assert BASE.is_dir() and (BASE/'preflight/validation.json').exists()
    assert (BASE/'profile40_on_control/failed.txt').exists()
    assert not RUN.exists(), 'Fresh root required'
    ROOT.mkdir(parents=True)
    shutil.copy2(HERE/'PROTOCOL.txt', RUN/'PROTOCOL.txt')
    names = ['experiment.py', 'run_case.py', 'source.env', 'source_runner.sh',
             'active_window.launch', 'glim_profile.launch',
             'grandtour_eth1_kimera_stereo.launch', 'kimera_vio_ros.launch',
             'kimera_vio_ros_newer_college_2021_stereo.launch',
             'receiver_e2.sh', 'verify_effective.py', 'coverage_checks.py',
             'validate_numerical_case.py', 'numerical_equivalence.py',
             'profile40_on.json']
    for name in names:
        write_new(ROOT/name, transform((BASE/name).read_text()))
    (ROOT/'receiver_e2.sh').chmod(0o755)
    shutil.copytree(BASE/'private_build', ROOT/'private_build', symlinks=True)
    for name in ['validate_native.py', 'validate_lifecycle.py',
                 'validate_diagnostics.py', 'admission_contract.py',
                 'lifecycle_contract.py']:
        write_new(RUN/name, transform((OLD/name).read_text()))
    old_preflight = HERE.parent/'e1b_profile40_20261012/preflight.py'
    p = transform(old_preflight.read_text())
    p = once(p,
        "for name,h in json.loads((LIB/'complete.json').read_text())['artifacts'].items(): old[str(LIB/name)]=h",
        """fixed=json.loads((LIB/'complete.json').read_text())
assert sha(LIB/'libglim_ros.so')==fixed['libglim_ros_sha256']
for name in ['libglim_ros.so','glim_ros.o','glim_factor_graph_inspector.o','complete.json']:
    old[str(LIB/name)]=sha(LIB/name)
sys.path.insert(0,str(HERE.parent/'e1b_startup_fix_20261012'))
from layout_guard import require_size
require_size(LIB/'glim_ros.o',3344)
assert (WS/'runs/e1b_startup_fix_20261012/layout/size.log').read_text().strip()=='GlimCbsBridge_size=3344'""")
    write_new(HERE/'preflight.py', p)
    # Every source input/setting remains in the previous prepared harness;
    # generated source/receiver output dirs are intentionally not copied.
    cfg = json.loads((ROOT/'profile40_on.json').read_text())
    assert cfg['receiver_model']=='source_wnoa'
    assert cfg['sources'][0]['session']==cfg['sources'][1]['session']
    assert FIXED_LIB in (ROOT/'glim_profile.launch').read_text()
    assert all(not (ROOT/n).exists() for n in
               ['profile40_on_source','profile40_on_receiver','preflight'])
    print(ROOT)


if __name__ == '__main__':
    main()
