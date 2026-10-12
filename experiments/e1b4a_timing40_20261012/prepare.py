"""Fresh harness derived from the validated E1b.3c; no estimator edits."""
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD=WS/'runs/gnssfgo_e1b_profile40_mapguard_20261012'
RUN=WS/'runs/gnssfgo_e1b4a_timing40_20261012'
ROOT=RUN/'prepared'
LIB='runs/e1b4a_local_update_batching_20261012/release_2'
def once(s,a,b):
    assert s.count(a)==1,(a,s.count(a))
    return s.replace(a,b)
def transform(s):
    return (s.replace(OLD.name,RUN.name).replace('11410','11414')
            .replace('eth1_e1b_profile40_mapguard_on_20261012','eth1_e1b4a_timing40_on_20261012')
            .replace('runs/e1b_startup_fix_20261012/fixed_release',LIB))
def new(p,s):
    with p.open('x') as f:f.write(s)
def main():
    assert not RUN.exists()
    ROOT.mkdir(parents=True)
    shutil.copy2(HERE/'PROTOCOL.txt',RUN/'PROTOCOL.txt')
    names=['experiment.py','run_case.py','source.env','source_runner.sh',
      'active_window.launch','glim_profile.launch','grandtour_eth1_kimera_stereo.launch',
      'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
      'receiver_e2.sh','coverage_checks.py','validate_numerical_case.py',
      'numerical_equivalence.py','profile40_on.json','verify_effective.py','map_guard.py']
    for name in names:
        s=transform((OLD/'prepared'/name).read_text())
        if name=='glim_profile.launch':
            s=once(s,'<env name="CBS_E1B_PROFILE_PATH"',
              '<env name="CBS_E1B_LOCAL_UPDATE_GPU_BATCHING" value="1"/>\n        <env name="CBS_E1B_PROFILE_PATH"')
        if name=='verify_effective.py':
            s=once(s,"assert 'CBS_E1B_PROFILE_PATH' not in env(pid)",
              "assert 'CBS_E1B_PROFILE_PATH' not in env(pid)\n"
              "assert env(glim_pid)['CBS_E1B_LOCAL_UPDATE_GPU_BATCHING']=='1'\n"
              "assert 'CBS_E1B_LOCAL_UPDATE_GPU_BATCHING' not in env(pid)\n"
              "assert not any('SNAPSHOT' in k or 'REAL_INPUT' in k or 'GLIM_E1_REAL_AUDIT_ROOT' == k for k in env(glim_pid))\n"
              "assert 'e1b4a_real_input' not in glim_loaded")
        new(ROOT/name,s)
    (ROOT/'receiver_e2.sh').chmod(0o755)
    shutil.copytree(OLD/'prepared/private_build',ROOT/'private_build',symlinks=True)
    for name in ['validate_native.py','validate_lifecycle.py','validate_diagnostics.py',
                 'admission_contract.py','lifecycle_contract.py']:
        new(RUN/name,transform((OLD/name).read_text()))
    s=transform((HERE.parent/'e1b_profile40_mapguard_20261012/preflight.py').read_text())
    s=once(s,"fixed=json.loads((LIB/'complete.json').read_text())\nassert sha(LIB/'libglim_ros.so')==fixed['libglim_ros_sha256']\nfor name in ['libglim_ros.so','glim_ros.o','glim_factor_graph_inspector.o','complete.json']:",
      "fixed=json.loads((LIB/'libraries_complete.json').read_text())\n"
      "for name,h in fixed['artifacts'].items():assert sha(LIB/name)==h,name\n"
      "for name in fixed['artifacts']:")
    s=s.replace("require_size(LIB/'glim_ros.o',3344)","require_size(LIB/'glim_ros.cpp.o',3344)")
    s=s.replace('runs/e1b_startup_fix_20261012/fixed_on/maps.txt',
                'runs/e1b4a_local_update_batching_20261012/no_data_startup/maps.txt')
    s=once(s,"manifest=dict(old)",
      "manifest=dict(old)\n"
      "def host_real(p):\n"
      "    while p.is_symlink():\n"
      "        target=p.readlink()\n"
      "        p=(WS/str(target).removeprefix('/workspace/cbs_gtsam4.3/')) if str(target).startswith('/workspace/cbs_gtsam4.3/') else (target if target.is_absolute() else p.parent/target)\n"
      "    return p.resolve()\n"
      "for p in LIB.glob('lib*.so*'):\n"
      "    q=host_real(p);manifest[str(q)]=sha(q)\n")
    s=once(s,"print('E1B_PROFILE40_PREFLIGHT_PASS'","print('E1B4A_TIMING40_PREFLIGHT_PASS'")
    new(HERE/'preflight.py',s)
    s=(HERE.parent/'e1b_profile40_mapguard_20261012/analyze_profile.py').read_text()
    s=once(s,'sync_ns / sync_calls / 1e6,','sync_ns / sync_calls / 1e6 if sync_calls else None,')
    new(HERE/'analyze_profile.py',s)
    print(ROOT)
if __name__=='__main__':main()
