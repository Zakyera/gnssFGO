"""Fresh B0-equivalent live harness with only the private provider selected."""
from pathlib import Path
import hashlib,json,shutil
HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD=WS/'runs/gnssfgo_shared_linearization_b0_20261012'
RUN=WS/'runs/gnssfgo_shared_linearization_s4_provider_20261012'
ROOT=RUN/'prepared'
BUILD=WS/'runs/shared_linearization_s4_provider_20261012/release_2'
def inside(p):return '/workspace/'+str(p.relative_to('/home/yeranis/repos/V4RL'))
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def swap(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)
def transform(s):
    return s.replace(OLD.name,RUN.name).replace('11418','11422').replace('eth1_shared_linearization_b0_on_20261012','eth1_shared_linearization_s4_provider_on_20261012')
def main():
    assert not RUN.exists();ROOT.mkdir(parents=True)
    records=[]
    for name in ['experiment.py','run_case.py','source.env','source_runner.sh','active_window.launch',
        'glim_profile.launch','grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch',
        'kimera_vio_ros_newer_college_2021_stereo.launch','receiver_e2.sh','coverage_checks.py',
        'validate_numerical_case.py','numerical_equivalence.py','profile40_on.json','verify_effective.py','map_guard.py']:
        src=OLD/'prepared'/name;s=transform(src.read_text())
        if name=='glim_profile.launch':
            s=s.replace('/workspace/cbs_gtsam4.3/runs/e1b4a_local_update_batching_20261012/release_2:',inside(BUILD)+':')
            s=swap(s,'<env name="CBS_E1B_LOCAL_UPDATE_GPU_BATCHING" value="1"/>',
              '<env name="CBS_E1B_LOCAL_UPDATE_GPU_BATCHING" value="1"/>\n        <env name="GLIM_SHARED_NATIVE_PROVIDER" value="shared_native"/>')
        if name=='verify_effective.py':
            s=s.replace('/workspace/cbs_gtsam4.3/runs/e1b4a_local_update_batching_20261012/release_2',inside(BUILD))
            s=swap(s,"assert env(glim_pid)['CBS_E1B_LOCAL_UPDATE_GPU_BATCHING']=='1'",
              "assert env(glim_pid)['CBS_E1B_LOCAL_UPDATE_GPU_BATCHING']=='1'\n"+
              "assert env(glim_pid)['GLIM_SHARED_NATIVE_PROVIDER']=='shared_native'\n"+
              "assert 'GLIM_SHARED_NATIVE_JOURNAL' not in env(glim_pid)\n"+
              "assert 'GLIM_SHARED_NATIVE_PROVIDER' not in env(pid)")
        if name=='map_guard.py':s=swap(s,"    'libglim_ros.so',","    'libglim_ros.so',\n    'libgtsam_points.so.1.2.1',")
        if name=='run_case.py':
            s=swap(s,"        while jobs['source'].poll() is None:","""        log_positions={}
        def provider_failures():
            for log in [source/'estimators.log',*source.glob('ros_logs/*/glim_rosnode-*-stdout.log')]:
                if not log.exists():continue
                with log.open() as f:
                    f.seek(log_positions.get(str(log),0));chunk=f.read();log_positions[str(log)]=f.tell()
                for token in ['SHARED_NATIVE_SHADOW_FAILURE','SHARED_NATIVE_SHADOW_SETUP_FAILURE','SHARED_NATIVE_PROVIDER_FAILURE']:
                    assert token not in chunk, token+': '+str(log)
        while jobs['source'].poll() is None:
            provider_failures()""")
            s=swap(s,"        assert jobs['source'].returncode==0","        provider_failures()\n        assert jobs['source'].returncode==0")
        (ROOT/name).write_text(s);records.append(dict(file=name,source=sha(src),candidate=sha(ROOT/name)))
    (ROOT/'receiver_e2.sh').chmod(0o755)
    shutil.copytree(OLD/'prepared/private_build',ROOT/'private_build',symlinks=True)
    for name in ['validate_native.py','validate_lifecycle.py','validate_diagnostics.py','admission_contract.py','lifecycle_contract.py']:
        (RUN/name).write_text(transform((OLD/name).read_text()))
    shutil.copy2(HERE/'PROTOCOL.md',RUN/'PROTOCOL.md')
    startup=(HERE.parent/'e1b_startup_fix_20261012/debug_startup.py').read_text()
    startup=swap(startup,"RUN=WS/'runs/e1b_startup_fix_20261012'",f"RUN=Path('{RUN}')")
    startup=swap(startup,"FAILED=WS/'runs/gnssfgo_e1b_profile40_20261012/prepared'",f"FAILED=Path('{OLD}/prepared')")
    startup=swap(startup,"LIB=WS/'runs/e1b_local_profile_20261012/release_2'",f"LIB=Path('{BUILD}')")
    startup=swap(startup,'PORT=11407','PORT=11421')
    startup=swap(startup,"        if a.profile: extra+=","        extra+='export CBS_E1B_LOCAL_UPDATE_GPU_BATCHING=1 GLIM_SHARED_NATIVE_PROVIDER=shared_native && '\n        if a.profile: extra+=")
    startup=swap(startup,"            assert inside(a.library/'libglim_ros.so') in maps",
      "            checked=docker(['python3',inside(RUN/'prepared/map_guard.py'),inside(a.library),inside(out/'maps.txt')],capture_output=True,text=True,check=True)\n            (out/'map_guard.log').write_text(checked.stdout+checked.stderr)")
    startup=swap(startup,"        for error in ['ERROR: AddressSanitizer'", "        assert 'SHARED_NATIVE_PROVIDER_SETUP enabled=1 independent_local=0 revision_covariance=unchanged_main' in text\n        for error in ['SHARED_NATIVE_PROVIDER_FAILURE','ERROR: AddressSanitizer'")
    (RUN/'startup.py').write_text(startup)
    (HERE/'preparation.json').write_text(json.dumps(dict(run=str(RUN),build=str(BUILD),files=records),indent=2))
    print(RUN)
if __name__=='__main__':main()
