"""Create an isolated 160-second harness from the validated S4d harness."""
from pathlib import Path
import ast
import hashlib
import json
import shutil

HERE = Path(__file__).resolve().parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD = WS/'runs/gnssfgo_shared_linearization_s4_provider_20261012'
RUN = WS/'runs/gnssfgo_shared_linearization_s5_live160_20261012'
ROOT = RUN/'prepared'
BUILD = WS/'runs/shared_linearization_s4_provider_20261012/release_2'
PRIOR = HERE.parent/'shared_linearization_s4_provider_20261012'

def inside(p):
    return '/workspace/'+str(p.relative_to('/home/yeranis/repos/V4RL'))

def sha(p):
    h = hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024), b''):
            h.update(b)
    return h.hexdigest()

def swap(s,a,b):
    assert s.count(a)==1,(a,s.count(a))
    return s.replace(a,b)

def transform(s):
    return (s.replace(OLD.name,RUN.name).replace('11422','11424')
            .replace('eth1_shared_linearization_s4_provider_on_20261012',
                     'eth1_shared_linearization_s5_live160_on_20261012')
            .replace('profile40','capacity160'))

def main():
    assert not RUN.exists()
    ROOT.mkdir(parents=True)
    records=[]
    names=['experiment.py','run_case.py','source.env','source_runner.sh',
           'active_window.launch','glim_profile.launch','grandtour_eth1_kimera_stereo.launch',
           'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
           'receiver_e2.sh','coverage_checks.py','validate_numerical_case.py',
           'numerical_equivalence.py','profile40_on.json','verify_effective.py','map_guard.py']
    for name in names:
        src=OLD/'prepared'/name
        s=transform(src.read_text())
        if name=='run_case.py':
            s=swap(s,'        duration=40','        duration=160')
            s=swap(s,'# Reuse the completed matched OFF; rerun only failed ON with the tested diagnostic fix.',
                   '# Retain the historical validation prerequisite; this task runs only the new ON capacity case.')
        if name=='source.env':
            s=swap(s,'${CBSMS_BAG_DURATION_SEC:-40.0}','${CBSMS_BAG_DURATION_SEC:-160.0}')
        if name=='source_runner.sh':
            s=swap(s,'"${duration_sec}" == 40.0','"${duration_sec}" == 160.0')
            s=swap(s,'Prepared 40-second ON E1b profiling capture only.',
                   'Prepared 160-second ON shared-native capacity capture only.')
        if name=='verify_effective.py':
            s=swap(s,"os.environ['CBSMS_BAG_DURATION_SEC'] == '40.0'",
                   "os.environ['CBSMS_BAG_DURATION_SEC'] == '160.0'")
        if name=='validate_numerical_case.py':
            s=swap(s,"metadata['sensor_window_sec']=='40.0'","metadata['sensor_window_sec']=='160.0'")
            s=swap(s,'check(*load(live/\'receiver\'/file),40,','check(*load(live/\'receiver\'/file),160,')
        if name=='profile40_on.json':
            cfg=json.loads(s);cfg['sequence']='ETH1 first160s S5 frozen shared-native covariance provider'
            s=json.dumps(cfg,indent=2)+'\n'
        dest=ROOT/name.replace('profile40','capacity160')
        if dest.suffix=='.py':ast.parse(s)
        dest.write_text(s)
        records.append(dict(file=dest.name,source=sha(src),candidate=sha(dest)))
    (ROOT/'receiver_e2.sh').chmod(0o755)
    shutil.copytree(OLD/'prepared/private_build',ROOT/'private_build',symlinks=True)
    for name in ['validate_native.py','validate_lifecycle.py','validate_diagnostics.py',
                 'admission_contract.py','lifecycle_contract.py','validate_provider.py']:
        s=transform((OLD/name).read_text())
        if name=='validate_provider.py':
            s=swap(s,'assert len(records)>=350','assert len(records)>=1550')
        ast.parse(s);(RUN/name).write_text(s)
    shutil.copy2(HERE/'PROTOCOL.md',RUN/'PROTOCOL.md')
    s=transform((PRIOR/'validate.py').read_text())
    (HERE/'validate.py').write_text(s)
    (HERE/'preparation.json').write_text(json.dumps(dict(run=str(RUN),build=str(BUILD),files=records),indent=2))
    print(RUN)

if __name__=='__main__':main()
