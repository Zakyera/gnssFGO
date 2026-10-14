"""Pin private adapter compatibility fix; preserve failed preflight evidence."""
import copy, hashlib, json, sys
from pathlib import Path
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_fixed_lag_live30_off_fl4a_20261014/prepared')
REPO=Path('/home/yeranis/repos/gnssFGO')
def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
sys.path.insert(0,str(P/'transport_tools'))
from live_receiver_protocol import validate_config
cfg=json.loads((P/'pilot30_off.json').read_text())
validate_config(cfg)
tests=1
for key,val in [('receiver_publication_policy','unknown'),('receiver_fixed_lag_seconds',8),('receiver_binary','/wrong')]:
    bad=copy.deepcopy(cfg);bad[key]=val
    try: validate_config(bad)
    except ValueError: tests+=1
    else: raise AssertionError(key)
bad=copy.deepcopy(cfg);bad.pop('receiver_publication_policy')
try: validate_config(bad)
except ValueError: tests+=1
else: raise AssertionError('legacy strict policy must remain enforced')
legacy=copy.deepcopy(bad);legacy['receiver_environment'].update(WNOJ_STRICT_PUBLICATION='1',WNOJ_INCREMENTAL_PROFILE='nominal')
validate_config(legacy);tests+=1
bad=copy.deepcopy(cfg);bad['receiver_environment']['WNOJ_STRICT_PUBLICATION']='1'
try: validate_config(bad)
except ValueError: tests+=1
else: raise AssertionError('must not claim nonlinear stationarity')
original=json.loads((P/'protected_manifest.json').read_text())
changed={str(P/name) for name in ('run_case.py','pilot30_off.json')}
for p,h in original.items():
    if p in changed: assert sha(P/'preflight_original'/Path(p).name)==h
    else: assert sha(p)==h,p
for name in ('launch_grandtour_live_receiver.py','grandtour_live_receiver_ros1.py'):
    assert sha(P/'transport_tools'/name)==sha(REPO/'tools'/name),name
updated={p:sha(p) for p in original}
for p in [*P.joinpath('transport_tools').glob('*.py'),*P.joinpath('preflight_original').iterdir(),*P.joinpath('pilot30_off_control').iterdir(),Path(__file__),Path(__file__).with_name('build_check.py')]:
    if p.is_file(): updated[str(p)]=sha(p)
updated['/home/yeranis/results/gnssfgo_fixed_lag_live_fl4a_20261014/live']=sha('/home/yeranis/results/gnssfgo_fixed_lag_live_fl4a_20261014/live')
with (P/'protected_manifest_v2.json').open('x') as f: json.dump(updated,f,indent=2)
with (P/'preflight/adapter_v2.json').open('x') as f:
    json.dump(dict(passed=True,tests=tests,changed_preparation_files=sorted(changed),protected_count=len(updated),note='First attempt failed before receiver/source/bag launch; original evidence preserved. Only private adapter policy validation changes.'),f,indent=2)
print('private policy tests passed',tests,'protected',len(updated))
