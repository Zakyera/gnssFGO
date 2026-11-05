"""Final independent artifact/phase accounting. No estimator replay or scoring."""
import csv,datetime,hashlib,importlib.util,json,subprocess,sys
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');P=W/'runs'/H.name
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def check(pins):
 for p,v in pins.items():assert sha(p)==v,p
def load(p):return json.loads(p.read_text())
assert not (H/'final_verification.json').exists()
assert load(H/'complete.json')['passed'] and load(H/'closeout.json')['passed']
for step in ('inspection','equation_inspection','equation_components'):
 assert load(H/(step+'_passed.json'))['passed']
 assert load(H/(step+'_cleanup.json'))['container']=='exited'
a=load(H/'analysis.json');g=load(H/'gravity_coupling.json')
check(a['checkpoint_sha256']);check(a['analysis_input_sha256']);check(g['input_sha256'])
assert len(a['checkpoint_sha256'])==15 and [x['k'] for x in a['updates']]==[21,22,23,24,25]
obj=list(csv.reader((P/'prepared/full_on_source/objective.csv').open()))
for row in a['updates']:
 k=row['k'];reference=list(map(float,obj[k]))
 assert abs(row['costs']['initialized']['total']-reference[4])<=1e-8*(1+abs(reference[4]))
 assert abs(row['costs']['refresh']['total']-reference[5])<=1e-8*(1+abs(reference[5]))
 assert abs(row['costs']['refresh']['families']['imu']-reference[7])<=1e-8*(1+abs(reference[7]))
for p in (P/'inspection/results').glob('update_*/*roundtrip.txt'):
 d=dict(csv.reader(p.open()));assert float(d['max_error_difference'])==float(d['max_relative_linear_difference'])==0
eq=list(csv.DictReader((P/'equation_inspection/results/equations.csv').open()))
cp=list(csv.DictReader((P/'equation_components/results/components.csv').open()))
assert len(eq)==len(cp)==15
assert all(float(r['fresh_vs_cache'])==0 and float(r['direct_vs_estimate'])<1e-6 for r in eq)
assert all(float(r['max_gravity_profile_difference'])==0 for r in cp)
module=H.parent/'eig2_on_mechanism_cpv43_20261029/analyze.py'
spec=importlib.util.spec_from_file_location('cpv43_readonly',module);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
source=P/'prepared/full_on_source';parsed=m.parse_source(source/'estimators.log')
matches=0;max_error=0.0
for k in range(21,26):
 rows={r[0]:np.array(r[1:],float) for r in csv.reader((P/'inspection/results'/f'update_{k}'/'after_refresh_values.csv').open())}
 for i,T in parsed['windows'][k].items():
  error=float(np.max(abs(m.pose(rows['x'+str(i)])-T)));assert error<1e-12
  max_error=max(max_error,error);matches+=1
logs=list(source.glob('ros_logs/*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log'));assert len(logs)==1
life=m.parse_stdout(logs[0])['life'];assert all(life[k]['marginal_keys']==0 for k in range(21,26))
first_marginal=min(k for k,r in life.items() if r['marginal_keys']>0);assert first_marginal==26
pins=load(P/'prepared/protected_manifest.json');check(pins)
old=load(H.parent/'eig2_refresh_full_cpv42_20261029/complete.json')['output_sha256'];check(old)
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert containers==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited']
for name in ('start.json','launch.json','release_process.json','san_process.json','backend_process.json','capture_on_process.json'):
 receipt=load(H/name);proc=Path('/proc')/str(receipt['pid'])/'cmdline'
 assert not proc.exists() or H.name.encode() not in proc.read_bytes(),name
hesai=subprocess.check_output(['python3',str(H.parent/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
result=dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),protected_files=len(pins),
 original_report_outputs=len(old),hesai=hesai,containers=containers,owned_jobs_exited=True,
 snapshots=15,all_factor_roundtrip_error=0,all_factor_roundtrip_linear_relative_error=0,
 native_main_pose_matches=matches,native_main_max_component_difference=max_error,first_marginalization=first_marginal,
 max_fresh_vs_cache=max(float(r['fresh_vs_cache']) for r in eq),
 max_direct_vs_estimate_tangent_norm=max(float(r['direct_vs_estimate']) for r in eq),
 max_tree_H_relative=max(float(r['tree_H_relative']) for r in cp),
 max_tree_rhs_relative=max(float(r['tree_rhs_relative']) for r in cp),
 covariance_investigated=False,accuracy_scored=False,full_source_runs=0,bounded_source_runs=1,
 output_sha256={str(p):sha(p) for p in H.iterdir() if p.is_file()},
 diagnostic_artifacts={str(p):sha(p) for root in ('inspection','equation_inspection','equation_components') for p in (P/root).rglob('*') if p.is_file()})
with (H/'final_verification.json').open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
print(json.dumps({k:v for k,v in result.items() if k not in ('output_sha256','diagnostic_artifacts')},indent=2))
