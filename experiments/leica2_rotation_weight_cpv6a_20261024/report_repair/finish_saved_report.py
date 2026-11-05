"""Finite report-only continuation; never starts/retries a source capture."""
from pathlib import Path
import datetime,fcntl,hashlib,json,os,subprocess,time
R=Path(__file__).resolve().parent;H=R.parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_rotation_weight_cpv6a_20261024/prepared')
V=Path('/home/yeranis/results/gnssfgo_leica2_rotation_weight_cpv6a_20261024')
PY='/home/yeranis/.local/share/pipx/venvs/evo/bin/python'
def now():return datetime.datetime.now().astimezone().isoformat()
def load(p):return json.loads(p.read_text())
def save(name,data):
 with (R/name).open('x') as f:json.dump(data,f,indent=2)
def step(name,cmd):
 with (R/(name+'.log')).open('x') as f:subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,check=True)
def verify():
 pins=load(P/'protected_manifest.json')
 for p,h in pins.items():
  with Path(p).open('rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==h,p
 return len(pins)
def main():
 lock=(R/'finish.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 assert not (R/'start.json').exists()
 assert load(R/'partial_weight_check.json')['passed']
 save('start.json',dict(pid=os.getpid(),started=now(),scope='wait for existing controller; saved-data reporting only'))
 started_container=False;failure=None;cleanup=None
 try:
  deadline=time.monotonic()+3600
  while True:
   s=load(H/'status.json')
   if s['state'] in ('complete','failed'):break
   assert time.monotonic()<deadline,'report wait deadline'
   time.sleep(15)
  assert s['state']=='failed' and s['failure']['stage']=='validate_on_weight',s
  assert s['cleanup_error'] is None and load(H/'closeout.json')['passed']
  # Prove the exact known reporting error, not numerical/coverage failure.
  err=(H/'validate_on_weight.log').read_text()
  assert 'np.allclose(c,want' in err and 'AssertionError' in err
  assert (P/'full_on_control/complete.json').exists() and not (P/'full_on_control/failed.txt').exists()
  for p in [P/'full_on_source/local_provenance.json',P/'full_on_source/velodyne_validation.json',P/'full_on_source/candidate_validation.json',V/'on/validation.json']:
   assert load(p)['passed'],p
  for _ in range(10):
   pid=int(s['pid']);proc=Path('/proc')/str(pid)
   if not proc.exists():break
   time.sleep(1)
  assert not proc.exists(),'original controller still active'
  assert subprocess.check_output(['docker','inspect','-f','{{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).split()==['exited','exited']
  count=verify()
  step('applied_weight',[PY,str(R/'validate_applied_weight.py')])
  assert load(P/'full_on_source/weight_validation.json')['passed']
  assert not (P/'evaluation_full').exists() and not (H/'results.json').exists()
  step('start_report_container',['docker','start','cbsms_ws_gpu']);started_container=True
  script='/workspace/cbs_gtsam4.3/runs/gnssfgo_leica2_rotation_weight_cpv6a_20261024/prepared/evaluate_fl4f.py'
  step('evaluate',['docker','exec','cbsms_ws_gpu','bash','-lc','source /opt/ros/noetic/setup.bash && python3 '+script])
  step('summarize',['python3',str(H/'summarize.py')])
  result=load(H/'results.json');assert result['passed']
  e=result['evaluation']['results'];comparisons={}
  for cohort in ('full_saved','common'):
   comparisons[cohort]={}
   for role in ('native_glim','native_kimera','fused_online','fused_finalized'):
    comparisons[cohort][role]={}
    for metric in ('ATE_RMSE_m','translation_RPE_1s_RMSE_m','rotation_RPE_1s_RMSE_deg'):
     b=e['baseline_on_'+role][cohort][metric];c=e['on_'+role][cohort][metric]
     comparisons[cohort][role][metric]=dict(original_on=b,candidate_on=c,change_percent=100*(c/b-1))
  save('comparison.json',comparisons)
  with (H/'REPORT.txt').open('a') as f:
   f.write('\nCAPTURE AND SAVED-DATA REPORT COMPLETE '+now()+'\n')
   f.write('Original report-only validator failure preserved; actual factor checker passes. No source retry.\n')
   f.write('Primary Kimera full-saved rotation RPE: '+json.dumps(comparisons['full_saved']['native_kimera']['rotation_RPE_1s_RMSE_deg'])+'\n')
   f.write('See RESULTS_REPORT.txt/results.json and report_repair/comparison.json for all three controls/candidate metrics.\n')
 except BaseException as exc:
  failure=repr(exc)
 finally:
  try:
   if started_container:
    top=subprocess.check_output(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'],text=True)
    assert all(l.split()[1]=='tail' for l in top.splitlines()[1:] if l.strip()),top
    step('stop_report_container',['docker','stop','cbsms_ws_gpu'])
   final=subprocess.check_output(['docker','inspect','-f','{{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).split()
   assert final==['exited','exited'],final
   count=verify()
  except BaseException as exc:cleanup=repr(exc)
  save('final.json',dict(passed=failure is None and cleanup is None,finished=now(),failure=failure,cleanup_error=cleanup,source_rerun=False,original_status_preserved=True))
  with (H/'REPORT.txt').open('a') as f:f.write('Report finisher closeout: '+json.dumps(load(R/'final.json'))+'\n')
 if failure or cleanup:raise SystemExit(1)
if __name__=='__main__':main()
