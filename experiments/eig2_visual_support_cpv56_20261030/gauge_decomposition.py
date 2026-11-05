"""Bounded export to explain CP56's failed rigid-coordinate check."""
import datetime,hashlib,json,subprocess
from pathlib import Path
B=Path(__file__).resolve().parent;H=B/'gauge_decomposition';H.mkdir(exist_ok=False)
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');K=W/'runs'/B.name/'gauge_decomposition'
C='/workspace/cbs_gtsam4.3';CK=C+'/runs/'+B.name+'/gauge_decomposition';OLD=C+'/runs/'+B.name+'/measure_gate_failure'
def sha(p):
 with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(n,v):
 with (H/n).open('x') as f:json.dump(v,f,indent=2)
def call(n,cmd):
 save(n+'_command.json',cmd)
 with (H/(n+'.log')).open('x') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
 save(n+'_receipt.json',dict(exit=r.returncode));assert r.returncode==0,(n,r.returncode)
pins={str(K/'Export.cpp'):sha(K/'Export.cpp'),str(B/'gauge_decomposition.py'):sha(B/'gauge_decomposition.py')}
pins.update(json.loads((B/'measure_gate_failure/protected_manifest.json').read_text()))
for p,h in pins.items():assert sha(p)==h,p
save('pins.json',pins)
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert states=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n';save('preflight.json',dict(containers=states))
call('start',['docker','start','cbsms_ws_gpu'])
try:
 for mode in ['release','san']:
  for kind in ['compile','link']:
   cmd=json.loads((B/'measure_gate_failure'/f'{mode}_{kind}_command.json').read_text())
   cmd=[x.replace(OLD,CK).replace('/VisualSupport.cpp','/Export.cpp') for x in cmd]
   call(mode+'_'+kind,cmd)
  cmd=json.loads((B/'measure_gate_failure'/f'{mode}_selftest_command.json').read_text());cmd=[x.replace(OLD,CK) for x in cmd];call(mode+'_selftest',cmd)
 call('export',['docker','exec','-e','LD_LIBRARY_PATH='+C+'/runs/kimera_on_incremental_refresh_cpv40_20261029/private_build:'+C+'/devel/lib','cbsms_ws_gpu',CK+'/release',C+'/runs/eig2_hesai_insertion_cpv51_20261030/prepared',CK+'/results'])
 for phase in ['ordinary','refresh']:
  assert sha(K/'results'/f'{phase}_before.bin')==sha(K/'results'/f'{phase}_after.bin')
 for p,h in pins.items():assert sha(p)==h,p
 save('complete.json',dict(completed=True,rotation_control_passed=True,translation_check_is_measurement_not_acceptance=True,optimizer_calls=0,time=datetime.datetime.now().astimezone().isoformat()))
finally:
 top=subprocess.check_output(['docker','top','cbsms_ws_gpu','-eo','pid,comm,args'],text=True)
 assert all(row.split()[1] in ('tail','zsh') for row in top.splitlines()[1:] if row.strip()),top
 call('restore',['docker','stop','cbsms_ws_gpu'])
 save('cleanup.json',dict(jobs=top,containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)))
