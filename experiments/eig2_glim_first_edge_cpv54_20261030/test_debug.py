import json,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent
def save(n,v):
 with (H/n).open('x') as f:json.dump(v,f,indent=2)
assert subprocess.check_output(['docker','inspect','-f','{{.State.Status}}','cbsms_ws_gpu'],text=True).strip()=='exited'
subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
try:
 for label in ('release_compile','release_test'):
  cmd=json.loads((H/(label+'_process.json')).read_text())['command']
  cmd=[s.replace('/Test.cpp','/Test_v2.cpp').replace('/test_release','/test_debug') for s in cmd]
  with (H/('debug_'+label+'.log')).open('x') as f:
   j=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT);save('debug_'+label+'_process.json',dict(pid=j.pid,command=cmd));rc=j.wait()
  save('debug_'+label+'_receipt.json',dict(exit=rc))
  if rc:break
finally:subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
