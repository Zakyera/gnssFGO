"""Host owner for one exclusive private gate. Restores the build container."""
from pathlib import Path
import datetime
import json
import os
import shutil
import subprocess
import sys
import time

H=Path(__file__).resolve().parent
P='/workspace/cbs_gtsam4.3/runs/'+H.name
mode,attempt=sys.argv[1:]
assert mode in ('release','san','backend')
assert shutil.disk_usage(H).free > 8*1024**3
stem=mode+'_'+attempt
def write(suffix,data):
    with (H/(stem+suffix)).open('x') as f:json.dump(data,f,indent=2)
def inspect():
    return subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}',
        'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
before=inspect()
assert all(x.endswith(' exited') for x in before), before
write('_start.json',{'host_pid':os.getpid(),'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    'before':before,'mode':mode,'attempt':attempt})
rc=None; start=time.time()
try:
    subprocess.run(['docker','start','cbsms_ws_gpu'],check=True)
    args=['docker','exec','-u',f'{os.getuid()}:{os.getgid()}','cbsms_ws_gpu',
          'python3',P+'/build.py',mode,attempt]
    with (H/(stem+'.log')).open('x') as log:
        proc=subprocess.Popen(args,stdout=log,stderr=subprocess.STDOUT)
        write('_process.json',{'host_pid':proc.pid,'args':args})
        rc=proc.wait()
finally:
    top=subprocess.check_output(['docker','top','cbsms_ws_gpu','-eo','pid,ppid,comm'],text=True)
    write('_prestop_jobs.json',{'jobs':top})
    # docker exec has returned. Fail closed instead of stopping unexpected jobs.
    assert not any(s in top for s in ('cc1plus','python3','rosbag','kimera','glim','/test'))
    subprocess.run(['docker','stop','cbsms_ws_gpu'],check=True)
    write('_receipt.json',{'exit':rc,'seconds':time.time()-start,'after':inspect(),
                          'free_bytes':shutil.disk_usage(H).free})
raise SystemExit(rc if rc is not None else 1)
