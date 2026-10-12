"""Low-rate, read-only process-memory observation of this owned capture."""
from pathlib import Path
import json
import subprocess
import time
from prepare import ROOT,RUN

code=r'''
from pathlib import Path
import json
rows=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  args=(p/'cmdline').read_bytes().split(b'\0')
  if not args or not any(args[0].endswith(x) for x in [b'/glim_rosnode',b'/kimera_vio_ros_node',b'/replay']):continue
  status=dict(line.split(':',1) for line in (p/'status').read_text().splitlines() if ':' in line)
  rows.append(dict(pid=int(p.name),executable=args[0].decode(),
     rss_kib=int(status.get('VmRSS','0 kB').split()[0]),
     peak_rss_kib=int(status.get('VmHWM','0 kB').split()[0]),
     threads=int(status.get('Threads','0'))))
 except (OSError,ValueError):pass
print(json.dumps(rows))
'''
control=ROOT/'capacity160_on_control'
with (RUN/'memory_observations.jsonl').open('x',buffering=1) as out:
    while not any((control/name).exists() for name in ['complete.json','failed.txt']):
        row=dict(wall_ns=time.time_ns(),containers={})
        status=ROOT/'capacity160_on_receiver/transport_status.json'
        try:row['transport']=json.loads(status.read_text())
        except (OSError,json.JSONDecodeError):pass
        for container in ['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']:
            r=subprocess.run(['docker','exec',container,'python3','-c',code],capture_output=True,text=True,timeout=10)
            if r.returncode==0:row['containers'][container]=json.loads(r.stdout)
            else:row['containers'][container]=dict(error=r.stderr.strip())
        out.write(json.dumps(row)+'\n')
        for _ in range(30):
            if any((control/name).exists() for name in ['complete.json','failed.txt']):break
            time.sleep(1)
print('MEMORY_OBSERVATION_COMPLETE',flush=True)
