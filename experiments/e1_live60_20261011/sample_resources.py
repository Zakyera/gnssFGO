#!/usr/bin/env python3
"""Read process memory at explicit checkpoints; not a peak-memory profiler."""
import json
from pathlib import Path
import subprocess
import time

ROOT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_live60_20261011/prepared')
status=json.loads((ROOT/'live60_on_receiver/transport_status.json').read_text())
probe=r'''
import json
from pathlib import Path
out=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  args=(p/'cmdline').read_bytes().split(b'\0')
  exe=args[0].decode(errors='replace')
  if not any(exe.endswith(s) for s in ['/glim_rosnode','/kimera_vio_ros_node','/replay']):continue
  values={k:v.strip() for k,v in (l.split(':',1) for l in (p/'status').read_text().splitlines() if ':' in l)}
  out.append({'pid':int(p.name),'executable':exe,'memory':{k:values[k] for k in ['VmRSS','VmHWM','VmSize'] if k in values}})
 except OSError:pass
print(json.dumps(out))
'''
entry={'wall_ns':time.time_ns(),'transport':status,'processes':{}}
for container in ['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']:
    r=subprocess.run(['docker','exec',container,'python3','-c',probe],capture_output=True,text=True,check=True)
    entry['processes'][container]=json.loads(r.stdout)
with (ROOT.parent/'resource_samples.jsonl').open('a') as f:f.write(json.dumps(entry)+'\n')
print(json.dumps({k:v for k,v in entry.items() if k!='transport'}))
