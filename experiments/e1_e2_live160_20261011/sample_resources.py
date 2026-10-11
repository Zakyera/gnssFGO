"""Read-only process provenance and memory snapshots during the owned run."""
import json
from pathlib import Path
import subprocess
import time

ROOT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live160_20261011/prepared')
status=json.loads((ROOT/'live160_on_receiver/transport_status.json').read_text())
probe=r'''
import hashlib,json,os,sys
from pathlib import Path
out=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit():continue
 try:
  args=(p/'cmdline').read_bytes().split(b'\0')
  exe=args[0].decode(errors='replace')
  if not any(exe.endswith(s) for s in ['/glim_rosnode','/kimera_vio_ros_node','/replay']):continue
  values={k:v.strip() for k,v in (l.split(':',1) for l in (p/'status').read_text().splitlines() if ':' in l)}
  env=dict(x.decode().split('=',1) for x in (p/'environ').read_bytes().split(b'\0') if b'=' in x)
  item={'pid':int(p.name),'executable':exe,'argv':[a.decode(errors='replace') for a in args if a],
        'memory':{k:values[k] for k in ['VmRSS','VmHWM','VmSize'] if k in values},
        'affinity':sorted(os.sched_getaffinity(int(p.name))),
        'processor_snapshot':int((p/'stat').read_text().split()[38]),
        'environment':{k:env.get(k) for k in ['LD_LIBRARY_PATH','OMP_NUM_THREADS','E2_EXACT_LINEARIZATION_CACHE','WNOJ_STRICT_PUBLICATION','WNOJ_INCREMENTAL_PROFILE']}}
  if exe.endswith('/replay'):
   assert exe=='/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011/candidate/replay'
   assert env['E2_EXACT_LINEARIZATION_CACHE']=='1' and env['WNOJ_STRICT_PUBLICATION']=='1' and env['WNOJ_INCREMENTAL_PROFILE']=='nominal'
   assert sys.argv[1] in '\0'.join(item['argv'])
   item['sha256']=hashlib.sha256((p/'exe').read_bytes()).hexdigest()
   assert item['sha256']=='9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0'
   item['library_maps']=[s for s in (p/'maps').read_text().splitlines() if any(x in s for x in ['libgtsam','libtbb'])]
  out.append(item)
 except OSError:pass
print(json.dumps(out))
'''
entry={'wall_ns':time.time_ns(),'transport':status,'processes':{}}
for container in ['cbsms_ws_gpu','ubuntu-jammy-humble-nvidia']:
    r=subprocess.run(['docker','exec',container,'python3','-c',probe,str(ROOT)],capture_output=True,text=True,check=True)
    entry['processes'][container]=json.loads(r.stdout)
assert len(entry['processes']['cbsms_ws_gpu'])==2
assert len(entry['processes']['ubuntu-jammy-humble-nvidia'])==1
with (ROOT.parent/'resource_samples.jsonl').open('a') as f:f.write(json.dumps(entry)+'\n')
print(json.dumps({c:[{k:p[k] for k in ['pid','executable','memory','processor_snapshot','affinity']} for p in v] for c,v in entry['processes'].items()}))
