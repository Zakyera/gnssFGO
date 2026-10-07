from pathlib import Path
import json, os, subprocess, sys, time
root=Path('/audit');mode=sys.argv[1]
assert mode in ['off','on']
out=root/(mode+'_wnoa')
assert not out.exists(), 'refuse duplicate/overwrite'
command=['/previous/build/wnoa/replay','--receiver-model','source_wnoa','--mode','sources','--timeline','record','--solver','isam2',
         '--support-policy','uniform_imu','--support-period-ns','100000000','--receiver-startup-samples','59',
         '--event-stream',str(root/'inputs'/f'{mode}_events.csv'),'--out',str(out)]
env=dict(os.environ,WNOJ_INCREMENTAL_PROFILE='nominal',WNOJ_STRICT_PUBLICATION='1',AUDIT_RECORD_ALL='1',WNOJ_AUDIT_INDICES='294,594,894,1192')
t=time.monotonic()
with (root/(mode+'_replay.log')).open('x') as f:p=subprocess.run(command,env=env,stdout=f,stderr=subprocess.STDOUT)
with (root/(mode+'_command.json')).open('x') as f:json.dump({'command':command,'returncode':p.returncode,'seconds':time.monotonic()-t,'environment':{k:env[k] for k in ['WNOJ_INCREMENTAL_PROFILE','WNOJ_STRICT_PUBLICATION','AUDIT_RECORD_ALL','WNOJ_AUDIT_INDICES']}},f,indent=2)
print(mode,p.returncode,time.monotonic()-t,flush=True)
sys.exit(p.returncode)
