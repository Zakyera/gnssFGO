"""Scoped, no-network build/solve/score launcher; no source processes or retries."""
from pathlib import Path
import json, subprocess, sys
ROOT=Path(__file__).resolve().parent
action=sys.argv[1];assert action in ['build','run','score']
image='sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d'
mounts={
 '/audit':(str(ROOT),'rw'),
 '/base':('/home/yeranis/results/gnssfgo_eth1_wnoa_mechanism_20261007','ro'),
 '/prior':('/home/yeranis/results/gnssfgo_eth1_wnoa_covariance_counterfactual_20261007','ro'),
 '/previous':('/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007','ro'),
 '/old_audit':('/home/yeranis/results/gnssfgo_eth1_wnoa_prediction_audit_20261007','ro'),
 '/source':('/home/yeranis/repos/gnssFGO','ro'),
 '/runtime':('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime','ro')}
entry=['python3','/audit/run.py',action]
if action=='score':
    image='sha256:7ad3854c699fb9a292aa5d42f8f890dfbd5dccb16633e043a93d135221418aaa'
    project='/home/yeranis/repos/V4RL/cbs_gtsam4.3';mounts={k:v for k,v in mounts.items() if k in ['/audit','/base','/prior']}
    mounts[project]=(project,'ro')
    entry=['bash','-c','source /opt/ros/noetic/setup.bash && python3 /audit/score.py']
cmd=['docker','run','--rm','--network','none','--read-only','--user','1000:1000','--tmpfs','/tmp:rw,exec,size=2g','-e','OMP_NUM_THREADS=1','-e','LD_LIBRARY_PATH=/runtime/lib']
for dest,(source,mode) in mounts.items():
    assert Path(source).is_dir(),source
    cmd+=['-v',f'{source}:{dest}:{mode}']
cmd+=[image,*entry]
with (ROOT/f'{action}_container_command.json').open('x') as f:json.dump(cmd,f,indent=2)
subprocess.run(cmd,check=True)
