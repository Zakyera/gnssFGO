"""Read-only bag coverage check; constructs no GLIM estimator or GPU factors."""
from pathlib import Path
import os,json,subprocess
W=Path('/workspace/cbs_gtsam4.3');R=W/'runs/main_gpu_batching_cpk16_20261020';P=R/'k16h_source_1'
out=P/'coverage_3';assert not out.exists();out.mkdir()
m=json.loads((P/'input_manifest.json').read_text())
env=os.environ.copy()
env.pop('LD_PRELOAD',None)
env['LD_LIBRARY_PATH']=':'.join(map(str,[R/'release_1',W/'runs/native_covariance_cache_cpi3_20261019/release_v2',W/'runs/local_revision_covariance_cpi1_20261019',W/'runs/shared_linearization_s4_provider_20261012/release_2']))+':'+env.get('LD_LIBRARY_PATH','')
cmd=[str(P/'driver_build_4/paced_input'),'inspect',str(P/'glim_config'),
     m['topics']['/boxi/cpt7/imu']['bag'],m['topics']['/boxi/hesai/points']['bag'],
     str(m['window_start_ns']),str(m['window_end_ns']),str(out),'90']
with (out/'command.json').open('x') as f:json.dump(cmd,f)
with (out/'inspect.log').open('x') as f:subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT,check=True,timeout=120)
print((out/'coverage_complete.json').read_text())
