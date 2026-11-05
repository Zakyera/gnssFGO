"""One exclusive reporting observer replay of the new CP-V29 OFF journal."""
import importlib.util, json, os, shutil, subprocess, time
from pathlib import Path
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_incremental_cpv29_20261028/prepared')
spec=importlib.util.spec_from_file_location('cpv28_replay',H.parent/'all_incremental_comparison_cpv28_20261028/replay.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)

def main():
    root=P/'full_off_receiver';out=H/'fusion_observer_replay';pub=H/'fusion_publications.csv'
    assert not out.exists() and not pub.exists()
    assert (P/'full_off_control/complete.json').exists()
    assert shutil.disk_usage(H).free>8*2**30
    cfg=json.loads((root/'resolved_config.json').read_text())
    assert cfg['support_period_ns']==100000000 and cfg['startup_samples']==59
    assert cfg['receiver_model']=='source_wnoa' and cfg['receiver_fixed_lag_seconds']==7
    assert cfg['receiver_environment']=={'LD_LIBRARY_PATH':m.LIB,'OMP_NUM_THREADS':'1'}
    pins=json.loads((m.H23/'dependency_pins.json').read_text())
    old=json.loads((m.H23/'input_pins.json').read_text())
    pins.update({p:d for p,d in old.items() if p.startswith(m.LIB+'/')})
    pins[str(m.EXE)]=json.loads((m.H23/'binary.json').read_text())['sha256']
    for n in (*m.FILES,'../events.csv','../resolved_config.json'):
        p=(root/'receiver'/n).resolve();pins[str(p)]=m.sha(p)
    assert all(m.sha(p)==d for p,d in pins.items())
    m.save(H/'fusion_replay_pins.json',pins)
    env=dict(os.environ)
    for k in list(env):
        if k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) or k=='LD_PRELOAD':env.pop(k)
    env.update(LD_LIBRARY_PATH=m.LIB,OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',
               E2_EXACT_LINEARIZATION_CACHE='0',CPV23_PUBLICATION_FILE=str(pub))
    cmd=['taskset','-c','2',str(m.EXE),'--absorbed-revision-policy','seal_at_absorption',
         '--mode','sources','--timeline','record','--solver','isam2',
         '--support-policy','uniform_imu','--support-period-ns','100000000',
         '--receiver-startup-samples','59','--receiver-model','source_wnoa',
         '--event-stream',str(root/'events.csv'),'--out',str(out)]
    with (H/'fusion_observer.log').open('xb') as log:
        proc=subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT)
        m.save(H/'fusion_replay_launch.json',dict(pid=proc.pid,command=cmd,time=time.time()))
        code=proc.wait()
    m.save(H/'fusion_replay_finish.json',dict(returncode=code,time=time.time()))
    assert code==0
    result=m.parity(root/'receiver',out,pub)
    assert all(m.sha(p)==d for p,d in pins.items())
    result['input_pins_unchanged']=len(pins)
    m.save(H/'fusion_parity.json',result)
    print(json.dumps(result),flush=True)

if __name__=='__main__':main()
