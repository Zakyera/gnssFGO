#!/usr/bin/env python3
"""Standalone fixture timing including JSON serialization/flush; no ROS run."""
import json
import os
from pathlib import Path
import statistics
import subprocess

ROOT=Path(__file__).resolve().parent
WS=Path('/workspace/cbs_gtsam4.3')
OLD=WS/'runs/glim_sync_compatible_batching_e1_20261011/private'
BUILD=ROOT/'release_2'

def main():
    out=BUILD/'io1';out.mkdir(exist_ok=False)
    env=os.environ.copy();env.pop('CBS_E1B_PROFILE_PATH',None)
    env['OMP_NUM_THREADS']='1'
    env['LD_LIBRARY_PATH']=f'{BUILD}:{OLD}:{WS}/devel/lib:'+env.get('LD_LIBRARY_PATH','')
    commands=[]
    def run(cmd,label,e=env):
        with (out/(label+'.log')).open('x') as log:
            p=subprocess.run(cmd,env=e,stdout=log,stderr=subprocess.STDOUT)
        commands.append(dict(command=cmd,label=label,returncode=p.returncode,
                             sink=e.get('CBS_E1B_PROFILE_PATH')))
        (out/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
        print(label,p.returncode,flush=True)
        assert p.returncode==0,(label,p.returncode)
        return (out/(label+'.log')).read_text()
    compile=next(x['command'] for x in json.loads((BUILD/'commands.json').read_text()) if x['label']=='test_solver.cpp')
    compile=[str(ROOT/'bench.cpp') if x==str(ROOT/'test_solver.cpp') else x for x in compile]
    compile[compile.index('-o')+1]=str(out/'bench')
    run(compile,'compile')
    # Bracket ON by OFF to make drift visible; do not hide a noisy baseline.
    samples={}
    for mode in ('off_before','on','off_after'):
        e=env.copy()
        if mode=='on':e['CBS_E1B_PROFILE_PATH']=str(out/'profile.jsonl')
        log=run(['taskset','-c','2',str(out/'bench'),'file_bench'],mode,e)
        samples[mode]=[int(x.split(',')[2])/350 for x in log.splitlines() if x.startswith('FILE_SAMPLE,')]
        assert len(samples[mode])==25
    rows=[json.loads(x) for x in (out/'profile.jsonl').read_text().splitlines()]
    assert len(rows)==8750
    for row in rows:
        assert not row['aborted'] and row['context']==2
        assert sum(c[2] for c in row['owner_wall'].values())==row['owner_wall']['root'][1]
    summary=dict(rows=len(rows),median_ns_per_update={k:statistics.median(v) for k,v in samples.items()},
        min_ns_per_update={k:min(v) for k,v in samples.items()},max_ns_per_update={k:max(v) for k,v in samples.items()},
        includes='Full fixture, timers, JSON serialization and fflush to local page cache; not fsync, source data, CUDA or disk saturation.')
    (out/'passed.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
