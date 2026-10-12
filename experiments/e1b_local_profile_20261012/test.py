#!/usr/bin/env python3
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parent
WS=Path('/workspace/cbs_gtsam4.3')
OLD=WS/'runs/glim_sync_compatible_batching_e1_20261011/private'

def main():
    p=argparse.ArgumentParser();p.add_argument('build');p.add_argument('--tag',default='tests1');args=p.parse_args()
    build=ROOT/args.build;assert (build/'complete.json').exists()
    out=build/args.tag;out.mkdir(exist_ok=False)
    env=os.environ.copy();env.pop('CBS_E1B_PROFILE_PATH',None)
    env['LD_LIBRARY_PATH']=f'{build}:{OLD}:{WS}/devel/lib:'+env.get('LD_LIBRARY_PATH','')
    env['OMP_NUM_THREADS']='1';env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
    records=[]
    for name in ('test_profile','test_solver','test_gpu'):
        cmd=['taskset','-c','2',str(build/name),'--gtest_output=xml:'+str(out/(name+'.xml'))]
        start=time.monotonic()
        with (out/(name+'.log')).open('x') as f:
            result=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT)
        records.append(dict(command=cmd,seconds=time.monotonic()-start,returncode=result.returncode))
        (out/'commands.json').write_text(json.dumps(records,indent=2)+'\n')
        print(name,result.returncode,flush=True)
        if result.returncode:
            print((out/(name+'.log')).read_text()[-5000:]);raise SystemExit(result.returncode)
    (out/'passed.json').write_text(json.dumps(dict(passed=True,environment=env['LD_LIBRARY_PATH']),indent=2)+'\n')

if __name__=='__main__':main()
