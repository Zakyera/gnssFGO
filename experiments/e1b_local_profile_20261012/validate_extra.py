#!/usr/bin/env python3
"""Original-library oracle, deterministic repeat, sink contract and overhead.
Only standalone tests. Never starts GLIM, ROS, a receiver or bag playback.
"""
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess

ROOT=Path(__file__).resolve().parent
WS=Path('/workspace/cbs_gtsam4.3')
OLD=WS/'runs/glim_sync_compatible_batching_e1_20261011/private'
BUILD=ROOT/'release_2'
OUT=BUILD/'extra1'

def main():
    OUT.mkdir(exist_ok=False)
    env=os.environ.copy();env.pop('CBS_E1B_PROFILE_PATH',None)
    env['OMP_NUM_THREADS']='1'
    env['LD_LIBRARY_PATH']=f'{BUILD}:{OLD}:{WS}/devel/lib:'+env.get('LD_LIBRARY_PATH','')
    commands=[]
    def run(cmd,label,environment=env):
        with (OUT/(label+'.log')).open('x') as log:
            proc=subprocess.run(cmd,env=environment,stdout=log,stderr=subprocess.STDOUT)
        commands.append(dict(command=cmd,label=label,returncode=proc.returncode,
                             ld=environment.get('LD_LIBRARY_PATH',''),
                             sink=environment.get('CBS_E1B_PROFILE_PATH')))
        (OUT/'commands.json').write_text(json.dumps(commands,indent=2)+'\n')
        print(label,proc.returncode,flush=True)
        if proc.returncode:
            print((OUT/(label+'.log')).read_text()[-5000:]);raise SystemExit(proc.returncode)
        return (OUT/(label+'.log')).read_text()
    def fingerprint(text):
        return next(x for x in text.splitlines() if x.startswith('E1B_MATH,'))
    repeated=run(['taskset','-c','2',str(BUILD/'test_solver')],'repeat')
    native=env.copy();native['E1B_NATIVE_ORACLE']='1'
    native['LD_LIBRARY_PATH']=f'{OLD}:{WS}/devel/lib:{BUILD}:'+os.environ.get('LD_LIBRARY_PATH','')
    original=run(['taskset','-c','2',str(BUILD/'test_solver')],'original_libraries',native)
    first=(BUILD/'tests1/test_solver.log').read_text()
    assert fingerprint(first)==fingerprint(repeated)==fingerprint(original)
    run(['ldd',str(BUILD/'test_solver')],'ldd_candidate')
    run(['ldd',str(BUILD/'test_solver')],'ldd_original',native)
    compile=next(x['command'] for x in json.loads((BUILD/'commands.json').read_text()) if x['label']=='test_solver.cpp')
    compile=[str(ROOT/'bench.cpp') if x==str(ROOT/'test_solver.cpp') else x for x in compile]
    compile[compile.index('-o')+1]=str(OUT/'bench')
    run(compile,'compile_bench')
    sample=run(['taskset','-c','2',str(OUT/'bench')],'overhead')
    pairs=[list(map(int,x.split(',')[1:])) for x in sample.splitlines() if x.startswith('PAIR,')]
    assert len(pairs)==25
    off=[x[1]/350 for x in pairs];on=[x[2]/350 for x in pairs]
    sink_env=env.copy();sink_path=OUT/'sink.jsonl';sink_env['CBS_E1B_PROFILE_PATH']=str(sink_path)
    run(['taskset','-c','2',str(OUT/'bench'),'sink'],'sink_enabled',sink_env)
    rows=[json.loads(x) for x in sink_path.read_text().splitlines()]
    assert len(rows)==35 and all(r['context']==2 and not r['aborted'] for r in rows)
    for r in rows:
        owner=r['owner_wall'];worker=r['worker_sum']
        assert sum(c[2] for c in owner.values())==owner['root'][1]
        assert sum(c[2] for c in worker.values())==worker.get('worker_root',[0,0])[1]
        assert all(0<=c[2]<=c[1] for c in list(owner.values())+list(worker.values()))
    before=hashlib.sha256(sink_path.read_bytes()).hexdigest()
    refused=run(['taskset','-c','2',str(OUT/'bench'),'sink'],'sink_refused',sink_env)
    assert 'E1B_PROFILE_SINK_ERROR' in refused
    assert hashlib.sha256(sink_path.read_bytes()).hexdigest()==before
    absent=run(['taskset','-c','2',str(OUT/'bench'),'sink'],'sink_disabled')
    assert 'E1B_PROFILE_SINK_ERROR' not in absent
    # Focused profiler memory/UB safety including independent worker contexts.
    sanitizer=OUT/'profile_asan_ubsan'
    run(['/usr/bin/c++','-std=c++17','-O1','-g','-pthread','-fsanitize=address,undefined',
         '-fno-sanitize-recover=all','-fno-omit-frame-pointer','-I'+str(ROOT),
         str(ROOT/'RuntimeProfile.cpp'),str(ROOT/'test_profile.cpp'),'-lgtest_main','-lgtest',
         '-o',str(sanitizer)],'compile_asan_ubsan')
    sanenv=env.copy();sanenv['ASAN_OPTIONS']='detect_leaks=1:halt_on_error=1'
    sanenv['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
    run([str(sanitizer),'--gtest_output=xml:'+str(OUT/'asan_ubsan.xml')],'asan_ubsan',sanenv)
    summary=dict(passed=True,fingerprint_sha256=hashlib.sha256(fingerprint(first).encode()).hexdigest(),
                 exact_numbers=len(fingerprint(first).split(','))-3,sink_rows=len(rows),
                 sink_no_overwrite=True,overhead=dict(pairs=25,fixture_updates_per_sample=350,
                 off_median_ns_per_update=statistics.median(off),on_median_ns_per_update=statistics.median(on),
                 paired_ratio_median=statistics.median([b/a for a,b in zip(off,on)]),
                 paired_extra_ns_median=statistics.median([b-a for a,b in zip(off,on)]),
                 note='Includes fixture setup, estimate/covariance queries and exact comparison; RAM records, no file I/O.'))
    (OUT/'passed.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))

if __name__=='__main__':main()
