"""Bounded saved-input validation/timing; never starts a ROS or estimator node."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess
import time

ROOT=Path(__file__).resolve().parent.parent
WS=Path('/workspace/cbs_gtsam4.3')
S3=WS/'runs/shared_linearization_s3_capture_20261012'
REAL=WS/'runs/gnssfgo_shared_linearization_s3_20261012/prepared/profile40_on_source/native_gaussian.bin'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['release','asan','asan_nopie']);a=p.parse_args()
    build=ROOT/a.mode;out=ROOT/(a.mode+'_results');out.mkdir(exist_ok=False)
    env=os.environ.copy();env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    if a.mode.startswith('asan'):env.update(ASAN_OPTIONS='detect_leaks=1:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
    records=[]
    def run(name,cmd,success=True):
        t=time.monotonic()
        with (out/(name+'.jsonl')).open('x') as log, (out/(name+'.stderr')).open('x') as err:
            r=subprocess.run(cmd,env=env,stdout=log,stderr=err)
        records.append(dict(name=name,command=cmd,returncode=r.returncode,seconds=time.monotonic()-t))
        (out/'commands.json').write_text(json.dumps(records,indent=2))
        assert (r.returncode==0)==success,(name,r.returncode,(out/(name+'.stderr')).read_text()[-3000:])
    run('unit',[str(build/'tests'),'--gtest_output=xml:'+str(out/'unit.xml')])
    fixtures=[('pose',S3/'release_1/fixture.bin'),('imu',S3/'mixed_fixture/imu.bin')]
    for name,src in fixtures:
        for variant in ['original','instrumented']:
            run(name+'_'+variant,[str(build/variant),str(src),str(out/(name+'_'+variant+'.bin'))])
        assert sha(out/(name+'_original.bin'))==sha(out/(name+'_instrumented.bin'))
    # Negative reader checks use new disposable copies only.
    fixture=fixtures[0][1].read_bytes()
    for name,data in [('bad_magic',b'XXXXXXXX'+fixture[8:]),('truncated',fixture[:-5])]:
        src=out/(name+'.input');src.write_bytes(data)
        run(name,[str(build/'instrumented'),str(src),str(out/(name+'.bin'))],False)
    print(a.mode+' fixture, timer-equivalence and parser checks passed',flush=True)
    if a.mode=='release':
        pairs=[]
        for rep in range(4):
            label='warmup' if rep==0 else 'measured'+str(rep)
            variants=['original','instrumented'] if rep%2==0 else ['instrumented','original']
            for v in variants:
                name=label+'_'+v
                run(name,[str(build/v),str(REAL),str(out/(name+'.bin'))])
                rows=[json.loads(s) for s in (out/(name+'.jsonl')).read_text().splitlines()]
                end=rows[-1];assert end['kind']=='summary' and end['epoch']==783 and end['queries']==20 and end['retirements']==313
            h=[sha(out/(label+'_'+v+'.bin')) for v in variants];assert h[0]==h[1]
            if pairs:assert h[0]==pairs[0]['sha256']
            pairs.append(dict(label=label,sha256=h[0],byte_identical=True))
            print(label+' complete: 783 epochs, 20 query matrices exact between variants',flush=True)
    else:pairs=[]
    result=dict(passed=True,mode=a.mode,pairs=pairs,inputs={str(src):sha(src) for _,src in fixtures},real_journal_sha256=sha(REAL))
    (out/'complete.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)
if __name__=='__main__':main()
