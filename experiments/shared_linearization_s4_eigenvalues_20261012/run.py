"""Correctness first; matched timing only after both validation markers exist."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess
import time

ROOT=Path(__file__).resolve().parent.parent
WS=Path('/workspace/cbs_gtsam4.3')
BASE=WS/'runs/shared_linearization_s4_timing_20261012'
S3=WS/'runs/shared_linearization_s3_capture_20261012'
REAL=WS/'runs/gnssfgo_shared_linearization_s3_20261012/prepared/profile40_on_source/native_gaussian.bin'
EXPECTED='3652c745b2f0005c4dbaf230dbcd5b26ecb89d2fd2cecc85bff8afd7d20841bd'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('phase',choices=['release','asan','timing']);a=p.parse_args()
    mode='release' if a.phase=='timing' else a.phase
    build=ROOT/mode;ref=BASE/('asan_nopie' if mode=='asan' else 'release')
    if a.phase=='timing':
        for m in ['release','asan']:
            assert json.loads((ROOT/(m+'_results/complete.json')).read_text())['passed']
    out=ROOT/(a.phase+'_results');out.mkdir(exist_ok=False)
    env=os.environ.copy();env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
    if mode=='asan':env.update(ASAN_OPTIONS='detect_leaks=1:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
    records=[];identities={}
    def run(name,cmd,success=True):
        t=time.monotonic()
        with (out/(name+'.jsonl')).open('x') as log,(out/(name+'.stderr')).open('x') as err:
            r=subprocess.run(cmd,env=env,stdout=log,stderr=err)
        records.append(dict(name=name,command=cmd,returncode=r.returncode,seconds=time.monotonic()-t))
        (out/'commands.json').write_text(json.dumps(records,indent=2))
        assert (r.returncode==0)==success,(name,r.returncode,(out/(name+'.stderr')).read_text()[-3000:])
    def replay(name,exe,src):
        output=out/(name+'.bin');run(name,[str(exe),str(src),str(output)])
        identities[name]=sha(output)
        if src==REAL:
            end=json.loads((out/(name+'.jsonl')).read_text().splitlines()[-1])
            assert end['kind']=='summary' and end['epoch']==783 and end['queries']==20 and end['retirements']==313,end
            assert identities[name]==EXPECTED,(name,identities[name])
        return identities[name]
    if a.phase!='timing':
        run('unit',[str(build/'tests'),'--gtest_output=xml:'+str(out/'unit.xml')])
        fixtures=[('pose',S3/'release_1/fixture.bin'),('imu',S3/'mixed_fixture/imu.bin')]
        for name,src in fixtures:
            assert replay(name+'_original',ref/'original',src)==replay(name+'_candidate',build/'candidate',src)
        fixture=fixtures[0][1].read_bytes()
        for name,data in [('bad_magic',b'XXXXXXXX'+fixture[8:]),('truncated',fixture[:-5])]:
            src=out/(name+'.input');src.write_bytes(data)
            run(name,[str(build/'candidate'),str(src),str(out/(name+'.bin'))],False)
        if a.phase=='release':
            replay('real_original',ref/'original',REAL);replay('real_candidate',build/'candidate',REAL)
    else:
        for rep in range(4):
            label='warmup' if rep==0 else 'measured'+str(rep)
            order=['original','candidate'] if rep%2==0 else ['candidate','original']
            for v in order:replay(label+'_'+v,ref/'original' if v=='original' else build/'candidate',REAL)
            print(label+' passed: full saved outputs byte-identical',flush=True)
        replay('profile_original',ref/'instrumented',REAL)
        replay('profile_candidate',build/'instrumented',REAL)
    result=dict(passed=True,phase=a.phase,outputs=identities,real_journal_sha256=sha(REAL))
    (out/'complete.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True)

if __name__=='__main__':main()
