"""Create-once observer gates; no source launch or optimizer policy changes."""
import csv
import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_linear_completion_s5e_20261013')
E2B=HERE.parent/'receiver_endpoint_reuse_e2b_20261013'
ORACLE=Path('/home/yeranis/results/gnssfgo_endpoint_reuse_e2b_20261013')
OLD=Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011')
RT=Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime')
JOURNAL=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared/capacity160_on_receiver/events.csv')
PREFIX=Path('/home/yeranis/results/gnssfgo_receiver_subphase_s5b_20261013/prefix5_valid.csv')

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m

def sha(path):
    h=hashlib.sha256()
    with open(path,'rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()

def integrity(phase):
    m=module('integrity_base',HERE.parent/'receiver_subphase_s5b_20261013/integrity.py')
    protected=json.loads((m.ROOT/'protected.json').read_text());prior=json.loads(m.PRIOR.read_text())
    for p,w in protected.items(): assert sha(p)==w,p
    for p,w in prior['raw_artifacts'].items():assert sha(p)==w['sha256'],p
    for p,w in prior['source_repositories'].items():
        got=dict(head=subprocess.check_output(['git','-C',p,'rev-parse','HEAD'],text=True).strip(),
                 status=subprocess.check_output(['git','-C',p,'status','--porcelain'],text=True).strip())
        assert got==w,(p,got,w)
    oracle={str(p):sha(p) for folder in ['full_on1','prefix_on'] for p in (ORACLE/folder).iterdir() if p.is_file()}
    oracle[str(JOURNAL)]=sha(JOURNAL)
    for p in Path('/home/yeranis/results/gnssfgo_publication_s5d_20261013/full_on').rglob('*'):
        if p.is_file():oracle[str(p)]=sha(p)
    if phase=='after':assert json.loads((ROOT/'before.json').read_text())['oracle']==oracle
    return dict(passed=True,protected_files=len(protected),raw_artifacts=len(prior['raw_artifacts']),
                source_repositories=len(prior['source_repositories']),oracle=oracle)

def select():
    def rows(name):return list(csv.DictReader((ORACLE/'full_on1'/name).open()))
    conv=[r for r in rows('publication_convergence.csv') if r['phase']=='support']
    recovery={int(r['index']) for r in rows('publication_steps.csv') if r['phase']=='support' and r['recovery']=='1'}
    late=[r for r in conv if int(r['index'])>=1400]
    picks={}
    for name,want in [('recovery',True),('ordinary',False)]:
        eligible=[r for r in late if (int(r['index']) in recovery)==want]
        r=sorted(eligible,key=lambda r:(-int(r['attempted']),int(r['index'])))[0]
        picks[name]=dict(index=int(r['index']),attempted=int(r['attempted']))
    indices=sorted(set([10,20,40,100,200,300,400,700,1000,1200,1300,1400,1420,1440,1460,1480]+[p['index'] for p in picks.values()]))
    return dict(passed=True,indices=indices,retained_selection=picks)

def command(kind):
    record='candidate_build.json' if kind=='replay' else 'candidate_unit.json'
    cmd=json.loads((OLD/record).read_text())['commands'][0]
    cmd[6:6]=['-I'+str(HERE),'-I'+str(E2B)]
    for i,a in enumerate(cmd):
        if a.endswith('.cpp'):
            p=HERE/Path(a).name
            if p.exists():cmd[i]=str(p)
            elif (E2B/Path(a).name).exists():cmd[i]=str(E2B/Path(a).name)
    cmd[-1]=str(ROOT/'build'/kind)
    if kind!='replay':cmd[cmd.index('-o'):cmd.index('-o')]=[str(HERE/'test_endpoints.cpp'),str(HERE/'test_linear_completion.cpp')]
    if kind.startswith('sanitize'):
        cmd.remove('-O3');cmd.remove('-DNDEBUG')
        cmd[1:1]=['-O1','-g1','-DEIGEN_MALLOC_ALREADY_ALIGNED=1','-fno-omit-frame-pointer','-fsanitize=address,undefined']
    assert not Path(cmd[-1]).exists()
    return cmd

def main():
    action=sys.argv[1];ROOT.mkdir(exist_ok=True);(ROOT/'build').mkdir(exist_ok=True)
    with (ROOT/'controller.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        with (ROOT/(action+'.started')).open('x') as f:f.write(str(os.getpid()))
        if action in ['before','after','after_final','select']:
            result=select() if action=='select' else integrity('after' if action=='after_final' else action)
        else:
            os.sched_setaffinity(0,{2})
            env={k:v for k,v in os.environ.items() if not k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','S5D_','S5E_','E4_','CACHE_FAILURE')) and k!='LD_PRELOAD'}
            env.update(LD_LIBRARY_PATH=str(RT/'lib'),OMP_NUM_THREADS='1',WNOJ_STRICT_PUBLICATION='1',
                       WNOJ_INCREMENTAL_PROFILE='nominal',E2_EXACT_LINEARIZATION_CACHE='1',E2B_ENDPOINT_CACHE='1',
                       ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
            if action.startswith('build_'):cmd=command(action[6:].removesuffix('_fixed'))
            elif action in ['unit_checked','sanitize','unit_consistent','sanitize_consistent']:
                build_action='build_'+action+('_fixed' if action=='unit_consistent' else '')
                assert json.loads((ROOT/(build_action+'.json')).read_text())['passed']
                cmd=[str(ROOT/'build'/action),'--gtest_output=xml:'+str(ROOT/(action+'.xml'))]
            elif action in ['prefix_off','prefix_on','full_on']:
                assert json.loads((ROOT/'unit_checked.json').read_text())['passed']
                prereq={'prefix_off':'sanitize','prefix_on':'prefix_off','full_on':'prefix_on'}[action]
                assert json.loads((ROOT/(prereq+'.json')).read_text())['passed']
                if action!='prefix_off':env['S5E_INDICES']=','.join(map(str,json.loads((ROOT/'select.json').read_text())['indices']))
                assert not (ROOT/action).exists()
                cmd=[str(ROOT/'build/replay'),'--receiver-model','source_wnoa','--mode','sources','--timeline','record',
                     '--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000',
                     '--receiver-startup-samples','59','--event-stream',str(JOURNAL if action=='full_on' else PREFIX),
                     '--out',str(ROOT/action)]
            else:raise ValueError(action)
            start=time.monotonic();error=None
            with (ROOT/(action+'.log')).open('x') as log:
                try:rc=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1800).returncode
                except Exception as ex:rc=-1;error=repr(ex)
            result=dict(passed=rc==0,returncode=rc,error=error,seconds=time.monotonic()-start,command=cmd)
            if rc==0 and action in ['prefix_off','prefix_on','full_on']:
                m=module('compare',HERE.parent/'receiver_subphase_s5b_20261013/compare.py')
                oracle=ORACLE/('full_on1' if action=='full_on' else 'prefix_on')
                result['comparison']=m.pair(oracle,ROOT/action)
                result['endpoint_counters_exact']=(oracle/'e2b_endpoints.csv').read_bytes()==(ROOT/action/'e2b_endpoints.csv').read_bytes()
                result['passed']=result['comparison']['passed'] and result['endpoint_counters_exact']
            if rc==0 and action.startswith('build_'):result['binary_sha256']=sha(cmd[-1])
        with (ROOT/(action+'.json')).open('x') as f:json.dump(result,f,indent=2)
        print(action,'PASS' if result['passed'] else 'FAIL',result.get('seconds',''),flush=True)
        if not result['passed']:raise SystemExit(1)
if __name__=='__main__':main()
