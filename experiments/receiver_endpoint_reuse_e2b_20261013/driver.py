"""Create-once private build/test/replay actions; no source estimator launch."""
import fcntl
import importlib.util
import json
import os
from pathlib import Path
import resource
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = Path('/home/yeranis/results/gnssfgo_endpoint_reuse_e2b_20261013')
OLD = Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011')
RT = Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime')
REF = Path('/home/yeranis/results/gnssfgo_receiver_sampling_s5c_20261013')
JOURNAL = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_shared_linearization_s5_live160_20261012/prepared/capacity160_on_receiver/events.csv')
PREFIX = Path('/home/yeranis/results/gnssfgo_receiver_subphase_s5b_20261013/prefix5_valid.csv')

def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); return m

def build(kind):
    record = 'candidate_build.json' if kind == 'replay' else 'candidate_unit.json'
    cmd = json.loads((OLD/record).read_text())['commands'][0]
    cmd.insert(6, '-I'+str(HERE))
    for i, arg in enumerate(cmd):
        if arg.endswith('.cpp') and (HERE/Path(arg).name).exists(): cmd[i] = str(HERE/Path(arg).name)
    cmd[-1] = str(ROOT/'build'/kind)
    if kind != 'replay': cmd.insert(cmd.index('-o'), str(HERE/'test_endpoints.cpp'))
    if kind == 'sanitize':
        cmd.remove('-O3'); cmd.remove('-DNDEBUG')
        cmd[1:1] = ['-O1','-g1','-DEIGEN_MALLOC_ALREADY_ALIGNED=1','-fno-omit-frame-pointer','-fsanitize=address,undefined']
    assert not Path(cmd[-1]).exists(), 'Refuse binary overwrite'
    return cmd

def metadata():
    freq = Path('/sys/devices/system/cpu/cpu2/cpufreq/scaling_cur_freq')
    return dict(affinity=sorted(os.sched_getaffinity(0)), load=os.getloadavg(),
                frequency_khz=freq.read_text().strip() if freq.exists() else None)

def main():
    action = sys.argv[1]
    ROOT.mkdir(exist_ok=True); (ROOT/'build').mkdir(exist_ok=True)
    with (ROOT/'controller.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        assert not (ROOT/(action+'.json')).exists(), 'Refuse action overwrite'
        # A crash leaves this marker: never automatically retry an uncertain action.
        with (ROOT/(action+'.started')).open('x') as f: f.write(str(os.getpid()))
        os.sched_setaffinity(0,{2})
        env = {k:v for k,v in os.environ.items() if not k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) and k!='LD_PRELOAD'}
        env.update(LD_LIBRARY_PATH=str(RT/'lib'), OMP_NUM_THREADS='1',
                   WNOJ_STRICT_PUBLICATION='1', WNOJ_INCREMENTAL_PROFILE='nominal',
                   E2_EXACT_LINEARIZATION_CACHE='1', E2B_ENDPOINT_CACHE='0',
                   ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',
                   UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
        comparison = None
        if action.startswith('build_'):
            # Explicit correction of the diagnostic include overlay; initial
            # compile log is retained, and no estimator replay is retried.
            cmd = build('replay' if action=='build_replay_overlay' else action[6:])
        elif action in ['unit_off','unit_on','sanitize_off','sanitize_on']:
            kind, switch = action.split('_'); env['E2B_ENDPOINT_CACHE'] = str(int(switch=='on'))
            assert (ROOT/('build_'+kind+'.json')).exists()
            cmd = [str(ROOT/'build'/kind), '--gtest_output=xml:'+str(ROOT/(action+'.xml'))]
        elif action in ['prefix_off','prefix_on','full_off1','full_on1','full_on2','full_off2']:
            prerequisites = dict(prefix_off='sanitize_on',prefix_on='prefix_off',full_off1='prefix_on',
                                 full_on1='full_off1',full_on2='full_on1',full_off2='full_on2')
            assert json.loads((ROOT/(prerequisites[action]+'.json')).read_text())['passed']
            assert not (ROOT/action).exists(), 'Refuse output overwrite'
            env['E2B_ENDPOINT_CACHE'] = str(int('_on' in action))
            cmd = [str(ROOT/'build/replay'),'--receiver-model','source_wnoa','--mode','sources',
                   '--timeline','record','--solver','isam2','--support-policy','uniform_imu',
                   '--support-period-ns','100000000','--receiver-startup-samples','59',
                   '--event-stream',str(PREFIX if action.startswith('prefix') else JOURNAL),
                   '--out',str(ROOT/action)]
            comparison = REF/('prefix_reference' if action.startswith('prefix') else 'reference')
        else: raise ValueError(action)
        before = metadata(); start = time.monotonic(); error = None
        with (ROOT/(action+'.log')).open('x') as log:
            try: rc = subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1800).returncode
            except Exception as e: rc = -1; error = repr(e)
        seconds = time.monotonic()-start; usage = resource.getrusage(resource.RUSAGE_CHILDREN)
        result = dict(action=action,command=cmd,returncode=rc,seconds=seconds,error=error,
                      before=before,after=metadata(),max_rss_kib=usage.ru_maxrss,
                      user_seconds=usage.ru_utime,system_seconds=usage.ru_stime,
                      environment={k:env[k] for k in ['LD_LIBRARY_PATH','OMP_NUM_THREADS',
                        'WNOJ_STRICT_PUBLICATION','WNOJ_INCREMENTAL_PROFILE',
                        'E2_EXACT_LINEARIZATION_CACHE','E2B_ENDPOINT_CACHE']},passed=rc==0)
        if rc==0 and comparison:
            m=load_module('s5b_compare',HERE.parent/'receiver_subphase_s5b_20261013/compare.py')
            result['validation']=m.pair(comparison,ROOT/action)
            result['passed']=result['validation']['passed']
        with (ROOT/(action+'.json')).open('x') as f: json.dump(result,f,indent=2)
        print(action,'PASS' if result['passed'] else 'FAIL',seconds,flush=True)
        if not result['passed']: raise SystemExit(1)

if __name__=='__main__': main()
