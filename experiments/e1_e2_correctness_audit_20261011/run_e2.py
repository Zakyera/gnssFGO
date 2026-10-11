"""Bounded E2 private builds and single-core exact frozen-journal controls."""
import hashlib,json,os,resource,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011')
OLD=ROOT.parent/'gnssfgo_e2_exact_evaluation_20261011'
REPO=HERE.parents[1]
RT=ROOT.parent/'gnssfgo_synthetic_uncertainty_20261004/baseline/runtime'
VW=ROOT.parent/'gnssfgo_vw_jacobian_20261011/build'
PRIOR=ROOT.parent/'gnssfgo_publication_cache_20261010/build'
action=sys.argv[1]
assert action in ['original_unit','candidate_unit','candidate_sanitize','candidate_build',
                  'original_micro','candidate_micro','off1','on1','on2','off2','reference_pinned','reference_unpinned']
record=ROOT/(action+'.json');assert not record.exists(),'Refuse repeated action'
env=os.environ.copy();env.update(OMP_NUM_THREADS='1',LD_LIBRARY_PATH=str(RT/'lib'),
    WNOJ_STRICT_PUBLICATION='1',WNOJ_INCREMENTAL_PROFILE='nominal')
for k in ['WNOJ_AUDIT_INDICES','CACHE_FAILURE_PROBE_DIR','E2_EXACT_LINEARIZATION_CACHE']:env.pop(k,None)
def snapshot():
    p=Path('/sys/devices/system/cpu/cpu2/cpufreq/scaling_cur_freq')
    return dict(affinity=sorted(os.sched_getaffinity(0)),load=os.getloadavg(),
      frequency_khz=p.read_text().strip() if p.exists() else None)
if action!='reference_unpinned':
    os.sched_setaffinity(0,{2});assert os.sched_getaffinity(0)=={2}
assert Path('/sys/bus/event_source/devices/cpu_core/cpus').read_text().strip()=='0-7'
before=snapshot();commands=[]
if '_' in action and not action.startswith('reference_'):
    variant,kind=action.split('_');build=ROOT/variant
    if kind=='micro':
        commands=[[str(build/'unit'),'--gtest_also_run_disabled_tests','--gtest_filter=E2Audit.DISABLED_Microbench']]
    else:
        include=[build,VW,PRIOR,RT/'include',REPO/'online_fgo',REPO/'online_fgo/include',Path('/usr/include/eigen3')]
        reference=json.loads((VW.parent/'command_build.json').read_text())['commands'][0]['command']
        include += [Path(a[2:]) for a in reference if a.startswith('-I/opt/ros/humble/include/')]
        files=[build/'test_e2_audit.cpp',VW/'test_cache.cpp',PRIOR/'test_refinement.cpp',build/'test_twist.cpp',
          *[REPO/'online_fgo/test'/n for n in ['test_transactional_isam2_rebuild.cpp','test_stationary_qr_consensus.cpp','test_bounded_isam2_refinement.cpp']]]
        flags=['-O3','-DNDEBUG','-fopenmp','-fPIC']
        if kind=='sanitize':
            flags=['-O1','-g1','-DEIGEN_MALLOC_ALREADY_ALIGNED=1','-fno-omit-frame-pointer','-fsanitize=address,undefined']
            env.update(ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
        if kind=='build':files=[build/'DiagnosticReplay.cpp']
        binary=build/('replay' if kind=='build' else 'sanitize' if kind=='sanitize' else 'unit')
        commands=[['c++','-std=c++20',*flags,*['-I'+str(p) for p in include],*map(str,files),
          '-L'+str(RT/'lib'),'-lonline_fgo_solver','-lonline_fgo_util',str(RT/'lib/libgtsam.so.4'),
          '-ltbb','-Wl,-rpath,'+str(RT/'lib'),*(['-lgtest_main','-lgtest','-lpthread'] if kind!='build' else []),'-o',str(binary)]]
        if kind!='build':commands.append([str(binary),'--gtest_output=xml:'+str(ROOT/(action+'.xml'))])
else:
    for gate in ['candidate_unit','candidate_sanitize','candidate_build']:
        assert json.loads((ROOT/(gate+'.json')).read_text())['passed'],gate
    if not action.startswith('reference_'):
        ordered=['off1','on1','on2','off2'];index=ordered.index(action)
        if index:assert json.loads((ROOT/(ordered[index-1]+'.json')).read_text())['passed']
        if index:assert json.loads((ROOT/'environment_control.json').read_text())['passed']
    out=ROOT/action;assert not out.exists()
    binary=VW/'replay' if action.startswith('reference_') else ROOT/'candidate/replay'
    env['E2_EXACT_LINEARIZATION_CACHE']='1' if action.startswith('on') else '0'
    journal=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_live60_20261011/prepared/live60_on_receiver/events.csv')
    commands=[[str(binary),'--receiver-model','source_wnoa','--mode','sources','--timeline','record',
      '--solver','isam2','--support-policy','uniform_imu','--support-period-ns','100000000',
      '--receiver-startup-samples','59','--event-stream',str(journal),'--out',str(out)]]
start=time.monotonic();code=0
with (ROOT/'logs'/(action+'.log')).open('x') as log:
    for cmd in commands:
        try:code=subprocess.run(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1800).returncode
        except subprocess.TimeoutExpired:code=124
        if code:break
usage=resource.getrusage(resource.RUSAGE_CHILDREN)
payload=dict(passed=code==0,returncode=code,seconds=time.monotonic()-start,commands=commands,
    before=before,after=snapshot(),max_rss_kib=usage.ru_maxrss,user_seconds=usage.ru_utime,
    system_seconds=usage.ru_stime,environment={k:v for k,v in env.items() if k.startswith(('E2_','WNOJ_','OMP_','LD_LIBRARY','ASAN_','UBSAN_'))})
if '_' not in action or action.startswith('reference_'):payload['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
with record.open('x') as f:json.dump(payload,f,indent=2)
print(action,code,payload['seconds'],flush=True)
if code:print((ROOT/'logs'/(action+'.log')).read_text()[-6000:])
raise SystemExit(code)
