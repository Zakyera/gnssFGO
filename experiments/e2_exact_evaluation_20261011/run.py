"""Run in existing Humble container; bounded private build/replay only."""
import json,os,resource,subprocess,sys,time
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_e2_exact_evaluation_20261011')
REPO=HERE.parents[1]
VW=Path('/home/yeranis/results/gnssfgo_vw_jacobian_20261011/build')
PRIOR=Path('/home/yeranis/results/gnssfgo_publication_cache_20261010/build')
RT=Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime')
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')
action=sys.argv[1]
assert action in ['build_probe','unit_probe','probe_live60','build_memo','unit_memo','sanitizer_memo',
                  'reference_live60','candidate_live60','reference_partial143','candidate_partial143',
                  'reference_live60_repeat','candidate_live60_repeat']
record=ROOT/f'command_{action}.json';assert not record.exists(),'Refuse duplicate action'
env=os.environ.copy();env.update(OMP_NUM_THREADS='1',LD_LIBRARY_PATH=str(RT/'lib'),
    WNOJ_STRICT_PUBLICATION='1',WNOJ_INCREMENTAL_PROFILE='nominal')
env.pop('WNOJ_AUDIT_INDICES',None);env.pop('CACHE_FAILURE_PROBE_DIR',None)
if action.startswith(('build_','unit_','sanitizer_')):
    memo=action.endswith('_memo');build=ROOT/('candidate_build' if memo else 'probe')
    include=[build,VW,PRIOR,RT/'include',REPO/'online_fgo',REPO/'online_fgo/include',Path('/usr/include/eigen3')]
    # Preserve the successful reference's ROS include list. Adding every current
    # directory can shadow libc headers (e.g. idl/string.h, features/features.h).
    reference=json.loads((VW.parent/'command_build.json').read_text())['commands'][0]['command']
    include += [Path(arg[2:]) for arg in reference if arg.startswith('-I/opt/ros/humble/include/')]
    unit=not action.startswith('build_')
    files=[str(HERE/('test_memo.cpp' if memo else 'test_probe.cpp')),str(VW/'test_cache.cpp'),str(PRIOR/'test_refinement.cpp')]
    if memo:files += [str(build/'test_twist.cpp'),*[str(REPO/'online_fgo/test'/name) for name in
        ['test_transactional_isam2_rebuild.cpp','test_stationary_qr_consensus.cpp','test_bounded_isam2_refinement.cpp']]]
    if not unit:files=[str(build/'DiagnosticReplay.cpp')]
    binary=build/(action if unit else 'replay')
    flags=['-O3','-DNDEBUG','-fopenmp','-fPIC']
    if action=='sanitizer_memo':
        flags=['-O1','-g1','-DEIGEN_MALLOC_ALREADY_ALIGNED=1','-fno-omit-frame-pointer','-fsanitize=address,undefined']
        env.update(ASAN_OPTIONS='detect_leaks=0:halt_on_error=1',UBSAN_OPTIONS='halt_on_error=1:print_stacktrace=1')
    command=['c++','-std=c++20',*flags,*['-I'+str(p) for p in include],
      *files,'-L'+str(RT/'lib'),'-lonline_fgo_solver','-lonline_fgo_util',
      str(RT/'lib/libgtsam.so.4'),'-ltbb','-Wl,-rpath,'+str(RT/'lib'),
      *(['-lgtest_main','-lgtest','-lpthread'] if unit else []),'-o',str(binary)]
else:
    probe=action=='probe_live60'
    for gate in (['build_probe','unit_probe'] if probe else ['unit_memo','sanitizer_memo','build_memo']):
        assert json.loads((ROOT/f'command_{gate}.json').read_text())['passed'],gate
    out=ROOT/action;assert not out.exists(),'Refuse output overwrite'
    partial='partial143' in action
    journal=WS/('gnssfgo_eth1_vw_jacobian_live160_20261011/prepared/full160_on_receiver/events.csv' if partial else
                'gnssfgo_e1_live60_20261011/prepared/live60_on_receiver/events.csv')
    binary=ROOT/'probe/replay' if probe else ROOT/'candidate_build/replay' if action.startswith('candidate') else VW/'replay'
    if action.startswith('candidate'):env['E2_EXACT_LINEARIZATION_CACHE']='1'
    else:env.pop('E2_EXACT_LINEARIZATION_CACHE',None)
    command=[str(binary),'--receiver-model','source_wnoa','--mode','sources',
      '--timeline','record','--solver','isam2','--support-policy','uniform_imu',
      '--support-period-ns','100000000','--receiver-startup-samples','59',
      '--event-stream',str(journal),
      '--out',str(out)]
start=time.monotonic()
with (ROOT/'logs'/f'{action}.log').open('x') as log:
    try:
        code=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=1800).returncode
        if code==0 and action.startswith(('unit_','sanitizer_')):
            code=subprocess.run([str(binary),'--gtest_output=xml:'+str(ROOT/(action+'.xml'))],env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180).returncode
    except subprocess.TimeoutExpired:code=124
usage=resource.getrusage(resource.RUSAGE_CHILDREN)
result=dict(passed=code==0,returncode=code,seconds=time.monotonic()-start,command=command,
    child_max_rss_kib=usage.ru_maxrss,child_user_seconds=usage.ru_utime,child_system_seconds=usage.ru_stime,
    environment={k:v for k,v in env.items() if k.startswith(('E2_','WNOJ_','OMP_','LD_LIBRARY','ASAN_','UBSAN_'))})
with record.open('x') as f:json.dump(result,f,indent=2)
print(action,code,result['seconds'],flush=True)
if code:print((ROOT/'logs'/f'{action}.log').read_text()[-6000:])
raise SystemExit(code)
