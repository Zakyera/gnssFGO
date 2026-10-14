"""Bounded numerical cross-check, no optimization and no replay."""
import fcntl,json,os,subprocess,time
from driver import HERE,ROOT,RT,A,D,sha,protect
with (ROOT/'controller.lock').open('a') as lock:
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    saved=json.loads((ROOT/'protected.json').read_text());assert protect()==saved
    with (ROOT/'crosscheck.started').open('x') as f:f.write(str(os.getpid()))
    cmd=json.loads((A/'build_snapshot.json').read_text())['command']
    cmd=[str(HERE/'GradientCurvature.cpp') if x.endswith('/SnapshotAudit.cpp') else x for x in cmd]
    cmd[-1]=str(ROOT/'gradient_curvature')
    env={k:v for k,v in os.environ.items() if not k.startswith(('WNOJ_','E2_','E2B_','S5B_','S5C_','E4_','CACHE_FAILURE')) and k!='LD_PRELOAD'}
    env.update(LD_LIBRARY_PATH=str(RT/'lib'),OMP_NUM_THREADS='1',E2B_ENDPOINT_CACHE='0',E2_EXACT_LINEARIZATION_CACHE='0')
    os.sched_setaffinity(0,{2});results=[]
    for name,command in [('build',cmd),('run',[str(ROOT/'gradient_curvature'),str(A/'diagnostics/snapshot'),str(D/'diagnostics/last_accepted.values'),str(ROOT/'diagnostics/mode'),str(ROOT/'diagnostics/gradient_curvature.csv')])]:
        start=time.monotonic()
        with (ROOT/('crosscheck_'+name+'.log')).open('x') as log:rc=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=180).returncode
        results.append(dict(phase=name,returncode=rc,seconds=time.monotonic()-start,command=command))
        if rc:break
    result=dict(phases=results,protected_unchanged=protect()==saved,code={p.name:sha(p) for p in HERE.iterdir() if p.is_file()})
    with (ROOT/'crosscheck.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result['phases']));assert len(results)==2 and all(x['returncode']==0 for x in results)
