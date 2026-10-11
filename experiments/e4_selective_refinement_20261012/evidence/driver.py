#!/usr/bin/env python3
"""Isolated E4 builds/replay; every output is create-once, no automatic retries."""
import hashlib,json,os,resource,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parent
BUILD=ROOT/'build'
OLD=Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011')
RUN=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live160_20261011/prepared')
ENV=dict(LD_LIBRARY_PATH='/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/lib',
         OMP_NUM_THREADS='1',WNOJ_STRICT_PUBLICATION='1',WNOJ_INCREMENTAL_PROFILE='nominal',E2_EXACT_LINEARIZATION_CACHE='1')
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1<<20),b''):h.update(b)
    return h.hexdigest()
def run(tag,cmd,env=None):
    assert not (ROOT/(tag+'.json')).exists(), 'Refuse duplicate command'
    start=time.monotonic()
    with (ROOT/(tag+'.log')).open('x') as log:
        p=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT,env=os.environ|ENV|(env or {}))
    r=dict(command=cmd,returncode=p.returncode,seconds=time.monotonic()-start,
           affinity=sorted(os.sched_getaffinity(0)),max_rss_kib=resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss)
    with (ROOT/(tag+'.json')).open('x') as f:json.dump(r,f,indent=2)
    print(json.dumps(r));return p.returncode
def build(kind,tag=None):
    j={'build':'candidate_build.json','unit':'candidate_unit.json','sanitize':'candidate_sanitize.json'}[kind]
    c=json.loads((OLD/j).read_text())['commands'][0]
    c.insert(1,'-I'+str(BUILD))
    c=[str(BUILD/'DiagnosticReplay.cpp') if s==str(OLD/'candidate/DiagnosticReplay.cpp') else s for s in c]
    out=BUILD/('replay' if kind=='build' else (tag or kind))
    c[c.index('-o')+1]=str(out)
    if kind!='build':c.insert(c.index('-o'),str(BUILD/'test_selective.cpp'))
    assert not out.exists()
    return run(tag or kind,c)
def preserve():
    paths=set(json.loads((RUN/'protected_manifest.json').read_text()))
    paths.update(str(p) for p in (OLD/'candidate').iterdir() if p.is_file())
    paths.add(str(RUN/'live160_on_receiver/events.csv'))
    paths.update(str(p) for p in (RUN/'live160_on_receiver/receiver').iterdir() if p.is_file())
    paths.add('/home/yeranis/results/gnssfgo_publication_cache_20261010/build/FullPublicationRefinement.h')
    hashes={p:sha(p) for p in sorted(paths)}
    with (ROOT/'protected.json').open('x') as f:json.dump(hashes,f,indent=2)
    print('Protected',len(hashes),'files')
def replay(tag):
    assert tag in ['reference','candidate','candidate_repeat']
    out=ROOT/tag;assert not out.exists()
    cmd=[str(BUILD/'replay'),'--receiver-model','source_wnoa','--mode','sources',
      '--timeline','record','--solver','isam2','--support-policy','uniform_imu',
      '--support-period-ns','100000000','--receiver-startup-samples','59',
      '--event-stream',str(RUN/'live160_on_receiver/events.csv'),'--out',str(out)]
    return run(tag,cmd,dict(E4_SELECTIVE_REFINEMENT='0' if tag=='reference' else '1'))
def freeze():
    paths=[p for p in BUILD.rglob('*') if p.is_file() and p.suffix in ['.h','.cpp']]
    paths += [BUILD/'replay',ROOT/'driver.py',ROOT/'analyze.py',
      Path('/home/yeranis/repos/gnssFGO/experiments/e4_selective_refinement_20261012/PROTOCOL.txt')]
    result=dict(files={str(p):sha(p) for p in paths},environment=ENV,
      reference_sha256=sha(OLD/'candidate/replay'),event_sha256=sha(RUN/'live160_on_receiver/events.csv'))
    assert result['reference_sha256']=='9d2ef832941c5ff2592da8428cb80104352590bcfea50c187624aaf4e90d60e0'
    with (ROOT/'frozen_candidate.json').open('x') as f:json.dump(result,f,indent=2)
    print('Frozen candidate and protocol before journal comparison')
if __name__=='__main__':
    action=sys.argv[1]
    if action=='preserve':preserve()
    elif action=='freeze':freeze()
    elif action in ['build','unit','sanitize']:sys.exit(build(action))
    elif action in ['unit_consistent','unit_verified']:sys.exit(build('unit',action))
    elif action in ['test_unit','test_sanitize','test_unit_consistent','test_unit_verified']:
        kind=action.removeprefix('test_')
        sys.exit(run(action,[str(BUILD/kind),'--gtest_output=xml:'+str(ROOT/(action+'.xml'))],
            dict(ASAN_OPTIONS='detect_leaks=0',UBSAN_OPTIONS='halt_on_error=1')))
    else:sys.exit(replay(action))
