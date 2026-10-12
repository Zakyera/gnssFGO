"""Standalone timing binaries; all original numerical statements retained."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import subprocess
import time

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
WS=Path('/workspace/cbs_gtsam4.3')

def timed(source):
    s='#include "Timing.h"\n'+source
    def put(a,b):
        nonlocal s
        assert s.count(a)==1,(a,s.count(a))
        s=s.replace(a,b)
    put('  try {\n    require(mode_', '  try {\n    diagnostic_timing::Span timing("stage.");\n    require(mode_')
    put('    std::set<Id> seen;', '    timing.mark("copy_and_prior_checks_s");\n    std::set<Id> seen;')
    put('    auto incident = committed_->prior.clone();','    timing.mark("validate_and_clone_native_s");\n    auto incident = committed_->prior.clone();')
    put('    if (!s.retiring.empty()) {','    timing.mark("partition_incident_s");\n    if (!s.retiring.empty()) {')
    put('    next->active = std::move(active);','    timing.mark("retirement_elimination_s");\n    next->active = std::move(active);')
    put('    pending_ = std::move(next);','    pending_ = std::move(next);\n    timing.mark("finalize_s");')
    put('  require(from != to', '  diagnostic_timing::Span timing("query.");\n  require(from != to')
    put('  auto h = native.hessian().first;', '  timing.mark("clone_graph_s");\n  auto h = native.hessian().first;\n  timing.mark("assemble_dense_hessian_s");')
    put('  positiveDefinite(h);','  positiveDefinite(h);\n  timing.mark("dense_rank_check_s");')
    put('  auto joint = tree->jointMarginalCovariance({from, to});','  timing.mark("sparse_elimination_s");\n  auto joint = tree->jointMarginalCovariance({from, to});\n  timing.mark("joint_marginal_s");')
    put('  positiveDefinite(out.joint_current);','  timing.mark("extract_and_transport_s");\n  positiveDefinite(out.joint_current);\n  timing.mark("endpoint_rank_check_s");')
    put('  out.relative = result.covariance;', '  out.relative = result.covariance;\n  timing.mark("relative_covariance_helper_s");')
    return s

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['release','asan']);a=p.parse_args()
    out=ROOT/a.mode;out.mkdir(exist_ok=False)
    source=(HERE/'NativeGaussianHistory.cpp').read_text()
    # Mechanical timer-only transformation, not an algorithm edit.
    (out/'instrumented.cpp').write_text(timed(source))
    opts=['-std=c++17','-O2','-DNDEBUG','-pthread']
    if a.mode=='asan':opts+=['-O1','-g1','-fno-omit-frame-pointer','-fsanitize=address,undefined','-fno-sanitize-recover=all','-DEIGEN_MALLOC_ALREADY_ALIGNED=1']
    includes=['-I'+str(HERE),'-I'+str(WS/'src/cbs/include'),'-isystem',str(WS/'devel/include/gtsam/3rdparty/Eigen'),'-isystem',str(WS/'devel/include')]
    lib=WS/'runs/e1b4a_local_update_batching_20261012/release_2'
    link=['-L'+str(lib),'-L'+str(WS/'devel/lib'),'-Wl,-rpath,'+str(lib)+':'+str(WS/'devel/lib'),'-lgtsam','-ltbb']
    commands=[];started=time.monotonic()
    def run(cmd):
        t=time.monotonic();r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
        commands.append(dict(command=cmd,seconds=time.monotonic()-t,returncode=r.returncode))
        (out/'commands.json').write_text(json.dumps(commands,indent=2));r.check_returncode()
    with (out/'build.log').open('x') as log:
        for name,src in [('original',HERE/'NativeGaussianHistory.cpp'),('instrumented',out/'instrumented.cpp'),('replay',HERE/'replay.cpp'),('tests',HERE/'test_native_gaussian.cpp')]:
            run(['c++',*opts,*includes,'-c',str(src),'-o',str(out/(name+'.o'))])
        for variant in ['original','instrumented']:
            run(['c++',*opts,str(out/(variant+'.o')),str(out/'replay.o'),'-o',str(out/variant),*link])
        run(['c++',*opts,str(out/'instrumented.o'),str(out/'tests.o'),'-o',str(out/'tests'),*link,'-lgtest_main','-lgtest'])
    summary=dict(passed=True,seconds=time.monotonic()-started,binaries={n:hashlib.sha256((out/n).read_bytes()).hexdigest() for n in ['original','instrumented','tests']})
    (out/'build_result.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
if __name__=='__main__':main()
