"""Private overlays only. Existing sources, objects and installed DSOs are read-only."""
from pathlib import Path
import argparse
import hashlib
import json
import shlex
import shutil
import subprocess
import time

WS=Path('/workspace/cbs_gtsam4.3')
ROOT=WS/'runs/shared_linearization_s3_20261012'
B0=WS/'runs/e1b4a_local_update_batching_20261012/release_2'
OLD=WS/'runs/e1b_local_profile_20261012'

def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def flags(p):
    lines=p.read_text().splitlines()
    return sum([shlex.split(next(s.split(' = ',1)[1] for s in lines if s.startswith(k+' = ')))
                for k in ['CXX_DEFINES','CXX_INCLUDES','CXX_FLAGS']],[])
def replace(text,old,new):
    assert text.count(old)==1,(old,text.count(old))
    return text.replace(old,new)

def main():
    p=argparse.ArgumentParser();p.add_argument('mode',choices=['release','ubsan','asan'])
    p.add_argument('attempt');p.add_argument('--bridge',action='store_true');a=p.parse_args()
    out=ROOT/(a.mode+'_'+a.attempt);out.mkdir(exist_ok=False)
    src=out/'source';shutil.copytree(ROOT/'authoring',src)
    # S2 is inherited without edits; the authoring snapshot includes these files.
    overlay=out/'overlay';overlay.mkdir()
    native=WS/'src/gtsam_points/src/gtsam_points/optimizers'
    inner=(native/'incremental_fixed_lag_smoother_ext.cpp').read_text()
    inner='#include "ShadowObserver.h"\n'+inner
    line='  isamResult_ = isam_.update(newFactors, newTheta, factorsToRemove, constrainedKeys, {}, additionalMarkedKeys);'
    inner=replace(inner,line,line+'\n  shared_native::beforeRetire(isam_, isamResult_, marginalizableKeys);')
    inner=replace(inner,'  // Remove marginalized keys from the KeyTimestampMap',
                  '  shared_native::afterRetire(isam_);\n\n  // Remove marginalized keys from the KeyTimestampMap')
    (overlay/'incremental_fixed_lag_smoother_ext.cpp').write_text(inner)
    wrapper=(native/'incremental_fixed_lag_smoother_ext_with_fallback.cpp').read_text()
    wrapper='#include "ShadowObserver.h"\n'+wrapper
    wrapper=replace(wrapper,'~IncrementalFixedLagSmootherExtWithFallback() {}',
                    '~IncrementalFixedLagSmootherExtWithFallback() { shared_native::detach(this); }')
    wrapper=replace(wrapper,'  //\n  fallback_happend = false;',
                    '  shared_native::UpdateScope shadow(this, newFactors, factorsToRemove);\n  //\n  fallback_happend = false;')
    wrapper=replace(wrapper,'  update_fallback_state();\n\n  return result;',
                    '  update_fallback_state();\n  shadow.commit();\n\n  return result;')
    line='void IncrementalFixedLagSmootherExtWithFallback::fallback_smoother() const {'
    wrapper=replace(wrapper,line,line+'\n  shared_native::reset(this);')
    (overlay/'incremental_fixed_lag_smoother_ext_with_fallback.cpp').write_text(wrapper)
    bridge=(B0/'libraries_source_snapshot/overlay/cbs_bridge.cpp').read_text()
    bridge='#include "ShadowObserver.h"\n'+bridge
    line='  updateSidecar(new_factors, new_values, new_stamps);'
    bridge=replace(bridge,line,
      '  shared_native::admitFromEnvironment(&smoother, smoother.getLinearizationPoint().empty(), new_factors);\n'+line)
    line='      const gtsam::Pose3 measured_from_to = from_pose.between(to_pose);'
    bridge=replace(bridge,line,line+'\n      shared_native::queryForLog(&smoother, from_key, to_key, from_pose, to_pose);')
    (overlay/'cbs_bridge.cpp').write_text(bridge)
    options=['-std=c++17','-O2','-DNDEBUG','-fPIC','-pthread']
    if a.mode!='release': options+=['-O1','-g1','-fno-omit-frame-pointer',
        '-fsanitize='+('undefined' if a.mode=='ubsan' else 'address,undefined'),'-fno-sanitize-recover=all']
    if a.mode=='asan':options+=['-DEIGEN_MALLOC_ALREADY_ALIGNED=1']
    inc=['-I'+str(src),'-I'+str(WS/'src/gtsam_points/include'),'-I'+str(WS/'src/cbs/include'),
         '-isystem',str(WS/'devel/include/gtsam/3rdparty/Eigen'),'-isystem',str(WS/'devel/include')]
    commands=[];protected={};start=time.monotonic()
    def run(cmd,cwd,label):
        for arg in cmd:
            q=Path(arg) if arg.startswith('/') else cwd/arg
            if q.is_file() and (arg.endswith('.o') or '.so' in q.name) and out not in q.parents:protected[str(q)]=sha(q)
        begin=time.monotonic()
        with (out/(label+'.log')).open('x') as log:
            result=subprocess.run(cmd,cwd=cwd,stdout=log,stderr=subprocess.STDOUT)
        commands.append(dict(command=cmd,cwd=str(cwd),returncode=result.returncode,seconds=time.monotonic()-begin))
        (out/'commands.json').write_text(json.dumps(commands,indent=2))
        (out/'protected.json').write_text(json.dumps(protected,indent=2))
        print(label,result.returncode,round(commands[-1]['seconds'],2),flush=True)
        if result.returncode:print((out/(label+'.log')).read_text()[-8000:]);raise SystemExit(result.returncode)
    replacements={}
    for name in ['NativeGaussianHistory','ShadowObserver','incremental_fixed_lag_smoother_ext',
                 'incremental_fixed_lag_smoother_ext_with_fallback']:
        path=(overlay if name.startswith('incremental_') else src)/(name+'.cpp')
        run(['c++',*inc,*options,'-c',str(path),'-o',str(out/(name+'.o'))],out,name)
        if name.startswith('incremental_'):
            replacements['CMakeFiles/gtsam_points.dir/src/gtsam_points/optimizers/'+name+'.cpp.o']=str(out/(name+'.o'))
    cwd=WS/'build/gtsam_points'
    cmd=[replacements.get(x,x) for x in shlex.split((cwd/'CMakeFiles/gtsam_points.dir/link.txt').read_text())]
    cmd[cmd.index('-o')+1]=str(out/'libgtsam_points.so.1.2.1')
    cmd += [str(out/(name+'.o')) for name in ['NativeGaussianHistory','ShadowObserver']]+options
    run(cmd,cwd,'gtsam_points_link')
    for name in ['libgtsam_points.so.1','libgtsam_points.so']:(out/name).symlink_to('libgtsam_points.so.1.2.1')
    if a.bridge:
        records=json.loads((B0/'libraries_commands.json').read_text())
        record=next(r for r in records if str(B0/'cbs_bridge.cpp.o') in r['command'] and '-c' in r['command'])
        cmd=[str(overlay/'cbs_bridge.cpp') if x.endswith('/overlay/cbs_bridge.cpp') else
             str(out/'cbs_bridge.cpp.o') if x==str(B0/'cbs_bridge.cpp.o') else x for x in record['command']]
        cmd[1:1]=inc;cmd+=options
        run(cmd,Path(record['cwd']),'bridge_compile')
        record=next(r for r in records if '-o' in r['command'] and str(B0/'libglim_ros.so') in r['command'])
        cmd=[str(out/'cbs_bridge.cpp.o') if x==str(B0/'cbs_bridge.cpp.o') else x for x in record['command']]
        cmd[cmd.index('-o')+1]=str(out/'libglim_ros.so');cmd+=['-L'+str(out),'-lgtsam_points']+options
        run(cmd,Path(record['cwd']),'bridge_link')
    for name in ['test_lifecycle','test_native_gaussian']:
        run(['c++',*inc,*options,'-c',str(src/(name+'.cpp')),'-o',str(out/(name+'.o'))],out,name)
    exe=out/'test_shadow'
    cmd=['c++',*options,str(out/'test_lifecycle.o'),str(out/'test_native_gaussian.o'),'-o',str(exe),
         '-L'+str(out),'-L'+str(B0),'-L'+str(OLD/'release_2'),'-L'+str(WS/'devel/lib'),
         '-lgtsam_points','-lgtsam','-lgtsam_points_cuda','-le1b_profile','-lgtest_main','-lgtest','-ltbb',
         '-L/usr/local/cuda/lib64','-lcudart','-fopenmp']
    run(cmd,out,'tests_link')
    cmd=['c++',*inc,*options,str(src/'control.cpp'),'-o',str(out/'control'),
         '-L'+str(out),'-L'+str(B0),'-L'+str(OLD/'release_2'),'-L'+str(WS/'devel/lib'),
         '-lgtsam_points','-lgtsam','-lgtsam_points_cuda','-le1b_profile','-ltbb',
         '-L/usr/local/cuda/lib64','-lcudart','-fopenmp']
    run(cmd,out,'control_link')
    assert all(sha(p)==h for p,h in protected.items())
    (out/'build_result.json').write_text(json.dumps(dict(seconds=time.monotonic()-start,protected_unchanged=len(protected),
         artifacts={p.name:sha(p) for p in out.iterdir() if p.is_file() and ('.so' in p.name or p==exe)}),indent=2))
if __name__=='__main__':main()
