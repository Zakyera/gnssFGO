"""Compile new diagnostic units only against frozen E1b.4a libraries."""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import subprocess
import time

WS=Path('/workspace/cbs_gtsam4.3')
ROOT=WS/'runs/e1b4a_snapshot_checker_20261012'
BATCH=WS/'runs/e1b4a_local_update_batching_20261012'
OLD=WS/'runs/e1b_local_profile_20261012'

def main():
    p=argparse.ArgumentParser();p.add_argument('variant',choices=['release','ubsan','asan']);p.add_argument('attempt');a=p.parse_args()
    out=ROOT/(a.variant+'_'+a.attempt);out.mkdir(exist_ok=False)
    shutil.copytree(ROOT/'authoring',out/'source');shutil.copytree(ROOT/'include',out/'include')
    base=json.loads((WS/'runs/glim_sync_compatible_batching_e1_20261011/private/test_attempt3_commands.json').read_text())[0]
    includes=['-I'+str(out/'include'),'-I'+str(out/'source'),'-I'+str(BATCH/'authoring'),
              '-I'+str(OLD),'-I'+str(OLD/'overlay/include')]
    libs=str(BATCH/'release_2')+':'+str(OLD/'release_2')
    extra=[] if a.variant=='release' else ['-O1','-g1','-fno-omit-frame-pointer',
        '-fsanitize='+('undefined' if a.variant=='ubsan' else 'address,undefined'),'-fno-sanitize-recover=all']
    if a.variant=='asan':extra+=['-DEIGEN_MALLOC_ALREADY_ALIGNED=1']
    cmd=[str(out/'source/test_snapshot.cpp') if x.endswith('/cbs_local_graph_linearizer_test.cpp') else x for x in base]
    cmd[cmd.index('-o')+1]=str(out/'test_snapshot')
    cmd[1:1]=includes+['-L'+str(BATCH/'release_2'),'-L'+str(OLD/'release_2')]
    cmd += [str(out/'source/ExactGpuSnapshot.cpp'),'-lcudart','-L/usr/local/cuda/lib64',
            '-ltbb','-le1b_profile','-Wl,-rpath,'+libs]+extra
    begin=time.monotonic()
    (out/'command.json').write_text(json.dumps(cmd,indent=2))
    with (out/'build.log').open('x') as log:r=subprocess.run(cmd,stdout=log,stderr=subprocess.STDOUT)
    result=dict(returncode=r.returncode,seconds=time.monotonic()-begin)
    if not r.returncode:result['sha256']=hashlib.sha256((out/'test_snapshot').read_bytes()).hexdigest()
    (out/'build_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)
    if r.returncode:print((out/'build.log').read_text()[-12000:])
    raise SystemExit(r.returncode)
if __name__=='__main__':main()
