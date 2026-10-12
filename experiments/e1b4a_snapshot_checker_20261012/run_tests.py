"""One test invocation per fresh output name, with loader and integrity evidence."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess
import time
import xml.etree.ElementTree as ET

WS=Path('/workspace/cbs_gtsam4.3')
ROOT=WS/'runs/e1b4a_snapshot_checker_20261012'
def main():
    p=argparse.ArgumentParser();p.add_argument('build');p.add_argument('name')
    p.add_argument('--filter',default='*');a=p.parse_args()
    out=ROOT/a.build/a.name;out.mkdir(exist_ok=False)
    env=os.environ.copy()
    env['LD_LIBRARY_PATH']=':'.join(str(x) for x in [
        WS/'runs/e1b4a_local_update_batching_20261012/release_2',
        WS/'runs/e1b_local_profile_20261012/release_2',WS/'devel/lib',Path('/usr/local/cuda/lib64')])
    env['ASAN_OPTIONS']='detect_leaks=0:halt_on_error=1'
    env['UBSAN_OPTIONS']='halt_on_error=1:print_stacktrace=1'
    executable=ROOT/a.build/'test_snapshot'
    linked=subprocess.check_output(['ldd',str(executable)],env=env,text=True)
    (out/'ldd.txt').write_text(linked)
    cmd=[str(executable),'--gtest_filter='+a.filter,'--gtest_output=xml:'+str(out/'tests.xml')]
    start=time.monotonic()
    with (out/'test.log').open('x') as f:r=subprocess.run(cmd,env=env,stdout=f,stderr=subprocess.STDOUT)
    result=dict(returncode=r.returncode,seconds=time.monotonic()-start,command=cmd,
                executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest())
    if (out/'tests.xml').exists():result['gtest']=ET.parse(out/'tests.xml').getroot().attrib
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result),flush=True)
    if r.returncode:print((out/'test.log').read_text()[-9000:])
    raise SystemExit(r.returncode)
if __name__=='__main__':main()
