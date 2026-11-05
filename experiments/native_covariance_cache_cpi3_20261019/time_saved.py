"""Run one bounded saved-equation benchmark after both correctness suites pass."""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import time

ROOT=Path('/workspace/cbs_gtsam4.3/runs/native_covariance_cache_cpi3_20261019')
WS=Path('/workspace/cbs_gtsam4.3')
JOURNAL=WS/'runs/gnssfgo_shared_linearization_s3_20261012/prepared/profile40_on_source/native_gaussian.bin'
BASE=WS/'runs/shared_linearization_s4_provider_20261012/release_2'

def main():
    assert (ROOT/'release_v2/passed.json').exists()
    assert (ROOT/'san/passed.json').exists()
    expected='25ab6edd4d33e55c6993abbaf63982b9711e2f634709d02a2b834460e258de1f'
    assert hashlib.sha256(JOURNAL.read_bytes()).hexdigest()==expected
    output=ROOT/'timing.jsonl';assert not output.exists()
    env=os.environ.copy()
    env['LD_LIBRARY_PATH']=':'.join(map(str,[ROOT/'release_v2',BASE,
        WS/'runs/e1b4a_local_update_batching_20261012/release_2',
        WS/'runs/e1b_local_profile_20261012/release_2',WS/'devel/lib',WS/'devel',
        '/opt/ros/noetic/lib','/usr/local/cuda/lib64']))
    env['OMP_NUM_THREADS']='1'
    command=[str(ROOT/'release_v2/benchmark'),str(JOURNAL)]
    started=time.monotonic()
    with output.open('x') as f:
        result=subprocess.call(command,env=env,stdout=f,stderr=subprocess.STDOUT)
    (ROOT/'timing_receipt.json').write_text(json.dumps(dict(command=command,
        returncode=result,wall_s=time.monotonic()-started,journal_sha256=expected,
        scope='saved ETH1 native Gaussian snapshots; multiple-query workload, no estimators'),indent=2)+'\n')
    assert result==0
    print('saved-equation benchmark passed')

if __name__=='__main__':main()
