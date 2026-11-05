from pathlib import Path
import hashlib
import json
import math
import statistics
import shutil

HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/native_covariance_cache_cpi3_20261019')

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def stats(xs):
    xs=sorted(xs)
    def pct(p):
        i=(len(xs)-1)*p;a=math.floor(i);b=math.ceil(i)
        return xs[a]*(b-i)+xs[b]*(i-a) if a!=b else xs[a]
    return dict(n=len(xs),mean=statistics.mean(xs),median=statistics.median(xs),p95=pct(.95),max=max(xs))

def main():
    output=HERE/'verification.json';assert not output.exists()
    protected=json.loads((ROOT/'protected.json').read_text())
    assert all(sha(Path(p))==h for p,h in protected.items())
    capture_manifest=ROOT.parent/'gnssfgo_eig2_local_cov60_cpr1_20261019/prepared/protected_manifest.json'
    capture_protected=json.loads(capture_manifest.read_text())
    assert all(sha(Path(p))==h for p,h in capture_protected.items())
    for mode in ['release_v2','san']:
        assert json.loads((ROOT/mode/'passed.json').read_text())['passed']
        for name in ['tests','repeat']:
            assert '[  PASSED  ] 55 tests.' in (ROOT/mode/(name+'.log')).read_text()
    rows=[json.loads(l) for l in (ROOT/'timing.jsonl').read_text().splitlines()]
    summary=rows[-1];assert summary['kind']=='summary'
    measured=[r for r in rows if r['kind']=='batch' and r['pass']>0]
    assert len(measured)==60 and summary['epoch']==783
    result=dict(protected=protected,original_hashes_unchanged=True,
        frozen_capture_protected_count=len(capture_protected),
        frozen_capture_manifest_sha256=sha(capture_manifest),
        summary=summary,all_batches=dict(original_s=stats([r['original_s'] for r in measured]),
        cached_s=stats([r['cached_s'] for r in measured]),
        speed_ratio=stats([r['original_s']/r['cached_s'] for r in measured])),
        filled_window_batches=dict(original_s=stats([r['original_s'] for r in measured if r['pairs']>=45]),
        cached_s=stats([r['cached_s'] for r in measured if r['pairs']>=45])),
        private_provider_sha256=sha(ROOT/'release_v2/libgtsam_points.so.1.2.1'),
        correctness='55 Release and 55 focused ASan/UBSan; each repeated',
        scope='Gaussian queries only; no live throughput certification')
    output.write_text(json.dumps(result,indent=2)+'\n')
    evidence=HERE/'evidence';evidence.mkdir()
    for rel in ['timing.jsonl','timing_receipt.json','NativeGaussianHistory.cpp.patch',
                'NativeGaussianHistory.h.patch','release/cache_tests.log',
                'release_v2/commands.json','release_v2/tests.log','release_v2/repeat.log',
                'san/commands.json','san/tests.log','san/repeat.log']:
        target=evidence/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/rel,target)
    print(json.dumps({k:v for k,v in result.items() if k!='protected'},indent=2))

if __name__=='__main__':main()
