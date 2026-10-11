"""Create private fixtures, hash protected originals; never overwrite a run."""
import hashlib,json,shutil
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011')
OLD=ROOT.parent/'gnssfgo_e2_exact_evaluation_20261011'
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
GPU=WS/'runs/e1_correctness_audit_20261011'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
protected=json.loads((OLD/'protected.json').read_text())
for p,h in protected.items():assert sha(p)==h,p
previous=json.loads((OLD/'provenance_final.json').read_text())
for group in ['private','authored']:
    for p,h in previous[group].items():
        assert sha(p)==h,p
        protected[p]=h
for folder in [WS/'runs/glim_sync_compatible_batching_e1_20261011',WS/'src/gtsam_points',WS/'src/glim_ros1']:
    # Protect candidate headers, tests and binaries used by E1, not huge logs.
    for p in folder.rglob('*'):
        if p.is_file() and (p.suffix in ['.cpp','.hpp','.h'] or '.so' in p.name or
                p.name in ['test_attempt3','ubsan_attempt1','sanitize_attempt1']):protected[str(p)]=sha(p)
ROOT.mkdir(exist_ok=False);GPU.mkdir(exist_ok=False)
(ROOT/'logs').mkdir()
with (ROOT/'protected.json').open('x') as f:json.dump(protected,f,indent=2)
for variant in ['original','candidate']:
    dest=ROOT/variant;dest.mkdir()
    for p in (OLD/'candidate_build').iterdir():
        if p.suffix in ['.h','.cpp']:shutil.copy2(p,dest/p.name)
    shutil.copy2(HERE.parent/'e2_exact_evaluation_20261011/test_memo.cpp',dest/'test_memo.cpp')
    shutil.copy2(HERE/'test_e2_audit.cpp',dest/'test_e2_audit.cpp')
    if variant=='candidate':shutil.copy2(HERE/'ExactLinearizationMemo.h',dest/'ExactLinearizationMemo.h')
for name in ['test_e1_audit.cpp','run_e1.py']:
    shutil.copy2(HERE/name,GPU/name)
shutil.copy2(HERE/'PROTOCOL.txt',ROOT/'PROTOCOL.txt')
shutil.copy2(HERE/'PROTOCOL.txt',GPU/'PROTOCOL.txt')
print('Protected',len(protected),'files; fresh CPU and GPU fixture roots created.')
