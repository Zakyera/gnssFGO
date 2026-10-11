"""Use the validated observing build; replace ONLY its observing wrapper."""
import hashlib,json,shutil
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_e2_exact_evaluation_20261011')
assert json.loads((ROOT/'probe_analysis.json').read_text())['worthwhile_gate_passed']
dest=ROOT/'candidate_build';dest.mkdir(exist_ok=False)
for p in (ROOT/'probe').iterdir():
    if p.suffix in ['.h','.cpp']:shutil.copy2(p,dest/p.name)
shutil.copy2(HERE/'ExactLinearizationMemo.h',dest/'ExactLinearizationMemo.h')
p=dest/'DiagnosticReplay.cpp';s=p.read_text()
for old,new,n in [('#include "ExactLinearizationProbe.h"','#include "ExactLinearizationMemo.h"',1),
                   ('e2_probe::ObservedFactor','e2_memo::MemoFactor',2),
                   ('e2_probe::Flush','e2_memo::Flush',1)]:
    assert s.count(old)==n,(old,s.count(old));s=s.replace(old,new)
p.write_text(s)
with (ROOT/'candidate_sources.json').open('x') as f:
    json.dump({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in dest.iterdir()},f,indent=2)
print('Prepared opt-in E2 candidate; no original modified')
