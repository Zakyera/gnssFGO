"""Mechanical private build instrumentation. Originals are copied, never edited."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
ROOT = Path('/home/yeranis/results/gnssfgo_e2_exact_evaluation_20261011')
VW = Path('/home/yeranis/results/gnssfgo_vw_jacobian_20261011/build')
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
ROOT.mkdir(exist_ok=False)
(ROOT/'probe').mkdir();(ROOT/'logs').mkdir()
protected=json.loads((WS/'gnssfgo_e1_live60_20261011/prepared/protected_manifest.json').read_text())
for p,h in protected.items():assert sha(p)==h,p
assert sha(VW/'replay')=='62124b496ad6f854079218862acb149892aa452d4af908c50bc96067e49c9dbb'
for p in VW.iterdir():
    if p.suffix in ('.h','.cpp'):
        protected[str(p)]=sha(p)
        shutil.copy2(p,ROOT/'probe'/p.name)
for p in (Path('/home/yeranis/results/gnssfgo_publication_cache_20261010/build')).glob('*.h'):
    protected[str(p)]=sha(p)
for case,rel in [('live60','gnssfgo_e1_live60_20261011/prepared/live60_on_receiver'),
                 ('partial143','gnssfgo_eth1_vw_jacobian_live160_20261011/prepared/full160_on_receiver')]:
    p=WS/rel
    protected[str(p/'events.csv')]=sha(p/'events.csv')
    for file in (p/'receiver').iterdir():
        if file.is_file():protected[str(file)]=sha(file)
shutil.copy2(HERE/'ExactLinearizationProbe.h',ROOT/'probe/ExactLinearizationProbe.h')
p=ROOT/'probe/SourceWnoaModel.h'
s=p.read_text();assert s.count('class WnoaRelativeFactor final :')==1
p.write_text(s.replace('class WnoaRelativeFactor final :','class WnoaRelativeFactor :'))
p=ROOT/'probe/DiagnosticReplay.cpp';s=p.read_text()
edits={
 '#include "FailureProbe.h"':'#include "FailureProbe.h"\n#include "ExactLinearizationProbe.h"',
 'emplace_shared<fgo::offline::WnoaRelativeFactor>':'emplace_shared<e2_probe::ObservedFactor<fgo::offline::WnoaRelativeFactor,1>>',
 'emplace_shared<fgo::factor::GPWNOAPrior>':'emplace_shared<e2_probe::ObservedFactor<fgo::factor::GPWNOAPrior,0>>',
 '    const auto result=numerical_audit::Refine(isam2_);':'    const auto result=numerical_audit::Refine(isam2_);\n    e2_probe::Flush(out,index,physical,phase);'}
for old,new in edits.items():
    assert s.count(old)==1,(old,s.count(old));s=s.replace(old,new)
p.write_text(s)
with (ROOT/'protected.json').open('x') as f:json.dump(protected,f,indent=2)
with (ROOT/'preparation.json').open('x') as f:
    json.dump(dict(protected=len(protected),repo_parent=subprocess.check_output(
      ['git','-C',str(HERE.parents[1]),'rev-parse','HEAD'],text=True).strip(),
      probe={str(p):sha(p) for p in (ROOT/'probe').iterdir()},
      protocol_sha256=sha(HERE/'PROTOCOL.txt')),f,indent=2)
print('Prepared observing-only E2:',len(protected),'protected hashes')
