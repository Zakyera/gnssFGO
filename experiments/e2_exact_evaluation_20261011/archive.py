"""Archive small E2 evidence, not journals, binaries or frozen source runs."""
import hashlib,json,shutil
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=Path('/home/yeranis/results/gnssfgo_e2_exact_evaluation_20261011')
for name in ['live60_comparison.json','partial143_comparison.json','live60_repeat_comparison.json','provenance_final.json']:
    assert json.loads((ROOT/name).read_text())['valid'],name
dest=HERE/'evidence';dest.mkdir(exist_ok=False)
files=[p for p in ROOT.iterdir() if p.suffix in ['.json','.xml']]
files+=list((ROOT/'logs').glob('*.log'))
files += [p for directory in ['probe','candidate_build'] for p in (ROOT/directory).iterdir() if p.suffix in ['.h','.cpp']]
for directory in ['probe_live60','reference_live60','candidate_live60','reference_partial143','candidate_partial143',
                  'reference_live60_repeat','candidate_live60_repeat']:
    for name in ['summary.json','publication_convergence.csv','publication_steps.csv','e2_probe.csv','e2_memo.csv']:
        p=ROOT/directory/name
        if p.exists():files.append(p)
manifest={};omitted=[]
for p in sorted(set(files)):
    if p.stat().st_size>2_000_000:
        omitted.append(str(p));continue
    q=dest/p.relative_to(ROOT);q.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(p,q)
    h=hashlib.sha256(p.read_bytes()).hexdigest();assert hashlib.sha256(q.read_bytes()).hexdigest()==h
    manifest[str(q.relative_to(HERE))]=dict(original=str(p),bytes=p.stat().st_size,sha256=h)
with (HERE/'ARCHIVE_MANIFEST.json').open('x') as f:json.dump(dict(files=manifest,omitted=omitted),f,indent=2)
print('archived',len(manifest),'files, bytes',sum(v['bytes'] for v in manifest.values()),'omitted',len(omitted))
