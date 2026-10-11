"""Preserve exact inputs, private build and baseline. Read-only except new record."""
import hashlib,json,sys
from pathlib import Path
ROOT=Path('/home/yeranis/results/gnssfgo_e2_exact_evaluation_20261011')
HERE=Path(__file__).resolve().parent
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
label=sys.argv[1];assert label in ['pre_replay','final']
expected=json.loads((ROOT/'protected.json').read_text())
for p,h in expected.items():assert sha(p)==h,p
fixtures=json.loads((ROOT/'fixture_mapping.json').read_text())
for p,v in fixtures.items():assert sha(p)==v['sha256'],p
private={str(p):sha(p) for p in (ROOT/'candidate_build').iterdir() if p.is_file()}
authored={str(p):sha(p) for p in HERE.iterdir() if p.is_file() and p.suffix in ['.h','.cpp','.py']}
if label=='final':
    old=json.loads((ROOT/'provenance_pre_replay.json').read_text())
    assert old['private']==private,'Private executable/source changed during replay'
payload=dict(valid=True,protected_files=len(expected),private=private,authored=authored,fixtures=fixtures)
if label=='final':
    # Retain identities even for large numerical products omitted from Git.
    payload['replay_products']={}
    for name in ['probe_live60','reference_live60','candidate_live60',
                 'reference_partial143','candidate_partial143',
                 'reference_live60_repeat','candidate_live60_repeat']:
        assert json.loads((ROOT/('command_'+name+'.json')).read_text())['passed'],name
        payload['replay_products'][name]={p.name:sha(p) for p in (ROOT/name).iterdir() if p.is_file()}
with (ROOT/f'provenance_{label}.json').open('x') as f:json.dump(payload,f,indent=2)
print(label,'protected',len(expected),'private',len(private),'all exact')
