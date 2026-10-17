"""Read-only identity verification and new compact diagnostic receipts."""
import csv,hashlib,json
from pathlib import Path
R=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_retirement_fl4m_20261015')
HERE=Path(__file__).resolve().parent
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(8*1024*1024),b''):h.update(block)
    return h.hexdigest()
def save(path,data):
    with path.open('x') as f:json.dump(data,f,indent=2,allow_nan=False)
protected=json.loads((R/'protected_before.json').read_text())
changed=[p for p,h in protected.items() if not Path(p).exists() or sha(Path(p))!=h]
assert not changed,changed
analysis=json.loads((R/'analysis.json').read_text())
first=analysis['pre_retirement_parity']['first_retirement_update']
native={};objectives={};flags={};validation={};completion={}
for mode in ['remove','retain']:
    d=R/(mode+'_capture')
    validation[mode]=json.loads((d/'validation.json').read_text())
    completion[mode]=json.loads((d/'complete.json').read_text())
    assert validation[mode]['valid'] and not analysis['cases'][mode]['guards']
    echo=list(csv.DictReader((d/'native.log').open()))
    ignored=[x for x in echo if x.get('field.header.seq') is None]
    assert all(x['%time'] in ['WARNING: no messages received and simulated time is active.','Is /clock being published?'] for x in ignored)
    native[mode]=[x for x in echo if x.get('field.header.seq') is not None]
    objectives[mode]=list(csv.reader((d/'objective.csv').open()))
    flags[mode]=json.loads((d/'preflight.json').read_text())['flags']
    assert len(native[mode])==454
assert all(flags['remove'][k]==flags['retain'][k] for k in flags['remove'] if k.startswith('cbs_'))
before=[(a,b) for a,b in zip(native['remove'],native['retain']) if int(a['field.header.seq'])<first]
fields=[k for k in native['remove'][0] if k!='%time']
assert len(before)==28 and all(a[k]==b[k] for a,b in before for k in fields)
assert all(a[2:]==b[2:] for a,b in zip(objectives['remove'][:first],objectives['retain'][:first]))
manifest={str(p.relative_to(R)):sha(p) for p in [
    R/'private_build/libkimera_vio.so',R/'private_build/libcbs.so',R/'test',
    R/'VioBackend.cpp',R/'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h',
    R/'RetirementAudit.h',R/'aligned_inputs.bag',R/'frozen_inputs.json']}
tests=json.loads((R/'test_commands.json').read_text())
tests['compile']=[s for s in tests['compile'] if '/src/cbs/tests/../include' not in s]
save(R/'actual_test_commands.json',tests)
summary=dict(protected_files_unchanged=len(protected),private_manifest=manifest,
    exact_pre_retirement_native_messages_excluding_receipt_time=True,
    exact_pre_retirement_objective_fields=True,pre_retirement_count=28,
    same_cbs_flags=True,validation=validation,completion=completion,
    reference=analysis['reference'],cases={k:{n:v for n,v in c.items() if n not in ['rows','guards']} for k,c in analysis['cases'].items()})
save(R/'final_validation.json',summary)
save(HERE/'receipt.json',summary)
print(json.dumps(dict(protected_files_unchanged=len(protected),pre_retirement_count=28,private_manifest=manifest),indent=2))
