"""Close out this saved-data report; never invoke an estimator or container."""
import hashlib
import json
from pathlib import Path
import numpy as np
import analyze as v

H=v.H;O=v.OUT
dest=H/'closeout.json'
assert not dest.exists(),'Refuse closeout overwrite'
summary=json.loads((O/'summary.json').read_text())
checks=json.loads((O/'report_checks.json').read_text())
assert checks['passed'] and checks['projection_design_check']['passed']
hashes={}
for obj in [summary,checks]+[x[k] for x in summary['cases'].values() for k in ('trajectories','peers')]:
    for p,sha in obj['input_sha256'].items():
        assert p not in hashes or hashes[p]==sha,(p,'inconsistent generation hashes')
        hashes[p]=sha
script=str(H/'report.py');old=hashes.pop(script)
current=(H/'report.py').read_text()
new="All Kimera and ONLINE fusion ATE direction conclusions survive matched\ncoverage. GLIM Hesai SPX-3 is effectively unchanged: its tiny full-saved gain\nreverses to a tiny common-mask increase, so it is NOT counted as a robust gain.\nAbsolute full-saved"
oldtext='All ATE direction conclusions survive matched coverage. Absolute full-saved'
assert current.count(new)==1
original=current.replace(new,oldtext)
newtitle='Incoming GLIM can be more accurate in one direction and less in another'
oldtitle='The same incoming factor can help one direction and hurt another'
assert original.count(newtitle)==1
original=original.replace(newtitle,oldtitle)
assert hashlib.sha256(original.encode()).hexdigest()==old,'Only the two documented editorial edits are allowed'
for p,sha in hashes.items():assert v.a.g.sha(Path(p))==sha,p
report=(H/'REPORT.txt').read_text()
assert new in report and oldtext not in report
assert (H/'PROPOSAL.txt').read_text().strip() in report
assert len(summary['cases'])==7
for case in summary['cases']:
    z=np.load(O/(case+'_series.npz'))
    assert np.all(np.diff(z['timestamp_ns'])==100_000_000)
    for field in z.files:
        if field.endswith('_deg') or field.endswith('_error'):assert np.isfinite(z[field]).all()
for name in ('01_decision_scorecard.png','02_kimera_rotation_horizons.png','03_peer_directional_tradeoff.png','04_velodyne_orientation.png'):
    assert (O/name).stat().st_size>10_000
artifacts={str(p.relative_to(H)):dict(bytes=p.stat().st_size,sha256=v.a.g.sha(p)) for p in H.rglob('*') if p.is_file()}
value=dict(passed=True,task='CP-V8',scope='Saved-data decision report only',
           completed_baseline_pairs=7,actual_verified_G2K_factors=10609,
           all_original_analysis_and_experiment_inputs_unchanged=True,
           unchanged_input_files=len(hashes),input_sha256=hashes,
           reviewed_report_generator_edit=dict(before_sha256=old,after_sha256=v.a.g.sha(Path(script)),
                                               exact_two_edits_reconstruct_original=True,evidence='EDITORIAL_REVIEW.txt'),
           figures_visually_reviewed=4,no_capture_replay_container_build_or_settings_change=True,
           output_bytes=sum(x['bytes'] for x in artifacts.values()),artifacts=artifacts,
           proposal_only=True,baseline_preserved=True,candidate_not_implemented=True)
v.a.save(dest,value)
print(json.dumps({k:x for k,x in value.items() if k not in ('input_sha256','artifacts')},indent=2))
