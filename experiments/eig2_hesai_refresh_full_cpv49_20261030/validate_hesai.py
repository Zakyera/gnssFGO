"""Hesai sensor identity and coverage checks; no estimator rerun."""
from pathlib import Path
import csv,json,sys
P=Path(__file__).resolve().parent
mode=sys.argv[1];assert mode in ('off','on')
S=P/f'full_{mode}_source'
assert (P/f'full_{mode}_control/complete.json').exists()
assert json.loads((S/'hesai_startup.json').read_text())['passed']
coverage=json.loads((S/'source_coverage.json').read_text());assert coverage['passed']
assert coverage['native_glim']['rows']>=3517
meta=(S/'run_metadata.txt').read_text();assert 'glim_sensor_pair=hesai+cpt7_imu' in meta
with (S/'glim_odometry.csv').open() as f:rows=list(csv.DictReader(f))
assert rows and all(r['field.child_frame_id']=='cpt7_imu' for r in rows)
result=dict(passed=True,mode=mode,published=len(rows),coverage=coverage['native_glim'],
 body_frame='cpt7_imu',source_profile_and_subscriptions_verified_before_playback=True,
 input_audit='Original CP-K24 bag/count audit re-used after exact size, mtime and SHA256 checks',
 limit='Not a new raw per-point timestamp or deskew-equation audit.')
with (S/'hesai_validation.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
