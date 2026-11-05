"""Inspect already-written diagnostic factor rows; no additional solves."""
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path
H = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
D = W/'runs'/H.name/'attempt_compile1/results'
pins = {}
def pin(p):
    with p.open('rb') as f:
        pins[str(p)] = hashlib.file_digest(f, 'sha256').hexdigest()
    return p
names = ['VALID','DEGENERATE','BEHIND_CAMERA','OUTLIER','FAR_POINT']
source = pin(W/'devel/include/gtsam/geometry/triangulation.h').read_text()
assert 'enum Status { VALID, DEGENERATE, BEHIND_CAMERA, OUTLIER, FAR_POINT };' in source
data = {}
for label in ('initial','A_ordinary','A_refresh','B_ordinary','B_refresh','OFF_refresh'):
    with pin(D/(label+'_factors.csv')).open() as f:
        data[label] = {r['slot']:r for r in csv.DictReader(f) if r['type']=='smart'}
assert all(len(rows)==338 for rows in data.values())
counts = {label:dict(Counter(names[int(r['status'])] for r in rows.values())) for label,rows in data.items()}
changes = []
for slot, b in data['B_refresh'].items():
    a = data['A_refresh'][slot]
    if a['status']!=b['status']:
        changes.append(dict(slot=int(slot), B_status=names[int(b['status'])], A_status=names[int(a['status'])], keys=a['keys']))
transitions = dict(Counter(r['B_status']+' -> '+r['A_status'] for r in changes))
out = dict(counts=counts, B_to_A_final_transitions=transitions, changed_slots=changes,
           input_sha256=pins,
           scope='Statuses after native error evaluation on independent copies of the same saved smart graph at each state.',
           limitations='Not a count of frontend feature loss, removed factors or exact live solver active rows; no follow-on causal ablation.')
with (H/'visual_support.json').open('x') as f:
    json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k not in ('input_sha256','changed_slots')},indent=2))
