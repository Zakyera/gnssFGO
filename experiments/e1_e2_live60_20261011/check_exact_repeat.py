"""Record strict equality additionally to the frozen numerical replay gate."""
import csv
import hashlib
import json
from pathlib import Path
RUN=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live60_20261011')
A=RUN/'prepared/live60_on_receiver/receiver'
B=RUN/'prepared/live60_on_numerical_repeat'
def payload(p):
    omit={'publication_convergence.csv':{'seconds'},'revision_events.csv':{'update_seconds'},
          'stream_progress.csv':{'wall_unix_ns'}}
    if p.name in omit:
        with p.open() as f:return [{k:v for k,v in r.items() if k not in omit[p.name]} for r in csv.DictReader(f)]
    if p.name=='summary.json':return {k:v for k,v in json.loads(p.read_text()).items() if k not in ['wall_seconds','revision_update_seconds']}
    return hashlib.sha256(p.read_bytes()).hexdigest()
exact={p.name:payload(p)==payload(B/p.name) for p in A.iterdir() if p.is_file()}
with (RUN/'repeat_exact_comparison.json').open('x') as f:json.dump(dict(all_exact=all(exact.values()),products=exact),f,indent=2)
print(json.dumps(exact,indent=2))
