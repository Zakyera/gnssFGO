"""Supplemental coverage accounting only; no scores or trajectories changed."""
import csv
import datetime
import hashlib
import json
from decimal import Decimal
from pathlib import Path
import numpy as np

H = Path(__file__).resolve().parent
E = H.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')
P = W/'eth1_prior_fix_live_cpv103_20261103/prepared'
S = P/'full_on_source'
read = lambda p: json.loads(Path(p).read_text())
epoch = read(P/'input_audit.json')['epoch_ns']
new = np.load(H/'evaluation/comparison.npz')
old = np.load(E/'eth1_split_imu_live_pair_cpv95_20261103_continuation1/evaluation/comparison.npz')
nt = new['timestamp_ns'][new['common_valid']]
ot = old['timestamp_ns'][old['common_valid']]
lost = np.setdiff1d(ot, nt)
added = np.setdiff1d(nt, ot)
assert len(ot) == 2969 and len(nt) == 2966 and len(lost) == 3 and len(added) == 0
causes = []
for t in lost:
    ix = np.flatnonzero(new['timestamp_ns'] == t)
    assert len(ix) == 1
    bad = [key.removesuffix('_support') for key in new.files
           if key.endswith('_support') and not bool(new[key][ix[0]])]
    causes.append(dict(timestamp_ns=int(t), relative_s=float((t-epoch)*1e-9), unsupported_curves=bad))
with (S/'glim_odometry.csv').open() as f:
    csvrows = list(csv.DictReader(f))
ct = np.asarray([int(r['field.header.stamp']) for r in csvrows], dtype=np.int64)
dump = [line.split() for line in (S/'odom_imu.txt').read_text().splitlines() if line.strip()]
dt = np.asarray([int(Decimal(row[0])*10**9) for row in dump], dtype=np.int64)
assert len(ct) == 3454 and len(dt) == 3455 and np.array_equal(ct, dt[:-1])
assert read(H/'results.json')['full_saved']['CP103_ON_glim']['rows'] == len(ct)
used = [Path(__file__), H/'results.json', H/'evaluation/comparison.npz',
        E/'eth1_split_imu_live_pair_cpv95_20261103_continuation1/evaluation/comparison.npz',
        S/'glim_odometry.csv', S/'odom_imu.txt', S/'source_coverage.json',
        P/'on/coverage_control.py', P/'on/source_runner_on.sh']
pins = {}
for p in used:
    with p.open('rb') as f:
        pins[str(p)] = hashlib.file_digest(f, 'sha256').hexdigest()
out = dict(passed=True, finished=datetime.datetime.now().astimezone().isoformat(),
           old_CP95_common_rows=len(ot), new_common_rows=len(nt), removed=causes, added=[],
           GLIM_first_published_csv_rows=len(ct), GLIM_native_dump_rows=len(dt),
           CSV_equals_dump_timestamp_prefix=True,
           final_dump_only_timestamp_ns=int(dt[-1]), final_dump_only_relative_s=float((dt[-1]-epoch)*1e-9),
           final_dump_only_step_s=float((dt[-1]-ct[-1])*1e-9),
           note='All 3454 first-published CSV poses were scored. The native shutdown dump has one additional final pose, not captured in the CSV. The dump can also contain revised means; it is not substituted for the established first-published evaluation. No sample was silently dropped by the evaluator. No gap filling or re-scoring.',
           input_sha256=pins)
with (H/'coverage_review.json').open('x') as f:
    json.dump(out, f, indent=2)
print(json.dumps({k:v for k,v in out.items() if k!='input_sha256'}, indent=2))
