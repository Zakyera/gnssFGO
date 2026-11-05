"""Separate fresh-evidence cycles from sparse-belief tails; no replay."""
import hashlib
import json
from pathlib import Path
import numpy as np

H = Path(__file__).resolve().parent
EPOCH = 1732549034915071665
data = np.load(H/'diagnostic_arrays.npz')
source = H.parent/'first_published_comparison_cpv10_20261026/outputs/velodyne_leica2.npz'
common = np.load(source)
grid = common['timestamp_ns']
valid = common['common_valid']
# CP-V10 comparison arrays use capture-relative integer nanoseconds.
if int(grid[0]) < 1_000_000_000_000_000:
    grid = grid + EPOCH
edges = np.flatnonzero(np.diff(np.r_[False, valid, False]))
segments = [(int(grid[a]), int(grid[b-1])) for a, b in zip(edges[::2], edges[1::2])]
out = {}
for mode in ('off', 'on'):
    t = data[mode+'_timestamp_ns']
    age = data[mode+'_frontier_age_ms']
    right = data[mode+'_latest_endpoint_right']
    k = np.arange(len(t))
    mask = np.zeros(len(t), bool)
    for a, b in segments:
        mask |= (t >= a) & (t <= b)
    fresh = np.all(np.isfinite(age), axis=1) & (age[:, 1] <= 500)
    entry = {'cohorts': {}}
    for name, cohort in [('inside_frozen_common_intervals', mask), ('kimera_endpoint_age_le500ms', fresh), ('first60s', (t-EPOCH < 60_000_000_000) & fresh)]:
        missing = cohort & ~np.all(np.isfinite(age), axis=1)
        cohort = cohort & np.all(np.isfinite(age), axis=1)
        entry['cohorts'][name] = {
            'excluded_initial_source_unseen_publications': int(missing.sum()),
            'publications': int(cohort.sum()),
            'neither_source_directly_touches_newest_percent': float(100*np.mean(~np.any(right[cohort] == k[cohort, None], axis=1))),
            'source_age_ms_median_p95': {s: np.percentile(age[cohort, j], [50, 95]).tolist() for j, s in enumerate(('glim','kimera'))},
        }
    bad = np.isfinite(age[:, 1]) & (age[:, 1] > 500)
    changes = np.flatnonzero(np.diff(np.r_[False, bad, False]))
    entry['kimera_belief_endpoint_age_over500ms_intervals'] = [
        {'first_publication_s': float((t[a]-EPOCH)*1e-9), 'last_publication_s': float((t[b-1]-EPOCH)*1e-9),
         'count': int(b-a), 'max_endpoint_age_ms': float(age[a:b, 1].max())}
        for a, b in zip(changes[::2], changes[1::2])]
    last_endpoint = int(t[-1])-int(round(age[-1, 1]*1e6))
    entry['last_received_usable_kimera_endpoint_s'] = (last_endpoint-EPOCH)*1e-9
    entry['warning'] = 'Native Kimera continues with sparse stationary-tail keyframes; this is relative-belief coverage, not proof of missing camera frames. Inside common intervals selects actual fusion samples, not new common-grid ATE.'
    out[mode] = entry
pins = {}
for p in (Path(__file__), H/'diagnostic_arrays.npz', source):
    with p.open('rb') as f:
        pins[str(p)] = hashlib.file_digest(f, 'sha256').hexdigest()
out['input_pins'] = pins
with (H/'frontier_cohorts_complete.json').open('x') as f:
    json.dump(out, f, indent=2, allow_nan=False)
print(json.dumps({k: v for k,v in out.items() if k != 'input_pins'}, indent=2))
