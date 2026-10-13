"""Locate the violating coordinates at every captured pass; read raw logs only."""
import csv
import json
from pathlib import Path

ROOT = Path('/home/yeranis/results/gnssfgo_publication_s5d_20261013')
assert json.loads((ROOT / 'full_on.json').read_text())['passed']
analysis = json.loads((ROOT / 'analysis.json').read_text())
records = []
for snapshot in analysis['snapshots']:
    index = snapshot['index']
    grouped = {}
    with (ROOT / 'full_on' / ('s5d_support_' + str(index)) / 'gradients.csv').open() as f:
        for row in csv.DictReader(f):
            frame = int(row['frame'])
            key = row['key']
            coord = int(row['coordinate'])
            block = ('rotation' if coord < 3 else 'translation') if key[0] == 'x' else ('velocity' if key[0] == 'v' else 'omega')
            grouped.setdefault(frame, []).append(dict(key=key, coordinate=coord,
                block=block, value=float(row['normalized']), age_s=(index-int(key[1:]))*.1))
    for frame, values in sorted(grouped.items()):
        top = max(values, key=lambda r: r['value'])
        meta = snapshot['frames'][frame]
        assert top['value'] == meta['gradient']
        violating = [r for r in values if r['value'] > 1e-5]
        block_counts = {b: sum(r['block'] == b for r in violating)
                        for b in ['rotation', 'translation', 'velocity', 'omega']}
        records.append(dict(index=index, frame=frame, kind=meta['kind'], accepted=meta['accepted'],
            top=top, coordinates=len(values), violating_coordinates=len(violating),
            violating_keys=len({r['key'] for r in violating}), total_keys=3*(index+1),
            violating_states=len({r['key'][1:] for r in violating}),
            violation_oldest_age_s=max([r['age_s'] for r in violating], default=None),
            violating_older5s=sum(r['age_s']>5 for r in violating), block_counts=block_counts))
with (ROOT / 'gradient_passes.json').open('x') as f:
    json.dump(records, f, indent=2)
for r in records:
    if r['index'] in [1424, 1438, 1440, 1480]:
        print(r['index'], r['frame'], r['kind'], r['top'], 'violating keys',
              r['violating_keys'], '/', r['total_keys'], 'oldest', r['violation_oldest_age_s'])
