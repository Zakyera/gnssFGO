"""Preserve malformed fixture; create corrected end-marker fixture once."""
import hashlib
import json
from pathlib import Path
root=Path('/home/yeranis/results/gnssfgo_receiver_subphase_s5b_20261013')
lines=(root/'prefix5.csv').read_text().splitlines(keepends=True)
assert lines[-1]=='E\n' and all(line[0] in ['I','B'] for line in lines[:-1])
last=int(lines[-2].split(',')[1])
with (root/'prefix5_valid.csv').open('x') as f:
    f.writelines(lines[:-1]);f.write(f'E,{last}\n')
with (root/'prefix_fix.json').open('x') as f:
    json.dump(dict(rows_unchanged=len(lines)-1,end_timestamp=last,
        sha256=hashlib.sha256((root/'prefix5_valid.csv').read_bytes()).hexdigest()),f,indent=2)
