"""Verify all saved evo archives and consolidated table rows, then seal outputs."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
from datetime import datetime

sys.dont_write_bytecode=True
import numpy as np
from evo.core import metrics
from evo.tools import file_interface

H=Path(__file__).resolve().parent


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream,'sha256').hexdigest()


pins=json.loads((H/'input_sha256.json').read_text())
assert all(sha(p)==digest for p,digest in pins.items())
all_results=json.loads((H/'results.json').read_text())
assert all_results['passed'] and len(all_results['cases'])==7
archive_count=0
for p in sorted(H.rglob('*.zip')):
    r=file_interface.load_res_file(p)
    assert np.isfinite(r.np_arrays['error_array']).all()
    # Statistics are recomputed only by evo, never a separate RMSE formula.
    metric=metrics.APE(metrics.PoseRelation.translation_part)
    metric.error=r.np_arrays['error_array']
    assert metric.get_all_statistics()==r.stats, p
    archive_count+=1
commands=[]
for case,d in all_results['cases'].items():
    assert len(d['results'])==12
    pairs=json.loads((H/case/'pair_indices.json').read_text())
    assert len(pairs)==d['RPE_pairs']
    commands.extend(json.loads((H/case/'commands.json').read_text()))
    for row in d['results'].values():
        a=file_interface.load_res_file(row['ATE_evo_archive'])
        assert a.stats['rmse']==row['ATE_RMSE_m']
        for r in row['RPE'].values():
            z=file_interface.load_res_file(r['evo_archive'])
            assert z.stats==r['statistics'] and len(z.np_arrays['error_array'])==len(pairs)
assert len(commands)==all_results['new_CLI_calls']==372
assert all(c['returncode']==0 and Path(c['archive']).is_file() for c in commands)
for method in ('METHOD1','METHOD2'):
    lines=(H/(method+'_TABLE.txt')).read_text().splitlines()
    rows=[line for line in lines if any(f'| {label} |' in line for label in ('GLIM','Kimera','Fusion'))]
    assert len(rows)==21, method
    for line in rows:
        assert len(line.split('|'))==10, line
assert archive_count==516
checkpoint_command=['python3','/home/yeranis/repos/gnssFGO/experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py','--originals']
p=subprocess.run(checkpoint_command,capture_output=True,text=True,timeout=60,check=False)
assert p.returncode==0 and '667 saved files' in p.stdout, p.stdout+p.stderr
out=dict(passed=True,time=datetime.now().astimezone().isoformat(),pinned_files_verified=len(pins),
         cases=7, table_rows=42, ATE_entries=84,RPE_entries=168,new_evo_CLI_calls=len(commands),
         new_evo_archives_read_back=archive_count,reused_ETH1_RPE_archives=24,
         checkpoint=dict(command=checkpoint_command,returncode=p.returncode,stdout=p.stdout,stderr=p.stderr),
         source_runs=0,receiver_replays=0,containers_started=0,builds=0,
         output_sha256={str(p):sha(p) for p in sorted(H.rglob('*')) if p.is_file()})
with (H/'closeout.json').open('x') as stream:
    json.dump(out,stream,indent=2,allow_nan=False);stream.write('\n')
print(json.dumps({k:v for k,v in out.items() if k!='output_sha256'},indent=2))
