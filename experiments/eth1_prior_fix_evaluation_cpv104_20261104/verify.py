"""Separate saved archive/mask/input verification; no scientific re-scoring."""
import datetime
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from evo.tools import file_interface

H=Path(__file__).resolve().parent
def read(p): return json.loads(Path(p).read_text())
def sha(p):
    with Path(p).open('rb') as f: return hashlib.file_digest(f,'sha256').hexdigest()

r=read(H/'results.json')
assert r['passed'] and r['evo_version']=='1.36.3'
assert r['source_runs']==r['receiver_replays']==0 and not r['new_matched_OFF_ON']
assert r['CP103_changed_backend'] and r['CP91_has_no_fusion']
assert r['verification']['CLI_API_error_max']<1e-8 and r['verification']['TUM_roundtrip_max']<1e-8
commands=read(H/'evaluation/commands.json')
assert len(commands)==r['new_evo_CLI_calls']
for c in commands:
    argv=c['command']
    assert c['exit']==0 and '--correct_scale' not in argv
    assert Path(argv[argv.index('--save_results')+1]).is_relative_to(H)
    if argv[0].endswith('evo_rpe'):
        assert argv[argv.index('-d')+1]=='0.5' and argv[argv.index('-u')+1]=='m' and '--pairs_from_reference' in argv
    else:
        assert argv[0].endswith('evo_ape') and '--align' in argv
    res=file_interface.load_res_file(Path(argv[argv.index('--save_results')+1]))
    err=res.np_arrays['error_array']
    assert np.isfinite(err).all() and len(err)>0
    assert abs(float(np.sqrt(np.mean(err**2)))-res.stats['rmse'])<1e-10
data=np.load(H/'evaluation/comparison.npz')
valid=data['common_valid']
assert valid.sum()==r['common_coverage']['valid_queries']
assert len(r['common'])==14 and len(r['primary_curves'])==11
pairs=read(H/'evaluation/common/pair_indices.json')
assert len(pairs)==r['common_RPE_pairs']
for pair in pairs:
    ids=np.flatnonzero((data['timestamp_ns']>=pair['start_ns'])&(data['timestamp_ns']<=pair['end_ns']))
    assert valid[ids].all() and pair['reference_distance_m']>=.5
for group in ('common','full_saved'):
    for key,row in r[group].items():
        assert row['rows']>100 and row['alignment']['scale']==1
        assert math.isfinite(row['ATE_RMSE_m']) and row['ATE_RMSE_m']>=0
        for kind in ('translation_m','rotation_deg'):
            z=row['RPE'][kind]
            res=file_interface.load_res_file(z['archive'])
            assert res.stats==z['statistics'] and len(res.np_arrays['error_array'])==z['pairs']
        if group=='common':
            assert row['rows']==int(valid.sum()) and data[key+'_support'][valid].all()
            assert row['RPE']['translation_m']['pairs']==len(pairs)
            assert row['RPE']['rotation_deg']['pairs']==len(pairs)
for path,digest in read(H/'evaluation/input_sha256.json').items(): assert sha(path)==digest,path
assert set(r['full_saved'])==set(r['common'])
assert sum(row['newly_scored'] for row in r['full_saved'].values())==4
out=dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),
    direct_EVO_CLI_archives=len(commands),common_rows=int(valid.sum()),common_pairs=len(pairs),
    primary_curves=11,delayed_curves=3,all_common_masks_equal=True,no_gap_crossing_pairs=True,
    inputs_verified=len(read(H/'evaluation/input_sha256.json')),source_runs=0,receiver_replays=0,
    changed_backend_not_matched_control=True,failed_CP99_ON_excluded=True)
with (H/'report_verification.json').open('x') as f: json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
