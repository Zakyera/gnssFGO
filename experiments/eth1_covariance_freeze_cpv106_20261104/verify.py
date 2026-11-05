"""Independent saved-score/mask/command check; no re-scoring or optimization."""
import math
import numpy as np
from evo.tools import file_interface
from common import *
r=read(H/'results.json');assert r['passed'] and r['evo_version']=='1.36.3'
assert r['receiver_replays']==2 and r['source_runs']==0 and r['not_a_new_live_OFF_ON']
assert r['verification']['CLI_API_error_max']<1e-8 and r['verification']['TUM_roundtrip_max']<1e-8
commands=read(H/'evaluation/commands.json');assert len(commands)==r['direct_EVO_calls']==24
for item in commands:
    cmd=item['command'];assert item['exit']==0 and '--correct_scale' not in cmd
    dest=Path(cmd[cmd.index('--save_results')+1]);assert dest.is_relative_to(H)
    if cmd[0].endswith('evo_ape'):assert '--align' in cmd
    else:
        assert cmd[0].endswith('evo_rpe') and '--pairs_from_reference' in cmd
        assert cmd[cmd.index('-d')+1]=='0.5' and cmd[cmd.index('-u')+1]=='m'
    z=file_interface.load_res_file(dest);err=z.np_arrays['error_array']
    assert len(err)>0 and np.isfinite(err).all()
    assert abs(float(np.sqrt(np.mean(err**2)))-z.stats['rmse'])<1e-10
S=E/'eth1_prior_fix_evaluation_cpv104_20261104'
old=read(S/'results.json');data=np.load(H/'evaluation/comparison.npz');prior=np.load(S/'evaluation/comparison.npz')
assert np.array_equal(data['timestamp_ns'],prior['timestamp_ns'])
assert np.array_equal(data['common_valid'],prior['common_valid'])
assert data['common_valid'].sum()==r['common_rows']==2966
assert read(H/'evaluation/common/pair_indices.json')==read(S/'evaluation/common/pair_indices.json')
assert r['common_RPE_pairs']==365
for group in ('common','full_saved'):
    assert set(r[group])=={'baseline_online','baseline_delayed','frozen_kimera_online','frozen_kimera_delayed'}
    for key,row in r[group].items():
        assert row['alignment']['scale']==1 and math.isfinite(row['ATE_RMSE_m'])
        assert row['rows']==(2966 if group=='common' else 3456)
        for kind in ('translation_m','rotation_deg'):
            x=row['RPE'][kind];z=file_interface.load_res_file(x['archive'])
            assert z.stats==x['statistics'] and len(z.np_arrays['error_array'])==x['pairs']
            if group=='common':assert x['pairs']==365
        if key.startswith('baseline'):
            expected=old[group]['CP103_ON_fusion_'+key.rsplit('_',1)[1]]
            values=[row['ATE_RMSE_m'],row['path_length_m'],row['reference_path_length_m']]
            targets=[expected['ATE_RMSE_m'],expected['path_length_m'],expected['reference_path_length_m']]
            assert max(abs(x-y) for x,y in zip(values,targets))<1e-8
count=check(read(H/'evaluation/input_sha256.json'))
assert read(H/'baseline_parity.json')['byte_identical_archive_files']==211422
assert read(H/'validation.json')['covariance_payload_error']==0
assert read(H/'container_restore.json')['passed']
save('report_verification.json',dict(passed=True,finished=now(),direct_EVO_archives=24,
    common_rows=2966,common_reference_distance_pairs=365,full_native_rows=3456,
    baseline_original_scores_reproduced=True,exact_fixed_mask=True,inputs_verified=count,
    source_runs=0,receiver_replays=2,production_fix_qualified=False))
print(json.dumps(read(H/'report_verification.json'),indent=2))
