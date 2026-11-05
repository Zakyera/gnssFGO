"""Independent final score/contract audit, no receiver or new scoring."""
import math
import numpy as np
from evo.tools import file_interface
from common import *
assert not (H/'report_verification.json').exists()
r=read(H/'results.json');assert r['passed'] and r['evo_version']=='1.36.3'
assert r['receiver_replays']==4 and r['source_runs']==0 and r['not_live']
assert r['verification']['CLI_API_error_max']<1e-8 and r['verification']['TUM_roundtrip_max']<1e-8
commands=read(H/'evaluation/commands.json');assert len(commands)==r['direct_EVO_calls']==48
for item in commands:
    cmd=item['command'];assert item['exit']==0 and '--correct_scale' not in cmd
    dest=Path(cmd[cmd.index('--save_results')+1]);assert dest.is_relative_to(H)
    if cmd[0].endswith('evo_ape'):assert '--align' in cmd
    else:
        assert cmd[0].endswith('evo_rpe') and '--pairs_from_reference' in cmd
        assert cmd[cmd.index('-d')+1]=='0.5' and cmd[cmd.index('-u')+1]=='m'
    z=file_interface.load_res_file(dest);err=z.np_arrays['error_array'];assert len(err)>0 and np.isfinite(err).all()
    assert abs(float(np.sqrt(np.mean(err**2)))-z.stats['rmse'])<1e-10
old=read(S/'results.json');data=np.load(H/'evaluation/comparison.npz');prior=np.load(S/'evaluation/comparison.npz')
assert np.array_equal(data['timestamp_ns'],prior['timestamp_ns']) and np.array_equal(data['common_valid'],prior['common_valid'])
assert data['common_valid'].sum()==r['common_rows']==3532
assert read(H/'evaluation/common/pair_indices.json')==read(S/'evaluation/common/pair_indices.json') and r['common_RPE_pairs']==349
for group in ('common','full_saved'):
    assert set(r[group])=={m+'_'+p for m in MODES for p in ('online','delayed')}
    for key,row in r[group].items():
        assert row['alignment']['scale']==1 and math.isfinite(row['ATE_RMSE_m']) and row['rows']==(3532 if group=='common' else 3545)
        for kind in ('translation_m','rotation_deg'):
            x=row['RPE'][kind];z=file_interface.load_res_file(x['archive']);assert z.stats==x['statistics'] and len(z.np_arrays['error_array'])==x['pairs']
            if group=='common':assert x['pairs']==349
        if key.startswith('baseline'):
            expected=old[group]['on_fusion_'+key.rsplit('_',1)[1]]
            assert max(abs(row[k]-expected[k]) for k in ('ATE_RMSE_m','path_length_m','reference_path_length_m'))<1e-8
epoch=read(P/'input_audit.json')['epoch_ns'];t=(data['timestamp_ns']-epoch)*1e-9;valid=data['common_valid'];gp=data['reference_position']
prefix={}
for mode in MODES:
    for product in ('online','delayed'):
        key=mode+'_'+product;pos=data[key+'_position'];al=old['common']['on_fusion_'+product]['alignment'];e=pos@np.asarray(al['rotation']).T+al['translation']-gp
        assert np.max(abs(e-data[key+'_frozen_error']))<1e-12
        # Earliest changed event is after52.5s; initial states and earlier ONLINE
        # publications must match. Finalized states can incorporate future data.
        if product=='online':
            ix=valid&(t<50);delta=float(np.max(abs(pos[ix]-data['baseline_online_position'][ix])));assert delta==0;prefix[mode]=delta
        for w in r['local_diagnostics'][key]['windows']:
            ix=valid&(t>=w['start_s'])&(t<w['end_s']);rmse=float(np.sqrt(np.mean(np.sum(e[ix]**2,axis=1))));assert abs(rmse-w['baseline_alignment_RMS_m'])<1e-12
count=check(read(H/'evaluation/input_sha256.json'));assert read(H/'baseline_parity.json')['byte_identical_archive_files']==221071
assert all(x['covariance_payload_error']==0 for x in read(H/'validation.json')['variants'].values())
assert read(H/'container_restore.json')['passed']
save('report_verification.json',dict(passed=True,at=now(),direct_EVO_archives=48,common_rows=3532,common_reference_distance_pairs=349,full_native_rows=3545,baseline_original_scores_reproduced=True,exact_fixed_mask=True,baseline_alignment_window_arithmetic=True,unchanged_online_prefix_max_abs_error=prefix,inputs_verified=count,source_runs=0,receiver_replays=4,production_fix_qualified=False))
print(json.dumps(read(H/'report_verification.json'),indent=2))
