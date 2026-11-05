"""Independent saved archive/mask/report verification; no re-scoring or capture."""
import hashlib,json,math
from pathlib import Path
import numpy as np
from evo.tools import file_interface
H=Path(__file__).resolve().parent
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
r=read(H/'results.json');assert r['passed'] and r['evo_version']=='1.36.3'
assert r['fresh_matched_off_on'] and r['live_bidirectional_ON'] and r['both_persistent_ON']
assert r['mixed_prior_feedback_user_accepted'] and not r['kimera_native_only_outgoing_information']
assert r['source_runs_this_reporting_stage']==r['receiver_replays']==0 and r['receiver_Qc_multiplier']==1
assert r['verification']['CLI_API_error_max']<1e-8 and r['verification']['TUM_roundtrip_max']<1e-8
expected={f'{m}_{k}' for m in ('off','on') for k in ('glim','kimera','fusion_online','fusion_delayed')}
for group in ('full_saved','common'):
    assert set(r[group])==expected
    for key,v in r[group].items():
        assert v['rows']>100 and v['reference_path_length_m']>100
        assert all(math.isfinite(v[k]) and v[k]>=0 for k in ('ATE_RMSE_m','path_length_m'))
        for k in ('translation_m','rotation_deg'):
            z=v['RPE'][k];res=file_interface.load_res_file(z['archive'])
            assert res.stats==z['statistics'] and len(res.np_arrays['error_array'])==z['pairs']>100
        if group=='common':
            assert v['rows']==r['common_coverage']['valid_queries']
            assert v['RPE']['translation_m']['pairs']==r['common_RPE_pairs']
commands=read(H/'evaluation/commands.json');assert len(commands)==r['new_evo_CLI_calls'] and commands
for c in commands:
    assert c['exit']==0;argv=c['command'];assert '--correct_scale' not in argv
    if argv[0].endswith('evo_rpe'):
        assert argv[argv.index('-d')+1]=='0.5' and argv[argv.index('-u')+1]=='m' and '--pairs_from_reference' in argv
    else:assert '--align' in argv
    res=file_interface.load_res_file(Path(argv[argv.index('--save_results')+1]));err=res.np_arrays['error_array']
    assert np.isfinite(err).all() and len(err)>0
    assert abs(float(np.sqrt(np.mean(err**2)))-res.stats['rmse'])<1e-10
pins=read(H/'evaluation/input_sha256.json')
for p,v in pins.items():assert sha(p)==v,p
for mode in ('off','on'):assert read(H/(mode+'_validated.json'))['passed']
assert read(H/'closeout.json')['passed']
out=dict(passed=True,direct_EVO_CLI_archives=len(commands),report_inputs=len(pins),fresh_pair=True,
         source_runs=0,receiver_replays=0,all_common_masks_equal=True,native_only_provenance_claimed=False)
with (H/'report_verification.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out,indent=2))
