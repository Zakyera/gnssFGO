"""Independently verify report products and all direct EVO archives, saved-only."""
import hashlib,json,math
from pathlib import Path
import numpy as np
from evo.tools import file_interface
H=Path(__file__).resolve().parent
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
r=read(H/'results.json');assert r['passed'] and r['evo_version']=='1.36.3'
assert r['live_bidirectional'] and r['kimera_factor_mode']=='persistent'
assert r['glim_factor_mode']=='inject_persistent'
assert r['mixed_prior_feedback_user_accepted'] and not r['kimera_native_only_outgoing_information']
assert r['no_fresh_matched_off_control'] and r['receiver_Qc_multiplier']==1
assert r['source_runs_this_reporting_stage']==r['receiver_replays']==0
assert r['verification']['CLI_API_error_max']<1e-8
assert r['verification']['TUM_roundtrip_max']<1e-8
for group in ('full_saved','common'):
    assert set(r[group])=={'on_glim','on_kimera','on_fusion_online','on_fusion_delayed'}
    for key,v in r[group].items():
        assert v['rows']>10 and v['reference_path_length_m']>25
        assert all(math.isfinite(v[k]) and v[k]>=0 for k in ('ATE_RMSE_m','path_length_m'))
        for k in ('translation_m','rotation_deg'):
            assert v['RPE'][k]['pairs']>40
            archive=file_interface.load_res_file(v['RPE'][k]['archive'])
            assert archive.stats==v['RPE'][k]['statistics']
            assert len(archive.np_arrays['error_array'])==v['RPE'][k]['pairs']
commands=read(H/'evaluation/commands.json')
assert len(commands)==r['new_evo_CLI_calls'] and commands
archives=[]
for c in commands:
    assert c['exit']==0
    argv=c['command'];assert '--correct_scale' not in argv
    if argv[0].endswith('evo_rpe'):
        assert argv[argv.index('-d')+1]=='0.5' and argv[argv.index('-u')+1]=='m'
        assert '--pairs_from_reference' in argv
    path=Path(argv[argv.index('--save_results')+1])
    obj=file_interface.load_res_file(path);errors=obj.np_arrays['error_array']
    assert np.isfinite(errors).all() and len(errors)>0
    assert abs(float(np.sqrt(np.mean(errors**2)))-obj.stats['rmse'])<1e-10
    archives.append(str(path))
pins=read(H/'evaluation/input_sha256.json')
for p,v in pins.items():assert sha(p)==v,p
assert read(H/'on_validated.json')['passed'] and read(H/'closeout.json')['passed']
v=dict(passed=True,evo_archives=len(archives),report_inputs=len(pins),archives=archives,
       native_only_provenance_claimed=False,mixed_prior_feedback_user_accepted=True,
       source_runs=0,receiver_replays=0)
with (H/'report_verification.json').open('x') as f:json.dump(v,f,indent=2)
print(json.dumps(v,indent=2))
