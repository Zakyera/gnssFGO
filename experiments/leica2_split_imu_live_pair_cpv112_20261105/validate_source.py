"""Saved full-source integrity/numerical gate; no accuracy threshold or gap filling."""
import argparse,csv,hashlib,json,re,sys
from pathlib import Path
import numpy as np
from paths import H,P
from injection_schema import validate_injections
parser=argparse.ArgumentParser()
parser.add_argument('mode',choices=['off','on'])
parser.add_argument('--source',type=Path)
parser.add_argument('--profile-root',type=Path)
parser.add_argument('--output',type=Path)
args=parser.parse_args();mode=args.mode
S=args.source or P/f'full_{mode}_source'
profile=args.profile_root or P/mode
output=args.output or H/f'{mode}_candidate_validation.json'
assert not output.exists()
def read(p):return json.loads(p.read_text())
coverage=read(S/'source_coverage.json');effective=read(S/'effective_mode_check.json')
assert (S/'complete.marker').exists() and coverage['passed'] and effective['passed']
assert effective['checked_before_playback'] and coverage['requested_s']==428.75
assert effective['kimera_candidate_env']['CPV40_REFRESH']=='1'
assert effective['glim']['glim_cbs_mode']=='inject_persistent'
assert effective['glim']['glim_cbs_receive_external_beliefs'] is (mode=='on')
assert coverage['frontend']['received_pairs']==4287
assert coverage['native_kimera']['rows']==coverage['backend_rows']>10
assert coverage['native_glim']['rows']>=4267
assert not (S/'acceptance.csv').exists()
assert effective['private_kimera_sha256']=='985c115d3573446f65ef4ce7cc4125e514b4922fed6445ed05cbc4dbdfb70a09'
assert Path(effective['prior_failure_archive']).name=='prior_failures'
assert (S/'prior_failures').is_dir() and not list((S/'prior_failures').iterdir())
assert effective['kimera_candidate_env']==dict(CPK9_PRIOR_REFRESH='1',FL4Q_ENABLE='0',FL4M_RETIREMENT='remove',CPV40_REFRESH='1',CPV38_OFF_REFRESH='0',CPV76_PERSISTENT_COMPAT='1')
assert effective['kimera']['cbs_experimental_native_imu_mean_alignment']=='true'
assert hashlib.sha256((profile/'private_build/libkimera_vio.so').read_bytes()).hexdigest()=='985c115d3573446f65ef4ce7cc4125e514b4922fed6445ed05cbc4dbdfb70a09'
logs=[S/'estimators.log',*S.glob('ros_logs/*/*stdout.log')]
for p in logs:
    for line in p.open(errors='strict'):
        for marker in ('SHARED_NATIVE_SHADOW_FAILURE','SHARED_NATIVE_SHADOW_SETUP_FAILURE',
                       'SHARED_NATIVE_PROVIDER_FAILURE','CPK9 ','CPV40 ','CPV76 ',
                       'active_window_rollback','active_window_retry_local_update_',
                       'Smoother recovery','Persistent BPSAM recovery','process has died',
                       'terminate called','Segmentation fault','FL4Q','CPK9_PRIOR_REFRESH,',
                       'rollback_and_retry_without_staged_cbs','rollback_remove_active_and_retry',
                       'CPV40_FATAL','CPV73_READINESS_FAILURE','active_window_temporary_graph_queued',
                       'GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,removed','active_window_temporary_odom_applied'):
            assert marker not in line,(str(p),line)
with (S/'objective.csv').open() as f:
    rows=list(csv.reader(f))
data=np.array(rows,float)
assert data.shape==(coverage['backend_rows'],10) and np.isfinite(data).all(),data.shape
assert np.array_equal(data[:,0],np.arange(len(data)))
assert np.all(data[:,1]==0),'expected incremental solver mode'
assert np.all(data[:,2]==0),'persistent mode must not retire temporary factors'
injections=validate_injections(mode,coverage['external_injections'])
result=dict(passed=True,mode=mode,source_coverage=coverage,native_updates=len(data),
    numerical_objective_finite=True,actual_sparse_native_keyframes_preserved=True,
    temporary_retirements=0,imu_factor_cost=dict(median=float(np.median(data[:,7])),maximum=float(data[:,7].max())),
    native_only_outgoing_information=False if mode=='on' else None,
    external_injections=injections,source=str(S),
    scope='Exact CP89/91 raw-input/init/EOF coverage gates plus finite objective and no recovery; not estimator health certification')
with output.open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
