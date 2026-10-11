"""Aggregate independent validations; preserves the launcher's failed status."""
from pathlib import Path
import hashlib
import json
import subprocess

ROOT=Path(__file__).resolve().parent.parent
def sha(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
data={}
for name in ['capture_review','coverage_validation','gate_on_native_validation',
             'diagnostics_validation','lifecycle_supplement','gate_on_validation','protected_validation']:
    data[name]=json.loads((ROOT/(name+'.json')).read_text())
    assert data[name].get('passed',data[name].get('captured_data_complete',False)),name
for path,expected in json.loads((ROOT/'protected_manifest.json').read_text()).items():assert sha(path)==expected,path
v=data['gate_on_validation']; d=data['diagnostics_validation']
status=json.loads((ROOT/'gate_on_receiver/transport_status.json').read_text())
repos={}
for repo in [Path('/home/yeranis/repos/gnssFGO')]+[ROOT.parents[2]/'src'/n for n in ['glim_ros1','glim','Kimera-VIO','Kimera-VIO-ROS','cbs','cbsms']]:
    def git(*args):return subprocess.check_output(['git','-C',str(repo),*args],text=True)
    subprocess.run(['git','-C',str(repo),'diff','--check'],check=True)
    repos[str(repo)]=dict(head=git('rev-parse','HEAD').strip(),status=git('status','--short'),
                        dirty_diff_sha256=hashlib.sha256(git('diff','HEAD').encode()).hexdigest())
files=[ROOT/(n+'.json') for n in data]
files += [ROOT/'gate_on_receiver/events.csv', ROOT/'gate_on_source/recordings/cbs.bag',
          ROOT/'gate_on_control/failed.txt',ROOT/'gate_on_control/source.log']
files += list((ROOT/'validation_v2').glob('*.py'))
out=dict(status='CAPTURE_INDEPENDENTLY_VALIDATED_WITH_HARNESS_POSTCHECK_FAILURE',
    original_harness_failed=True, failure_reason='Inherited minimum100 GLIM poses incorrectly applied to10s gate',
    original_failure_preserved=True,no_estimator_rerun=True,no_160s_run=True,
    covariance_or_estimation_changes_during_run=False,
    diagnostic_samples={'Kimera_to_GLIM':len(d['glim_samples']),'GLIM_to_Kimera':len(d['kimera_samples'])},
    sampled_factor_algebra_max_relative_error=max(e for rows in [d['glim_samples'],d['kimera_samples']]
        for row in rows for e in row['algebra_relative_errors'].values()),
    revisions_conserved=v['versions'],stable_ids_received=v['raw_source_ids'],
    active_source_ids=v['active_source_ids'],replacements=v['replacements'],
    unsupported_versions=len(v['unsupported']),live_output_age_s=v['live_output_age_s'],
    exact_queue_peak=status['max_queue'],queue_at_end=status['queued'],
    max_publication_gradient=v['max_publication_gradient'],
    trajectory_and_state_replay_byte_identical=all(x['byte_identical'] for x in v['repeat_files'].values()),
    source_lifecycle_counts=v['source_lifecycle_counts'],
    artifact_sha256={str(p.relative_to(ROOT)):sha(p) for p in files},repositories=repos,
    limits='Short logging gate only; not proof of zero observer effect or160s throughput. '
           'No GT scoring or OFF/ON accuracy conclusion. Full fusion-family audit remains offline after capture.')
with (ROOT/'completion_review.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps({k:v for k,v in out.items() if k not in ['artifact_sha256','repositories']},indent=2))
