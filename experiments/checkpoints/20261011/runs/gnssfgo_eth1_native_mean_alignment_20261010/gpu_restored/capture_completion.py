"""Final validation/provenance aggregation, not a source or solver run."""
from pathlib import Path
import json
import subprocess
from experiment import ROOT, BINARY, file_sha

validation=json.loads((ROOT/'pilot_on_validation.json').read_text())
native=json.loads((ROOT/'pilot_on_native_validation.json').read_text())
lifecycle=json.loads((ROOT/'lifecycle_supplement.json').read_text())
evaluation=json.loads((ROOT/'full40_evaluation/summary.json').read_text())
assert all(x['passed'] for x in [validation,native,lifecycle,evaluation])
manifests={}
for name in ['protected_manifest.json','acceptance_manifest.json']:
    manifest=json.loads((ROOT/name).read_text())
    for path, expected in manifest.items(): assert file_sha(path)==expected, path
    manifests[name]=dict(entries=len(manifest),sha256=file_sha(ROOT/name))
ws=ROOT.parents[2]
repos={}
for repo in [Path('/home/yeranis/repos/gnssFGO')]+[ws/'src'/n for n in ['Kimera-VIO','Kimera-VIO-ROS','glim','glim_ros1','cbs','cbsms']]:
    def git(*args):return subprocess.check_output(['git','-C',str(repo),*args],text=True)
    subprocess.run(['git','-C',str(repo),'diff','--check'],check=True)
    changed=git('diff','--name-only','HEAD').splitlines()+git('ls-files','--others','--exclude-standard').splitlines()
    repos[str(repo)]=dict(head=git('rev-parse','HEAD').strip(),status=git('status','--short'),
        changed_file_hashes={p:file_sha(repo/p) for p in changed if (repo/p).is_file()})
status=json.loads((ROOT/'pilot_on_receiver/transport_status.json').read_text())
result=dict(status='NATIVE_MEAN_ALIGNMENT_40S_VALIDATED',
    acceptance='Numerical state equivalence plus exact event/accounting/covariance diagnostics; not claimed byte identity.',
    no_estimator_changes_during_resume=True,covariance_model_unchanged=True,defaults_disabled=True,
    fuser_binary_sha256=file_sha(BINARY),validated_manifests=manifests,
    total_revisions=validation['versions'],active_ids=validation['active_source_ids'],
    replacements=validation['replacements'],output_age_s=validation['live_output_age_s'],
    queue_peak=status['max_queue'],queue_at_end=status['queued'],
    source_lifecycle={k:{a:b for a,b in v.items() if a!='removals'} for k,v in lifecycle.items() if isinstance(v,dict)},
    native_alignment_calls={k:v['aligned_calls'] for k,v in native['receivers'].items()},
    evaluator_reference=evaluation['reference'],
    results={k:dict(full_ATE_m=v['full']['ATE_RMSE_m'],full_rows=v['full']['rows'],
        span_s=v['full']['timestamp_span_s'],max_gap_s=v['full']['max_pose_gap_s'],
        path_m=v['full']['full_CPT7_polyline_path_m'],coverage_percent=v['full']['reference_coverage_percent'],
        common_ATE_m=v['common']['ATE_RMSE_m'],common_RPE_1s=v['common_RPE_1s']) for k,v in evaluation['results'].items()},
    repositories=repos)
with (ROOT/'completion.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='repositories'},indent=2))
