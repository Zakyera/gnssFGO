#!/usr/bin/env python3
"""All original gates plus explicitly authorized numerical repeat equivalence."""
import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import shlex
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from experiment import ROOT, REFERENCE, REPO, BINARY, ENV, file_sha
OLD = REFERENCE / 'pilot_on_source'
from numerical_equivalence import compare
sys.path.insert(0,str(REPO/'tools'))
from live_receiver_protocol import parse_belief

def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('tag', choices=['profile40_on'])
    args=parser.parse_args()
    tag=args.tag
    phase,mode=tag.split('_')[:2]
    live=ROOT/(tag+'_receiver')
    source=ROOT/(tag+'_source')
    assert (live/'complete.marker').exists()
    assert (source/'complete.marker').exists()
    assert (ROOT/(tag+'_control/complete.json')).exists()
    assert not (ROOT/(tag+'_control/failed.txt')).exists()
    result=dict(tag=tag,passed=False)
    for name,expected in json.loads((ROOT/'protected_manifest.json').read_text()).items():
        assert file_sha(name)==expected,name
    # Parameters should differ from the retained matched run only in output paths.
    param_differences={}
    for name in ['resolved_glim_params.yaml','resolved_kimera_params.yaml']:
        a=yaml.safe_load((OLD/name).read_text())
        b=yaml.safe_load((source/name).read_text())
        differences={k:[a.get(k),b.get(k)] for k in set(a)|set(b) if a.get(k)!=b.get(k)}
        param_differences[name]=differences
        for key,(old,new) in differences.items():
            if key=='glim_cbs_experimental_local_gpu_batching':
                assert old is None and new is True
                continue
            if key.startswith('glim_cbs_active_factor_diagnostics_'):
                assert old is None and new == {'glim_cbs_active_factor_diagnostics_enable':False,
                    'glim_cbs_active_factor_diagnostics_stride':20,
                    'glim_cbs_active_factor_diagnostics_max_updates':40}[key]
                continue
            if mode=='off' and key in ['glim_cbs_receive_external_beliefs','cbs_belief_receive_enable']:
                assert old is True and new is False
                continue
            if mode=='off' and key=='glim_cbs_mode':
                assert old=='active_window_temporary' and new=='observe_only'
                continue
            if key in ['glim_cbs_receiver_local_competition_audit_max_samples',
                       'glim_cbs_receiver_local_competition_audit_stride']:
                assert (old,new) in [(0,40),(1,20)], (key,old,new)
                continue
            if key == 'glim_cbs_mode':
                assert (old,new) == ('inject_persistent','active_window_temporary')
                continue
            if key == 'glim_cbs_experimental_native_imu_mean_alignment':
                assert old is None and new is True
                continue
            if key == 'glim_dump_path':
                assert new.endswith('/'+tag+'_source/glim_dump')
                continue
            assert isinstance(old,str) and isinstance(new,str) and (
                key.endswith('recording_id') or str(OLD.name) in old), (key,old,new)
    result['parameter_path_differences']=param_differences
    for directory in ['glim_config','kimera_profile']:
        old=OLD/directory
        assert {str(p.relative_to(old)):hashlib.sha256(p.read_bytes()).hexdigest() for p in old.rglob('*') if p.is_file()} == {
          str(p.relative_to(source/directory)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (source/directory).rglob('*') if p.is_file()}
    metadata=dict(line.split('=',1) for line in (source/'run_metadata.txt').read_text().splitlines() if '=' in line)
    assert metadata['cbs_protocol']=='classic' and metadata['cbs_covariance']=='schur_relative_between'
    assert metadata['presentation_enable']=='false'
    assert metadata['glim_receive_cbs_beliefs']==metadata['kimera_receive_cbs_beliefs']==str(mode=='on').lower()
    assert metadata['sensor_window_sec']=='40.0' and metadata['bag_rate']=='0.10'
    assert metadata['glim_cbs_mode']==('active_window_temporary' if mode=='on' else 'observe_only')
    assert metadata['kimera_cbs_odom_factor_mode']=='active_window_temporary'
    assert json.loads((source/'effective_mode_check.json').read_text())['passed']
    result['source_metadata']=metadata
    glim_logs=[p for p in (source/'ros_logs').glob('*/glim_rosnode-*-stdout.log') if p.parent.name!='latest']
    assert len(glim_logs)==1
    expected={}
    for kind,sid,path in [('glim','glim_hesai',glim_logs[0]),('kimera','kimera_front',source/'estimators.log')]:
        for line in path.open(errors='strict'):
            belief=parse_belief(kind,line)
            if belief is None:
                continue
            key=(sid,belief['i'],belief['j'])
            payload=(belief['ti'],belief['tj'],*belief['mean'],*belief['covariance'])
            history=expected.setdefault(key,[])
            if not history or history[-1]!=payload:
                history.append(payload)
    received={}
    imus=0
    for line in (live/'events.csv').open():
        fields=line.strip().split(',')
        if fields[0]=='I':
            imus+=1
        if fields[0]!='B':
            continue
        assert int(fields[1])>=int(fields[9])
        key=(fields[2],int(fields[6]),int(fields[7]))
        history=received.setdefault(key,[])
        assert int(fields[5])==len(history)
        history.append((int(fields[8]),int(fields[9]),*map(float,fields[10:])))
    assert received==expected, 'source revision payload/order loss'
    summary=json.loads((live/'receiver/summary.json').read_text())
    assert summary['receiver_model']=='source_wnoa' and summary['streaming']
    assert summary['experimental_qc_diagonal']==[.00375]*3+[.015]*3
    assert summary['source_wnoa_initial_sigmas']==dict(pose_rad_m=1e-5,velocity_m_s=10,omega_rad_s=3)
    assert summary['imu_factor_count']==summary['bias_variable_count']==summary['acceleration_variable_count']==0
    assert summary['pending_at_end']==summary['revision_duplicates']==summary['revision_stale']==0
    assert summary['source_model_noise']=={}
    assert summary['maximum_factor_covariance_abs_difference']<1e-12
    assert summary['revision_messages_loaded']==sum(map(len,received.values()))
    clock={int(r['state_index']):int(r['available_timestamp_ns']) for r in read(live/'receiver/support_clock.csv')}
    active={}; replacements=0
    for r in read(live/'receiver/causal_dispatch.csv'):
        assert int(r['arrival_ns'])<=int(r['insertion_ns'])
        assert int(r['required_support'])<=int(r['created_support'])
        assert clock[int(r['required_support'])]<=int(r['insertion_ns'])
        key=tuple(r[k] for k in ['source','session','from_index','to_index'])
        revision=int(r['revision'])
        if r['outcome']=='insert':
            assert key not in active
        elif r['outcome']=='replace':
            assert key in active and revision>active[key]
            replacements+=1
        else:
            raise AssertionError(r)
        active[key]=revision
    assert len(active)==summary['beliefs_accepted'] and replacements==summary['revision_removals']
    unsupported=read(live/'receiver/unsupported_beliefs.csv')
    result['unsupported']=unsupported
    conv=read(live/'receiver/publication_convergence.csv')
    assert len(conv)==summary['support_states']
    assert all(r['status']=='stationary' and float(r['gradient_after'])<=1e-5 for r in conv)
    result['max_publication_gradient']=max(float(r['gradient_after']) for r in conv)
    monitor=[json.loads(line) for line in (ROOT/(tag+'_control')/'monitor.jsonl').open()]
    live_monitor=[r for r in monitor if r['published_poses']>0 and r['last_imu_ns']>0]
    assert len(live_monitor)>5, 'no sustained co-running output'
    ages=np.array([r['output_age_seconds'] for r in live_monitor])
    result['live_output_age_s']=dict(zip(['median','p95','max'],map(float,np.quantile(ages,[.5,.95,1]))))
    result['maximum_observed_queue']=max(r['queued'] for r in monitor)
    transport=json.loads((live/'transport_status.json').read_text())
    assert transport['queued']==0 and transport['max_queue']<1000
    result['final_transport_status']=transport
    from coverage_checks import check,load
    imu_times=[int(row.split(',')[2])*1e-9 for row in (live/'events.csv').read_text().splitlines() if row.startswith('I,')]
    coverage=json.loads((source/'source_coverage.json').read_text())
    for role,file in [('fused_online','online_trajectory.tum'),('fused_final','trajectory.tum')]:
        coverage[role]=check(*load(live/'receiver'/file),40,imu_times[0],imu_times[-1],role)
    result['coverage']=coverage
    # Operational smoke gate, not an accuracy/model tuning threshold.
    assert np.quantile(ages,.95)<1.0 and max(r['queued'] for r in monitor)<1000, 'live throughput gate failed'
    if True:  # One saved-journal validation, not another source run.
        repeat=ROOT/(tag+'_numerical_repeat')
        assert not repeat.exists(), 'Refuse duplicate validation replay'
        args=['env',*[k+'='+v for k,v in sorted(ENV.items())],
          str(BINARY),'--receiver-model','source_wnoa','--mode','sources','--timeline','record','--solver','isam2',
          '--support-policy','uniform_imu','--support-period-ns','100000000','--receiver-startup-samples','59',
          '--event-stream',str(live/'events.csv'),'--out',str(repeat)]
        command=['docker','exec','ubuntu-jammy-humble-nvidia','bash','-lc',
                 'source /opt/ros/humble/setup.bash && exec '+shlex.join(args)]
        run=subprocess.run(command,capture_output=True,text=True)
        with (ROOT/(tag+'_numerical_repeat.log')).open('x') as f: f.write(run.stdout+run.stderr)
        assert run.returncode==0,run.stderr
    numerical = compare(live/'receiver', repeat)
    identities={}
    for name in ['trajectory.tum','states.csv','online_trajectory.tum','online_states.csv','support_clock.csv',
                 'source_model_covariance.csv','model_covariance.csv','causal_dispatch.csv','measurement_events.csv']:
        a=(live/'receiver'/name).read_bytes()
        identities[name]=dict(live_sha256=hashlib.sha256(a).hexdigest(),
                             repeat_sha256=file_sha(repeat/name),
                             byte_identical=a==(repeat/name).read_bytes())
    result.update(passed=True,imu_samples=imus,revision_payloads_conserved=True,
                  raw_source_ids=len(received),versions=sum(map(len,received.values())),
                  active_source_ids=len(active),replacements=replacements,summary=summary,
                  repeat_files=identities,numerical_repeat=numerical)
    # Read source lifecycle evidence, do not infer removal from a trajectory.
    tokens = {
        'glim_queued':'GLIM_CBS_ODOM_INJECT_ROW',
        'glim_removed':'GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW,active_window_remove_committed',
        'kimera_tracked':'active_window_slot_tracked',
        'kimera_removed':'active_window_remove_scheduled',
        'kimera_pre_guard_drop':'action=drop_before_stage',
        'kimera_post_guard_rollback':'action=rollback_and_retry_without_staged_cbs',
        'kimera_active_guard_rollback':'action=rollback_remove_active_and_retry'}
    counts = collections.Counter()
    lifecycle_logs = {p.resolve() for p in (source/'ros_logs').glob('*/*stdout.log') if p.parent.name != 'latest'}
    lifecycle_logs.add((source/'estimators.log').resolve())
    for log in sorted(lifecycle_logs):
        for line in log.open():
            if 'injected_persistent' in line or 'persistent_odom_applied' in line:
                raise AssertionError('Unexpected persistent CBS factor application')
            for key, token in tokens.items():
                if token in line:
                    if key == 'glim_queued' and 'active_window_temporary_graph_queued' not in line:
                        continue
                    if key == 'kimera_removed' and 'CBS_BPSAM_' not in line:
                        continue
                    counts[key] += 1
    required=['glim_queued','kimera_tracked']
    required+=['glim_removed','kimera_removed']
    for key in required:
        assert (counts[key]>0)==(mode=='on'), ('unexpected/missing active-window lifecycle evidence',key)
    result['source_lifecycle_counts']=dict(counts)
    result['native_alignment']=json.loads((ROOT/(tag+'_native_validation.json')).read_text())
    assert result['native_alignment']['passed']
    result['diagnostics']=json.loads((ROOT/(tag+'_diagnostics_validation.json')).read_text())
    assert result['diagnostics']['passed']
    if mode=='on':
        result['lifecycle']=json.loads((ROOT/(tag+'_lifecycle_validation.json')).read_text())
        assert result['lifecycle']['passed']
    result['interpretation_limit']='Mode includes legacy guards. Counts validate lifecycle activity, not a numerical proof of zero inherited information.'
    path=ROOT/(phase+'_'+mode+'_validation.json')
    with path.open('x') as f:
        json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in ['summary','unsupported','source_metadata','parameter_path_differences','repeat_files','native_alignment','numerical_repeat']},indent=2))

if __name__=='__main__':
    main()
