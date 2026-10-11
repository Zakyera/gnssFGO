#!/usr/bin/env python3
"""Capture conservation, frozen settings, causal accounting and exact repeat."""
import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np
import yaml
from prepare import ROOT, OLD, REPO, BINARY, ENV
sys.path.insert(0,str(REPO/'tools'))
from live_receiver_protocol import parse_belief

def read(path):
    with path.open() as f:
        return list(csv.DictReader(f))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('tag')
    args=parser.parse_args()
    tag=args.tag
    mode=tag.split('_')[0]
    loss='huber' if tag=='on_huber' else 'gaussian'
    live=ROOT/(tag+'_receiver')
    source=ROOT/(tag+'_source')
    assert (live/'complete.marker').exists() and (source/'complete.marker').exists()
    result=dict(tag=tag,passed=False)
    for name,expected in json.loads((ROOT/'protected_manifest.json').read_text()).items():
        assert hashlib.sha256(Path(name).read_bytes()).hexdigest()==expected,name
    # Parameters should differ from the retained matched run only in output paths.
    param_differences={}
    for name in ['resolved_glim_params.yaml','resolved_kimera_params.yaml']:
        a=yaml.safe_load((OLD/('smoke_'+mode+'_source')/name).read_text())
        b=yaml.safe_load((source/name).read_text())
        differences={k:[a.get(k),b.get(k)] for k in set(a)|set(b) if a.get(k)!=b.get(k)}
        param_differences[name]=differences
        for key,(old,new) in differences.items():
            if key=='glim_cbs_source_loss':
                assert old is None and new==loss
                continue
            assert isinstance(old,str) and isinstance(new,str) and (
                key.endswith('recording_id') or str(OLD.name) in old), (key,old,new)
    result['parameter_path_differences']=param_differences
    for directory in ['glim_config','kimera_profile']:
        old=OLD/('smoke_'+mode+'_source')/directory
        assert {str(p.relative_to(old)):hashlib.sha256(p.read_bytes()).hexdigest() for p in old.rglob('*') if p.is_file()} == {
          str(p.relative_to(source/directory)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (source/directory).rglob('*') if p.is_file()}
    metadata=dict(line.split('=',1) for line in (source/'run_metadata.txt').read_text().splitlines() if '=' in line)
    assert metadata['cbs_protocol']=='classic' and metadata['cbs_covariance']=='schur_relative_between'
    assert metadata['presentation_enable']=='false'
    assert metadata['glim_receive_cbs_beliefs']==metadata['kimera_receive_cbs_beliefs']==str(mode=='on').lower()
    result['source_metadata']=metadata
    assert (ROOT/(tag+'_control')/'runtime_maps.json').exists()
    loss_records={}
    for kind, pattern in [('glim','*/glim_rosnode-*-stdout.log'),
                          ('kimera','*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log')]:
        paths=sorted({p.resolve() for p in (source/'ros_logs').glob(pattern)})
        assert len(paths)==1,(kind,paths)
        rows=[line.split('CBS_SOURCE_LOSS_',1)[1].strip() for line in paths[0].open(errors='strict') if 'CBS_SOURCE_LOSS_' in line]
        created=[r for r in rows if r.startswith('CREATE,')]
        states=[r for r in rows if r.startswith('STATE,')]
        assert bool(created)==(mode=='on'),(kind,len(created))
        assert all(r.split(',')[5]==loss for r in created)
        for r in created:
            covariance=np.array(list(map(float,r.split(',')[6:]))).reshape(6,6)
            assert np.isfinite(covariance).all()
            assert np.linalg.eigvalsh((covariance+covariance.T)*.5).min()>0
        phases=collections.Counter(r.split(',')[3] for r in states)
        if mode=='on': assert all(phases[p]>0 for p in ['pre','linearize','post_fresh']),phases
        for row in states:
            r=row.split(','); s,w,cost=map(float,r[6:9])
            expected_w=3/s if loss=='huber' and s>3 else 1.0
            expected_cost=3*s-4.5 if loss=='huber' and s>3 else .5*s*s
            assert np.isclose(w,expected_w,rtol=1e-10,atol=1e-12),(kind,row)
            assert np.isclose(cost,expected_cost,rtol=1e-9,atol=1e-10),(kind,row)
        loss_records[kind]=dict(created=len(created),states=len(states),phases=dict(phases))
    result['source_loss_records']=loss_records
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
    # Operational smoke gate, not an accuracy/model tuning threshold.
    assert np.quantile(ages,.95)<1.0 and max(r['queued'] for r in monitor)<1000, 'live throughput gate failed'
    repeat=ROOT/(tag+'_repeat')
    command=['docker','exec','ubuntu-jammy-humble-nvidia','env',*[k+'='+v for k,v in ENV.items()],
      str(BINARY),'--receiver-model','source_wnoa','--mode','sources','--timeline','record','--solver','isam2',
      '--support-policy','uniform_imu','--support-period-ns','100000000','--receiver-startup-samples','59',
      '--event-stream',str(live/'events.csv'),'--out',str(repeat)]
    run=subprocess.run(command,capture_output=True,text=True)
    (ROOT/(tag+'_repeat.log')).write_text(run.stdout+run.stderr)
    assert run.returncode==0,run.stderr
    identities={}
    for name in ['trajectory.tum','states.csv','online_trajectory.tum','online_states.csv','support_clock.csv',
                 'source_model_covariance.csv','model_covariance.csv','causal_dispatch.csv','measurement_events.csv']:
        a=(live/'receiver'/name).read_bytes()
        assert a==(repeat/name).read_bytes(),name
        identities[name]=hashlib.sha256(a).hexdigest()
    result.update(passed=True,imu_samples=imus,revision_payloads_conserved=True,
                  raw_source_ids=len(received),versions=sum(map(len,received.values())),
                  active_source_ids=len(active),replacements=replacements,summary=summary,
                  exact_repeat_sha256=identities)
    path=ROOT/(tag+'_validation.json')
    with path.open('x') as f:
        json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in ['summary','unsupported','source_metadata','parameter_path_differences','exact_repeat_sha256']},indent=2))

if __name__=='__main__':
    main()
