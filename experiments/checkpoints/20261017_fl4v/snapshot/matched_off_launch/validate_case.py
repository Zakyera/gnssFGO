"""Validate the one completed live OFF pilot and its deterministic journal replay."""
import collections, csv, hashlib, json, math, os, shlex, statistics, subprocess, sys, time
from pathlib import Path
P=Path(__file__).resolve().parent
MODE=sys.argv[2] if len(sys.argv)>2 else 'off'
assert MODE in ('off','on')
SRC=P/f'live120_{MODE}_source'
REC=P/f'live120_{MODE}_receiver'
OUT=REC/'receiver'
CTL=P/f'live120_{MODE}_control'
EXE=Path('/home/yeranis/results/gnssfgo_fixed_lag_failure_fl4e_20261014/candidate_live')
RESULT=Path(f'/home/yeranis/results/gnssfgo_fixed_lag_live_fl4t_20261016/{MODE}')
LIB=Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/lib')

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(1048576),b''):h.update(block)
    return h.hexdigest()

def rows(path):
    with path.open(newline='') as f:return list(csv.reader(f))

def dicts(path):
    with path.open(newline='') as f:return list(csv.DictReader(f))

def stat(x):
    x=sorted(map(float,x))
    assert x
    return dict(n=len(x),mean=statistics.fmean(x),median=statistics.median(x),
                p95=x[min(len(x)-1,math.ceil(.95*len(x))-1)],max=x[-1])

def check():
    assert (CTL/'complete.json').is_file() and not (CTL/'failed.txt').exists()
    assert (SRC/'complete.marker').is_file() and (REC/'complete.marker').is_file()
    assert not (REC/'transport_failed.txt').exists() and not (REC/'failed.txt').exists()
    manifest=json.loads((P/'protected_manifest.json').read_text())
    for path,want in manifest.items():assert sha(path)==want,path
    cfg=json.loads((REC/'resolved_config.json').read_text())
    effective=json.loads((SRC/'effective_mode_check.json').read_text())
    assert effective['passed'] and effective['checked_before_playback']
    assert effective['glim']['glim_cbs_mode']=='active_window_temporary'
    assert effective['glim']['glim_cbs_receive_external_beliefs'] is (MODE=='on')
    assert effective['glim']['glim_cbs_publish_beliefs'] is True
    assert cfg['receiver_fixed_lag_seconds']==7
    assert cfg['receiver_publication_policy']=='fl4a_fixed_lag_incremental'
    assert cfg['source_protocol']=='classic'
    assert json.loads((OUT/'summary.json').read_text())['stationarity']=='not_checked_incremental_policy'
    source_cov=json.loads((SRC/'source_coverage.json').read_text())
    assert source_cov['passed'] and source_cov['requested_s']==120
    assert source_cov['native_glim']['rows']>=1170 and source_cov['native_kimera']['rows']>=570
    metadata=(SRC/'run_metadata.txt').read_text()
    markers=['mode=cbs_'+MODE,
             'glim_receive_cbs_beliefs='+str(MODE=='on').lower(),
             'kimera_receive_cbs_beliefs='+str(MODE=='on').lower(),
             'glim_cbs_mode=active_window_temporary']
    if MODE=='off':
        markers.extend(('k2g_injected=0','g2k_injected=0'))
    for marker in markers:
        assert marker in metadata,marker
    events=rows(REC/'events.csv')
    assert events[-1][0]=='E' and sum(r[0]=='E' for r in events)==1
    I=[r for r in events if r[0]=='I'];B=[r for r in events if r[0]=='B']
    assert I and B and sum(r[2]=='glim_hesai' for r in B)>0 and sum(r[2]=='kimera_front' for r in B)>0
    transport=json.loads((REC/'transport_complete.json').read_text())
    summary=json.loads((OUT/'summary.json').read_text())
    assert summary['complete'] and summary['lag_seconds']==7
    assert summary['events']==len(events)
    assert transport['imu']==len(I) and transport['beliefs']==len(B)
    assert summary['received']==len(B) and summary['pending_at_end']==0
    assert transport['published_poses']==summary['publications']
    assert transport['max_queue']<1000
    assert summary['retained_supports']<=71 and summary['peak_slots']<=200
    assert summary['inserted']==summary['active']+summary['absorbed']

    incoming={}
    for r in B:
        assert len(r)==52
        key=(r[2],r[3],r[6],r[7],r[5])
        assert key not in incoming,key
        incoming[key]=r
    action=collections.Counter()
    online_records=[];final_records=[];end=0;absorbed=0;mean_error=0.;cov_error=0.
    for directory in sorted((OUT/'archive/commits').iterdir(),key=lambda p:int(p.name)):
        for r in rows(directory/'records.txt'):
            kind=r[0]
            if kind=='belief':
                status=r[1];action[status]+=1
                key=tuple(r[2:7]);b=incoming.pop(key)
                assert r[7]==b[4] and r[8:11]==b[8:10]+[b[1]]
                mean_error=max(mean_error,max(abs(float(x)-float(y)) for x,y in zip(r[13:19],b[10:16])))
                cov_error=max(cov_error,max(abs(float(x)-float(y)) for x,y in zip(r[19:],b[16:])))
            elif kind=='online_incremental':online_records.append(r)
            elif kind=='finalized_incremental':final_records.append(r)
            elif kind=='absorbed':absorbed+=1
            elif kind=='end_incremental':end+=1
    assert not incoming and end==1
    assert cov_error==0 and mean_error<1e-12
    assert action['insert']==summary['inserted'] and action['replace']==summary['replaced']
    assert action['before_initialization']==summary['excluded']
    assert action['unsupported_at_end']==summary['unsupported_end']==0
    assert absorbed==summary['absorbed']
    assert summary['received']==sum(action.values())
    assert len(online_records)==summary['publications']
    assert len(final_records)==summary['finalized']
    assert action['duplicate']==0 and action['stale']==0

    online=dicts(OUT/'online_states.csv');final=dicts(OUT/'finalized_states.csv')
    assert len(online)==len(online_records)
    assert len(final)==len(final_records)
    for name,products,committed in [('online',online,online_records),('finalized',final,final_records)]:
        for p,r in zip(products,committed):
            expected=[r[1],r[2]]+r[4:]+[r[3]]
            assert list(p.values())==expected,(name,p['state_index'])
            vals=[float(p[k]) for k in ('px','py','pz','qx','qy','qz','qw','vx','vy','vz','wx','wy','wz')]
            assert all(math.isfinite(x) for x in vals)
            assert abs(sum(x*x for x in vals[3:7])-1)<1e-7
            assert int(p['availability_ns'])>=int(p['timestamp_ns'])
        indices=[int(p['state_index']) for p in products]
        assert indices==list(range(len(indices))),name
    times=[int(p['timestamp_ns']) for p in online]
    assert all(b-a==100_000_000 for a,b in zip(times,times[1:]))
    assert (times[-1]-times[0])*1e-9>=118.
    assert [r[1:3] for r in online_records]==[r[1:3] for r in final_records]
    path={}
    for name,products in [('online',online),('finalized',final)]:
        xyz=[[float(p[k]) for k in ('px','py','pz')] for p in products]
        path[name]=sum(math.dist(a,b) for a,b in zip(xyz,xyz[1:]))

    progress=dicts(OUT/'stream_progress.csv')
    assert len(progress)==len(events)
    assert all(int(p['event'])==i and p['kind']==e[0] for i,(p,e) in enumerate(zip(progress,events),1))
    cycles=[];bucket=0.;previous=0
    for p in progress:
        bucket+=float(p['service_seconds'])
        published=int(p['publications'])
        if published>previous:
            assert published==previous+1
            cycles.append(bucket);bucket=0.;previous=published
    service=stat(cycles)
    assert service['n']==len(online)
    assert service['p95']<=.8
    receipts=[json.loads(x) for x in (REC/'receipts.jsonl').read_text().splitlines()]
    assert len(receipts)==len(progress)
    latency=[(int(p['processed_wall_ns'])-r['wall_ns'])*1e-9 for p,r in zip(progress,receipts)]
    assert min(latency)>=0
    latency=stat(latency)
    monitored=[json.loads(x) for x in (CTL/'monitor.jsonl').read_text().splitlines()]
    assert monitored and max(m['max_queue'] for m in monitored)<1000
    published_during_source=max(m['published_poses'] for m in monitored)
    assert published_during_source>=max(1,len(online)-5)
    ages=[m['output_age_seconds'] for m in monitored if m.get('output_age_seconds') is not None]
    assert ages
    assert max(m['queued'] for m in monitored)<1000
    result=dict(passed=True,source_coverage=source_cov,transport=transport,
        summary=summary,actions=dict(action),mean_log_roundtrip_error=mean_error,
        covariance_payload_error=cov_error,online_path_m=path['online'],
        finalized_path_m=path['finalized'],fused_span_s=(times[-1]-times[0])*1e-9,
        support_cycle_seconds=service,receipt_to_processing_seconds=latency,
        monitor_output_age_s=stat(ages),published_during_source=published_during_source,
        max_monitor_queue=max(m['queued'] for m in monitored),
        protected_unchanged=len(manifest),
        claim='live 0.1x bounded pilot; fixed-lag incremental without stationarity certificate')
    RESULT.mkdir(parents=True,exist_ok=False)
    with (RESULT/'validation.json').open('x') as f:json.dump(result,f,indent=2)
    return result

def repeat():
    assert json.loads((RESULT/'validation.json').read_text())['passed']
    out=RESULT/'repeat_live'
    assert not out.exists()
    cmd=[str(EXE),'--event-stream',str(REC/'events.csv'),'--out',str(out),
         '--mode','sources','--timeline','record','--solver','isam2',
         '--support-policy','uniform_imu','--support-period-ns','100000000',
         '--receiver-startup-samples','59','--receiver-model','source_wnoa']
    # The capture ran in the Jammy receiver container. The host deliberately
    # lacks its Boost 1.74 runtime, so repeat in that same pinned container.
    shell='source /opt/ros/humble/setup.bash && exec '+shlex.join([
        'env','LD_LIBRARY_PATH='+str(LIB),'OMP_NUM_THREADS=1',
        'E2B_ENDPOINT_CACHE=0','E2_EXACT_LINEARIZATION_CACHE=0',
        'taskset','-c','2',*cmd])
    with (RESULT/'repeat_container_escalated.log').open('x') as f:
        started=time.monotonic()
        rc=subprocess.run(['docker','exec','ubuntu-jammy-humble-nvidia','bash','-lc',shell],
                          stdout=f,stderr=subprocess.STDOUT,timeout=1200).returncode
    assert rc==0
    files=['online_states.csv','finalized_states.csv','online_trajectory.tum',
           'finalized_trajectory.tum','summary.json']
    for name in files:assert sha(OUT/name)==sha(out/name),name
    archive=OUT/'archive';again=out/'archive';n=0
    for file in archive.rglob('*'):
        if file.is_file():
            target=again/file.relative_to(archive)
            assert target.is_file() and sha(file)==sha(target),str(file)
            n+=1
    assert n==sum(file.is_file() for file in again.rglob('*'))
    result=dict(passed=True,seconds=time.monotonic()-started,archive_files_byte_identical=n,
                numeric_products_byte_identical=files,receiver_executable_sha256=sha(EXE))
    with (RESULT/'repeat_validation.json').open('x') as f:json.dump(result,f,indent=2)
    return result

if __name__=='__main__':
    phase=sys.argv[1]
    print(json.dumps(check() if phase=='check' else repeat() if phase=='repeat' else None,indent=2))
