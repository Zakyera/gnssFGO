"""Additional bounded context for the three edges; no estimator execution."""
import csv
import importlib.util
import json
from pathlib import Path
import re
import sys
sys.dont_write_bytecode=True
spec=importlib.util.spec_from_file_location('trace',Path(__file__).with_name('analyze.py'))
t=importlib.util.module_from_spec(spec);spec.loader.exec_module(t)
np=t.np
out=t.HERE/'outputs'; data=json.loads((out/'all_versions.json').read_text())
ref=t.geo.Reference(t.BAG)
answer={'initialization':{},'same_update_comparison':{},'reference_motion':[]}
for mode in ('off','on'):
    source=t.P/f'full_{mode}_source'
    profile=source/'kimera_profile/BackendParams.yaml';t.pin(profile)
    profile_text=profile.read_text()
    vals={k:float(re.search(r'^'+k+r':\s*([0-9.e+-]+)',profile_text,re.M)[1])
          for k in ('autoInitialize','initialVelocitySigma')}
    assert vals=={'autoInitialize':1.,'initialVelocitySigma':.001}
    native=source/'kimera_odometry.csv'; t.pin(native)
    with native.open() as f: first=next(csv.DictReader(f))
    vel=[float(first['field.twist.twist.linear.'+axis]) for axis in 'xyz']
    assert vel==[0.,0.,0.]
    log=source/'estimators.log';t.pin(log)
    warnings=[];first_injection=None
    for n,line in enumerate(log.open(),1):
        if 'InitializationFromImu: assumes' in line: warnings.append(dict(line=n,text=line.strip()))
        marker='KIMERA_CBS_MEAN_ALIGNMENT_ROW,'
        if marker in line and first_injection is None:
            a=line.split(marker,1)[1].strip().split(',')
            first_injection=dict(line=n,update=int(a[0]),sender_edge=list(map(int,a[1:3])),receiver_edge=list(map(int,a[3:5])))
    assert len(warnings)==1
    assert first_injection is None if mode=='off' else first_injection['update']==5
    answer['initialization'][mode]=dict(profile_values=vals,first_pose_ns=int(first['field.header.stamp']),
        first_published_velocity=vel,warning=warnings[0],first_external_alignment=first_injection)
    a={ (r['from_id'],r['revision']):r for r in data if r['mode']==mode}
    if mode=='off': off=a
    else:
        for i,j in t.EDGES:
            different=[]
            for rev in range(25):
                x,y=off[i,rev],a[i,rev]
                if not (np.array_equal(x['mean_log'],y['mean_log']) and np.array_equal(x['covariance'],y['covariance'])):
                    different.append(dict(revision=rev,update=y['update_id']))
            answer['same_update_comparison'][f'{i}->{j}']=dict(first_nonidentical=different[0])
for i,j in t.EDGES:
    r=next(r for r in data if r['mode']=='on' and r['from_id']==i and r['revision']==0)
    answer['reference_motion'].append(dict(edge=[i,j],start_s=r['physical_start_s'],end_s=r['physical_end_s'],
       duration_s=(r['end_ns']-r['start_ns'])*1e-9,
       chord_speed_mps=r['reference_distance_mm']/((r['end_ns']-r['start_ns'])*1e-6)))
stamp=answer['initialization']['on']['first_pose_ns']
answer['initial_reference_speed_estimates_mps']={str(dt):float(np.linalg.norm(ref.at(stamp+dt)[:3,3]-ref.at(stamp-dt)[:3,3])/(2*dt*1e-9)) for dt in (20_000_000,50_000_000)}
# Revisit precisely the existing confidence-ranking cohort; no new selection
# based on reference error. Only remove the two receiver-excluded stable IDs.
samples=t.HERE.parent/'covariance_ranking_full3_20261023/outputs/samples.csv'; t.pin(samples)
points=list(csv.DictReader(samples.open()))
answer['ranking_cohort']={}
for mode in ('off','on'):
    chosen=[r for r in points if r['sequence']=='SPX-3' and r['source']=='kimera_front' and r['mode']==mode and r['stage']=='near4s']
    assert len(chosen)==875
    groups={'all_received':chosen,'only_receiver_eligible':[r for r in chosen if int(r['from_id'])>=2]}
    for name,rows in groups.items():
        v=np.array([float(r['translation_trace_m2']) for r in rows]);e=np.array([float(r['translation_error_mm']) for r in rows])
        cutoff=np.sort(v)[(len(v)+1)//2-1];mask=v<=cutoff
        rms=float(np.sqrt(np.mean(e*e)));low=float(np.sqrt(np.mean(e[mask]*e[mask])))
        answer['ranking_cohort'][f'{mode}_{name}']=dict(n=len(e),kept=int(mask.sum()),all_rms_mm=rms,confident_half_rms_mm=low,ratio=low/rms)
for relative in ['src/initial/InitializationFromImu.cpp','include/kimera-vio/backend/VioBackend.h','src/backend/VioBackend.cpp']:
    t.pin(Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/Kimera-VIO')/relative)
t.pin(Path(__file__));answer['input_sha256']=t.HASHES
t.dump(out/'context.json',answer)
for path,expected in t.HASHES.items():assert t.geo.sha(Path(path))==expected
print(json.dumps({k:v for k,v in answer.items() if k!='input_sha256'},indent=2))
