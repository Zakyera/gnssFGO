"""Direct EVO for a frozen saved-event intervention, not a live OFF/ON run."""
import copy,importlib.metadata,importlib.util,sys
import numpy as np
from common import *
sys.dont_write_bytecode=True
S=E/'eth1_prior_fix_evaluation_cpv104_20261104'
spec=importlib.util.spec_from_file_location('cp106_saved_helpers',S/'evaluate.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
d=b.d;a=d.a
assert importlib.metadata.version('evo')=='1.36.3'
assert read(H/'validation.json')['passed'] and read(H/'baseline_parity.json')['passed']
assert not (H/'evaluation').exists() and not (H/'results.json').exists()
O=H/'evaluation';O.mkdir()
audit=d.read(P/'input_audit.json');epoch=audit['epoch_ns'];d.epoch=epoch;d.d39.epoch=epoch
old=d.read(S/'results.json');data=np.load(d.pin(S/'evaluation/comparison.npz'))
grid=data['timestamp_ns'];valid=data['common_valid'];gp=data['reference_position'];gr=data['reference_rotation']
ids=np.flatnonzero(valid);cuts=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
assert len(ids)==2966 and len(cuts)==3
curves={}
for mode in ('baseline','frozen_kimera'):
    for product,filename in [('online','online_trajectory.tum'),('delayed','finalized_trajectory.tum')]:
        curves[mode+'_'+product]=b.physical(a.load_curve(d.pin(H/mode/filename)),'cpt7')
folder=O/'common';rp,rt,refs,pairs,info=b.refs(folder,grid,gp,gr,cuts)
assert pairs==d.read(S/'evaluation/common/pair_indices.json') and len(pairs)==365
common={};full={};arrays=dict(timestamp_ns=grid,common_valid=valid,reference_position=gp,reference_rotation=gr)
for key,curve in curves.items():
    pos,rot,good=a.sample(curve,grid,500_000_000);assert good[valid].all()
    common[key]=d.score_series(folder,key,grid,pos,rot,gp,gr,cuts,refs,rp,rt)
    arrays[key+'_position']=pos;arrays[key+'_rotation']=rot
    product=key.rsplit('_',1)[1];oldkey='CP103_ON_fusion_'+product
    ts,fp,fr=curve
    originalref=d.file_interface.read_tum_trajectory_file(d.pin(S/('evaluation/full_'+oldkey+'/reference.tum')))
    assert len(originalref.timestamps)==len(ts)==3456
    assert np.max(abs(originalref.timestamps-(ts-epoch)*1e-9))<1e-8
    fg=originalref.positions_xyz;fgr=np.asarray(originalref.poses_se3)[:,:3,:3]
    seg=[np.arange(len(ts))];fdir=O/('full_'+key)
    frp,frt,frefs,fpairs,finfo=b.refs(fdir,ts,fg,fgr,seg)
    full[key]=d.score_series(fdir,key,ts,fp,fr,fg,fgr,seg,frefs,frp,frt)
    full[key].update(first_s=float((ts[0]-epoch)*1e-9),last_s=float((ts[-1]-epoch)*1e-9),
        supported_s=float((ts[-1]-ts[0])*1e-9),coverage_percent=100*float((ts[-1]-ts[0])*1e-9)/346.32,
        maximum_gap_s=float(np.diff(ts).max()*1e-9),gaps=0)
    assert fpairs==d.read(S/('evaluation/full_'+oldkey+'/pair_indices.json'))
    if key.startswith('baseline'):
        for group,rows in [('common',common),('full_saved',full)]:
            expected=old[group][oldkey]
            actual=rows[key]
            for x,y in zip(b.metric_values(actual),b.metric_values(expected)):assert abs(x-y)<1e-8,(key,group,x,y)
np.savez_compressed(O/'comparison.npz',**arrays)
effects={}
for group,rows in [('common',common),('full_saved',full)]:
    effects[group]={}
    for product in ('online','delayed'):
        orig=rows['baseline_'+product];v=rows['frozen_kimera_'+product]
        effects[group][product]=dict(ATE_improvement_percent=100*(1-v['ATE_RMSE_m']/orig['ATE_RMSE_m']),
            translation_RPE_improvement_percent=100*(1-v['RPE']['translation_m']['statistics']['rmse']/orig['RPE']['translation_m']['statistics']['rmse']),
            rotation_RPE_improvement_percent=100*(1-v['RPE']['rotation_deg']['statistics']['rmse']/orig['RPE']['rotation_deg']['statistics']['rmse']))
for mod in (b,b.b,d,d.m,d.m.c,d.m.c.b,a,a.g,d.u,d.d39,d.d39.v):d.pin(mod.__file__)
for p in [Path(__file__),H/'PROTOCOL.txt',H/'prepared.json',H/'baseline_parity.json',H/'validation.json',H/'variant_events.csv']:
    d.pin(p)
import evo
for p in Path(evo.__file__).parent.rglob('*.py'):d.pin(p)
d.PINS.update(a.HASHES)
d.save(O/'commands.json',d.commands);d.save(O/'input_sha256.json',d.PINS)
assert len(d.commands)==24,len(d.commands)
save('results.json',dict(passed=True,finished=now(),evo_version='1.36.3',method=1,
    reference='IE-TC; shared CPT7, not independent prism ground truth',
    physical_point='calibrated prism',alignment='SE3 no scale',
    common=common,full_saved=full,effects=effects,common_rows=len(ids),common_RPE_pairs=len(pairs),
    common_coverage=old['common_coverage'],common_segments=info,
    baseline_scores_reproduce_CP104=True,direct_EVO_calls=len(d.commands),verification=d.checks,
    receiver_replays=2,source_runs=0,not_a_new_live_OFF_ON=True,
    intervention='Only subsequent Kimera covariance held at first accepted full6x6; all mean revisions and event order preserved',
    interpretation_limit='Conditional effect of covariance schedule in one fixed saved stream; not production qualification or calibration proof'))
print(json.dumps(dict(common={k:b.metric_values(v) for k,v in common.items()},effects=effects),indent=2))
