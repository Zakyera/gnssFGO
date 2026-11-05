"""Frozen-mask direct EVO and local fit-sensitivity diagnostics, no receiver run."""
import importlib.metadata,importlib.util,sys
import numpy as np
from common import *
sys.dont_write_bytecode=True
sys.path.insert(0,str(S))
spec=importlib.util.spec_from_file_location('cp108_helpers',S/'evaluate.py');b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b)
d=b.d;a=d.a
assert read(H/'validation.json')['passed'] and importlib.metadata.version('evo')=='1.36.3'
assert not (H/'evaluation').exists();O=H/'evaluation';O.mkdir()
old=d.read(S/'results.json');data=np.load(d.pin(S/'evaluation/comparison.npz'));epoch=d.read(P/'input_audit.json')['epoch_ns'];d.epoch=epoch;d.d39.epoch=epoch
grid=data['timestamp_ns'];valid=data['common_valid'];gp=data['reference_position'];gr=data['reference_rotation'];t=(grid-epoch)*1e-9
ids=np.flatnonzero(valid);cuts=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1);assert len(ids)==3532 and len(cuts)==2
folder=O/'common';rp,rt,refs,pairs,info=b.reference_files(folder,grid,gp,gr,cuts)
assert pairs==d.read(S/'evaluation/common/pair_indices.json') and len(pairs)==349
common={};full={};diagnostic={};arrays=dict(timestamp_ns=grid,common_valid=valid,reference_position=gp,reference_rotation=gr)
def metrics(x):return [x['ATE_RMSE_m'],x['path_length_m'],x['reference_path_length_m'],x['RPE']['translation_m']['statistics']['rmse'],x['RPE']['rotation_deg']['statistics']['rmse']]
for mode in MODES:
    for product,filename in [('online','online_trajectory.tum'),('delayed','finalized_trajectory.tum')]:
        key=mode+'_'+product;oldkey='on_fusion_'+product
        curve=b.physical(a.load_curve(d.pin(H/mode/filename)),'cpt7');pos,rot,good=a.sample(curve,grid,500_000_000);assert good[valid].all()
        common[key]=d.score_series(folder,key,grid,pos,rot,gp,gr,cuts,refs,rp,rt);arrays[key+'_position']=pos;arrays[key+'_rotation']=rot
        ts,fp,fr=curve;originalref=d.file_interface.read_tum_trajectory_file(d.pin(S/('evaluation/full_'+oldkey+'/reference.tum')))
        assert len(ts)==len(originalref.timestamps)==3545 and np.max(abs(originalref.timestamps-(ts-epoch)*1e-9))<1e-8
        fg=originalref.positions_xyz;fgr=np.asarray(originalref.poses_se3)[:,:3,:3];seg=[np.arange(len(ts))]
        frp,frt,frefs,fpairs,finfo=b.reference_files(O/('full_'+key),ts,fg,fgr,seg)
        assert fpairs==d.read(S/('evaluation/full_'+oldkey+'/pair_indices.json'))
        full[key]=d.score_series(O/('full_'+key),key,ts,fp,fr,fg,fgr,seg,frefs,frp,frt)
        full[key].update(first_s=float((ts[0]-epoch)*1e-9),last_s=float((ts[-1]-epoch)*1e-9),supported_s=float((ts[-1]-ts[0])*1e-9),maximum_gap_s=float(np.diff(ts).max()*1e-9))
        if mode=='baseline':
            for actual,expected in ((common[key],old['common'][oldkey]),(full[key],old['full_saved'][oldkey])):assert max(abs(x-y) for x,y in zip(metrics(actual),metrics(expected)))<1e-8
        own=common[key]['alignment'];frozen=old['common'][oldkey]['alignment'];eo=pos@np.array(own['rotation']).T+own['translation']-gp;ef=pos@np.array(frozen['rotation']).T+frozen['translation']-gp
        local=np.full(len(grid),np.nan)
        for ix in cuts:
            if len(ix)<51:continue
            aa,bb=ix[:-50],ix[50:];de=np.einsum('nji,nj->ni',rot[aa],pos[bb]-pos[aa]);dg=np.einsum('nji,nj->ni',gr[aa],gp[bb]-gp[aa]);local[bb]=np.linalg.norm(de-dg,axis=1)
        arrays[key+'_own_error']=eo;arrays[key+'_frozen_error']=ef;arrays[key+'_relative5_m']=local
        windows=[]
        for lo,hi in ((0,50),(50,70),(70,130),(138,146),(170,178),(275,290),(290,310),(298,304),(310,350)):
            ix=valid&(t>=lo)&(t<hi);windows.append(dict(start_s=lo,end_s=hi,n=int(ix.sum()),own_global_RMS_m=float(np.sqrt(np.mean(np.sum(eo[ix]**2,axis=1)))),baseline_alignment_RMS_m=float(np.sqrt(np.mean(np.sum(ef[ix]**2,axis=1)))),relative5_RMS_m=float(np.sqrt(np.nanmean(local[ix]**2)))))
        diagnostic[key]=dict(windows=windows,baseline_alignment_whole_RMS_m=float(np.sqrt(np.mean(np.sum(ef[valid]**2,axis=1)))))
        print(key,metrics(common[key]),flush=True)
effects={}
for product in ('online','delayed'):
    base=common['baseline_'+product]['ATE_RMSE_m'];effects[product]={mode:100*(1-common[mode+'_'+product]['ATE_RMSE_m']/base) for mode in MODES[1:]}
for mod in (b,d,d.m,d.m.c,d.m.c.b,a,a.g,d.u,d.d39,d.d39.v):d.pin(mod.__file__)
for p in [Path(__file__),H/'PROTOCOL.txt',H/'prepared.json',H/'baseline_parity.json',H/'validation.json']+[H/(m+'_events.csv') for m in MODES[1:]]:d.pin(p)
d.PINS.update(a.HASHES);d.save(O/'commands.json',d.commands);d.save(O/'input_sha256.json',d.PINS);assert len(d.commands)==48
np.savez_compressed(O/'comparison.npz',**arrays)
save('results.json',dict(passed=True,at=now(),evo_version='1.36.3',common=common,full_saved=full,local_diagnostics=diagnostic,ATE_improvement_percent=effects,common_rows=3532,common_RPE_pairs=349,common_coverage=old['common_coverage'],baseline_CP108_reproduced=True,direct_EVO_calls=48,verification=d.checks,source_runs=0,receiver_replays=4,not_live=True,intervention='Regional Kimera late revisions at age2.5s; covariance/mean/both factorial holds',limits='Conditional saved-stream ablation, not calibrated uncertainty or production fix. IE-TC sharesCPT7.'))
print(json.dumps(effects,indent=2))
