"""Saved-only Method1 EVO full-window and prior-pilot-prefix report."""
import importlib.metadata,importlib.util,json,sys
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';S=P/'off_source';O=H/'evaluation';OLD=E/'eth1_alpha_bias_rw_off60_cpv88_20261102'
spec=importlib.util.spec_from_file_location('cp89_saved_helpers',OLD/'evaluate.py')
b=importlib.util.module_from_spec(spec);spec.loader.exec_module(b);d=b.d;a=d.a
audit=json.loads((P/'input_audit.json').read_text());epoch=audit['epoch_ns'];duration=audit['duration_s']
d.epoch=epoch;d.d39.epoch=epoch
def physical(c,body):
    ts,p,r=c;T=np.asarray(audit['T_'+body+'_prism'])
    return ts,p+np.einsum('nij,j->ni',r,T[:3,3]),r@T[:3,:3]
def blocks(grid,good):
    ids=np.flatnonzero(good);assert len(ids)>100
    cuts=(np.diff(ids)>1)|(np.diff(grid[ids])>100_000_001)
    return np.split(ids,np.flatnonzero(cuts)+1)
def reference_files(folder,grid,gp,gr,segments):
    folder.mkdir();ids=np.concatenate(segments);rp=folder/'reference.tum'
    rt=d.export(rp,d.curve(grid[ids],gp[ids],gr[ids]))
    refs,pairs,info=d.reference_segments(folder,grid,gp,gr,segments)
    return rp,rt,refs,pairs,info
def main():
    assert importlib.metadata.version('evo')=='1.36.3' and duration==346.32
    assert d.read(H/'capture_complete.json')['passed'] and d.read(H/'closeout.json')['passed']
    assert not O.exists() and not (H/'results.json').exists()
    for mod in (b,d,d.m,d.m.c,d.m.c.b,a,a.g,d.u,d.d39,d.d39.v):d.pin(Path(mod.__file__))
    for p in (Path(__file__),H/'PROTOCOL.txt',H/'profile_parity.json',P/'input_audit.json',S/'effective_mode_check.json',S/'source_coverage.json'):d.pin(p)
    old=d.read(OLD/'results.json')
    record=next(r for r in d.m.c.b.ROWS if r[0]=='hesai_eth1');_,_,ref=d.m.c.b.load(record)
    reference=physical((ref.times,np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3]),'cpt7')
    kg=b.kimera(S,'alphasense')
    curves={'GLIM_CPT7':physical(a.load_curve(S/'glim_odometry.csv'),'cpt7'),'Kimera_Alphasense':physical(kg,'alphasense')}
    first=max(c[0][0] for c in curves.values());last=min(c[0][-1] for c in curves.values())
    grid=np.arange(((int(first)+99_999_999)//100_000_000)*100_000_000,int(last)+1,100_000_000,dtype=np.int64)
    gp,gr,valid=a.sample(reference,grid,50_000_000);samples={}
    for name,c in curves.items():
        p,r,good=a.sample(c,grid,500_000_000);valid &= good;samples[name]=(p,r)
    segments=blocks(grid,valid);ids=np.concatenate(segments);O.mkdir()
    folder=O/'common';rp,rt,refs,pairs,info=reference_files(folder,grid,gp,gr,segments)
    common={};full={}
    for name,(p,r) in samples.items():
        common[name]=d.score_series(folder,name,grid,p,r,gp,gr,segments,refs,rp,rt)
        ts,p,r=curves[name];fp,fr,good=a.sample(reference,ts,50_000_000);assert good.all()
        cuts=np.split(np.arange(len(ts)),np.flatnonzero(np.diff(ts)>500_000_000)+1)
        ff=O/('full_'+name);frp,frt,rfs,prs,inf=reference_files(ff,ts,fp,fr,cuts)
        full[name]=d.score_series(ff,name,ts,p,r,fp,fr,cuts,rfs,frp,frt)
        supported=sum(float((ts[ii[-1]]-ts[ii[0]])*1e-9) for ii in cuts)
        full[name].update(first_s=float((ts[0]-epoch)*1e-9),last_s=float((ts[-1]-epoch)*1e-9),
            supported_s=supported,coverage_percent=100*supported/duration,gaps=len(cuts)-1,segments=inf)
    # A full-length alignment cannot be compared directly with the pilot's alignment.
    # Rescore just the new Kimera prefix on the exact saved pilot mask.
    oldref=d.file_interface.read_tum_trajectory_file(d.pin(OLD/'evaluation/common/reference.tum'))
    pg=np.rint(oldref.timestamps*1e9).astype(np.int64)+epoch
    pp,pr,pvalid=a.sample(curves['Kimera_Alphasense'],pg,500_000_000)
    rg,rr,rgood=a.sample(reference,pg,50_000_000);pvalid &= rgood
    prefix=dict(identical_mask=bool(pvalid.all()),scored=False)
    if pvalid.all():
        pf=O/'pilot_prefix';ps=blocks(pg,pvalid);prp,prt,prefs,ppairs,pinfo=reference_files(pf,pg,rg,rr,ps)
        assert np.max(abs(prt.positions_xyz-oldref.positions_xyz))<1e-8
        assert np.max(abs(prt.orientations_quat_wxyz-oldref.orientations_quat_wxyz))<1e-8
        priorpairs=d.read(OLD/'evaluation/common/pair_indices.json');assert len(priorpairs)==len(ppairs)
        for x,y in zip(priorpairs,ppairs):
            for k in ('segment','start_ns','end_ns'):assert x[k]==y[k]
            assert abs(x['reference_distance_m']-y['reference_distance_m'])<1e-8
        prefix.update(scored=True,new=d.score_series(pf,'Kimera_new_prefix',pg,pp,pr,rg,rr,ps,prefs,prp,prt),
            saved_CP88=old['common']['Kimera_Alphasense_bias_rw'],rows=len(pg),pairs=len(ppairs))
    else:prefix['omission']='New prefix does not cover exact pilot mask; no unlike-mask comparison.'
    revision,events=d.d39.window_summary(S/'estimators.log',kg[0],b.native_csv(S/'kimera_odometry.csv'),kg)
    d.save(H/'revision_events.json',events);timing=[];cov=[]
    for line in (S/'estimators.log').open():
        for marker,dest in [('KIMERA_OPTIMIZE_TIMING_ROW,',timing),('KIMERA_CBS_REVISION_EDGE_ROW,',cov)]:
            if marker in line:dest.append(d.m.ANSI.sub('',line.split(marker,1)[1]).strip().split(','))
    times=np.asarray(timing,float);mat=np.asarray([x[28:] for x in cov],float).reshape(-1,6,6)
    assert np.isfinite(mat).all() and np.linalg.eigvalsh(mat).min()>0 and np.max(abs(mat-mat.transpose(0,2,1)))<1e-10
    diag=dict(revisions=revision,optimize_ms=d.d39.stats(times[:,1]),covariance=dict(rows=len(mat),
        rotation_trace_rad2=d.d39.stats(np.trace(mat[:,:3,:3],axis1=1,axis2=2)),
        translation_trace_m2=d.d39.stats(np.trace(mat[:,3:,3:],axis1=1,axis2=2)),calibration_claim=False))
    supported=sum(float((grid[ii[-1]]-grid[ii[0]])*1e-9) for ii in segments)
    result=dict(task='CP-V89',passed=True,scope='full346.32s ETH1 split-IMU CBS-OFF validation of CP88 candidate',
        common=common,full_saved=full,common_supported_s=supported,common_coverage_percent=100*supported/duration,
        common_rows=len(ids),common_segments=info,common_RPE_pairs=len(pairs),pilot_prefix=prefix,
        diagnostics=diag,CP88_diagnostics=old['diagnostics'],checks=d.checks,no_CBS_ON=True,no_fusion=True,
        limitations=['Not a matched full CPT7 or original-Alpha control','Bias RW candidate not certified Alpha calibration',
        'IE-TC shares CPT7,not independent prismGT; path is estimated polyline','Native/common masks distinct; gaps>0.5s not bridged',
        'Actual bias/physical IMU residual histories unavailable','Covariance consistency and nonlinear stationarity not certified'])
    lines=['CP-V89 FULL ETH1 CBS-OFF: GLIM CPT7 / Kimera Alphasense,CP88 biasRW unchanged.',
        'Method1 first/newest absolute at prism;EVO1.36.3 SE3/no-scale ATE,reference-distance0.5m RPE.',
        f'COMMON100ms support{supported:.6f}s/{duration}s;{len(ids)}rows,{len(pairs)}RPEpairs.',
        'Estimator | ATE m | tRPE m | rRPE deg | estimated path m | reference path m']
    for k,row in common.items():lines.append(f"{k} | {row['ATE_RMSE_m']:.6f} | {row['RPE']['translation_m']['statistics']['rmse']:.6f} | {row['RPE']['rotation_deg']['statistics']['rmse']:.6f} | {row['path_length_m']:.6f} | {row['reference_path_length_m']:.6f}")
    lines+=['Full-saved cadence,CP88 prefix and revision/timing/covariance:results.json.',
        'No fusion/CBS-ON;prior60s controls are NOT full-length controls. IE-TC is not independent prismGT.',
        'Report-process pass does not certify estimator accuracy,uncertainty or further-run authorization.']
    for p in [S/'glim_odometry.csv',S/'kimera_odometry.csv',S/'estimators.log']:d.pin(p)
    d.PINS.update(a.HASHES);assert all(d.sha(Path(p))==v for p,v in d.PINS.items())
    d.save(O/'commands.json',d.commands);d.save(O/'input_sha256.json',d.PINS);d.save(H/'results.json',result)
    with (H/'RESULTS_REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
    d.save(H/'report_verification.json',dict(passed=True,checks=d.checks,CLI_calls=len(d.commands),input_hashes=len(d.PINS),
        pilot_prefix_identical=prefix['identical_mask'],outputs={str(p):d.sha(p) for p in [H/'results.json',H/'RESULTS_REPORT.txt',O/'commands.json']}))
    print('\n'.join(lines))
if __name__=='__main__':main()
