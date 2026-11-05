"""One saved-only matched-settings OFF/ON EVO report, after source cleanup."""
import importlib.metadata,importlib.util,json,sys
from pathlib import Path
import numpy as np
sys.dont_write_bytecode=True
H=Path(__file__).resolve().parent;E=H.parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
P=W/'runs'/H.name/'prepared';OLD=E/'eth1_alpha_bias_rw_full_off_cpv89_20261102';Q=W/'runs'/OLD.name/'prepared';O=H/'evaluation'
spec=importlib.util.spec_from_file_location('cp91_saved_helpers',OLD/'evaluate.py')
v=importlib.util.module_from_spec(spec);spec.loader.exec_module(v);b=v.b;d=b.d;a=d.a
audit=json.loads((P/'input_audit.json').read_text());epoch=audit['epoch_ns'];duration=audit['duration_s'];d.epoch=epoch;d.d39.epoch=epoch
def physical(c,body):
    ts,p,r=c;T=np.asarray(audit['T_'+body+'_prism'])
    return ts,p+np.einsum('nij,j->ni',r,T[:3,3]),r@T[:3,:3]
def refs(folder,ts,p,r,cuts):
    folder.mkdir();ids=np.concatenate(cuts);rp=folder/'reference.tum';rt=d.export(rp,d.curve(ts[ids],p[ids],r[ids]))
    rr,pairs,info=d.reference_segments(folder,ts,p,r,cuts);return rp,rt,rr,pairs,info
def main():
    assert importlib.metadata.version('evo')=='1.36.3' and duration==346.32
    assert d.read(H/'capture_complete.json')['passed'] and d.read(H/'closeout.json')['passed']
    assert d.read(H/'frame_validation.json')['passed'] and d.read(H/'frame_preflight.json')['passed']
    assert not O.exists() and not (H/'results.json').exists()
    for mod in (v,b,d,d.m,d.m.c,d.m.c.b,a,a.g,d.u,d.d39,d.d39.v):d.pin(Path(mod.__file__))
    for p in (Path(__file__),H/'PROTOCOL.txt',H/'profile_parity.json',P/'input_audit.json'):d.pin(p)
    record=next(r for r in d.m.c.b.ROWS if r[0]=='hesai_eth1');_,_,ref=d.m.c.b.load(record)
    reference=physical((ref.times,np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3]),'cpt7')
    curves={};native_k={};diagnostics={};events={}
    for mode,S in [('OFF',Q/'off_source'),('ON',P/'on_source')]:
        effective=d.read(S/'effective_mode_check.json');coverage=d.read(S/'source_coverage.json')
        assert effective['passed'] and coverage['passed']
        assert effective['glim']['glim_cbs_receive_external_beliefs'] is (mode=='ON')
        native_k[mode]=b.kimera(S,'alphasense')
        curves[mode+'_GLIM']=physical(a.load_curve(S/'glim_odometry.csv'),'cpt7')
        curves[mode+'_Kimera']=physical(native_k[mode],'alphasense')
        revision,events[mode]=d.d39.window_summary(S/'estimators.log',native_k[mode][0],b.native_csv(S/'kimera_odometry.csv'),native_k[mode])
        times=[];cov=[]
        for line in (S/'estimators.log').open():
            for marker,dest in [('KIMERA_OPTIMIZE_TIMING_ROW,',times),('KIMERA_CBS_REVISION_EDGE_ROW,',cov)]:
                if marker in line:dest.append(d.m.ANSI.sub('',line.split(marker,1)[1]).strip().split(','))
        t=np.asarray(times,float);C=np.asarray([x[28:] for x in cov],float).reshape(-1,6,6)
        assert np.isfinite(C).all() and np.linalg.eigvalsh(C).min()>0 and np.max(abs(C-C.transpose(0,2,1)))<1e-10
        frame=d.read(H/('frame_preflight.json' if mode=='OFF' else 'frame_validation.json'))
        diagnostics[mode]=dict(revisions=revision,optimize_ms=d.d39.stats(t[:,1]),coverage=coverage,
            transported_covariance=frame['transported_covariance'],
            Kimera_logged_native_covariance=dict(rows=len(C),rotation_trace_rad2=d.d39.stats(np.trace(C[:,:3,:3],axis1=1,axis2=2)),
                translation_trace_m2=d.d39.stats(np.trace(C[:,3:,3:],axis1=1,axis2=2))),
            frame_audit=frame['receiver_raw_match'],calibration_claim=False)
        for p in (S/'estimators.log',S/'kimera_odometry.csv',S/'glim_odometry.csv'):d.pin(p)
    first=max(c[0][0] for c in curves.values());last=min(c[0][-1] for c in curves.values())
    grid=np.arange(((int(first)+99_999_999)//100_000_000)*100_000_000,int(last)+1,100_000_000,dtype=np.int64)
    gp,gr,valid=a.sample(reference,grid,50_000_000);samples={}
    for key,c in curves.items():
        p,r,good=a.sample(c,grid,500_000_000);samples[key]=(p,r);valid &= good
    ids=np.flatnonzero(valid);assert len(ids)>100
    cuts=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1);O.mkdir();folder=O/'common'
    rp,rt,rfs,pairs,info=refs(folder,grid,gp,gr,cuts);common={};full={}
    for key,(p,r) in samples.items():
        common[key]=d.score_series(folder,key,grid,p,r,gp,gr,cuts,rfs,rp,rt)
        ts,p,r=curves[key];fg,fr,good=a.sample(reference,ts,50_000_000);assert good.all()
        fc=np.split(np.arange(len(ts)),np.flatnonzero(np.diff(ts)>500_000_000)+1)
        ff=O/('full_'+key);frp,frt,rr,pp,inf=refs(ff,ts,fg,fr,fc)
        full[key]=d.score_series(ff,key,ts,p,r,fg,fr,fc,rr,frp,frt)
        supported=sum(float((ts[ii[-1]]-ts[ii[0]])*1e-9) for ii in fc)
        full[key].update(first_s=float((ts[0]-epoch)*1e-9),last_s=float((ts[-1]-epoch)*1e-9),
            supported_s=supported,coverage_percent=100*supported/duration,gaps=len(fc)-1,segments=inf)
    improvement={role:100*(1-common['ON_'+role]['ATE_RMSE_m']/common['OFF_'+role]['ATE_RMSE_m']) for role in ('GLIM','Kimera')}
    supported=sum(float((grid[ii[-1]]-grid[ii[0]])*1e-9) for ii in cuts)
    result=dict(task='CP-V91',passed=True,evo_version='1.36.3',scope='one new full ETH1 split-IMU source CBS-ON; saved CP89 matched-settings OFF',
        common=common,full_saved=full,common_supported_s=supported,common_coverage_percent=100*supported/duration,
        common_rows=len(ids),common_segments=info,common_RPE_pairs=len(pairs),ATE_improvement_percent=improvement,
        diagnostics=diagnostics,checks=d.checks,no_fusion=True,live_bidirectional=True,both_persistent=True,
        kimera_native_only_outgoing_information=False,limitations=[
        'Only ON is a new capture; saved CP89 OFF has identical settings except reception.',
        'Separate IMUs do not remove peer-feedback correlation; persistent mixed-prior history remains.',
        'Bias RW candidate not certified Alpha calibration; covariance calibration and independence unproven.',
        'IE-TC shares CPT7,not independent prismGT; estimated path is not physical travel.',
        'Method1 newest absolute backend poses; native incremental ROS path not scored for Kimera.',
        'Native/common masks distinct; gaps>0.5s not bridged, incomplete supported coverage explicit.',
        'Actual bias/physical IMU residual histories unavailable; no nonlinear stationarity claim.',
        'No fusion or qualified split-body receiver adapter in this experiment.'])
    lines=['CP-V91 FULL ETH1 SPLIT-IMU CBS-ON vs saved CP89 OFF',
        'GLIM Hesai+CPT7/HG4930; Kimera stereo+Alphasense; BOTH persistent, unchanged CP89 profiles/binaries.',
        'One fresh live bidirectional ON capture, no fusion. OFF is saved matched-settings CP89, not a new OFF capture.',
        'Method1 first/newest absolute at the physical prism; EVO1.36.3 SE3/no-scale ATE and reference-distance0.5m RPE.',
        f'COMMON100ms: {len(ids)}rows, {supported:.6f}s/{duration}s ({100*supported/duration:.3f}%), {len(pairs)}RPEpairs.',
        'Role | ATE OFF -> ON m | improvement | tRPE OFF -> ON m | rRPE OFF -> ON deg | path OFF -> ON m | reference m']
    for role in ('GLIM','Kimera'):
        x=common['OFF_'+role];y=common['ON_'+role]
        lines.append(f"{role} | {x['ATE_RMSE_m']:.6f} -> {y['ATE_RMSE_m']:.6f} | {improvement[role]:+.2f}% | "
            f"{x['RPE']['translation_m']['statistics']['rmse']:.6f} -> {y['RPE']['translation_m']['statistics']['rmse']:.6f} | "
            f"{x['RPE']['rotation_deg']['statistics']['rmse']:.6f} -> {y['RPE']['rotation_deg']['statistics']['rmse']:.6f} | "
            f"{x['path_length_m']:.6f} -> {y['path_length_m']:.6f} | {x['reference_path_length_m']:.6f}")
    lines+=['','FULL-SAVED NATIVE cadence (different support/pairs; no gap chords):',
        'Role | rows | support % | ATE m | tRPE m | rRPE deg | estimated/reference path m | gaps']
    for key,row in full.items():
        lines.append(f"{key} | {row['rows']} | {row['coverage_percent']:.3f} | {row['ATE_RMSE_m']:.6f} | "
            f"{row['RPE']['translation_m']['statistics']['rmse']:.6f} | {row['RPE']['rotation_deg']['statistics']['rmse']:.6f} | "
            f"{row['path_length_m']:.6f}/{row['reference_path_length_m']:.6f} | {row['gaps']}")
    lines+=['','Detailed revision/timing/wire and native covariance/payload/input omissions: results.json.',*result['limitations']]
    d.PINS.update(a.HASHES);assert all(d.sha(Path(p))==v for p,v in d.PINS.items())
    d.save(H/'revision_events.json',events);d.save(O/'commands.json',d.commands);d.save(O/'input_sha256.json',d.PINS);d.save(H/'results.json',result)
    with (H/'RESULTS_REPORT.txt').open('x') as f:f.write('\n'.join(lines)+'\n')
    d.save(H/'report_verification.json',dict(passed=True,checks=d.checks,CLI_calls=len(d.commands),input_hashes=len(d.PINS),
        common_rows=len(ids),common_RPE_pairs=len(pairs),outputs={str(p):d.sha(p) for p in [H/'results.json',H/'RESULTS_REPORT.txt',O/'commands.json']}))
    print('\n'.join(lines))
if __name__=='__main__':main()
