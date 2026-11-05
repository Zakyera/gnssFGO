"""All seven METHOD2 pairs; reconstruction and scoring never run sources."""
import collections, csv, importlib.util, json, sys
from decimal import Decimal
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
from scipy.spatial.transform import Rotation as R

H=Path(__file__).resolve().parent;E=H.parent
def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
base=module('cpv26',E/'leica2_all_incremental_cpv26_20261028/analyze.py')
fullmod=module('cpv27',E/'newest_absolute_comparison_cpv27_20261028/analyze.py')
c=base.c;a=c.a
N=lambda x:np.linalg.norm(x,axis=-1)
pin=base.pin;load=base.load;sha=base.sha
H10=E/'first_published_comparison_cpv10_20261026/outputs'
H26=E/'leica2_all_incremental_cpv26_20261028'
H23=E/'leica2_incremental_full_cpv23_20261027'
ROLES=('glim','kimera','fusion')
SOURCE_G=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/local_revision_covariance_cpi1_20261019/cbs_bridge.cpp')
SOURCE_K=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_current_chart_fl4q_20261016/VioBackend.cpp')

def verify_chain(ts,g,prev):
    o=base.chain(g,prev)
    qr=R.from_matrix(g[0,:3,:3]);point=g[0,:3,3].copy();ind=[base.matrix(point,qr.as_quat())]
    for k in range(1,len(g)):
        rp=R.from_matrix(prev[k,:3,:3]);rc=R.from_matrix(g[k,:3,:3])
        point=point+qr.apply(rp.inv().apply(g[k,:3,3]-prev[k,:3,3]));qr=qr*rp.inv()*rc
        ind.append(base.matrix(point,qr.as_quat()))
    control=base.chain(g,np.concatenate([g[:1],g[:-1]]))
    correction=np.asarray([g[k]@base.inverse(o[k]) for k in range(len(g))])
    po,_=a.prism(o[:,:3,3],o[:,:3,:3]);pg,_=a.prism(g[:,:3,3],g[:,:3,:3]);pp,_=a.prism(prev[:,:3,3],prev[:,:3,:3])
    checks=dict(rows=len(ts),independent_quaternion_chain_max=float(np.max(abs(o-np.asarray(ind)))),
                telescoping_max=float(np.max(abs(control-g))),graph_reconstruction_max=float(np.max(abs(correction@o-g))),
                prism_edge_length_max_m=float(np.max(abs(N(np.diff(po,axis=0))-N(pg[1:]-pp[1:])))))
    assert max(v for k,v in checks.items() if k!='rows')<1e-8
    arrays=dict(timestamp_ns=ts,graph_body_T=g,previous_same_update_body_T=prev,chained_body_T=o,graph_T_odometry=correction)
    return (ts,o[:,:3,3],o[:,:3,:3]),arrays,checks

def native_matrices(path):
    ts,p,r=a.load_curve(pin(path));g=np.repeat(np.eye(4)[None],len(ts),axis=0);g[:,:3,3]=p;g[:,:3,:3]=r
    return ts,g

def glim(source):
    logs=list((source/'ros_logs').glob('**/glim_rosnode*-stdout.log'));assert len(logs)==1
    selected={};counts=collections.Counter();last=-1
    with pin(logs[0]).open() as f:
        for line in f:
            if 'GLIM_CBS_ACTIVE_POSE_ROW,' not in line:continue
            row=line.split('GLIM_CBS_ACTIVE_POSE_ROW,',1)[1].split('\x1b')[0].strip().split(',')
            assert len(row)==12 and row[0]=='g'
            k,i=int(row[1]),int(row[3]);assert k>=last;last=k;counts[k]+=1
            if i in (k-1,k):assert (k,i) not in selected;selected[k,i]=row
    with pin(source/'glim_odometry.csv').open() as f:pub=list(csv.DictReader(f))
    ts,native=native_matrices(source/'glim_odometry.csv')
    assert sorted(counts)==list(range(len(pub))) and [int(x['field.header.seq']) for x in pub]==list(range(len(pub)))
    assert np.all(np.diff(ts)>0)
    g=[];prev=[];rounded=[]
    for k,p in enumerate(pub):
        assert p['field.header.frame_id']=='map' and p['field.child_frame_id']=='cpt7_imu'
        row=selected[k,k];values=np.array(row[5:],float)
        assert np.isfinite(values).all() and abs(N(values[3:])-1)<1e-6
        pose=base.matrix(values[:3],values[3:]);g.append(pose)
        rounded.append(abs(int(Decimal(row[4])*10**9)-int(ts[k])))
        assert Decimal(row[2])>=Decimal(row[4])
        if k==0:prev.append(pose)
        else:
            old=selected[k,k-1];assert old[2]==row[2]
            rounded.append(abs(int(Decimal(old[4])*10**9)-int(ts[k-1])))
            v=np.array(old[5:],float);assert np.isfinite(v).all() and abs(N(v[3:])-1)<1e-6
            prev.append(base.matrix(v[:3],v[3:]))
    g,prev=map(np.asarray,(g,prev));parity=float(np.max(abs(g-native)))
    assert parity<1e-12 and max(rounded)<1000
    curve,arrays,checks=verify_chain(ts,g,prev)
    checks.update(active_pose_rows=sum(counts.values()),publication_pose_max=parity,rounded_stamp_max_error_ns=max(rounded),missing_pairs=0)
    return curve,arrays,checks

def fusion(case,mode,original):
    pub=H/'replay_evidence'/(case+'_'+mode+'_publications.csv')
    assert load(H/(case+'_'+mode+'_parity.json'))['pass_']
    with pin(pub).open() as f:rows=list(csv.DictReader(f))
    groups={}
    for row in rows:groups.setdefault(int(row['newest']),[]).append(row)
    ts,native=native_matrices(original)
    assert sorted(groups)==list(range(len(ts)))
    g=[];prev=[];av=[]
    for k,ss in groups.items():
        assert [int(s['index']) for s in ss]==([0] if k==0 else [k-1,k])
        assert len({s['event'] for s in ss})==1
        assert all(int(s['physical_ns'])==int(ts[k]) for s in ss)
        assert all(int(s['availability_ns'])>=int(ts[k]) for s in ss)
        def pose(s):return base.matrix([float(s[x]) for x in ('px','py','pz')],[float(s[x]) for x in ('qx','qy','qz','qw')])
        g.append(pose(ss[-1]));prev.append(pose(ss[0]));av.append(int(ss[-1]['availability_ns']))
    g,prev=map(np.asarray,(g,prev));parity=float(np.max(abs(g-native)))
    assert parity<1e-12 and np.all(np.diff(ts)==100000000) and np.all(np.diff(av)>=0)
    curve,arrays,checks=verify_chain(ts,g,prev);checks.update(publication_pose_max=parity,missing_pairs=0)
    arrays['availability_ns']=np.array(av,np.int64)
    return curve,arrays,checks

def write_tum(path,curve):
    t,p,r=curve
    with path.open('x') as f:
        for stamp,pos,q in zip(t,p,R.from_matrix(r).as_quat()):
            f.write(f'{stamp//10**9}.{stamp%10**9:09d} '+' '.join(format(x,'.17g') for x in [*pos,*q])+'\n')

def reconstruct_glim(case):
    base.PINS.clear();old=load(H10/(case+'.json'))
    target=H/(case+'_glim_checks.json');assert not target.exists()
    checks={};arrays={};curves={}
    for mode in ('off','on'):
        source=Path(old['input_products'][mode+'_kimera']['path']).parent
        manifest=load(source.parent/'protected_manifest.json')
        for path in (SOURCE_G,SOURCE_K):assert sha(pin(path))==manifest[str(path)]
        settings=load(source/'effective_mode_check.json')
        assert settings['passed'] and '--no_incremental_pose=false' in json.dumps(settings)
        assert settings['glim']['glim_cbs_export_endpoint_audit_enable'] is True
        curve,aa,checks[mode]=glim(source)
        assert len(curve[0])==old['input_products'][mode+'_glim_initial']['rows']
        curves[mode]=curve;arrays.update({mode+'_'+k:v for k,v in aa.items()})
    for path in (Path(__file__),H/'PROTOCOL.txt',Path(base.__file__)):pin(path)
    assert all(sha(p)==v for p,v in base.PINS.items())
    for mode,curve in curves.items():write_tum(H/(case+'_'+mode+'_glim.tum'),curve)
    np.savez_compressed(H/(case+'_glim_products.npz'),**arrays)
    a.save(target,dict(checks=checks,input_pins=base.PINS,synthetic=base.synthetic_checks()))
    print(json.dumps(dict(stage='glim_reconstructed',case=case,checks=checks)),flush=True)

def analyze(row):
    case,sensor,sequence,_,_,duration,epoch=row
    base.PINS.clear();a.HASHES.clear();old=load(H10/(case+'.json'))
    assert not (H/(case+'.json')).exists()
    saved=np.load(pin(H10/(case+'.npz')));grid,valid=saved['timestamp_ns'],saved['common_valid']
    _,_,ref=c.b.load(row)
    reference=ref.times,np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3]
    gp,gr,gtok=a.sample(reference,grid,50_000_000);gp,gr=a.prism(gp,gr)
    assert np.array_equal(gp,saved['reference_position']) and np.array_equal(gr,saved['reference_rotation'])
    curves={};checks={};products={};paths={}
    prior=load(H26/'results.json') if case=='velodyne_leica2' else None
    if prior:assert load(H26/'closeout.json')
    else:
        assert load(H/'replay_complete.json')['pass_']
        gg=load(H/(case+'_glim_checks.json'));checks['glim']=gg['checks']
        for p,v in gg['input_pins'].items():assert sha(pin(p))==v
    for mode in ('off','on'):
        src=Path(old['input_products'][mode+'_kimera']['path']).parent
        setting=load(src/'effective_mode_check.json');assert setting['passed'] and '--no_incremental_pose=false' in json.dumps(setting)
        native=pin(src/'kimera_odometry.csv');assert sha(native)==old['input_products'][mode+'_kimera']['sha256']
        curves[mode+'_kimera']=a.load_curve(native);paths[mode+'_kimera']=str(native)
        gpath=H26/(mode+'_glim_chained.tum') if prior else H/(case+'_'+mode+'_glim.tum')
        curves[mode+'_glim']=a.load_curve(pin(gpath));paths[mode+'_glim']=str(gpath)
        orig=Path(old['input_products'][mode+'_fusion_online']['path']);assert sha(pin(orig))==old['input_products'][mode+'_fusion_online']['sha256']
        if prior:
            cp23=load(H23/'reconstruction_checks.json')
            fpath=Path(cp23['cases'][mode]['chained_trajectory']);assert sha(pin(fpath))==cp23['cases'][mode]['chained_sha256']
            curves[mode+'_fusion']=a.load_curve(fpath);checks[mode+'_fusion_reused']=cp23['cases'][mode]
        else:
            curve,aa,checks[mode+'_fusion']=fusion(case,mode,orig)
            curves[mode+'_fusion']=curve;products.update({mode+'_'+k:v for k,v in aa.items()})
            fpath=H/(case+'_'+mode+'_fusion.tum')
        paths[mode+'_fusion']=str(fpath)
    full={};common={};reproduced={};arrays=dict(timestamp_ns=grid,common_valid=valid,reference_position=gp,reference_rotation=gr)
    current_valid=gtok.copy()
    for key,curve in curves.items():
        p,r,good=a.sample(curve,grid,500_000_000);current_valid &= good;assert good[valid].all()
        ep,er=a.prism(p,r);common[key],res=c.metrics(ep,er,gp,gr,valid,grid)
        full[key]=fullmod.full_metric(curve,reference,epoch,duration)
        arrays[key+'_position']=ep;arrays[key+'_rotation']=er
        errors=np.full(len(grid),np.nan);errors[valid]=N(res);arrays[key+'_aligned_error_m']=errors
        if key.endswith('_kimera'):
            dc=abs(common[key]['ATE_RMSE_m']-old['common'][key]['ATE_RMSE_m'])
            df=abs(full[key]['ATE_RMSE_m']-old['full_saved'][key]['ATE_RMSE_m'])
            assert max(dc,df)<1e-9;reproduced[key]=dict(common_delta_m=dc,full_delta_m=df)
        if prior:
            kk=key+'_method2'
            d1=abs(common[key]['ATE_RMSE_m']-prior['common'][kk]['ATE_RMSE_m'])
            d2=abs(full[key]['ATE_RMSE_m']-prior['full_saved'][kk]['ATE_RMSE_m'])
            assert max(d1,d2)<1e-8;reproduced[key]=dict(CPV26_common_delta_m=d1,CPV26_full_delta_m=d2)
    assert np.array_equal(current_valid,valid),'common mask changed'
    for p in (Path(__file__),H/'PROTOCOL.txt',Path(base.__file__),Path(fullmod.__file__),Path(c.__file__),Path(c.b.__file__),Path(a.__file__),Path(a.g.__file__)):pin(p)
    base.PINS.update(a.HASHES);assert all(sha(p)==v for p,v in base.PINS.items())
    if not prior:
        for mode in ('off','on'):write_tum(Path(paths[mode+'_fusion']),curves[mode+'_fusion'])
        np.savez_compressed(H/(case+'_fusion_products.npz'),**products)
    np.savez_compressed(H/(case+'.npz'),**arrays)
    data=dict(case=case,sensor=sensor,sequence=sequence,capture_s=duration,epoch_ns=epoch,method=2,
              policy='accumulated same-update relative poses, both path and RMSE',original_receiver_Qc_multiplier=1.,
              common=common,full_saved=full,common_coverage=old['common_coverage'],reference=old['reference'],
              original_terminal_omissions=old['original_terminal_omissions'],trajectories=paths,reconstruction=checks,
              historical_score_checks=reproduced,mask_identical_to_CPV10=True,input_sha256=base.PINS,input_hashes_unchanged=True,
              off_on={r:c.improve(common['off_'+r]['ATE_RMSE_m'],common['on_'+r]['ATE_RMSE_m']) for r in ROLES})
    a.save(H/(case+'.json'),data)
    print(json.dumps(dict(stage='scored',case=case,ATE={k:v['ATE_RMSE_m'] for k,v in common.items()})),flush=True)

if __name__=='__main__':
    task,selection=sys.argv[1:]
    for row in c.b.ROWS:
        if selection not in ('all',row[0]):continue
        if task=='glim':
            if row[0]!='velodyne_leica2':reconstruct_glim(row[0])
        elif task=='score':analyze(row)
        else:raise ValueError(task)
