"""Validate live output and compare policies without changing source inputs."""
import csv, importlib.util, json, sys
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
H=Path(__file__).resolve().parent
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_incremental_cpv29_20261028/prepared')
spec=importlib.util.spec_from_file_location('cpv28',H.parent/'all_incremental_comparison_cpv28_20261028/analyze.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
a=m.a;c=m.c

def save(path,data):
    with path.open('x') as f:json.dump(data,f,indent=2);f.write('\n')
def csvrows(path):
    with m.pin(path).open() as f:return list(csv.DictReader(f))
def poses(rows,label):
    vals=np.array([[float(x[label+'_'+k]) for k in ('px','py','pz','qx','qy','qz','qw')] for x in rows])
    assert np.isfinite(vals).all() and np.max(abs(np.linalg.norm(vals[:,3:],axis=1)-1))<1e-6
    return np.array([m.base.matrix(v[:3],v[3:]) for v in vals])

def live_glim(src):
    curve,arrays,checks=m.glim(src)
    rows=csvrows(src/'glim_incremental_pairs.csv');n=len(rows)
    assert n==len(curve[0]) and n>=3430
    assert [int(x['index']) for x in rows]==list(range(n))
    ts=np.array([int(x['stamp_ns']) for x in rows],np.int64)
    assert np.array_equal(ts,curve[0])
    av=np.array([int(x['availability_ros_ns']) for x in rows],np.int64)
    wall=np.array([int(x['wall_ns']) for x in rows],np.int64)
    assert np.all(av>=ts) and np.all(np.diff(av)>=0) and np.all(np.diff(wall)>0)
    pair={k:poses(rows,k) for k in ('current','previous','incremental','graph_T_odom')}
    errors={k:float(np.max(abs(pair[k]-arrays[v]))) for k,v in
            [('current','graph_body_T'),('previous','previous_same_update_body_T'),
             ('incremental','chained_body_T'),('graph_T_odom','graph_T_odometry')]}
    assert max(errors.values())<1e-8,errors
    rr=csvrows(src/'glim_incremental_ros.csv')
    assert len(rr)==n and [int(x['field.header.seq']) for x in rr]==list(range(n))
    assert [int(x['field.header.stamp']) for x in rr]==ts.tolist()
    assert {x['field.header.frame_id'] for x in rr}=={'glim_incremental_odom_cpv29'}
    ros=np.array([m.base.matrix([float(x['field.pose.position.'+k]) for k in 'xyz'],
                               [float(x['field.pose.orientation.'+k]) for k in 'xyzw']) for x in rr])
    errors['ros_vs_live_csv']=float(np.max(abs(ros-pair['incremental'])))
    assert errors['ros_vs_live_csv']<1e-12
    checks.update(passed=True,live_vs_independent_max_matrix_errors=errors,
                  live_ros_rows=len(rr),newest_pose_body='cpt7_imu',
                  incremental_frame='glim_incremental_odom_cpv29',
                  physical_stamps_exact=True,latency_ros_ms=np.percentile((av-ts)*1e-6,[50,95,100]).tolist())
    arrays['availability_ros_ns']=av;arrays['wall_ns']=wall
    actual=(ts,pair['incremental'][:,:3,3],pair['incremental'][:,:3,:3])
    return actual,arrays,checks

def fusion(original):
    assert json.loads((H/'fusion_parity.json').read_text())['pass_']
    rows=csvrows(H/'fusion_publications.csv');groups={}
    for r in rows:groups.setdefault(int(r['newest']),[]).append(r)
    ts,native=m.native_matrices(original);assert sorted(groups)==list(range(len(ts)))
    g=[];prev=[];av=[]
    for k,ss in groups.items():
        assert [int(s['index']) for s in ss]==([0] if k==0 else [k-1,k])
        assert len({s['event'] for s in ss})==1
        assert all(int(s['physical_ns'])==int(ts[k]) and int(s['availability_ns'])>=int(ts[k]) for s in ss)
        def pose(s):return m.base.matrix([float(s[x]) for x in ('px','py','pz')],[float(s[x]) for x in ('qx','qy','qz','qw')])
        g.append(pose(ss[-1]));prev.append(pose(ss[0]));av.append(int(ss[-1]['availability_ns']))
    g,prev=np.array(g),np.array(prev)
    assert np.max(abs(g-native))<1e-12 and np.all(np.diff(ts)==100000000) and np.all(np.diff(av)>=0)
    return m.verify_chain(ts,g,prev)

def main():
    assert not (H/'results.json').exists()
    src=P/'full_off_source';receiver=P/'full_off_receiver/receiver'
    live,ga,gchecks=live_glim(src)
    fchain,fa,fchecks=fusion(receiver/'online_trajectory.tum')
    kg,_,kchecks=m.fullmod.kimera(src)
    curves={'glim_absolute':a.load_curve(m.pin(src/'glim_odometry.csv')),
            'glim_incremental_live':live,'kimera_absolute_graph':kg,
            'kimera_incremental_native':a.load_curve(m.pin(src/'kimera_odometry.csv')),
            'fusion_absolute_online':a.load_curve(m.pin(receiver/'online_trajectory.tum')),
            'fusion_incremental_observer':fchain,
            'fusion_delayed_finalized':a.load_curve(m.pin(receiver/'finalized_trajectory.tum'))}
    row=c.b.ROWS[0];assert row[0]=='hesai_eth1';duration,epoch=row[-2:]
    _,_,ref=c.b.load(row)
    reference=ref.times,np.array(ref.poses)[:,:3,3],np.array(ref.poses)[:,:3,:3]
    saved=np.load(m.pin(m.H10/'hesai_eth1.npz'));grid=saved['timestamp_ns'];oldvalid=saved['common_valid']
    gp,gr,gtok=a.sample(reference,grid,50_000_000);gp,gr=a.prism(gp,gr)
    assert np.array_equal(gp,saved['reference_position']) and np.array_equal(gr,saved['reference_rotation'])
    common_valid=oldvalid & gtok;sampled={}
    for key,curve in curves.items():
        p,r,good=a.sample(curve,grid,500_000_000);ep,er=a.prism(p,r)
        sampled[key]=(ep,er,good);common_valid &= good
    assert common_valid.sum()>2500
    full={};common={};own_grid={}
    arrays=dict(timestamp_ns=grid,common_valid=common_valid,original_common_valid=oldvalid,
                reference_position=gp,reference_rotation=gr)
    for key,curve in curves.items():
        ep,er,good=sampled[key]
        common[key],_=c.metrics(ep,er,gp,gr,common_valid,grid)
        full[key]=m.fullmod.full_metric(curve,reference,epoch,duration)
        own_grid[key],_=c.metrics(ep,er,gp,gr,good & gtok,grid)
        own_grid[key]['coverage']=c.coverage(grid,good & gtok,epoch,duration)
        arrays[key+'_position']=ep;arrays[key+'_rotation']=er
        m.write_tum(H/(key+'.tum'),curve)
    # Re-score old curves on the exact new shared mask if new gaps shrink it.
    old=json.loads(m.pin(m.H10/'hesai_eth1.json').read_text());oldcurves={}
    for name,product in [('glim_absolute','off_glim_initial'),('kimera_incremental_native','off_kimera'),('fusion_absolute_online','off_fusion_online')]:
        oldcurves[name]=a.load_curve(m.pin(Path(old['input_products'][product]['path'])))
    oldcurves['glim_incremental_live']=a.load_curve(m.pin(m.H/'hesai_eth1_off_glim.tum'))
    oldcurves['fusion_incremental_observer']=a.load_curve(m.pin(m.H/'hesai_eth1_off_fusion.tum'))
    oldfull={};oldcommon={}
    for key,curve in oldcurves.items():
        p,r,good=a.sample(curve,grid,500_000_000);assert good[common_valid].all()
        ep,er=a.prism(p,r);oldcommon[key],_=c.metrics(ep,er,gp,gr,common_valid,grid)
        oldfull[key]=m.fullmod.full_metric(curve,reference,epoch,duration)
    for file in (Path(__file__),H/'PROTOCOL.txt',Path(m.__file__),Path(m.base.__file__),Path(m.fullmod.__file__),
                 Path(c.__file__),Path(c.b.__file__),Path(a.__file__),Path(a.g.__file__)):m.pin(file)
    pins={**m.base.PINS,**a.HASHES};assert all(m.sha(p)==d for p,d in pins.items())
    results=dict(passed=True,case='Hesai ETH1 CBS-OFF CP-V29',capture_s=duration,Qc_multiplier=1,
       GLIM_live_checks=gchecks,Kimera_checks=kchecks,Fusion_chain_checks=fchecks,
       full_saved=full,common=common,own_100ms_grid=own_grid,
       common_coverage=c.coverage(grid,common_valid,epoch,duration),
       frozen_common_mask_identical=bool(np.array_equal(common_valid,oldvalid)),
       frozen_mask_excluded_new_queries=int(np.sum(oldvalid & ~common_valid)),
       old_full_saved=oldfull,old_common_same_mask=oldcommon,input_pins=pins,
       caveats=['IE-TC shares CPT7; not independent prism ground truth.',
                'Method2 is the same output rule, not equal latency or cadence.',
                'GLIM incremental output is additional and never fed back into estimation.',
                'Fusion incremental pairs come from byte-parity-verified reporting replay.',
                'Delayed finalized fusion is not online or full-history.',
                'Full native polyline may include gap chords; common path does not bridge gaps.',
                'New capture may differ stochastically; observer does not improve optimizer.'])
    np.savez_compressed(H/'products.npz',**arrays)
    np.savez_compressed(H/'glim_live_products.npz',**ga)
    np.savez_compressed(H/'fusion_incremental_products.npz',**fa)
    save(H/'results.json',results)
    with (H/'RESULTS_REPORT.txt').open('x') as f:
        f.write('CP-V29 FULL ETH1 HESAI CBS-OFF — live incremental GLIM output\n\n')
        f.write('Full saved native cadence; ATE rigid SE3/no scale; metres.\n')
        f.write('Product | ATE RMSE | estimated path | reference path | rows | span seconds\n')
        for key,s in full.items():f.write(f"{key} | {s['ATE_RMSE_m']:.6f} | {s['native_polyline_length_m']:.3f} | {s['reference_native_polyline_length_m']:.3f} | {s['rows']} | {s['span_s']:.3f}\n")
        f.write('\nCommon mask: '+json.dumps(results['common_coverage'])+'\n')
        f.write('Product | ATE RMSE | path | 1s translation RPE | 1s rotation RPE deg\n')
        for key,s in common.items():
            r=s['horizons']['1'];f.write(f"{key} | {s['ATE_RMSE_m']:.6f} | {s['path_length_m']:.3f} | {r['translation_RMSE_m']:.6f} | {r['rotation_RMSE_deg']:.6f}\n")
        f.write('\nLive GLIM parity checks: '+json.dumps(gchecks)+'\n')
        f.write('\nOld full saved -> fresh full saved ATE / path:\n')
        for key,s in oldfull.items():
            n=full[key];f.write(f"{key}: {s['ATE_RMSE_m']:.6f} -> {n['ATE_RMSE_m']:.6f} m; {s['native_polyline_length_m']:.3f} -> {n['native_polyline_length_m']:.3f} m\n")
        f.write('\n'+'\n'.join(results['caveats'])+'\n')
    print(json.dumps(dict(passed=True,glim_checks=gchecks,full_saved=full)),flush=True)

if __name__=='__main__':main()
