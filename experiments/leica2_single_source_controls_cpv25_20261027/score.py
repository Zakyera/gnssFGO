"""Unchanged first-publication scoring for native and source-subset receiver controls."""
import csv, hashlib, importlib.util, json, sys
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
from scipy.spatial.transform import Rotation

H=Path(__file__).resolve().parent;E=H.parent
R=Path('/home/yeranis/results/gnssfgo_leica2_single_source_controls_cpv25_20261027')
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
spec=importlib.util.spec_from_file_location('metrics14',E/'leica2_motion_prior_cpv14_20261026/score.py')
s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s);c,a=s.c,s.a
EPOCH=1732549034915071665
pins={}
def pin(path):
    path=Path(path)
    with path.open('rb') as f:pins[str(path)]=hashlib.file_digest(f,'sha256').hexdigest()
    return path
def save(path,x):
    with path.open('x') as f:json.dump(x,f,indent=2,allow_nan=False)
def N(x):return np.linalg.norm(x,axis=-1)

assert json.loads(pin(H/'replay_complete.json').read_text())['all_pass']
for case in ('glim_only','kimera_only'):
    assert json.loads(pin(H/(case+'_validation.json')).read_text())['pass_']
prior=json.loads(pin(E/'leica2_incremental_full_cpv23_20261027/results.json').read_text())
common=np.load(pin(E/'first_published_comparison_cpv10_20261026/outputs/velodyne_leica2.npz'))
grid,valid=common['timestamp_ns'],common['common_valid']
ref=a.g.Reference(pin(a.GT))
reference=np.asarray(ref.times,np.int64),np.asarray(ref.poses)[:,:3,3],np.asarray(ref.poses)[:,:3,:3]
curves={};full={};comm={};checks={};arrays={}
roots={'both':P/'full_off_receiver/receiver','glim_only':R/'glim_only','kimera_only':R/'kimera_only'}
for case,root in roots.items():
    for product,name in (('online','online_trajectory.tum'),('finalized','finalized_trajectory.tum')):
        key=case+'_'+product
        t,p,r=a.load_curve(pin(root/name));curves[key]=(t,p,r)
        assert len(t)==4282 and np.all(np.diff(t)==100_000_000)
        if key!='both_online':assert np.array_equal(t,curves['both_online'][0])
        gp,gr,ok=a.sample(reference,t,50_000_000);assert ok.all()
        ep,er=a.prism(p,r);gp,gr=a.prism(gp,gr)
        full[key],aligned,step,_,_=s.metrics(ep,er,gp,gr,np.ones(len(t),bool))
        full[key]['coverage']=c.coverage(t,np.ones(len(t),bool),EPOCH,428.75)
        full[key]['reference_supported_saved_rows']=int(ok.sum())
        pp,rr,ok=a.sample((t,p,r),grid,500_000_000);assert ok[valid].all()
        pp,rr=a.prism(pp,rr)
        comm[key],residual=c.metrics(pp,rr,common['reference_position'],common['reference_rotation'],valid,grid)
        arrays[key+'_common_error_m']=N(residual)
        arrays[key+'_full_aligned_p']=aligned
        arrays[key+'_full_step_error_m']=step
        if case=='both':
            oldkey='off_absolute' if product=='online' else 'off_delayed_finalized'
            for cohort,new in (('full',full),('common',comm)):
                delta=abs(new[key]['ATE_RMSE_m']-prior[cohort][oldkey]['ATE_RMSE_m'])
                assert delta<1e-9;checks[key+'_'+cohort+'_ATE_reproduction_m']=delta

native={}
for role,name in (('glim_initial','glim_odometry.csv'),('kimera','kimera_odometry.csv')):
    curve=a.load_curve(pin(P/'full_off_source'/name));curves['native_'+role]=curve
    p,r,ok=a.sample(curve,grid,500_000_000);assert ok[valid].all()
    p,r=a.prism(p,r)
    native[role],_=c.metrics(p,r,common['reference_position'],common['reference_rotation'],valid,grid)
    delta=abs(native[role]['ATE_RMSE_m']-prior['unchanged_native_common']['off_'+role]['ATE_RMSE_m'])
    assert delta<1e-9;checks['native_'+role+'_common_ATE_reproduction_m']=delta

# Source-specific actual endpoint cohorts, all products sampled at IDENTICAL
# physical queries. Linear position / SO(3) interpolation of saved outputs;
# not a recovered current graph snapshot or new causal publication product.
endpoints={src:[] for src in ('glim_velodyne','kimera_front')};seen=set()
with pin(P/'full_off_receiver/events.csv').open() as f:
    for row in csv.reader(f):
        if row[0]!='B':continue
        identity=(row[2],row[3],row[6],row[7])
        if identity in seen:continue
        seen.add(identity);endpoints[row[2]].append((int(row[8]),int(row[9])))
endpoint_scores={}
for source,pairs in endpoints.items():
    pairs=np.asarray(pairs,np.int64);query=np.r_[pairs[:,0],pairs[:,1]];n=len(pairs)
    gp,gr,ok=a.sample(reference,query,50_000_000);gp,gr=a.prism(gp,gr)
    mask=ok[:n]&ok[n:];sampled={}
    # Native counterpart plus BOTH and BOTH source-subset controls, so each
    # endpoint table has one fixed eligibility mask regardless of outcome.
    role='glim_initial' if source=='glim_velodyne' else 'kimera'
    keys=list(roots.keys())
    names=[k+'_'+p for k in keys for p in ('online','finalized')]+['native_'+role]
    for key in names:
        p,r,ok=a.sample(curves[key],query,500_000_000);p,r=a.prism(p,r)
        sampled[key]=(p,r);mask &= ok[:n]&ok[n:]
    ii=np.flatnonzero(mask);jj=ii+n
    summary={}
    for key,(p,r) in sampled.items():
        dt,dr,_=a.rpe(p,r,gp,gr,ii,jj)
        summary[key]={'translation_RMSE_m':float(np.sqrt(np.mean(N(dt)**2))),
                      'rotation_RMSE_deg':float(np.sqrt(np.mean(N(dr)**2)))}
    endpoint_scores[source]={'eligible_intervals':int(mask.sum()),'original_unique_intervals':n,
        'first_endpoint_s':float((pairs[mask,0].min()-EPOCH)*1e-9),
        'last_endpoint_s':float((pairs[mask,1].max()-EPOCH)*1e-9),
        'duration_s_median':float(np.median((pairs[mask,1]-pairs[mask,0])*1e-9)),
        'products':summary,'interpolated_saved_curves_not_same_solution_factors':True}
    arrays[source+'_endpoint_pairs_ns']=pairs;arrays[source+'_endpoint_valid']=mask

frontier=np.load(pin(E/'leica2_fusion_gap_diagnosis_cpv24_20261027/diagnostic_arrays.npz'))
assert np.array_equal(frontier['off_timestamp_ns'],curves['both_online'][0])
age=frontier['off_frontier_age_ms'];ages={}
for j,case in enumerate(('glim_only','kimera_only')):
    finite=np.isfinite(age[:,j]);normal=finite&(age[:,j]<=500)
    ages[case]={'all_seen_endpoint_age_ms':s.stat(age[finite,j]),'endpoint_age_le500ms':s.stat(age[normal,j]),
                'note':'Selected source frontier unchanged by removing other B rows. Physical evidence age, not wall-time solver latency.'}

result={'task':'CP-V25','source_mode':'CBS_OFF','Qc_multiplier':1.0,'full':full,'common':comm,
        'native_common':native,'native_full_saved':{role:prior['unchanged_native_full']['off_'+role] for role in native},
        'common_coverage':prior['common_coverage'],'source_endpoint_relative_scores':endpoint_scores,
        'source_evidence_age':ages,'baseline_checks':checks,'synthetic_checks':a.tests(),
        'limits':['Offline saved receiver controls, not native estimators or live co-running.',
                  'DELAYED finalized output is not ONLINE.','IE-TC shares CPT7; not independent prism GT.',
                  'Only other-source B rows removed; unchanged Qc/means/C/times/lag/priors.',
                  'Source-endpoint scores interpolate saved output curves, not simultaneous graph states.']}
for p in (Path(__file__),H/'PROTOCOL.txt',Path(s.__file__),Path(c.__file__),Path(c.b.__file__),Path(a.__file__),Path(a.g.__file__)):
    pin(p)
pins.update(a.HASHES)
for path,digest in pins.items():
    with open(path,'rb') as f:assert hashlib.file_digest(f,'sha256').hexdigest()==digest,path
save(H/'results.json',result);save(H/'scoring_pins.json',pins)
arrays['common_time_s']=(grid[valid]-EPOCH)*1e-9
arrays['full_time_s']=(curves['both_online'][0]-EPOCH)*1e-9
assert not (H/'score_arrays.npz').exists();np.savez_compressed(H/'score_arrays.npz',**arrays)
print(json.dumps({'common':comm,'native_common':native,'endpoints':endpoint_scores},indent=2))
