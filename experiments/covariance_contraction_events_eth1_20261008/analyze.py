"""Retained-log analysis only; never imports/runs an estimator."""
import argparse, csv, hashlib, importlib.util, json, re, sys
from collections import Counter, defaultdict
from pathlib import Path
sys.dont_write_bytecode = True
import numpy as np

HERE = Path(__file__).resolve().parent
OLD = HERE.parent/'covariance_quality_eth1_20261008'
spec = importlib.util.spec_from_file_location('quality', OLD/'analyze.py')
q = importlib.util.module_from_spec(spec); spec.loader.exec_module(q)
ROOT = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live_wnoa_20261007')
DIRS = {'off':ROOT/'full_off_source', 'on':ROOT/'bounded120_on_source'}
PARTS = {'rotation':slice(0,3), 'rho':slice(3,6)}

def dump(p,x):
    with p.open('x') as f: json.dump(x,f,indent=2,allow_nan=False)

def parse_main(p):
    rev=defaultdict(list); updates={}; retires={}; poses=defaultdict(list)
    merges={}; adds=[]; markers=Counter(); pending=[]
    for ln,line in enumerate(p.open(),1):
        m=re.search(r'(KIMERA_[A-Z_]+_ROW|CBS_BPSAM_ODOM_ADD_ROW_G2K),',line)
        if not m: continue
        r=line[m.start():].strip().split(','); tag=r[0]; markers[tag]+=1
        if tag=='KIMERA_CBS_REVISION_EDGE_ROW':
            assert len(r)==65,(ln,len(r))
            rec=dict(u=int(r[1]),i=int(r[3]),j=int(r[4]),a=float(r[5]),b=float(r[6]),
                revision=int(r[7]),kind=r[8],mu=np.array(r[23:29],float),
                C=np.array(r[29:65],float).reshape(6,6),line=ln)
            rev[(rec['i'],rec['j'])].append(rec)
        elif tag=='KIMERA_CBS_OUTGOING_TIMING_ROW':
            u=int(r[1]); assert u not in updates
            updates[u]=dict(u=u,local_factor_count=int(r[6]),outgoing=int(r[5]),
                queued_cbs=pending,line=ln); pending=[]
        elif tag=='KIMERA_CBS_REVISION_EDGE_RETIRE_ROW':
            key=(int(r[2]),int(r[3])); assert key not in retires
            retires[key]=dict(u=int(r[1]),revision=int(r[4]),line=ln)
        elif tag=='KIMERA_CBS_REVISION_POSE_RETIRE_ROW':
            poses[int(r[1])].append(dict(id=int(r[2]),stamp=float(r[3]),line=ln))
        elif tag=='KIMERA_CBS_POSE_MERGE_ROW':
            u=int(r[1]); assert u not in merges
            merges[u]=dict(edge=r[2],before_rotation=float(r[3]),before_rho=float(r[4]),
                after_rotation=float(r[5]),after_rho=float(r[6]),line=ln)
        elif tag=='CBS_BPSAM_ODOM_ADD_ROW_G2K':
            x=dict(source_edge=r[1],receiver_edge=r[2],status=r[3],trace=float(r[4]),action=r[5],line=ln)
            pending.append(x); adds.append(x)
    return rev,updates,retires,poses,merges,dict(markers)

def parse_stdout(p):
    cov=defaultdict(list); dec=defaultdict(list); markers=Counter(); routes=Counter(); factors=[]
    for ln,line in enumerate(p.open(),1):
        m=re.search(r'(CBS_[A-Z_]+_ROW),',line)
        if not m: continue
        r=line[m.start():].strip().split(','); tag=r[0]; markers[tag]+=1
        if tag=='CBS_ODOM_RELATIVE_COVARIANCE_ROW':
            assert len(r)==26 and r[1:4]==['K2G','k','g']
            key=(int(r[4][1:]),int(r[5][1:])); assert r[6]=='schur_relative_between'
            cov[key].append(dict(line=ln,trace=float(r[8]),rotation=float(r[16]),rho=float(r[17]),
                rank=int(r[18]),h_condition=float(r[19]),direct_difference=float(r[20]),
                jitter=int(r[21]),jitter_added=float(r[22]),status=r[25]))
        elif tag=='CBS_ODOM_RELATIVE_COVARIANCE_DECOMP_ROW':
            assert len(r)==12
            key=(int(r[4][1:]),int(r[5][1:]))
            dec[key].append(dict(line=ln,ff=float(r[6]),tt=float(r[7]),from_term=float(r[8]),
                to_term=float(r[9]),cross_term=float(r[10]),total=float(r[11])))
        elif tag=='CBS_JOINT_MARGINAL_INFORMATION_TIMING_ROW': routes[r[2]]+=1
        elif tag=='CBS_ODOM_FACTOR_COVARIANCE_ROW':
            factors.append(dict(i=int(r[4][1:]),j=int(r[5][1:]),action=r[6],line=ln))
    return cov,dec,dict(markers),dict(routes),factors

def compact_step(s):
    return {k:v for k,v in s.items() if k not in ('C','mu')}

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--out',required=True); args=ap.parse_args()
    out=Path(args.out); out.mkdir(parents=True,exist_ok=False)
    man=json.loads((q.BASE/'manifest.json').read_text()); cut=man['cutoff_ns']; start=man['first_imu_ns']
    src=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src')
    paths=[HERE/'PROTOCOL.txt',Path(__file__),OLD/'analyze.py',q.BASE/'manifest.json',
        src/'Kimera-VIO/src/backend/VioBackend.cpp',src/'cbs/src/bpsam/bpsam.cpp',
        src/'cbs/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h',
        src/'cbs/include/cbs/utils/relative_pose_covariance.h']
    std={}
    for mode,d in DIRS.items():
        std[mode]=list((d/'ros_logs').glob('*/*kimera_vio_ros_node*stdout.log'))
        assert len(std[mode])==1;std[mode]=std[mode][0]
        paths += [d/'estimators.log',d/'kimera_odometry.csv',std[mode],d/'kimera_profile/BackendParams.yaml',
                  *list(d.glob('run_metadata.txt')),*list((d/'ros_logs').glob('*/roslaunch-*.log')),
                  q.BASE/'inputs'/f'{mode}_events.csv',q.BASE/'payloads'/f'{mode}_final.csv']
    protected=dict(man['protected'])
    for p in paths: protected[str(p)]=q.sha(p)
    for p,h in protected.items(): assert q.sha(p)==h,p
    dump(out/'inputs_before.json',protected)
    summary=dict(scope=dict(start_ns=start,cutoff_ns=cut,no_runs=True),modes={})
    all_edges={}; all_steps={}; all_updates={}
    for mode,d in DIRS.items():
        H,_,_=q.load(mode); H={k:h for k,h in H.items() if k[0]=='kimera_front'}
        rev,updates,retires,poses,merges,markers=parse_main(d/'estimators.log')
        cov,dec,smarkers,routes,factors=parse_stdout(std[mode])
        # Per-edge call ordering is exact; verify every exported numeric trace.
        assert rev.keys()==cov.keys()==dec.keys()
        max_trace=0.; checks=[]
        for key,rs in rev.items():
            assert len(rs)==len(cov[key])==len(dec[key]),key
            for r,c,z in zip(rs,cov[key],dec[key]):
                delta=abs(c['trace']-np.trace(r['C'])); max_trace=max(max_trace,delta)
                assert np.isclose(c['trace'],np.trace(r['C']),rtol=2e-10,atol=1e-18),(key,r['u'])
                for part,sl in PARTS.items(): assert np.isclose(c[part],np.trace(r['C'][sl,sl]),rtol=2e-10,atol=1e-18)
                r['diagnostic']=c;r['decomposition']=z
        time_map={}
        with (d/'kimera_odometry.csv').open() as f:
            for r in csv.DictReader(f):
                u=int(r['field.header.seq']); assert u not in time_map
                time_map[u]=int(r['field.header.stamp'])
        accepted=[]; steps=[]; edge_rows=[]
        for key,h in H.items():
            pair=(h[0]['i'],h[0]['j']); raw=rev[pair]
            joined=[]
            for x in h:
                matches=[r for r in raw if r['revision']==x['revision'] and np.array_equal(r['mu'],x['mu']) and np.array_equal(r['C'],x['cov'])]
                assert len(matches)==1,(mode,pair,x['revision'],len(matches))
                r=matches[0]; joined.append(r)
                assert abs(round(r['a']*1e9)-key[1])<=256 and abs(round(r['b']*1e9)-key[2])<=256
            assert all(b['u']==a['u']+1 for a,b in zip(joined,joined[1:])),pair
            assert all(time_map[r['u']]<=h[n]['arrival']<=cut for n,r in enumerate(joined))
            u0=joined[0]['u']; ulast=joined[-1]['u']; retirement=retires.get(pair)
            retired=retirement is not None and retirement['u']==ulast+1 and time_map[retirement['u']]<=cut
            ref=q.ref.relative_reference(key[1],key[2]); assert ref is not None
            errs=np.array([q.ref.discrepancy(r['mu'],ref)[0] for r in joined])
            sig=np.array([[np.sqrt(np.trace(r['C'][sl,sl])) for sl in PARTS.values()] for r in joined])
            ages=np.array([(x['arrival']-h[0]['arrival'])*1e-9 for x in h])
            physical=np.array([(time_map[r['u']]-key[2])*1e-9 for r in joined])
            age2=max((i for i,a in enumerate(ages) if a<=2),default=0)
            qualify=h[0]['arrival']+2_000_000_000<=cut
            row=dict(i=pair[0],j=pair[1],first_u=u0,last_u=ulast,retire_u=retirement['u'] if retired else None,
                retired=retired,first_arrival_ns=h[0]['arrival'],age2_qualified=qualify,
                revisions=len(h),first_physical_s=(key[2]-start)*1e-9,age2_index=age2,
                physical_age_final=physical[-1],arrival_age_final=ages[-1],evolution=[])
            for n,(r,x) in enumerate(zip(joined,h)):
                er=[float(np.linalg.norm(errs[n,sl])) for sl in PARTS.values()]
                c=r['diagnostic']; z=r['decomposition']; u=r['u']
                v=dict(u=u,revision=x['revision'],age_updates=u-u0,age_arrival_s=float(ages[n]),
                    age_physical_s=float(physical[n]),sigma=sig[n].tolist(),error=er,line=r['line'],
                    numeric=c,decomposition=z)
                row['evolution'].append(v);accepted.append(r)
                if n==0:continue
                s=dict(i=pair[0],j=pair[1],u=u,from_u=joined[n-1]['u'],elapsed_s=(time_map[u]-start)*1e-9,
                    age_updates=u-u0,age_arrival_s=float(ages[n]),age_physical_s=float(physical[n]),
                    remaining_updates=ulast-u if retired else None,retired=retired,
                    late=bool(n>age2 and qualify),sigma_ratio=(sig[n]/sig[n-1]).tolist(),
                    error_ratio=[er[l]/row['evolution'][n-1]['error'][l] for l in range(2)],
                    local_factors=updates[u]['local_factor_count'],
                    local_factor_delta=updates[u]['local_factor_count']-updates[u-1]['local_factor_count'],
                    retire_count=len(poses.get(u,[])),queued_cbs=len(updates[u]['queued_cbs']),
                    merge=merges.get(u),line=r['line'],stdout_line=c['line'])
                steps.append(s)
            for p,idx in [('rotation',0),('rho',1)]:
                a=np.array([x['sigma'][idx] for x in row['evolution']]); loss=np.maximum(0,-np.diff(np.log(a)))
                late_loss=loss[age2:]
                row[p]=dict(last_over_age2=float(a[-1]/a[age2]),last_step_ratio=float(a[-1]/a[-2]) if len(a)>1 else None,
                    last6_steps_ratio=float(a[-1]/a[-7]) if len(a)>6 else None,
                    late_log_contraction=float(late_loss.sum()),
                    last_step_share_of_late_contraction=float(late_loss[-1]/late_loss.sum()) if len(late_loss) and late_loss.sum()>0 else None)
            edge_rows.append(row)
        maxu=max(r['u'] for r in accepted); us=[]
        grouped=defaultdict(list)
        for s in steps:grouped[s['u']].append(s)
        for u,g in sorted(grouped.items()):
            late=[x for x in g if x['late']]
            row=dict(u=u,elapsed_s=(time_map[u]-start)*1e-9,n=len(g),n_late=len(late),
                     local_factors=updates[u]['local_factor_count'],local_factor_delta=g[0]['local_factor_delta'],
                     retire_count=len(poses.get(u,[])),queued_cbs=len(updates[u]['queued_cbs']),merge=merges.get(u))
            for p,idx in [('rotation',0),('rho',1)]:
                a=np.array([x['sigma_ratio'][idx] for x in g]); b=np.array([x['sigma_ratio'][idx] for x in late])
                row[p]=dict(all_median_ratio=float(np.median(a)),late_median_ratio=float(np.median(b)) if len(b) else None,
                    late_fraction_drop5=float(np.mean(b<.95)) if len(b) else None,
                    late_log_contraction=float(np.maximum(0,-np.log(b)).sum()))
            us.append(row)
        result=dict(accepted_edges=len(H),accepted_versions=len(accepted),max_u=maxu,
                    joined_raw_trace_max_difference=max_trace,main_markers=markers,stdout_markers=smarkers,
                    covariance_routes_all_log=routes,
                    numerical_accepted=dict(jitter_count=sum(r['diagnostic']['jitter'] for r in accepted),
                        min_rank=min(r['diagnostic']['rank'] for r in accepted),
                        max_h_condition=max(r['diagnostic']['h_condition'] for r in accepted),
                        max_direct_difference=max(r['diagnostic']['direct_difference'] for r in accepted),
                        max_relative_direct_difference=max(r['diagnostic']['direct_difference']/r['diagnostic']['trace'] for r in accepted),
                        statuses=dict(Counter(r['diagnostic']['status'] for r in accepted))),
                    retirement=dict(first_u=min(poses),first_elapsed_s=(time_map[min(poses)]-start)*1e-9,
                        first=poses[min(poses)],updates_present=sum(u in poses for u in range(min(poses),maxu+1)),
                        updates_expected=maxu-min(poses)+1,retired_edges=sum(x['retired'] for x in edge_rows)),
                    source_factor_applications=dict(count_all_log=len(factors),actions=dict(Counter(x['action'] for x in factors)),
                        unique_receiver_edges=len(set((x['i'],x['j']) for x in factors))),
                    age_profile={},remaining_profile={},top_synchronized={},controls={},within_update_age={})
        for p,idx in [('rotation',0),('rho',1)]:
            result['age_profile'][p]={str(age):q.stats([s['sigma_ratio'][idx] for s in steps if s['age_updates']==age]) for age in range(1,26)}
            result['remaining_profile'][p]={str(age):q.stats([s['sigma_ratio'][idx] for s in steps if s['remaining_updates']==age]) for age in range(25)}
            eligible=[u for u in us if u['n_late']>=10]
            result['top_synchronized'][p]=sorted(eligible,key=lambda x:x[p]['late_median_ratio'])[:8]
            late=[s for s in steps if s['late']]; eligible_edges=[e for e in edge_rows if e['age2_qualified']]
            result['controls'][p]=dict(late_step_ratio=q.stats([s['sigma_ratio'][idx] for s in late]),
                paired_last_over_age2=q.stats([e[p]['last_over_age2'] for e in eligible_edges]),
                retired_last_step_ratio=q.stats([e[p]['last_step_ratio'] for e in edge_rows if e['retired']]),
                retired_last6_steps_ratio=q.stats([e[p]['last6_steps_ratio'] for e in edge_rows if e['retired']]),
                last_step_contraction_share=q.stats([e[p]['last_step_share_of_late_contraction'] for e in eligible_edges if e['retired'] and e[p]['last_step_share_of_late_contraction'] is not None]))
            age_corr=[]
            for u,g in sorted(grouped.items()):
                same=[s for s in g if s['late']]
                if len(same)<10: continue
                val=q.spearman([s['age_updates'] for s in same],[s['sigma_ratio'][idx] for s in same])
                if val is not None:age_corr.append(val)
            result['within_update_age'][p]=dict(correlation=q.stats(age_corr),
                negative_fraction=float(np.mean(np.array(age_corr)<0)),
                meaning='negative = older beliefs contract faster at the SAME backend update')
        summary['modes'][mode]=result;all_edges[mode]=edge_rows;all_steps[mode]=steps;all_updates[mode]=us
        print(mode,'joined',len(accepted),'versions',len(H),'edges',flush=True)
    # Same physical-edge cohort across modes; downstream accepted mapping checked by q.load.
    assert [(e['i'],e['j']) for e in all_edges['off']]==[(e['i'],e['j']) for e in all_edges['on']]
    dump(out/'summary.json',summary);dump(out/'edges.json',all_edges)
    dump(out/'steps.json',all_steps);dump(out/'updates.json',all_updates)
    after={p:q.sha(p) for p in protected}; assert after==protected
    dump(out/'inputs_after.json',after)
    dump(out/'validation.json',dict(passed=True,protected_files=len(protected),exact_source_payload_join=True,
                                   exact_per_edge_covariance_call_count=True,no_estimator_runs=True))

if __name__=='__main__':main()
