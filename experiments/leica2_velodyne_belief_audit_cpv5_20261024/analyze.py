"""CP-V5: read saved LEICA-2 evidence; never execute an estimator/replay."""
import csv
import gzip
import importlib.util
import json
import math
import sys
from collections import Counter, defaultdict
from itertools import zip_longest
from pathlib import Path

sys.dont_write_bytecode = True
import numpy as np
from scipy.stats import spearmanr
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

H = Path(__file__).resolve().parent
P = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
BASE = H.parent/'velodyne_full_campaign_cpv4_20261024/leica2'
GT = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/datasets/GrandTour/LEICA-2/2024-11-25-16-36-19_cpt7_ie_tc.bag')
OUT = H/'outputs'
START = 1732549034915071665
SOURCES = ('glim_velodyne', 'kimera_front')
MODES = ('off', 'on')
AGE, TOL = 4_000_000_000, 150_000_000
spec = importlib.util.spec_from_file_location('fl4z', H.parent/'fixed_lag_covariance_fl4z_20261017/analyze.py')
a = importlib.util.module_from_spec(spec); spec.loader.exec_module(a)
g = a.geo
HASHES = {}


def pin(p):
    p = Path(p)
    if str(p) not in HASHES: HASHES[str(p)] = g.sha(p)
    return p


def native_times(mode):
    result = {}
    for source, stem in zip(SOURCES, ('glim', 'kimera')):
        path = pin(P/f'full_{mode}_source/{stem}_odometry.csv')
        t = {}
        for r in csv.DictReader(path.open()):
            assert r['field.child_frame_id'] == 'cpt7_imu'
            idx = int(r['field.header.seq']); assert idx not in t
            t[idx] = int(r['field.header.stamp'])
        assert np.all(np.diff(list(t.values())) > 0)
        result[source] = t
    return result


def read_mode(mode):
    times = native_times(mode)
    first, near, seen = {}, defaultdict(dict), {}
    counts = Counter(); max_rounding = 0; covs = []
    min_eig, max_sym = math.inf, 0.
    def check_covs():
        nonlocal min_eig, max_sym
        c = np.asarray(covs)
        assert np.isfinite(c).all()
        max_sym = max(max_sym, float(np.max(np.abs(c-c.transpose(0,2,1)))))
        min_eig = min(min_eig, float(np.linalg.eigvalsh(c).min()))
        assert max_sym <= 1e-12 and min_eig > 0
        covs.clear()
    ep = pin(P/f'full_{mode}_receiver/events.csv')
    rp = pin(P/f'full_{mode}_receiver/receipts.jsonl')
    with ep.open() as ef, rp.open() as rf:
        for event, (line, receipt) in enumerate(zip_longest(ef, rf)):
            assert line is not None and receipt is not None
            if not line.startswith('B,'): continue
            r = line.rstrip().split(','); m = json.loads(receipt)
            assert len(r) == 52 and m['event'] == event and m['kind'] == 'B'
            assert m['receipt_ros_ns'] == int(r[1])
            meta = m['metadata']; source = r[2]
            assert meta['source'] == source and source in SOURCES
            ordinal, rev, i, j, ti, tj = map(int, r[4:10])
            stable = source, r[3], i, j
            assert rev == seen.get(stable, -1)+1
            seen[stable] = rev; counts[source] += 1
            assert ti < tj <= int(r[1])
            if source == SOURCES[0] and int(meta['source_revision']) == 0:
                assert a.stamp(meta['source_update']) == tj
                update = j
            else: update = int(meta['source_update'])
            assert update >= j
            for index, stamp in ((i, ti), (j, tj)):
                delta = abs(times[source][index]-stamp)
                max_rounding = max(max_rounding, delta); assert delta <= 1000
            update_ns = times[source][update]; age = update_ns-tj
            C = np.asarray(r[16:], float).reshape(6,6)
            mu = np.asarray(r[10:16], float); assert np.isfinite(mu).all()
            covs.append(C)
            if len(covs) == 4096: check_covs()
            if rev != 0 and abs(age-AGE) > TOL: continue
            x = dict(source=source, mode=mode, i=i, j=j, a=ti, b=tj,
                     arrival=int(r[1]), event_line=event+1, revision=rev,
                     source_revision=int(meta['source_revision']), update=update,
                     update_ns=update_ns, age_s=age*1e-9,
                     time_s=(tj-START)*1e-9, interval_s=(tj-ti)*1e-9,
                     mu=mu, cov=C)
            key = source, ti, tj
            if rev == 0:
                assert key not in first; first[key] = x
            if abs(age-AGE) <= TOL:
                assert update_ns not in near[key]
                near[key][update_ns] = x
    if covs: check_covs()
    return first, near, dict(versions=dict(counts), first_counts=dict(Counter(k[0] for k in first)),
                            max_endpoint_rounding_ns=max_rounding, min_eigenvalue=min_eig,
                            max_symmetry_error=max_sym)


def score(x, ref):
    target = g.inv(ref.at(x['a'])) @ ref.at(x['b'])
    d = a.measure(x, ref, target)
    dt = (x['b']-x['a'])*1e-9
    d['speed_m_s'] = float(np.linalg.norm(target[:3,3])/dt)
    d['angular_speed_deg_s'] = float(np.linalg.norm(g.log(target)[:3])*180/np.pi/dt)
    d['motion'] = 'quiet' if d['speed_m_s'] < .05 and d['angular_speed_deg_s'] < 2 else 'moving'
    return d


def ranking(v, e):
    v, e = np.asarray(v, float), np.asarray(e, float)
    assert len(v) and len(v) == len(e) and np.isfinite(v).all() and min(v) > 0
    assert np.isfinite(e).all()
    cuts = np.unique(np.quantile(v, np.linspace(0,1,11)))
    assigned = np.searchsorted(cuts[1:-1], v, side='right')
    bins = []
    for b in sorted(set(assigned)):
        mask = assigned == b
        bins.append(dict(n=int(mask.sum()), norm=float(np.sqrt(v[mask].mean())),
                         error_rms=float(np.sqrt(np.mean(e[mask]**2)))))
    assert sum(b['n'] for b in bins) == len(v)
    rms = float(np.sqrt(np.mean(e*e)))
    risk = []
    for percent in range(10,101,5):
        threshold = np.sort(v)[math.ceil(percent*len(v)/100)-1]
        mask = v <= threshold
        er = float(np.sqrt(np.mean(e[mask]**2)))
        risk.append(dict(requested_percent=percent, n=int(mask.sum()),
                         actual_percent=float(mask.mean()*100), error_rms=er,
                         relative_rms=er/rms if rms else None))
    assert risk[-1]['n'] == len(v)
    return dict(n=len(v), error_rms=rms, reported_norm_rms=float(np.sqrt(v.mean())),
                norm_median=float(np.sqrt(np.median(v))),
                error_median=float(np.median(e)), error_p95=float(np.percentile(e,95)),
                rank=float(spearmanr(v,e).statistic) if np.ptp(v)>0 and np.ptp(e)>0 else None,
                bins=bins, risk=risk)


def summary(xs):
    out = dict(n=len(xs))
    if not xs: return out
    for block, ef, vf, scale in [('translation','rho_error_mm','translation_trace_m2',1e6),
                                  ('rotation','rotation_error_deg','rotation_trace_rad2',(180/np.pi)**2)]:
        out[block] = ranking([x[vf]*scale for x in xs], [x[ef] for x in xs])
        out[block]['sqrt_normalized_discrepancy'] = a.stat([np.sqrt(x[block+'_normalized_squared']) for x in xs])
    out['sqrt_joint_discrepancy'] = a.stat([np.sqrt(x['joint_normalized_squared']) for x in xs])
    return out


def make_cohorts(first, near, ref):
    for mode in MODES:
        for x in first[mode].values(): x['metric'] = score(x,ref)
    summaries, coverage, records, worst = {}, {}, [], []
    for source in SOURCES:
        common = sorted(k for k in set(first['off']) & set(first['on']) if k[0] == source)
        eligible, missing = {}, []
        for key in common:
            epochs = set(near['off'].get(key,{})) & set(near['on'].get(key,{}))
            if epochs: eligible[key] = min(epochs, key=lambda t:(abs(t-key[2]-AGE),t))
            else: missing.append(key[1:])
        coverage[source] = dict(common_first=len(common), same_age_eligible=len(eligible),
                                missing_common_near4s=missing,
                                unmatched={m:[k[1:] for k in first[m] if k[0]==source and k not in common] for m in MODES})
        summaries[source] = {}
        for group in ('all_first','common_first','first','near4s'):
            summaries[source][group] = {}
            for mode in MODES:
                if group == 'all_first': xs = [x for k,x in first[mode].items() if k[0]==source]
                elif group == 'common_first': xs = [first[mode][k] for k in common]
                elif group == 'first': xs = [first[mode][k] for k in eligible]
                else:
                    xs = [near[mode][k][t] for k,t in eligible.items()]
                    for x in xs: x['metric'] = score(x,ref)
                d = summary([x['metric'] for x in xs])
                d['motion_groups'] = {motion:summary([x['metric'] for x in xs if x['metric']['motion']==motion])
                                      for motion in ('quiet','moving')}
                d['interval_s'] = a.stat([x['interval_s'] for x in xs])
                d['age_s'] = a.stat([x['age_s'] for x in xs])
                d['endpoint_envelope_s'] = [min(x['time_s'] for x in xs),max(x['time_s'] for x in xs)]
                summaries[source][group][mode] = d
                if group not in ('first','near4s'): continue
                for x in xs: records.append(dict(stage=group, **x))
                variances = np.array([x['metric']['translation_trace_m2'] for x in xs])
                confident = [x for x in xs if x['metric']['translation_trace_m2'] <= np.quantile(variances,.25)]
                total = sum(x['metric']['rho_error_mm']**2 for x in xs)
                for x in sorted(confident,key=lambda x:x['metric']['translation_normalized_squared'],reverse=True)[:5]:
                    worst.append(dict(stage=group, **x, translation_uncertainty_percentile=float(np.mean(variances<=x['metric']['translation_trace_m2'])*100),
                                      squared_error_share=float(x['metric']['rho_error_mm']**2/total)))
    return summaries, coverage, records, worst


def idx(s):
    assert s[0] == 'x'; return int(s[1:])


def peer_audit(first, ref):
    src = P/'full_on_source'
    main = pin(src/'estimators.log')
    paths = {d:pin(next(src.glob('ros_logs/*/'+pat))) for d,pat in
             [('G2K','kimera_vio_ros-*-stdout.log'),('K2G','glim_rosnode-*-stdout.log')]}
    aligned = {'G2K':[], 'K2G':[]}; frozen = {}; kim = {}
    for tag,f in g.rows(main,['KIMERA_CBS_MEAN_ALIGNMENT_ROW','FL4M_FROZEN_FACTOR_ROW','KIMERA_CBS_REVISION_EDGE_ROW']):
        if tag == 'KIMERA_CBS_REVISION_EDGE_ROW':
            cur, i, j = int(f[0]),int(f[2]),int(f[3])
            mu = np.array(f[22:28],float); C = np.array(f[28:64],float).reshape(6,6)
            assert np.max(np.abs(g.exp(mu)-g.inv(g.pose(f[8:15]))@g.pose(f[15:22]))) < 1e-10
            key = cur,i,j; assert key not in kim
            kim[key] = mu,C
        elif tag == 'FL4M_FROZEN_FACTOR_ROW':
            key = int(f[3]),int(f[4]); assert key not in frozen
            frozen[key] = g.matrix(f[5]).ravel(),g.matrix(f[6])
        else:
            cur, si,sj,ri,rj = map(int,f[:5]); sa,sb,ra,rb,limit = map(int,f[5:10])
            aligned['G2K'].append(dict(cur=cur,si=si,sj=sj,ri=ri,rj=rj,sa=sa,sb=sb,a=ra,b=rb,limit=limit,
                                       raw=g.matrix(f[10]).ravel(),mu=g.matrix(f[11]).ravel(),cov=g.matrix(f[12])))
    for _,f in g.rows(paths['K2G'],['GLIM_CBS_MEAN_ALIGNMENT_ROW']):
        si,sj,ri,rj = map(int,f[:4]); sa,sb,ra,rb,limit = map(a.stamp,f[4:9])
        aligned['K2G'].append(dict(si=si,sj=sj,ri=ri,rj=rj,sa=sa,sb=sb,a=ra,b=rb,limit=limit,
                                 raw=g.matrix(f[9]).ravel(),mu=g.matrix(f[10]).ravel(),cov=g.matrix(f[11])))
    lookup = {(x['source'],x['i'],x['j']):x for x in first.values()}
    # Classic temporary factors may consume a newer publication before a
    # particular insertion. Prove the payload, do not assume revision zero.
    pending=defaultdict(list)
    for direction,rr in aligned.items():
        source=SOURCES[0] if direction=='G2K' else SOURCES[1]
        for x in rr:
            origin=lookup[source,x['si'],x['sj']]
            if np.allclose(x['cov'],origin['cov'],rtol=1e-10,atol=1e-18) and np.allclose(x['raw'],origin['mu'],rtol=1e-10,atol=1e-12):
                x['_source_matches']=[origin]
            else:
                x['_source_matches']=[]
                pending[source,x['si'],x['sj']].append(x)
    if pending:
        nt=native_times('on')
        with (P/'full_on_receiver/events.csv').open() as ef, (P/'full_on_receiver/receipts.jsonl').open() as rf:
            for event,(line,receipt) in enumerate(zip_longest(ef,rf)):
                assert line is not None and receipt is not None
                if not line.startswith('B,'):continue
                r=line.rstrip().split(','); key=r[2],int(r[6]),int(r[7])
                if key not in pending:continue
                C=np.array(r[16:],float).reshape(6,6);mu=np.array(r[10:16],float)
                for x in pending[key]:
                    if not (np.allclose(x['cov'],C,rtol=1e-10,atol=1e-18) and np.allclose(x['raw'],mu,rtol=1e-10,atol=1e-12)):continue
                    meta=json.loads(receipt)['metadata'];update=int(meta['source_update'])
                    assert nt[key[0]][update]<=x['limit']+1000
                    origin=dict(source=key[0],i=key[1],j=key[2],a=int(r[8]),b=int(r[9]),mu=mu,cov=C,
                                revision=int(r[5]),update=update,update_ns=nt[key[0]][update],event_line=event+1)
                    origin['metric']=score(origin,ref);x['_source_matches'].append(origin)
        assert all(x['_source_matches'] for rr in pending.values() for x in rr), 'Unmatched installed peer payload'
    result = {}
    for direction, rr in aligned.items():
        matrices, slots, pre, explicit, info = {}, {}, {}, {}, {}
        tags = ['CBS_ODOM_FACTOR_MATRIX_ROW','CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW',
                'GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW','CBS_ODOM_PREINJECTION_RESIDUAL_ROW','CBS_COVARIANCE_EXCHANGE_MATRIX_ROW']
        for tag,f in g.rows(paths[direction],tags):
            if tag == tags[0] and f[0] == direction:
                key = idx(f[1]),idx(f[2]); assert key not in matrices
                matrices[key] = f[3],g.matrix(f[4])
            elif tag == tags[1] and direction == 'G2K' and f[3]=='active_window_slot_tracked':
                key=tuple(map(idx,f[2].split('->'))); assert key not in slots; slots[key]=f[4:]
            elif tag == tags[2] and direction == 'K2G' and f[0]=='slot_tracked':
                key=idx(f[1]),idx(f[2]); assert key not in slots; slots[key]=f[3:]
            elif tag == tags[3] and f[0]==direction:
                key=idx(f[3]),idx(f[4]); assert key not in pre; pre[key]=np.array(f[7:],float)
            elif tag == tags[4] and f[0]==direction:
                key=idx(f[5]),idx(f[6])
                if f[8:10]==['receiver_preinjection_state','relative_pose_matrix']:
                    assert key not in explicit; explicit[key]=g.matrix(f[12])
                if f[8:10]==['receiver_inserted_factor','information_omega']:
                    assert key not in info; info[key]=g.matrix(f[12])
        keys = {(x['ri'],x['rj']) for x in rr}
        assert len(keys)==len(rr) and keys==set(matrices)==set(slots)
        if direction=='G2K': assert keys==set(frozen)
        maxc=maxmu=maxq=maxinfo=maxpre=0.; maxrounding=0
        missing_pre=[]; failed_pre=[]; joint=[]; pre_samples=[]
        for x in rr:
            key=x['ri'],x['rj']; C=x['cov']; mu=x['mu']
            assert x['sa']<x['sb']<=x['limit'] and x['a']<x['b']<=x['limit']
            action,installed=matrices[key]
            assert action==('active_window_temporary_odom_applied' if direction=='G2K' else 'active_window_temporary_graph_queued')
            maxc=max(maxc,float(np.max(np.abs(C-installed))))
            if direction=='G2K':
                fm,fc=frozen[key]; maxc=max(maxc,float(np.max(np.abs(fc-C))))
                maxmu=max(maxmu,float(np.max(np.abs(g.exp(fm)-g.exp(mu)))))
            source=SOURCES[0] if direction=='G2K' else SOURCES[1]
            candidates=x.pop('_source_matches')
            origin=min(candidates,key=lambda r:r['revision'])
            x['matching_source_revisions']=[c['revision'] for c in candidates]
            assert np.allclose(C,origin['cov'],rtol=1e-10,atol=1e-18)
            assert np.allclose(x['raw'],origin['mu'],rtol=1e-10,atol=1e-12)
            delta=max(abs(origin['a']-x['sa']),abs(origin['b']-x['sb']))
            maxrounding=max(maxrounding,delta); assert delta<=1000
            x['source_event_line']=origin['event_line']; x['source_revision']=origin['revision']
            x['time_s']=(x['b']-START)*1e-9
            x['raw_exact_source']=origin['metric']
            x['aligned']=score(x,ref)
            x['unaligned_on_receiver']=score(dict(x,mu=x['raw']),ref)
            x['from_shift_ms']=(x['a']-origin['a'])/1e6
            x['to_shift_ms']=(x['b']-origin['b'])/1e6
            x['installed_action']=action; x['slot']=slots[key]
            if key in info: maxinfo=max(maxinfo,float(np.max(np.abs(C@info[key]-np.eye(6)))))
            if key in explicit:
                e=g.log(g.inv(g.exp(mu))@explicit[key]); q=float(e@np.linalg.solve(C,e))
                maxq=max(maxq,abs(q-pre[key][7])/(1+q))
                x['explicit_pre']=score(dict(x,mu=g.log(explicit[key])),ref)
                pre_samples.append(x)
            if direction!='G2K': continue
            pk=x['cur']-1,*key; qk=x['cur'],*key
            if pk not in kim or qk not in kim:
                missing_pre.append([x['cur'],*key]); continue
            premu,preC=kim[pk]; postmu,_=kim[qk]
            e=g.log(g.inv(g.exp(mu))@g.exp(premu))
            vals=[np.linalg.norm(e),np.linalg.norm(e[:3]),np.linalg.norm(e[3:])]
            normdiff=float(np.max(np.abs(vals-pre[key][:3])))
            q=float(e@np.linalg.solve(C,e)); qdiff=abs(q-pre[key][7])/(1+q)
            explicitdiff=float(np.max(np.abs(g.exp(premu)-explicit[key]))) if key in explicit else 0.
            diff=max(normdiff,qdiff,explicitdiff); maxpre=max(maxpre,diff)
            if diff>1e-8:
                failed_pre.append(dict(cur=x['cur'],key=key,difference=diff)); continue
            x['verified_pre']=score(dict(x,mu=premu,cov=preC),ref)
            x['post_update']=score(dict(x,mu=postmu,cov=preC),ref)
            # Post covariance is not interpreted: preC used solely to reuse the
            # error routine. Retain only error scalars for post-update output.
            x['post_update']={k:v for k,v in x['post_update'].items() if k in ('rho_error_mm','cartesian_error_mm','rotation_error_deg')}
            x['pre_mu']=premu; x['pre_cov']=preC; x['post_mu']=postmu
            post_residual=g.log(g.inv(g.exp(mu))@g.exp(postmu))
            x['closer_to_incoming_after_update']={
                'translation':bool(np.linalg.norm(post_residual[3:])<np.linalg.norm(e[3:])),
                'rotation':bool(np.linalg.norm(post_residual[:3])<np.linalg.norm(e[:3])),
                'joint_covariance_weighted':bool(post_residual@np.linalg.solve(C,post_residual)<q)}
            joint.append(x)
        assert maxc==0 and maxmu<1e-10 and maxq<1e-8 and maxinfo<1e-8
        def comparisons(xs,pre_name):
            if not xs: return dict(n=0)
            d=dict(n=len(xs),endpoint_envelope_s=[min(x['time_s'] for x in xs),max(x['time_s'] for x in xs)])
            for block,ef,vf in [('translation','rho_error_mm','translation_trace_m2'),('rotation','rotation_error_deg','rotation_trace_rad2')]:
                ae=np.array([x['aligned'][ef] for x in xs]); pe=np.array([x[pre_name][ef] for x in xs])
                bd=dict(peer_error_rms=float(np.sqrt(np.mean(ae*ae))),pre_error_rms=float(np.sqrt(np.mean(pe*pe))),
                        peer_better_fraction=float(np.mean(ae<pe)))
                if pre_name=='verified_pre':
                    av=np.array([x['aligned'][vf] for x in xs]); pv=np.array([x[pre_name][vf] for x in xs])
                    post=np.array([x['post_update'][ef] for x in xs])
                    choose=av<pv
                    selected=np.where(choose,ae,pe)
                    bd.update(peer_lower_covariance_fraction=float(choose.mean()),
                              lower_trace_picks_lower_error_fraction=float(np.mean(choose==(ae<pe))),
                              lower_trace_choice_error_rms=float(np.sqrt(np.mean(selected**2))),
                              oracle_lower_error_rms=float(np.sqrt(np.mean(np.minimum(ae,pe)**2))),
                              post_error_rms=float(np.sqrt(np.mean(post*post))),
                              post_worse_than_pre_fraction=float(np.mean(post>pe)),
                              closer_to_peer_and_worse_than_pre_fraction=float(np.mean([(post[i]>pe[i]) and x['closer_to_incoming_after_update'][block] for i,x in enumerate(xs)])),
                              post_worse_when_peer_worse_fraction=float(np.mean(post[ae>pe]>pe[ae>pe])) if (ae>pe).any() else None)
                d[block]=bd
            return d
        result[direction]=dict(n=len(rr), checks=dict(all_exported_payloads_matched=True,source_revision_counts=dict(Counter(x['source_revision'] for x in rr)),all_installed_slots_matched=True,
            max_installed_covariance_difference=maxc,max_frozen_mean_difference=maxmu,max_timestamp_rounding_ns=maxrounding,
            sampled_explicit_pre_count=len(explicit),max_explicit_pre_q_relative_error=maxq,max_information_inverse_error=maxinfo,
            verified_pre_count=len(joint),missing_pre=missing_pre,failed_pre_checks=failed_pre,max_reconstructed_pre_difference=maxpre),
            summaries={stage:summary([x[stage] for x in rr]) for stage in ('raw_exact_source','aligned','unaligned_on_receiver')},
            verified_pre_comparison=comparisons(joint,'verified_pre'),explicit_pre_comparison=comparisons(pre_samples,'explicit_pre'),
            motion_comparisons={m:comparisons([x for x in joint if x['aligned']['motion']==m],'verified_pre') for m in ('quiet','moving')},
            time_comparisons=[dict(from_s=t,to_s=t+30,comparison=comparisons([x for x in joint if t<=x['time_s']<t+30],'verified_pre')) for t in range(0,450,30)],
            shifts={k:a.stat([x[k] for x in rr]) for k in ('from_shift_ms','to_shift_ms')},records=rr)
    return result


def serial(x):
    if isinstance(x,np.ndarray): return x.tolist()
    if isinstance(x,np.generic): return x.item()
    raise TypeError(type(x).__name__)


def save(path,obj):
    with path.open('x') as f: json.dump(obj,f,indent=2,default=serial,allow_nan=False)


def render(summaries,records,peers):
    plt.rcParams.update({'font.size':10,'savefig.facecolor':'white','svg.fonttype':'none'})
    colors={'off':'#176db5','on':'#dc6b16'}
    names=dict(zip(SOURCES,('GLIM / rear Velodyne','Kimera / front stereo')))
    for block,unit in [('translation','mm'),('rotation','degrees')]:
        fig,axs=plt.subplots(2,2,figsize=(13,9))
        for row,stage in enumerate(('first','near4s')):
            for col,source in enumerate(SOURCES):
                ax=axs[row,col]
                for mode in MODES:
                    d=summaries[source][stage][mode][block]
                    ax.plot([q['actual_percent'] for q in d['risk']], [q['relative_rms'] for q in d['risk']],'-o',ms=3,color=colors[mode],
                            label=f'{mode.upper()}: all RMS {d["error_rms"]:.3g} {unit}, rank {d["rank"]:.2f}')
                ax.axhline(1,color='gray',ls='--',lw=1)
                ax.set_title(f'{names[source]} — {stage} ({summaries[source][stage]["off"]["n"]:,} intervals)')
                ax.set_xlabel('Beliefs retained, lowest covariance trace first (%)')
                ax.set_ylabel('Retained error RMS / all-belief error RMS')
                ax.legend(fontsize=8); ax.grid(alpha=.2)
        fig.suptitle(f'LEICA-2 Velodyne: does {block} confidence select healthier beliefs?',fontsize=15)
        fig.text(.04,.018,'Below 1 = cleaner subset. Same intervals / physical revision age OFF–ON. Shared-CPT7 reference; no factors removed.',fontsize=10)
        fig.tight_layout(rect=(0,.05,1,.96)); fig.savefig(OUT/f'{block}_ranking.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(13,8.5))
    for row,source in enumerate(SOURCES):
        for mode in MODES:
            xs=[x for x in records if x['stage']=='first' and x['source']==source and x['mode']==mode]
            bins=[]
            for start in range(0,450,10):
                rr=[x for x in xs if start<=x['time_s']<start+10]
                if not rr: continue
                bins.append((start+5,np.sqrt(np.mean([x['metric']['translation_trace_m2'] for x in rr]))*1000,
                             np.sqrt(np.mean([x['metric']['rho_error_mm']**2 for x in rr]))))
            z=np.array(bins)
            axs[row,0].plot(z[:,0],z[:,1],color=colors[mode],label=mode.upper())
            axs[row,1].plot(z[:,0],z[:,2],color=colors[mode],label=mode.upper())
        for col in (0,1):
            axs[row,col].set_title(names[source]+(' — reported norm scale' if col==0 else ' — actual relative-error RMS'))
            axs[row,col].set_yscale('log');axs[row,col].set_xlabel('Seconds after capture start')
            axs[row,col].set_ylabel('mm');axs[row,col].grid(alpha=.2);axs[row,col].legend()
    fig.suptitle('LEICA-2: first-publication translation uncertainty and error (10 s bins)',fontsize=15)
    fig.text(.04,.018,'Same eligible OFF/ON interval cohort. sqrt(mean trace) is not a confidence bound; short relative error is not ATE.',fontsize=10)
    fig.tight_layout(rect=(0,.05,1,.96));fig.savefig(OUT/'translation_time.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(13,5.5))
    rr=[x for x in peers['G2K']['records'] if 'verified_pre' in x]
    for ax,(block,ef,vf,scale,unit) in zip(axs,[('translation','rho_error_mm','translation_trace_m2',1e6,'mm'),('rotation','rotation_error_deg','rotation_trace_rad2',(180/np.pi)**2,'deg')]):
        for field,label,color in [('aligned','Incoming GLIM','#cb3b36'),('verified_pre','Kimera before update','#176db5')]:
            d=ranking([x[field][vf]*scale for x in rr],[x[field][ef] for x in rr])
            ax.plot([b['norm'] for b in d['bins']],[b['error_rms'] for b in d['bins']],'-o',color=color,label=label)
        limits=[v for line in ax.lines for xy in (line.get_xdata(),line.get_ydata()) for v in xy]
        lo,hi=min(limits)*.7,max(limits)*1.4
        ax.plot([lo,hi],[lo,hi],'--',color='gray',lw=1,label='Equal magnitude guide')
        ax.set_xscale('log');ax.set_yscale('log');ax.set_xlabel(f'Reported norm: sqrt(mean trace) ({unit})')
        ax.set_ylabel(f'Actual error RMS ({unit})');ax.set_title(block.capitalize());ax.legend(fontsize=8);ax.grid(alpha=.2)
    fig.suptitle(f'Actual GLIM → Kimera factors versus verified Kimera pre-state ({len(rr):,} intervals)',fontsize=14)
    fig.text(.04,.02,'Identical receiver endpoints. Incoming mean aligned; covariance unchanged. Bins are descriptive, not calibrated probabilities.',fontsize=9)
    fig.tight_layout(rect=(0,.07,1,.94));fig.savefig(OUT/'actual_g2k.png',dpi=160);plt.close(fig)


def main():
    assert not OUT.exists(), 'Refuse overwrite; use saved outputs once created'
    assert g.self_test()
    v=np.arange(1,201,dtype=float)
    assert ranking(v,np.sqrt(v))['risk'][8]['relative_rms']<1
    assert ranking(v,np.sqrt(v[::-1]))['risk'][8]['relative_rms']>1
    assert all(r['n']==200 for r in ranking(np.ones(200),v)['risk'])
    assert np.allclose([r['relative_rms'] for r in ranking(v,v)['risk']], [r['relative_rms'] for r in ranking(100*v,v)['risk']])
    for p in (Path(__file__),H/'PROTOCOL.txt',Path(a.__file__),Path(g.__file__),BASE/'results.json',BASE/'closeout.json',P/'protected_manifest.json'):
        pin(p)
    for mode in MODES:
        for name in ('local_provenance.json','candidate_validation.json','velodyne_validation.json'):
            assert json.loads(pin(P/f'full_{mode}_source/{name}').read_text())['passed']
        pin(P/f'full_{mode}_control/complete.json')
    first,near,checks={},{},{}
    for mode in MODES:
        print('Reading immutable journal:',mode,flush=True)
        first[mode],near[mode],checks[mode]=read_mode(mode)
    print('Scoring exact source intervals and matched revision ages',flush=True)
    ref=g.Reference(pin(GT))
    summaries,coverage,records,worst=make_cohorts(first,near,ref)
    print('Joining actual installed peer factors and receiver pre/post states',flush=True)
    peers=peer_audit(first['on'],ref)
    bins=[]
    for source in SOURCES:
        for stage in ('first','near4s'):
            for mode in MODES:
                xs=[x for x in records if x['source']==source and x['stage']==stage and x['mode']==mode]
                for t in range(0,450,30):
                    rr=[x for x in xs if t<=x['time_s']<t+30]
                    if rr: bins.append(dict(source=source,stage=stage,mode=mode,from_s=t,to_s=t+30,summary=summary([x['metric'] for x in rr])))
    print('Verifying all input hashes before writing new audit outputs',flush=True)
    assert all(g.sha(Path(p))==h for p,h in HASHES.items())
    OUT.mkdir()
    save(OUT/'summary.json',summaries); save(OUT/'coverage.json',coverage)
    save(OUT/'worst_confident.json',worst);save(OUT/'time_bins.json',bins)
    save(OUT/'peer_summary.json',{d:{k:v for k,v in data.items() if k!='records'} for d,data in peers.items()})
    with gzip.open(OUT/'source_samples.jsonl.gz','xt') as f:
        # Include all first versions once, plus eligible near4s versions.
        for mode in MODES:
            for x in first[mode].values(): f.write(json.dumps(dict(stage='all_first',**x),default=serial,allow_nan=False)+'\n')
        for x in records:
            if x['stage']=='near4s': f.write(json.dumps(x,default=serial,allow_nan=False)+'\n')
    with gzip.open(OUT/'peer_samples.jsonl.gz','xt') as f:
        for direction,d in peers.items():
            for x in d['records']: f.write(json.dumps(dict(direction=direction,**x),default=serial,allow_nan=False)+'\n')
    render(summaries,records,peers)
    save(OUT/'audit.json',dict(passed=True,source_checks=checks,reference_max_bracket_ns=ref.max_bracket_ns,
         input_sha256=HASHES,inputs_unchanged=True,geometry_and_ranking_tests=True,
         dense_whitening_checked=True,matched_records=len(records),no_estimator_or_replay=True))
    print(json.dumps(dict(source_checks=checks,matched_records=len(records),peer_checks={d:x['checks'] for d,x in peers.items()}),indent=2))


if __name__=='__main__': main()
