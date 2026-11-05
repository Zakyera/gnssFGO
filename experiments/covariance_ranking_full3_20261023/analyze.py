"""Two read-only ranking diagnostics on completed full GrandTour captures."""
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from itertools import zip_longest

import numpy as np

sys.dont_write_bytecode = True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/evo/lib/python3.12/site-packages')
from scipy.stats import spearmanr
HERE = Path(__file__).resolve().parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    return mod


prev = load('cov_error', HERE.parent/'covariance_accuracy_full3_20261023/make_plots.py')
old, geo, plt = prev.old, prev.geo, prev.plt
OUT = HERE/'outputs'
AGE = 4_000_000_000
TOLERANCE = 150_000_000
SOURCES = tuple(old.SOURCES)
MODES = ('off', 'on')
STAGES = ('first', 'near4s')
BLOCKS = ('translation', 'rotation')


def write_json(path, obj):
    with path.open('x') as f: json.dump(obj, f, indent=2, allow_nan=False)


def pin(path, hashes):
    hashes[str(path)] = old.digest(path)


def stats(a):
    return prev.stat(a)


def physical_key(p):
    return p['source'], p['from_timestamp_ns'], p['to_timestamp_ns']


def source_times(root, mode, hashes):
    maps = {}
    for source, stem in [('glim_hesai', 'glim'), ('kimera_front', 'kimera')]:
        path = root/f'full_{mode}_source/{stem}_odometry.csv'; pin(path, hashes)
        mapping = {}
        for r in csv.DictReader(path.open()):
            assert r['field.child_frame_id'] == 'cpt7_imu'
            index, t = int(r['field.header.seq']), int(r['field.header.stamp'])
            assert index not in mapping
            mapping[index] = t
        assert np.all(np.diff(list(mapping.values())) > 0)
        maps[source] = mapping
    return maps


def read_case(seq, mode, prior, old_samples, hashes):
    root = old.RUNS/old.CONFIG[seq]['run']/'prepared'
    times = source_times(root, mode, hashes)
    path = root/f'full_{mode}_receiver/events.csv'
    receipt = root/f'full_{mode}_receiver/receipts.jsonl'
    expected_hash = prior['input_audit'][seq]['cases'][mode]['sha256']
    eh, rh = hashlib.sha256(), hashlib.sha256()
    first, candidates, seen = {}, {}, {}
    counts = dict.fromkeys(SOURCES, 0)
    max_endpoint_rounding_ns = 0
    with path.open('rb') as f, receipt.open('rb') as g:
        for event, (raw, rr) in enumerate(zip_longest(f, g)):
            assert raw is not None and rr is not None
            eh.update(raw); rh.update(rr)
            if not raw.startswith(b'B,'): continue
            meta = json.loads(rr)
            r = raw.rstrip().decode().split(',')
            assert len(r) == 52 and meta['event'] == event and meta['kind'] == 'B'
            assert meta['receipt_ros_ns'] == int(r[1])
            m = meta['metadata']; source = r[2]
            assert m['source'] == source
            ordinal, rev, i, j, ti, tj = map(int, r[4:10])
            stable = source, r[3], i, j
            assert rev == seen.get(stable, -1)+1
            seen[stable] = rev; counts[source] += 1
            if source == 'glim_hesai' and int(m['source_revision']) == 0:
                from decimal import Decimal
                assert int(Decimal(m['source_update'])*Decimal(10**9)) == tj
                update = j
            else:
                update = int(m['source_update'])
            assert update >= j
            # Source header seq is the estimator state ID, already used in the
            # preceding source audits; explicit per-edge checks guard this join.
            for index, stamp in [(i, ti), (j, tj)]:
                rounding = abs(times[source][index]-stamp)
                max_endpoint_rounding_ns = max(max_endpoint_rounding_ns, rounding)
                assert rounding <= 1000, (source, index, rounding)
            update_ns = times[source][update]
            age_ns = update_ns-tj
            if rev != 0 and abs(age_ns-AGE) > TOLERANCE: continue
            C = np.asarray(r[16:], float).reshape(6,6)
            mu = np.asarray(r[10:16], float)
            assert np.isfinite(C).all() and np.isfinite(mu).all()
            assert np.max(np.abs(C-C.T)) <= 1e-12 and np.linalg.eigvalsh(C).min() > 0
            p = dict(sequence=seq, mode=mode, source=source, session=r[3], ordinal=ordinal,
                     revision=rev, from_id=i, to_id=j, from_timestamp_ns=ti,
                     to_timestamp_ns=tj, arrival_ns=int(r[1]), update_id=update,
                     update_timestamp_ns=update_ns, source_age_s=age_ns*1e-9,
                     physical_end_s=(tj-old.CONFIG[seq]['start_ns'])*1e-9,
                     interval_s=(tj-ti)*1e-9, event_line=event+1,
                     translation_trace_m2=float(np.trace(C[3:,3:])),
                     rotation_trace_rad2=float(np.trace(C[:3,:3])), mu=mu)
            key = physical_key(p)
            if rev == 0:
                assert key not in first
                first[key] = p
                original = old_samples[(mode, source, i, j)]
                for name in ['from_timestamp_ns','to_timestamp_ns','event_line','arrival_ns']:
                    assert p[name] == int(original[name])
                for name in ['translation_trace_m2','rotation_trace_rad2']:
                    assert math.isclose(p[name], float(original[name]), rel_tol=1e-14)
                assert np.array_equal(mu, np.asarray([original[f'belief_log_{k}'] for k in range(6)],float))
            if abs(age_ns-AGE) <= TOLERANCE:
                history = candidates.setdefault(key, {})
                assert update_ns not in history, ('duplicate candidate at one source epoch', seq, mode, key, update)
                history[update_ns] = p
    assert eh.hexdigest() == expected_hash
    hashes[str(path)] = eh.hexdigest(); hashes[str(receipt)] = rh.hexdigest()
    for source in SOURCES:
        expected = prior['input_audit'][seq]['cases'][mode]['checks'][source]
        assert counts[source] == expected['versions']
        assert sum(k[0]==source for k in first) == expected['stable_ids']
    return first, candidates, dict(version_counts=counts,
           max_endpoint_rounding_ns=max_endpoint_rounding_ns, first_ids=len(first))


def measure(p, ref):
    target = geo.inv(ref.at(p['from_timestamp_ns'])) @ ref.at(p['to_timestamp_ns'])
    Z = geo.exp(p['mu']); e = geo.log(geo.inv(Z) @ target)
    p['translation_error_mm'] = float(np.linalg.norm(e[3:])*1000)
    p['cartesian_translation_error_mm'] = float(np.linalg.norm(Z[:3,3]-target[:3,3])*1000)
    p['rotation_error_deg'] = float(np.linalg.norm(e[:3])*180/np.pi)
    p['reference_translation_m'] = float(np.linalg.norm(target[:3,3]))
    for i,v in enumerate(p.pop('mu')): p[f'belief_log_{i}'] = float(v)


def curves(v, e):
    v, e = np.asarray(v,float), np.asarray(e,float)
    assert len(v) == len(e) and len(v) >= 100
    assert np.isfinite(v).all() and np.isfinite(e).all() and min(v) > 0
    # Quantile boundaries include ties together. Repeated boundaries collapse.
    cuts = np.unique(np.quantile(v, np.linspace(0,1,11)))
    assignments = np.searchsorted(cuts[1:-1], v, side='right')
    bins = []
    for b in range(max(1,len(cuts)-1)):
        mask = assignments == b
        if not mask.any(): continue
        bins.append(dict(n=int(mask.sum()), predicted_norm=float(np.sqrt(v[mask].mean())),
                         error_rms=float(np.sqrt(np.mean(e[mask]**2))),
                         variance_min=float(v[mask].min()), variance_max=float(v[mask].max()),
                         error_median=float(np.median(e[mask])), error_p95=float(np.percentile(e[mask],95))))
    assert sum(b['n'] for b in bins) == len(v)
    assert all(bins[i]['variance_max'] <= bins[i+1]['variance_min'] for i in range(len(bins)-1))
    base = float(np.sqrt(np.mean(e*e)))
    ordered = np.sort(v)
    risk = []
    for percent in range(10, 101, 5):
        threshold = ordered[math.ceil(percent*len(v)/100)-1]
        mask = v <= threshold
        rms = float(np.sqrt(np.mean(e[mask]**2)))
        risk.append(dict(requested_percent=percent, kept=int(mask.sum()),
                         actual_percent=float(mask.mean()*100), threshold_variance=float(threshold),
                         error_rms=rms, rms_over_all=rms/base))
    assert risk[-1]['kept'] == len(v) and math.isclose(risk[-1]['rms_over_all'],1.)
    return dict(n=len(v), full_rms=base, predicted_rms_norm=float(np.sqrt(v.mean())),
                rank_correlation=(float(spearmanr(v,e).statistic)
                                  if np.ptp(v)>0 and np.ptp(e)>0 else None), bins=bins, risk=risk)


def summarize(points):
    return {block:curves([p['translation_trace_m2']*1e6 if block=='translation'
                           else p['rotation_trace_rad2']*(180/np.pi)**2 for p in points],
                          [p['translation_error_mm'] if block=='translation' else p['rotation_error_deg'] for p in points])
            for block in BLOCKS}


def render(summary):
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.labelsize':10,
                         'svg.fonttype':'none','savefig.facecolor':'white','path.simplify':False})
    for block in BLOCKS:
        unit = 'mm' if block=='translation' else 'deg'
        for stage in STAGES:
            label = 'First publication' if stage=='first' else 'Near 4 s source age'
            for kind in ('reliability','risk'):
                fig, axs = plt.subplots(3,2,figsize=(14,13))
                fig.subplots_adjust(top=.87,bottom=.07,left=.09,right=.98,hspace=.42,wspace=.24)
                title = ('Does uncertainty track error?' if kind=='reliability' else 'Does selecting confident beliefs reduce error?')
                fig.suptitle(f'{block.capitalize()} — {title}',fontsize=17,x=.06,ha='left',y=.98)
                fig.text(.06,.945,label+' · same matched belief cohort for both ages and CBS OFF/ON',fontsize=11)
                fig.text(.06,.919,('Each point: uncertainty-bin RMS; diagonal is a magnitude guide, not a calibration certificate.'
                                   if kind=='reliability' else 'Below 1: retained beliefs have lower error than the full cohort. No factors were actually removed.'),fontsize=10)
                fig.text(.31,.895,'GLIM',ha='center',fontsize=12,weight='bold')
                fig.text(.78,.895,'KIMERA',ha='center',fontsize=12,weight='bold')
                for row,seq in enumerate(old.CONFIG):
                    for col,source in enumerate(SOURCES):
                        ax = axs[row,col]
                        vals=[]
                        for mode in MODES:
                            d = summary[seq][source][stage][mode][block]
                            if kind=='reliability':
                                x=[b['predicted_norm'] for b in d['bins']]
                                y=[b['error_rms'] for b in d['bins']]
                                text=f'{mode.upper()} (rank r={d["rank_correlation"]:.2f})'
                            else:
                                x=[b['actual_percent'] for b in d['risk']]
                                y=[b['rms_over_all'] for b in d['risk']]
                                text=f'{mode.upper()} (all RMS {d["full_rms"]:.3g} {unit})'
                            ax.plot(x,y,'o-',ms=4,lw=1.4,color=old.COLOR[mode],label=text)
                            vals.extend(x if kind=='reliability' else [])
                            vals.extend(y)
                        if kind=='reliability':
                            lo,hi=10**math.floor(math.log10(min(vals))),10**math.ceil(math.log10(max(vals)))
                            ax.plot([lo,hi],[lo,hi],'--',color='#777777',lw=1,label='Equal magnitude')
                            ax.set_xscale('log');ax.set_yscale('log')
                            ax.set_xlim(lo,hi);ax.set_ylim(lo,hi)
                            ax.set_xlabel(f'Reported norm scale: √mean(trace) ({unit})')
                            ax.set_ylabel(f'Actual relative-error RMS ({unit})')
                        else:
                            ax.axhline(1,color='#777777',ls='--',lw=1)
                            ax.set_xlim(8,102);ax.set_ylim(0,max(1.1,max(vals)*1.08))
                            ax.set_xlabel('Beliefs retained, lowest uncertainty first (%)')
                            ax.set_ylabel('Retained RMS / all-belief RMS')
                        n=summary[seq][source][stage]['off'][block]['n']
                        ax.set_title(f'{seq} · {n:,} identical intervals per mode',loc='left',fontsize=11)
                        ax.grid(True,which='major',color='#D6DDE5',lw=.6)
                        ax.spines[['top','right']].set_visible(False)
                        ax.legend(loc='best',fontsize=8)
                fig.text(.06,.027,'Same-endpoint SE(3) tangent errors vs shared-CPT7 IE-TC; overlapping edges are correlated. No calibration fit or fusion run.',fontsize=9)
                stem=OUT/f'{block}_{stage}_{kind}'
                for ext in ('png','svg'):fig.savefig(stem.with_suffix('.'+ext),dpi=165)
                plt.close(fig)


def self_test():
    assert prev.self_test()
    v=np.linspace(1,100,200)
    perfect=curves(v,np.sqrt(v))
    assert all(math.isclose(b['predicted_norm'],b['error_rms']) for b in perfect['bins'])
    assert perfect['risk'][8]['rms_over_all'] < 1
    inverse=curves(v,np.sqrt(v[::-1]))
    assert inverse['risk'][8]['rms_over_all'] > 1
    flat=curves(np.ones(200),np.linspace(.1,2,200))
    assert all(r['kept']==200 for r in flat['risk'])
    scaled=curves(v*100,np.sqrt(v))
    assert np.allclose([r['rms_over_all'] for r in perfect['risk']], [r['rms_over_all'] for r in scaled['risk']])


def main():
    assert not OUT.exists(), 'Refuse to overwrite an existing analysis'
    # Constant covariance yields undefined Spearman correlation, intentionally
    # tested only for selection ties, never emitted as a numeric correlation.
    self_test(); OUT.mkdir()
    hashes={}
    for path in [Path(__file__),HERE/'PROTOCOL.txt',Path(prev.__file__),Path(old.__file__),Path(geo.__file__)]:pin(path,hashes)
    prior_path=old.OUT/'audit.json';pin(prior_path,hashes)
    prior=json.loads(prior_path.read_text());assert prior['passed']
    previous_audit=prev.OUT/'audit.json';pin(previous_audit,hashes)
    assert json.loads(previous_audit.read_text())['passed']
    audit, summary, all_records = {}, {}, []
    for seq in old.CONFIG:
        sample_path=prev.OUT/seq.lower()/'covariance_and_error_samples.csv';pin(sample_path,hashes)
        samples={(p['mode'],p['source'],int(p['from_id']),int(p['to_id'])):p
                 for p in csv.DictReader(sample_path.open()) if p['stage']=='first'}
        first, candidates, checks = {}, {}, {}
        for mode in MODES:first[mode],candidates[mode],checks[mode]=read_case(seq,mode,prior,samples,hashes)
        bag=geo.WORK/'src/datasets/GrandTour'/prev.BAGS[seq];pin(bag,hashes)
        ref=geo.Reference(bag)
        audit[seq]=dict(input_checks=checks,cohorts={},all_first_context={})
        summary[seq]={}
        for mode in MODES:
            for p in first[mode].values():
                measure(p,ref)
                old_point=samples[(mode,p['source'],p['from_id'],p['to_id'])]
                assert math.isclose(p['translation_error_mm'],float(old_point['tangent_translation_error_mm']),abs_tol=1e-9)
                assert math.isclose(p['rotation_error_deg'],float(old_point['rotation_error_deg']),abs_tol=1e-9)
            audit[seq]['all_first_context'][mode]={s:summarize([p for k,p in first[mode].items() if k[0]==s]) for s in SOURCES}
        common=set(first['off'])&set(first['on'])
        for source in SOURCES:
            keys=sorted(k for k in common if k[0]==source)
            eligible=[]; omitted=[]
            populations={stage:{m:[] for m in MODES} for stage in STAGES}
            for key in keys:
                shared=set(candidates['off'].get(key,{}))&set(candidates['on'].get(key,{}))
                if not shared:
                    omitted.append(key);continue
                epoch=min(shared,key=lambda t:(abs(t-key[2]-AGE),t))
                eligible.append(key)
                for mode in MODES:
                    p0=dict(first[mode][key],stage='first')
                    p4=candidates[mode][key][epoch]
                    measure(p4,ref);p4=dict(p4,stage='near4s')
                    assert abs(p4['source_age_s']-4)<=TOLERANCE*1e-9
                    populations['first'][mode].append(p0);populations['near4s'][mode].append(p4)
                    all_records.extend([p0,p4])
            assert eligible
            audit[seq]['cohorts'][source]=dict(
                all_first_counts={m:sum(k[0]==source for k in first[m]) for m in MODES},
                exact_common_intervals=len(keys),eligible=len(eligible),
                no_shared_near4s_update=len(omitted),excluded_interval_endpoints=omitted,
                unmatched={m:sum(k[0]==source for k in set(first[m])-common) for m in MODES},
                physical_endpoint_range_s=[min(p['physical_end_s'] for p in populations['first']['off']),max(p['physical_end_s'] for p in populations['first']['off'])],
                actual_source_age_s=stats([p['source_age_s'] for p in populations['near4s']['off']]),
                interval_duration_s=stats([p['interval_s'] for p in populations['first']['off']]))
            summary[seq][source]={stage:{m:summarize(points) for m,points in pop.items()}
                                  for stage,pop in populations.items()}
        audit[seq]['reference']=dict(path=str(bag),max_bracket_ns=ref.max_bracket_ns,unsupported_rows=0)
        print(json.dumps({seq:{s:{k:v for k,v in c.items() if k!='excluded_interval_endpoints'} for s,c in audit[seq]['cohorts'].items()}}),flush=True)
    render(summary)
    with (OUT/'samples.csv').open('x') as f:
        w=csv.DictWriter(f,fieldnames=list(all_records[0]));w.writeheader();w.writerows(all_records)
    flat=[]
    for seq in summary:
        for source in SOURCES:
            for stage in STAGES:
                for mode in MODES:
                    for block,d in summary[seq][source][stage][mode].items():
                        halves={r['requested_percent']:r for r in d['risk']}
                        flat.append(dict(sequence=seq,source=source,stage=stage,mode=mode,block=block,
                          n=d['n'],rank_correlation=d['rank_correlation'],all_rms=d['full_rms'],
                          reported_rms_norm=d['predicted_rms_norm'],
                          lowest_half_rms=halves[50]['error_rms'],lowest_half_rms_ratio=halves[50]['rms_over_all'],
                          lowest_80pct_rms_ratio=halves[80]['rms_over_all']))
    with (OUT/'summary.csv').open('x') as f:
        w=csv.DictWriter(f,fieldnames=list(flat[0]));w.writeheader();w.writerows(flat)
    assert all(old.digest(Path(p))==h for p,h in hashes.items()), 'Input changed'
    write_json(OUT/'audit.json',dict(passed=True,inputs_unchanged=True,input_hashes=hashes,
               self_tests_passed=True,experiment_executed=False,age_target_s=4,age_tolerance_s=.15,
               cohort_audit=audit,summaries=summary,selected_rows=len(all_records),figures=8))
    print('PASS: two diagnostics, translation/rotation, first and common near-4s age.',flush=True)


if __name__=='__main__':main()
