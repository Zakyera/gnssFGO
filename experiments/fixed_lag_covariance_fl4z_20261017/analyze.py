"""Saved numeric telemetry audit only; no estimator or optimizer execution."""
import csv
import importlib.util
import json
import sys
from collections import defaultdict
from decimal import Decimal
from pathlib import Path

import numpy as np
from scipy.stats import chi2, spearmanr

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
spec = importlib.util.spec_from_file_location('fl4y', HERE.parent / 'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py')
geo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geo)
CASES = {
    'eth1': dict(off='gnssfgo_fixed_lag_live120_off_on_fl4t_20261016',
                 on='gnssfgo_fixed_lag_live120_on_fl4v_20261017', prefix='live120',
                 bag='2024-10-01-11-29-55_cpt7_ie_tc.bag',
                 lower=1727775011599997300, upper=1727775116999896800),
    'spx3': dict(off='gnssfgo_spx3_live60_off_on_fl4w_20261017',
                 on='gnssfgo_spx3_live60_off_on_fl4w_20261017', prefix='live60',
                 bag='SPX-3/2024-11-02-17-43-10_cpt7_ie_tc.bag', lower=0, upper=2**63-1),
    'eig2': dict(off='gnssfgo_eig2_live60_off_on_fl4x_20261017',
                 on='gnssfgo_eig2_live60_off_on_fl4x_20261017', prefix='live60',
                 bag='EIG-2/2024-11-03-13-59-54_cpt7_ie_tc.bag', lower=0, upper=2**63-1),
}
SOURCES = ('glim_hesai', 'kimera_front')


def stamp(s):
    return int((Decimal(s)*Decimal(1000000000)).to_integral_value())


def stat(a):
    a = np.asarray(a, float)
    assert a.size and np.isfinite(a).all()
    return dict(n=len(a), minimum=float(a.min()), median=float(np.median(a)),
                mean=float(a.mean()), rms=float(np.sqrt(np.mean(a*a))),
                p95=float(np.percentile(a, 95)), maximum=float(a.max()))


def rank(a, b):
    if len(set(a)) < 2 or len(set(b)) < 2:
        return None
    return float(spearmanr(a, b).statistic)


def write(path, data):
    with path.open('x') as f:
        json.dump(data, f, indent=2, allow_nan=False)


def read_events(root, prefix, mode, cfg, inputs):
    folder = root / f'{prefix}_{mode}_receiver'
    files = [folder/'events.csv', folder/'receipts.jsonl']
    for p in files:
        inputs[str(p)] = geo.sha(p)
    metadata = {}
    with files[1].open() as f:
        for line in f:
            r = json.loads(line)
            if r['kind'] == 'B':
                metadata[r['event']] = r
    versions = defaultdict(dict)
    first = {}
    raw_counts = defaultdict(int)
    all_covs = []
    excluded = 0
    update_times = defaultdict(dict)
    with files[0].open() as f:
        for idx, r in enumerate(csv.reader(f)):
            if r[0] != 'B':
                continue
            assert len(r) == 52
            receipt = metadata.pop(idx)
            m = receipt['metadata']
            assert m['source'] == r[2] and receipt['receipt_ros_ns'] == int(r[1])
            source, i, j = r[2], int(r[6]), int(r[7])
            a, b, arrival = int(r[8]), int(r[9]), int(r[1])
            assert a < b <= arrival
            mu, cov = np.asarray(r[10:16], float), np.asarray(r[16:], float).reshape(6, 6)
            all_covs.append(cov)
            raw_counts[source] += 1
            if source == 'glim_hesai':
                # EXPORT logs use the newest pose time in this field, whereas
                # REVISION logs use its state index. Normalize to state index.
                if int(m['source_revision']) == 0:
                    assert stamp(m['source_update']) == b
                    update = j
                else:
                    update = int(m['source_update'])
                assert update >= j
            else:
                update = int(m['source_update'])
                for k, t in [(i, a), (j, b)]:
                    if k in update_times[source]:
                        assert update_times[source][k] == t
                    update_times[source][k] = t
            if a < cfg['lower'] or b > cfg['upper'] or arrival > cfg['upper']:
                excluded += 1
                continue
            key = (source, a, b)
            x = dict(source=source, source_i=i, source_j=j, a=a, b=b,
                     arrival=arrival, update=update, revision=int(r[5]),
                     source_revision=int(m['source_revision']), mu=mu, cov=cov)
            if update in versions[key]:
                old = versions[key][update]
                assert np.array_equal(old['mu'], mu) and np.array_equal(old['cov'], cov)
            versions[key][update] = x
            if key not in first:
                assert x['revision'] == 0, ('missing first publication', key)
                first[key] = x
    assert not metadata
    cs = np.asarray(all_covs)
    assert np.isfinite(cs).all()
    sym = float(np.max(np.abs(cs - cs.transpose(0, 2, 1))))
    eig = np.linalg.eigvalsh(cs)
    assert sym <= 1e-12 and eig.min() > 0
    checks = dict(versions=dict(raw_counts), excluded_versions=excluded,
                  selected_edges=len(first), max_symmetry_error=sym,
                  min_eigenvalue=float(eig.min()), finite_spd=True)
    return versions, first, checks, update_times


def measure(x, reference, target=None):
    C = x['cov']
    Z = geo.exp(x['mu'])
    if target is None:
        target = geo.inv(reference.at(x['a'])) @ reference.at(x['b'])
    e = geo.log(geo.inv(Z) @ target)
    tr = float(np.trace(C[:3, :3]))
    tt = float(np.trace(C[3:, 3:]))
    rvals = np.linalg.eigvalsh(C[:3, :3])
    tvals = np.linalg.eigvalsh(C[3:, 3:])
    white = np.linalg.solve(np.linalg.cholesky(C), e)
    d6 = float(e @ np.linalg.solve(C, e))
    assert abs(d6-white@white) <= 1e-7*(1+d6)
    return dict(rotation_trace_rad2=tr, translation_trace_m2=tt,
                rotation_sigma_norm_deg=float(np.sqrt(tr)*180/np.pi),
                translation_sigma_norm_mm=float(np.sqrt(tt)*1000),
                rotation_error_deg=float(np.linalg.norm(e[:3])*180/np.pi),
                rho_error_mm=float(np.linalg.norm(e[3:])*1000),
                cartesian_error_mm=float(np.linalg.norm(Z[:3, 3]-target[:3, 3])*1000),
                rotation_normalized_squared=float(e[:3]@np.linalg.solve(C[:3, :3], e[:3])),
                translation_normalized_squared=float(e[3:]@np.linalg.solve(C[3:, 3:], e[3:])),
                joint_normalized_squared=d6,
                rotation_block_eigen_ratio=float(rvals[-1]/rvals[0]),
                translation_block_eigen_ratio=float(tvals[-1]/tvals[0]),
                phi=e[:3].tolist(), rho=e[3:].tolist())


def summarize(records):
    assert records
    result = {key: stat([r[key] for r in records]) for key in records[0]
              if isinstance(records[0][key], (int, float))}
    for part, error, variance in [
        ('rotation', 'rotation_error_deg', 'rotation_trace_rad2'),
        ('translation', 'rho_error_mm', 'translation_trace_m2')]:
        err = np.asarray([r[error] for r in records])
        var = np.asarray([r[variance] for r in records])
        sigma = [r[part+'_sigma_norm_'+('deg' if part == 'rotation' else 'mm')] for r in records]
        cut1, cut3 = np.quantile(err, [.25, .75])
        lo, hi = err <= cut1, err >= cut3
        d = np.asarray([r[part+'_normalized_squared'] for r in records])
        eq = np.quantile(var, [.25, .75])
        result[part+'_quality'] = dict(
            spearman_variance_vs_error=rank(var.tolist(), err.tolist()),
            low_error_quartile=dict(n=int(lo.sum()), error_max=float(cut1),
                median_variance=float(np.median(var[lo])), median_sigma_norm=float(np.median(np.asarray(sigma)[lo]))),
            high_error_quartile=dict(n=int(hi.sum()), error_min=float(cut3),
                median_variance=float(np.median(var[hi])), median_sigma_norm=float(np.median(np.asarray(sigma)[hi]))),
            high_over_low_error_quartile_variance=float(np.median(var[hi])/np.median(var[lo])),
            lowest_variance_quartile_error_rms=float(np.sqrt(np.mean(err[var<=eq[0]]**2))),
            highest_variance_quartile_error_rms=float(np.sqrt(np.mean(err[var>=eq[1]]**2))),
            nominal_95_ellipsoid_fraction_inside=float(np.mean(d<=chi2.ppf(.95, 3))),
            rms_error_over_rms_reported_norm=float(np.sqrt(np.mean(err**2)/np.mean(np.asarray(sigma)**2))))
    result['joint_nominal_95_ellipsoid_fraction_inside'] = float(np.mean([
        r['joint_normalized_squared']<=chi2.ppf(.95, 6) for r in records]))
    return result


def paired_summary(pairs):
    result = {}
    for part, e, v in [('rotation', 'rotation_error_deg', 'rotation_trace_rad2'),
                       ('translation', 'rho_error_mm', 'translation_trace_m2')]:
        vr = np.asarray([p['on'][v]/p['off'][v] for p in pairs])
        er = np.asarray([p['on'][e]/p['off'][e] for p in pairs])
        result[part] = dict(on_over_off_variance=stat(vr), on_over_off_error=stat(er),
            fraction_variance_down_error_up=float(np.mean((vr<1)&(er>1))),
            fraction_variance_down_error_down=float(np.mean((vr<1)&(er<1))),
            fraction_variance_up_error_up=float(np.mean((vr>1)&(er>1))),
            fraction_variance_up_error_down=float(np.mean((vr>1)&(er<1))),
            change_rank_correlation=rank(vr.tolist(), er.tolist()))
    return result


def injection_records(root, cfg, reference, first, versions, inputs):
    src = root / f"{cfg['prefix']}_on_source"
    files = dict(G2K=src/'estimators.log',
                 K2G=next((src/'ros_logs').glob('*/glim_rosnode-*-stdout.log')))
    lookup = {(x['source'], x['source_i'], x['source_j']): x for x in first.values()}
    history = {(x['source'], x['source_i'], x['source_j']): list(versions[key].values())
               for key, x in first.items()}
    result, checks = {}, {}
    for direction, path in files.items():
        inputs[str(path)] = geo.sha(path)
        tag = 'KIMERA_CBS_MEAN_ALIGNMENT_ROW' if direction == 'G2K' else 'GLIM_CBS_MEAN_ALIGNMENT_ROW'
        records = []
        cov_diff, mean_diff, total, selected, matched = 0., 0., 0, 0, 0
        differs_from_first, matches_exported = [], 0
        for _, f in geo.rows(path, [tag]):
            total += 1
            if direction == 'G2K':
                cur, si, sj, ri, rj = map(int, f[:5])
                a, b, ra, rb, limit = map(int, f[5:10])
                raw, aligned, C = geo.matrix(f[10]).ravel(), geo.matrix(f[11]).ravel(), geo.matrix(f[12])
            else:
                si, sj, ri, rj = map(int, f[:4])
                a, b, ra, rb, limit = map(stamp, f[4:9])
                raw, aligned, C = geo.matrix(f[9]).ravel(), geo.matrix(f[10]).ravel(), geo.matrix(f[11])
            if min(a, ra)<cfg['lower'] or max(b, rb, limit)>cfg['upper']:
                continue
            selected += 1
            source = SOURCES[0] if direction == 'G2K' else SOURCES[1]
            key = source, si, sj
            if key in lookup:
                x = lookup[key]
                matched += 1
                cov_diff = max(cov_diff, float(np.max(np.abs(C-x['cov']))))
                mean_diff = max(mean_diff, float(np.max(np.abs(geo.exp(raw)-geo.exp(x['mu'])))))
                if not np.allclose(C,x['cov'],rtol=1e-10,atol=1e-18) or not np.allclose(raw,x['mu'],rtol=1e-10,atol=1e-12):
                    differs_from_first.append([si,sj,ri,rj])
                if any(np.allclose(C,y['cov'],rtol=1e-10,atol=1e-18)
                       and np.allclose(raw,y['mu'],rtol=1e-10,atol=1e-12) for y in history[key]):
                    matches_exported += 1
            rec = dict(source_i=si, source_j=sj, receiver_i=ri, receiver_j=rj,
                       source_from_ns=a, source_to_ns=b, receiver_from_ns=ra,
                       receiver_to_ns=rb, causal_limit_ns=limit, cov=C.tolist(),
                       raw_mu=raw.tolist(), aligned_mu=aligned.tolist())
            rec['raw'] = measure(dict(a=a,b=b,mu=raw,cov=C), reference)
            rec['aligned'] = measure(dict(a=ra,b=rb,mu=aligned,cov=C), reference)
            records.append(rec)
        checks[direction] = dict(total=total, selected=selected,
            matched_to_first_source_payload=matched, max_covariance_abs_difference=cov_diff,
            max_raw_transform_abs_difference=mean_diff,
            differs_from_first_payload=differs_from_first, matches_exported_version=matches_exported)
        result[direction] = dict(raw=summarize([x['raw'] for x in records]),
            aligned=summarize([x['aligned'] for x in records]), records=records)
    return result, checks


def analyze(name):
    cfg = CASES[name]
    out = HERE/f'{name}_results.json'
    assert not out.exists(), out
    roots = {mode: W/'runs'/cfg[mode]/'prepared' for mode in ('off', 'on')}
    inputs, versions, first, checks, keytimes = {}, {}, {}, {}, {}
    print(name, 'read saved journals', flush=True)
    for mode in roots:
        versions[mode], first[mode], checks[mode], keytimes[mode] = read_events(
            roots[mode], cfg['prefix'], mode, cfg, inputs)
    common_k = set(keytimes['off']['kimera_front']) & set(keytimes['on']['kimera_front'])
    assert all(keytimes['off']['kimera_front'][k] == keytimes['on']['kimera_front'][k] for k in common_k)
    bag = W/'src/datasets/GrandTour'/cfg['bag']
    inputs[str(bag)] = geo.sha(bag)
    reference = geo.Reference(bag)
    common = set(first['off']) & set(first['on'])
    records, unavailable = [], []
    for key in sorted(common):
        updates = set(versions['off'][key]) & set(versions['on'][key])
        if not updates:
            unavailable.append(key)
            continue
        for stage in ('first', 'last_common_update'):
            pair = dict(source=key[0], a=key[1], b=key[2], stage=stage)
            for mode in ('off', 'on'):
                x = first[mode][key] if stage == 'first' else versions[mode][key][max(updates)]
                pair[mode] = measure(x, reference)
                pair[mode+'_metadata'] = {k:v for k,v in x.items() if k not in ('mu','cov')}
                pair[mode+'_mu'] = x['mu'].tolist()
                pair[mode+'_cov'] = x['cov'].tolist()
            if stage == 'first':
                assert pair['off_metadata']['update'] == pair['on_metadata']['update'], key
            records.append(pair)
    assert not unavailable
    summaries = {}
    for source in SOURCES:
        summaries[source] = {}
        for stage in ('first', 'last_common_update'):
            rr = [r for r in records if r['source']==source and r['stage']==stage]
            summary = {mode:summarize([r[mode] for r in rr]) for mode in roots}
            summary['matched'] = paired_summary(rr)
            summary['coverage'] = dict(edges=len(rr), from_ns=min(r['a'] for r in rr),
                to_ns=max(r['b'] for r in rr), duration_s=stat([(r['b']-r['a'])/1e9 for r in rr]),
                off_revision=stat([r['off_metadata']['revision'] for r in rr]),
                on_revision=stat([r['on_metadata']['revision'] for r in rr]))
            summaries[source][stage] = summary
    print(name, 'score actual injections', flush=True)
    injections, injection_checks = injection_records(roots['on'], cfg, reference, first['on'], versions['on'], inputs)
    assert all(geo.sha(Path(p))==h for p,h in inputs.items())
    result = dict(sequence=name, config=cfg, checks=checks, common_edges=len(common),
        unmatched_edges={mode:len(set(first[mode])-common) for mode in roots},
        reference=dict(path=str(bag), max_used_bracket_ns=reference.max_bracket_ns,
                       caveat='shared-CPT7 IE-TC, not independent error-free GT'),
        nominal_thresholds=dict(d3=float(chi2.ppf(.95,3)),d6=float(chi2.ppf(.95,6))),
        sources=summaries, injections=injections, injection_checks=injection_checks,
        input_hashes=inputs, records=records, inputs_unchanged=True)
    write(out,result)
    print(json.dumps(dict(sequence=name,checks=checks,common_edges=len(common),
        injection_checks=injection_checks),indent=2), flush=True)


if __name__ == '__main__':
    geo.self_test()
    analyze(sys.argv[1])
