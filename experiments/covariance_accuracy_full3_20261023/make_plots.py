"""Augment saved full-run covariance plots with same-belief reference errors.

Read-only access to completed journals, bags, previous plots and math helpers.
No estimator, optimizer, replay, container or historical evaluator is executed.
"""
from __future__ import annotations

import csv
import importlib.util
import json
import math
from pathlib import Path
import sys
import zipfile
from collections import defaultdict

import numpy as np

sys.dont_write_bytecode = True
sys.path.append('/home/yeranis/.local/share/pipx/venvs/evo/lib/python3.12/site-packages')
HERE = Path(__file__).resolve().parent
EXPERIMENTS = HERE.parent


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


old = module('previous_covariance_plots', EXPERIMENTS/'covariance_timeseries_full3_20261022/make_plots.py')
geo = module('verified_relative_pose_math', EXPERIMENTS/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py')
plt = old.plt
OUT = HERE/'plots'
BAGS = {
    'ETH1': '2024-10-01-11-29-55_cpt7_ie_tc.bag',
    'EIG-2': 'EIG-2/2024-11-03-13-59-54_cpt7_ie_tc.bag',
    'SPX-3': 'SPX-3/2024-11-02-17-43-10_cpt7_ie_tc.bag',
}
INTEGER_FIELDS = ['ordinal', 'revision', 'from_id', 'to_id', 'from_timestamp_ns',
                  'to_timestamp_ns', 'arrival_ns', 'event_line']
FLOAT_FIELDS = ['physical_end_s', 'arrival_s', 'interval_s', 'rotation_trace_rad2',
                'translation_trace_m2']
ROWS = [(s, stage) for s in old.SOURCES for stage in ('first', 'last')]


def stat(values):
    a = np.asarray(values, float)
    assert a.size and np.isfinite(a).all()
    return dict(n=len(a), min=float(a.min()), median=float(np.median(a)),
                rms=float(np.sqrt(np.mean(a*a))), p95=float(np.percentile(a, 95)),
                max=float(a.max()))


def self_test():
    assert geo.self_test()
    A = geo.exp([.2, -.1, .3, 1., 2., 3.])
    Z = geo.exp([.01, .03, -.02, .1, .2, .04])
    B = A @ Z
    G = geo.exp([-.3, .2, .1, 50., -20., 4.])
    assert np.max(np.abs(geo.inv(A) @ B - Z)) < 1e-12
    assert np.max(np.abs(geo.inv(G @ A) @ (G @ B) - Z)) < 1e-12
    assert geo.error(Z, Z)['translation_m'] == 0
    Zbad = Z.copy(); Zbad[:3, 3] += np.array([.003, .004, 0.])
    assert math.isclose(geo.error(Zbad, Z)['translation_m'], .005, abs_tol=1e-14)
    return dict(exp_log_against_matrix_exponential=True, global_SE3_invariance=True,
                exact_belief_zero_error=True, known_5mm_translation_error=True)


def read_case(seq, mode, points, ref, prior, hashes):
    cfg = old.CONFIG[seq]
    path = old.RUNS/cfg['run']/'prepared'/f'full_{mode}_receiver/events.csv'
    hashes[str(path)] = prior['input_audit'][seq]['cases'][mode]['sha256']
    needed = defaultdict(list)
    for p in points:
        needed[p['event_line']].append(p)
    checked = 0
    import hashlib
    digest = hashlib.sha256()
    max_symmetry = 0.
    min_eig = math.inf
    for line_no, raw in enumerate(path.open('rb'), 1):
        digest.update(raw)
        if line_no not in needed:
            continue
        r = raw.rstrip().split(b',')
        assert r[0] == b'B' and len(r) == 52
        mu = np.asarray(r[10:16], float)
        C = np.asarray(r[16:], float).reshape(6, 6)
        assert np.isfinite(mu).all() and np.isfinite(C).all()
        max_symmetry = max(max_symmetry, float(np.max(np.abs(C-C.T))))
        min_eig = min(min_eig, float(np.linalg.eigvalsh(C).min()))
        assert max_symmetry <= 1e-12 and min_eig > 0
        for p in needed.pop(line_no):
            assert r[2].decode() == p['source'] and r[3].decode() == p['session']
            assert list(map(int, r[4:10])) == [p[k] for k in INTEGER_FIELDS[:6]]
            assert int(r[1]) == p['arrival_ns']
            for key, block in [('translation_trace_m2', C[3:, 3:]),
                               ('rotation_trace_rad2', C[:3, :3])]:
                assert math.isclose(float(np.trace(block)), p[key], rel_tol=1e-14)
            A = ref.at(p['from_timestamp_ns']); B = ref.at(p['to_timestamp_ns'])
            target = geo.inv(A) @ B
            belief = geo.exp(mu)  # Same Pose3::Expmap encoding as the receiver.
            error = geo.error(belief, target)
            p.update(translation_error_mm=error['translation_m']*1000,
                     tangent_translation_error_mm=error['rho_m']*1000,
                     rotation_error_deg=error['rotation_deg'], reference_valid=True,
                     reference_translation_norm_m=float(np.linalg.norm(target[:3, 3])))
            for k in range(6):
                p[f'belief_log_{k}'] = float(mu[k])
            for k in range(3):
                p[f'belief_translation_{k}_m'] = float(belief[k, 3])
                p[f'reference_translation_{k}_m'] = float(target[k, 3])
            # Independent direct expression for the reference relative translation.
            direct = A[:3, :3].T @ (B[:3, 3]-A[:3, 3])
            assert np.max(np.abs(direct-target[:3, 3])) < 1e-9
            assert math.isclose(np.linalg.norm(belief[:3, 3]-direct)*1000,
                                p['translation_error_mm'], abs_tol=1e-6)
            checked += 1
    assert not needed and checked == len(points)
    assert digest.hexdigest() == hashes[str(path)], 'Journal changed since previous plots'
    return dict(selected_rows=checked, all_selected_endpoints_supported=True,
                max_covariance_symmetry_error=max_symmetry, min_covariance_eigenvalue=min_eig)


def axis(ax, seq, source, stage, records, key, limits, title):
    for mode in ('off', 'on'):
        points = records[seq][mode][source][stage]
        x, y = old.line_with_gaps(points, key)
        ax.plot(x, y, color=old.COLOR[mode], lw=.8, marker='.', ms=1.7,
                alpha=.87, label=f'CBS {mode.upper()}')
    latest = max(records[seq][m][source][stage][-1]['physical_end_s'] for m in ('off', 'on'))
    if old.CONFIG[seq]['duration_s']-latest > 1:
        ax.axvspan(latest, old.CONFIG[seq]['duration_s'], color='#CBD5E1', alpha=.35)
        ax.text((latest+old.CONFIG[seq]['duration_s'])/2, .5, 'No new belief endpoints',
                transform=ax.get_xaxis_transform(), rotation=90, ha='center', va='center',
                fontsize=8, color='#475569')
    ax.set_xlim(0, old.CONFIG[seq]['duration_s'])
    ax.set_yscale('log')
    ax.set_ylim(limits[(source, key)])
    ax.set_title(title, loc='left', fontsize=11, pad=8)
    ax.set_ylabel('Trace (m², log scale)' if key == 'translation_trace_m2'
                  else 'Relative translation error (mm, log scale)')
    ax.set_xlabel('Physical belief endpoint time since capture start (s)')
    ax.grid(True, which='major', color='#D6DDE5', lw=.6)
    ax.spines[['top', 'right']].set_visible(False)
    ax.legend(loc='best', fontsize=8, framealpha=.9)


def plot(records):
    plt.rcParams.update({'font.family':'DejaVu Sans', 'font.size':10,
                         'axes.labelsize':9, 'xtick.labelsize':8, 'ytick.labelsize':8,
                         'svg.fonttype':'none', 'path.simplify':False, 'savefig.facecolor':'white'})
    keys = ['translation_trace_m2', 'translation_error_mm']
    limits = {}
    for s in old.SOURCES:
        for key in keys:
            values = [p[key] for seq in records for m in ('off', 'on')
                      for stage in ('first', 'last') for p in records[seq][m][s][stage]]
            assert min(values) > 0, 'Zero errors require an explicit non-log rendering'
            limits[s, key] = (10**math.floor(math.log10(min(values))),
                               10**math.ceil(math.log10(max(values))))
    for seq in records:
        slug = seq.lower()
        fig, axs = plt.subplots(4, 2, figsize=(16, 14))
        fig.subplots_adjust(top=.92, bottom=.075, left=.075, right=.98, hspace=.48, wspace=.22)
        fig.suptitle(f'{seq}  |  Exported translation uncertainty and same-interval accuracy', fontsize=17, y=.98)
        fig.text(.075, .952, 'Blue: CBS OFF   Orange: CBS ON     Each covariance/error pair uses the same exported belief and exact physical endpoints.', fontsize=10)
        for row, (source, stage) in enumerate(ROWS):
            label = old.SOURCES[source]+' — '+('first publication' if stage == 'first' else 'last available revision (retrospective)')
            axis(axs[row, 0], seq, source, stage, records, keys[0], limits, label+'\nReported covariance')
            axis(axs[row, 1], seq, source, stage, records, keys[1], limits, label+'\nError against IE-TC reference')
            single, sa = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
            single.subplots_adjust(top=.86, bottom=.14, left=.1, right=.97, hspace=.3)
            single.suptitle(seq+' | '+label, fontsize=15, y=.97)
            for n, key in enumerate(keys):
                axis(sa[n], seq, source, stage, records, key, limits,
                     'Reported translation covariance' if n == 0 else 'Same-belief translation error against IE-TC')
            sa[0].set_xlabel('')
            single.text(.1, .045, 'Raw samples; no smoothing. Relative intervals ≈0.20 s, not trajectory ATE. IE-TC shares CPT7; not independent truth.', fontsize=9)
            stem = OUT/slug/f'{row+1:02}_{old.SOURCES[source].lower()}_{stage}_covariance_and_error'
            for ext in ('png', 'svg'):
                single.savefig(stem.with_suffix('.'+ext), dpi=170)
            plt.close(single)
        fig.text(.075, .033, 'Raw samples, no clipping/smoothing. Relative intervals ≈0.20 s, not trajectory ATE. Identical y-scales per source across sequences.', fontsize=9)
        fig.text(.075, .018, 'IE-TC shares CPT7; these are reference discrepancies, not a covariance-calibration test. Last revisions have different ages and are not all receiver-accepted.', fontsize=9)
        for ext in ('png', 'svg'):
            fig.savefig(OUT/f'{slug}_covariance_and_translation_error.{ext}', dpi=160)
        plt.close(fig)
    return {s:{k:list(limits[s,k]) for k in keys} for s in old.SOURCES}


def main():
    assert not OUT.exists(), 'Preserve existing artifacts; no automatic overwrite'
    checks = self_test()
    OUT.mkdir()
    prior_path = old.OUT/'audit.json'
    prior = json.loads(prior_path.read_text())
    assert prior['passed']
    hashes = {str(p):old.digest(p) for p in [Path(__file__), HERE/'README.txt',
              prior_path, Path(old.__file__), Path(geo.__file__)]}
    records, audit = {}, {}
    for seq, cfg in old.CONFIG.items():
        slug = seq.lower(); (OUT/slug).mkdir()
        csv_path = old.OUT/slug/'plotted_samples.csv'
        hashes[str(csv_path)] = old.digest(csv_path)
        rows = list(csv.DictReader(csv_path.open()))
        for p in rows:
            assert p['sequence'] == seq
            for key in INTEGER_FIELDS: p[key] = int(p[key])
            for key in FLOAT_FIELDS: p[key] = float(p[key])
        bag = geo.WORK/'src/datasets/GrandTour'/BAGS[seq]
        hashes[str(bag)] = old.digest(bag)
        reference = geo.Reference(bag)
        records[seq], audit[seq] = {}, dict(config=cfg, modes={})
        for mode in ('off', 'on'):
            subset = [p for p in rows if p['mode'] == mode]
            audit[seq]['modes'][mode] = read_case(seq, mode, subset, reference, prior, hashes)
            records[seq][mode] = {s:{stage:sorted([p for p in subset if p['source']==s and p['stage']==stage],
                         key=lambda p:(p['to_timestamp_ns'],p['from_timestamp_ns']))
                         for stage in ('first','last')} for s in old.SOURCES}
            audit[seq]['modes'][mode]['summaries'] = {s:{stage:{key:stat([p[key] for p in points])
                for key in ['translation_trace_m2','translation_error_mm','tangent_translation_error_mm','rotation_error_deg','interval_s']}
                for stage,points in stages.items()} for s,stages in records[seq][mode].items()}
        with (OUT/slug/'covariance_and_error_samples.csv').open('x') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
        audit[seq]['reference'] = dict(path=str(bag), topic='/boxi/inertial_explorer/tc/odometry',
              body='cpt7_imu', records=len(reference.times), first_ns=int(reference.times[0]),
              last_ns=int(reference.times[-1]), max_used_bracket_ns=reference.max_bracket_ns,
              scored_selected_rows=len(rows), unsupported_rows=0)
        print(json.dumps({seq:audit[seq]['reference']}), flush=True)
    limits = plot(records)
    assert all(old.digest(Path(p)) == h for p,h in hashes.items()), 'Input changed during analysis'
    result = dict(passed=True, self_tests=checks, input_hashes=hashes, inputs_unchanged=True,
                  sequences=audit, shared_plot_limits=limits, combined_individual_figures=12,
                  eight_axis_overviews=3, exact_prior_sample_selection_preserved=True,
                  error='norm(translation(Expmap(mu))-translation(inv(Tref(ti))*Tref(tj)))',
                  reference_caveat='Shared-CPT7 IE-TC; not independent noiseless ground truth')
    with (OUT/'audit.json').open('x') as f: json.dump(result, f, indent=2, allow_nan=False)
    with zipfile.ZipFile(OUT/'covariance_and_translation_error_all_sequences.zip','x',zipfile.ZIP_DEFLATED) as z:
        z.write(HERE/'README.txt','README.txt')
        for p in sorted(OUT.rglob('*')):
            if p.is_file() and p.suffix != '.zip': z.write(p, p.relative_to(OUT))
    print('PASS: 12 paired figures, 3 eight-axis overviews, CSV samples and provenance archive.', flush=True)


if __name__ == '__main__':
    main()
