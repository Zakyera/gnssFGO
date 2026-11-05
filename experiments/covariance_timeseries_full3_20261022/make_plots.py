"""Read-only analysis of completed CP-K23/24/26 journals; no estimator execution."""
from __future__ import annotations

import csv
import hashlib
import json
import math
import sys
import zipfile
from collections import defaultdict
from pathlib import Path

import numpy as np

# The bundled runtime lacks matplotlib. Reuse the existing plotting package,
# keeping the bundled interpreter/numpy and all dependency directories unchanged.
try:
    import matplotlib
except ModuleNotFoundError:
    sys.path.append('/home/yeranis/.local/share/pipx/venvs/evo/lib/python3.12/site-packages')
    import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import LogLocator, NullFormatter

HERE = Path(__file__).resolve().parent
ROOT = Path('/home/yeranis/repos/gnssFGO')
RUNS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')
OUT = HERE / 'plots'
CONFIG = {
    'ETH1': dict(run='gnssfgo_eth1_awake_full_cpk23_20261021',
                 report='eth1_awake_full_cpk23_20261021/results.json',
                 start_ns=1727775007855396309, duration_s=346.32, offset_s=0, checkpoint='CP-K23'),
    'EIG-2': dict(run='gnssfgo_eig2_full_cpk24_20261021',
                  report='eig2_full_cpk24_20261021/results_eof_review.json',
                  start_ns=1730638816563546034, duration_s=355.02, offset_s=5.7, checkpoint='CP-K24/24A'),
    'SPX-3': dict(run='gnssfgo_spx3_awake_full_cpk26_20261021',
                  report='spx3_awake_full_cpk26_20261021/results.json',
                  start_ns=1730565902872888651, duration_s=194.02, offset_s=32, checkpoint='CP-K26'),
}
SOURCES = {'glim_hesai': 'GLIM', 'kimera_front': 'Kimera'}
BLOCKS = [('glim_hesai', 'translation'), ('glim_hesai', 'rotation'),
          ('kimera_front', 'translation'), ('kimera_front', 'rotation')]
COLOR = {'off': '#0072B2', 'on': '#D55E00'}
STAGES = {'first': 'First exported version', 'last': 'Last exported version (retrospective)'}


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(2**20), b''):
            h.update(chunk)
    return h.hexdigest()


def parse_case(sequence, mode, cfg, reference):
    path = RUNS / cfg['run'] / 'prepared' / f'full_{mode}_receiver/events.csv'
    seen = {}
    pooled = defaultdict(lambda: {'rotation': [], 'translation': []})
    counts = defaultdict(int)
    h = hashlib.sha256()
    record_count = 0
    with path.open('rb') as stream:
        for line_no, raw in enumerate(stream, 1):
            h.update(raw)
            record_count += 1
            if not raw.startswith(b'B,'):
                continue
            row = raw.rstrip().split(b',')
            assert len(row) == 52, (path, line_no, len(row))
            source = row[2].decode()
            assert source in SOURCES, source
            session = row[3].decode()
            ordinal, rev, i, j, ti, tj = map(int, row[4:10])
            arrival = int(row[1])
            assert ti < tj <= arrival
            key = (source, session, i, j)
            diag = [float(row[16 + 7*k]) for k in range(6)]
            assert np.isfinite(diag).all() and min(diag) > 0
            rotation, translation = sum(diag[:3]), sum(diag[3:])
            pooled[source]['rotation'].append(rotation)
            pooled[source]['translation'].append(translation)
            counts[source] += 1
            point = dict(sequence=sequence, mode=mode, source=source, session=session,
                         ordinal=ordinal, revision=rev, from_id=i, to_id=j,
                         from_timestamp_ns=ti, to_timestamp_ns=tj, arrival_ns=arrival,
                         physical_end_s=(tj-cfg['start_ns'])*1e-9,
                         arrival_s=(arrival-cfg['start_ns'])*1e-9,
                         interval_s=(tj-ti)*1e-9, rotation_trace_rad2=rotation,
                         translation_trace_m2=translation, event_line=line_no)
            if key not in seen:
                assert rev == 0, (path, key, rev)
                seen[key] = {'first': point, 'last': point, 'n': 1}
            else:
                previous = seen[key]['last']
                assert rev == previous['revision'] + 1
                assert (ti,tj,ordinal) == (previous['from_timestamp_ns'],previous['to_timestamp_ns'],previous['ordinal'])
                assert arrival >= previous['arrival_ns']
                seen[key]['last'] = point
                seen[key]['n'] += 1

    series = defaultdict(dict)
    checks = {}
    for source in SOURCES:
        histories = [v for k, v in seen.items() if k[0] == source]
        expected = reference['evaluation']['covariance'][mode][source]
        assert counts[source] == expected['versions']
        assert len(histories) == expected['stable_ids']
        assert sum(v['n'] > 1 for v in histories) == expected['revised_ids']
        checks[source] = dict(versions=counts[source], stable_ids=len(histories),
                              revised_ids=sum(v['n'] > 1 for v in histories))
        for block in ('rotation', 'translation'):
            values = np.asarray(pooled[source][block])
            unit = 'rad2' if block == 'rotation' else 'm2'
            saved = expected[f'{block}_trace_{unit}']
            median, p95 = float(np.median(values)), float(np.percentile(values,95))
            assert math.isclose(median, saved['median'], rel_tol=2e-13)
            assert math.isclose(p95, saved['p95'], rel_tol=2e-13)
            checks[source][block] = dict(pooled_median=median, pooled_p95=p95,
                                         minimum=float(values.min()), maximum=float(values.max()))
        for stage in STAGES:
            points = sorted((v[stage] for v in histories), key=lambda p:(p['to_timestamp_ns'],p['from_timestamp_ns']))
            assert all(0 <= p['physical_end_s'] <= cfg['duration_s'] for p in points)
            series[source][stage] = points
        first = series[source]['first']
        last = series[source]['last']
        checks[source]['physical_endpoint_span_s'] = [first[0]['physical_end_s'], first[-1]['physical_end_s']]
        checks[source]['long_interval_count_over_0p30s'] = sum(p['interval_s'] > 0.30 for p in first)
        checks[source]['max_interval_s'] = max(p['interval_s'] for p in first)
        checks[source]['last_revision_age_s'] = {
            'min': min((p['arrival_ns']-p['to_timestamp_ns'])*1e-9 for p in last),
            'median': float(np.median([(p['arrival_ns']-p['to_timestamp_ns'])*1e-9 for p in last])),
            'max': max((p['arrival_ns']-p['to_timestamp_ns'])*1e-9 for p in last),
        }
    return dict(path=str(path), sha256=h.hexdigest(), records=record_count, checks=checks), dict(series)


def line_with_gaps(points, key):
    x, y = [], []
    previous = None
    for p in points:
        t = p['physical_end_s']
        if previous is not None and t-previous > 0.5:
            x.append(float('nan')); y.append(float('nan'))
        x.append(t); y.append(p[key]); previous = t
    return x, y


def draw(ax, sequence, source, block, stage, data, limits, number):
    unit = 'm²' if block == 'translation' else 'rad²'
    key = 'translation_trace_m2' if block == 'translation' else 'rotation_trace_rad2'
    for mode in ('off','on'):
        points = data[sequence][mode][source][stage]
        x, y = line_with_gaps(points,key)
        ax.plot(x,y,color=COLOR[mode],lw=0.85,alpha=0.9,
                marker='.',ms=1.9,label=f'CBS {mode.upper()}',rasterized=False)
    latest = max(data[sequence][mode][source][stage][-1]['physical_end_s'] for mode in ('off','on'))
    if CONFIG[sequence]['duration_s']-latest > 1:
        ax.axvspan(latest,CONFIG[sequence]['duration_s'],color='#CBD5E1',alpha=.3,zorder=0)
        ax.text((latest+CONFIG[sequence]['duration_s'])/2,.5,'No new belief endpoints',
                transform=ax.get_xaxis_transform(),ha='center',va='center',rotation=90,
                fontsize=8,color='#475569')
    ax.set_yscale('log')
    ax.set_ylim(*limits[(source,block)])
    ax.set_xlim(0,CONFIG[sequence]['duration_s'])
    ax.set_title(f'{number}. {SOURCES[source]} {block} — {"first" if stage == "first" else "last available"}',loc='left',pad=10)
    ax.set_ylabel(f'Covariance trace ({unit}, log scale)')
    ax.set_xlabel('Physical endpoint time since capture start (s)')
    ax.grid(True,which='major',color='#D6DDE5',linewidth=0.6)
    ax.yaxis.set_minor_locator(LogLocator(base=10,subs=(2,5)))
    ax.yaxis.set_minor_formatter(NullFormatter())
    ax.spines[['top','right']].set_visible(False)
    ax.spines[['left','bottom']].set_color('#98A2B3')
    ax.legend(loc='best',frameon=True,framealpha=0.85,facecolor='white',edgecolor='none',fontsize=8)


def main():
    refresh = sys.argv[1:] == ['--refresh-plots']
    if OUT.exists():
        if not refresh:
            raise RuntimeError('Refusing to overwrite an existing plot set; inspect it first.')
        previous = json.loads((OUT/'audit.json').read_text())
        assert previous['passed'] and previous['plot_count'] == 24
        for seq in previous['input_audit'].values():
            for case in seq['cases'].values():
                assert digest(Path(case['path'])) == case['sha256']
    OUT.mkdir(exist_ok=refresh)
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10,'axes.titlesize':11,
                         'axes.labelsize':9,'xtick.labelsize':8,'ytick.labelsize':8,
                         'svg.fonttype':'none','path.simplify':False,'savefig.facecolor':'white'})
    data, audit = {}, {}
    for sequence, cfg in CONFIG.items():
        reference_path = ROOT / 'experiments' / cfg['report']
        reference = json.loads(reference_path.read_text())
        assert reference['passed']
        audit[sequence] = dict(config=cfg, reference=str(reference_path),
                               reference_sha256=digest(reference_path), cases={})
        data[sequence] = {}
        for mode in ('off','on'):
            info, series = parse_case(sequence,mode,cfg,reference)
            audit[sequence]['cases'][mode] = info
            data[sequence][mode] = series
            print(f'{sequence} {mode}: counts and pooled traces match the saved report',flush=True)

    # Same vertical scale for each estimator/block across OFF/ON, first/last,
    # and all three missions. No percentile clipping or smoothing.
    limits = {}
    for source,block in BLOCKS:
        key = 'translation_trace_m2' if block == 'translation' else 'rotation_trace_rad2'
        values = [p[key] for seq in data for mode in ('off','on') for stage in STAGES
                  for p in data[seq][mode][source][stage]]
        limits[(source,block)] = [10**(math.floor(math.log10(min(values)))-0.08),
                                  10**(math.ceil(math.log10(max(values)))+0.08)]
    catalog = []
    for sequence,cfg in CONFIG.items():
        folder = OUT / sequence.lower()
        folder.mkdir(exist_ok=refresh)
        allpoints = []
        for mode in ('off','on'):
            for source in SOURCES:
                for stage in STAGES:
                    allpoints.extend(dict(stage=stage,**p) for p in data[sequence][mode][source][stage])
        with (folder/'plotted_samples.csv').open('w' if refresh else 'x',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(allpoints[0]));writer.writeheader();writer.writerows(allpoints)
        fig,axes=plt.subplots(4,2,figsize=(14.8,16.4))
        fig.subplots_adjust(left=.09,right=.98,top=.88,bottom=.07,hspace=.43,wspace=.23)
        fig.suptitle(f'{sequence} — exported relative-belief covariance over time',fontsize=19,x=.06,ha='left',y=.98)
        fig.text(.06,.949,f'{cfg["checkpoint"]} · complete {cfg["duration_s"]:.2f} s capture · CBS OFF vs ON · unchanged LOCAL covariance',fontsize=11)
        fig.text(.06,.927,'Blue: CBS OFF    Orange: CBS ON    |    Every sample shown; no smoothing or percentile clipping',fontsize=10)
        fig.text(.27,.899,'FIRST EXPORTED VERSION',ha='center',fontsize=11,fontweight='bold')
        fig.text(.76,.899,'LAST EXPORTED VERSION — RETROSPECTIVE',ha='center',fontsize=11,fontweight='bold')
        fig.text(.06,.027,'Time is the physical end of each relative interval, not arrival time. Logarithmic y axes; rotation and translation stay separate.\nLines break at endpoint gaps >0.50 s. Gray tail = no new belief endpoints, not zero covariance. Last export may be excluded by the fuser.',fontsize=9)
        for row,(source,block) in enumerate(BLOCKS):
            for col,stage in enumerate(STAGES):
                number=row*2+col+1
                draw(axes[row,col],sequence,source,block,stage,data,limits,number)
                individual,ax=plt.subplots(figsize=(12.6,5.8))
                individual.subplots_adjust(left=.10,right=.98,top=.80,bottom=.17)
                draw(ax,sequence,source,block,stage,data,limits,number)
                individual.suptitle(f'{sequence}: {SOURCES[source]} {block} covariance trace',x=.10,ha='left',fontsize=16)
                individual.text(.10,.88,STAGES[stage]+' · full capture · CBS OFF vs ON',fontsize=11)
                individual.text(.10,.025,'Actual samples; no smoothing/clipping. Gaps >0.50 s are not connected. Gray tail = no new belief endpoints.\n'+('First transport version per stable belief ID.' if stage=='first' else 'Last exported version per ID, including any expired/EOF-excluded versions; not an online curve.'),fontsize=9)
                filename=f'{number:02d}_{SOURCES[source].lower()}_{block}_{stage}'
                for ext in ('png','svg'):
                    individual.savefig(folder/f'{filename}.{ext}',dpi=170)
                plt.close(individual)
                catalog.append(dict(sequence=sequence,number=number,source=source,block=block,stage=stage,
                                    png=str(folder/f'{filename}.png'),svg=str(folder/f'{filename}.svg')))
        for ext in ('png','svg'):
            fig.savefig(OUT/f'{sequence.lower()}_eight_plots.{ext}',dpi=170)
        plt.close(fig)
        print(f'{sequence}: eight plots and overview rendered',flush=True)

    # Make the limited matched-endpoint cohort explicit; do not divide curves
    # interpolated across different keyframes or silently match numeric IDs.
    for sequence in data:
        audit[sequence]['physical_pair_overlap']={}
        for source in SOURCES:
            keys=[]
            for mode in ('off','on'):
                keys.append({(p['from_timestamp_ns'],p['to_timestamp_ns']) for p in data[sequence][mode][source]['first']})
            audit[sequence]['physical_pair_overlap'][source]=dict(off=len(keys[0]),on=len(keys[1]),exactly_shared=len(keys[0]&keys[1]))
    for sequence in audit:
        for mode,case in audit[sequence]['cases'].items():
            assert digest(Path(case['path'])) == case['sha256'], 'Source journal changed during analysis'
        assert digest(Path(audit[sequence]['reference'])) == audit[sequence]['reference_sha256']
    result=dict(passed=True,experiment_execution=False,source_inputs_unchanged=True,
                input_audit=audit,plot_count=len(catalog),catalog=catalog,
                scales={f'{s}/{b}':v for (s,b),v in limits.items()},
                scope='Exported relative Pose3 measurement covariances, not absolute trajectory marginals or WNOA covariance.',
                axis='to_timestamp_ns minus exact selected-topic capture start; not wall-clock or arrival time',
                version='first transport revision zero / last observed transport revision per (source,session,fromID,toID)',
                caveats=['Last available is retrospective, not a joint-snapshot graph, and revision age varies.',
                         'Includes initialization exclusions, expired ETH1 revisions and terminal EOF omissions.',
                         'OFF/ON keyframes and relative interval durations need not match; no universal scale inferred.',
                         'No smoothing, clipping, GT gating, trace normalization, covariance tuning or solver run.'])
    (OUT/'audit.json').write_text(json.dumps(result,indent=2)+'\n')
    with zipfile.ZipFile(OUT/'covariance_plots_all_three_sequences.zip','w' if refresh else 'x',compression=zipfile.ZIP_DEFLATED) as z:
        for path in sorted(OUT.rglob('*')):
            if path.is_file() and path.suffix!='.zip':
                z.write(path,path.relative_to(OUT))
        z.write(HERE/'README.txt','README.txt')
    print(json.dumps({'passed':True,'individual_plots':len(catalog),'overviews':3,'output':str(OUT)}),flush=True)


if __name__=='__main__':
    main()
