#!/usr/bin/env python3
"""Isolated input intervention; the frozen receiver is never edited/rebuilt."""
from pathlib import Path
from collections import Counter
from itertools import zip_longest
import argparse
import csv
import hashlib
import json
import math
import os
import subprocess
import time
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = Path('/home/yeranis/results/gnssfgo_eth1_wnoa_kimera_scale2_20261008')
BASE = Path('/home/yeranis/results/gnssfgo_eth1_wnoa_mechanism_20261007')
LIVE = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_live_wnoa_20261007')
BINARY = Path('/home/yeranis/results/gnssfgo_eth1_local_prediction_20261007/build/wnoa/replay')
RUNTIME = Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/lib')
EXPECTED = 'fb56ffc5c6c9cb575d6129289688caeabb4e3baba32e429b857f3d6a30e50d1f'
ENV = dict(LD_LIBRARY_PATH=str(RUNTIME), OMP_NUM_THREADS='1',
           WNOJ_INCREMENTAL_PROFILE='nominal', WNOJ_STRICT_PUBLICATION='1',
           AUDIT_RECORD_ALL='1', WNOJ_AUDIT_INDICES='294,594,894,1192')

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for data in iter(lambda: f.read(2**20), b''):
            h.update(data)
    return h.hexdigest()

def save(path, value):
    with Path(path).open('x') as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write('\n')

def rows(path):
    with Path(path).open() as f:
        return list(csv.DictReader(f))

def transformed(line, alpha):
    f = line.rstrip('\n').split(',')
    if f[0] != 'B' or f[2] != 'kimera_front' or alpha == 1:
        return line
    assert len(f) == 52 and alpha == 2
    f[16:] = [format(float(x) * alpha, '.17g') for x in f[16:]]
    return ','.join(f) + '\n'

def protect_check():
    manifest = json.loads((ROOT/'manifest.json').read_text())
    for path, expected in manifest['protected'].items():
        assert sha(path) == expected, ('protected artifact changed', path)
    return len(manifest['protected'])

def prepare():
    assert not ROOT.exists(), 'refuse overwrite'
    assert sha(BINARY) == EXPECTED
    old = json.loads((BASE/'manifest.json').read_text())
    ROOT.mkdir()
    (ROOT/'inputs').mkdir()
    protected = {str(BINARY): sha(BINARY)}
    paths = list(RUNTIME.glob('*')) + [BASE/'metrics.json', BASE/'manifest.json']
    for mode in ('off','on'):
        paths += list((BASE/(mode+'_wnoa')).glob('*'))
        paths += [BASE/f'validation_{mode}.json', BASE/f'{mode}_command.json', BASE/'inputs'/f'{mode}_events.csv']
    # Preserve original source output/config and runtime identities too.
    for path in json.loads((LIVE/'protected_manifest_v2.json').read_text()):
        paths.append(Path(path))
    for path in old['protected']:
        if '_source/' in path or path.endswith('/events.csv') or 'score_grandtour_' in path:
            paths.append(Path(path))
    for path in paths:
        if path.is_file():
            protected[str(path)] = sha(path)
    inputs = {}
    for mode in ('off','on'):
        source = BASE/'inputs'/f'{mode}_events.csv'
        assert sha(source) == old['inputs'][mode]['clipped_sha256']
        assert json.loads((BASE/f'validation_{mode}.json').read_text())['passed']
        target = ROOT/'inputs'/f'{mode}_kimera_covariance_x2.events'
        counts = Counter()
        minimum = float('inf')
        max_symmetry = 0.
        with source.open() as fi, target.open('x') as fo:
            for line in fi:
                assert transformed(line, 1) == line
                f = line.rstrip('\n').split(',')
                assert f[0] in ('I','B','E')
                new = transformed(line, 2)
                g = new.rstrip('\n').split(',')
                counts[f[0]] += 1
                if f[0] == 'B':
                    assert len(f) == 52 and f[2] in ('glim_hesai','kimera_front')
                    counts[f[2]] += 1
                    assert g[:16] == f[:16]
                    a = np.array(f[16:], dtype=float).reshape(6,6)
                    b = np.array(g[16:], dtype=float).reshape(6,6)
                    assert np.array_equal(b, a * (2 if f[2]=='kimera_front' else 1))
                    assert np.isfinite(b).all()
                    eigenvalue = float(np.linalg.eigvalsh(b).min())
                    symmetry = float(np.max(np.abs(b-b.T)))
                    assert eigenvalue > 0 and symmetry <= 1e-12
                    minimum = min(minimum, eigenvalue)
                    max_symmetry = max(max_symmetry, symmetry)
                if f[0] != 'B' or f[2] != 'kimera_front':
                    assert new == line
                fo.write(new)
        assert counts['B'] == old['inputs'][mode]['counts']['B']
        assert counts['E'] == 1
        inputs[mode] = dict(original=str(source), original_sha256=sha(source), derived=str(target),
                            derived_sha256=sha(target), counts=dict(counts),
                            minimum_eigenvalue=minimum, maximum_symmetry_error=max_symmetry,
                            every_non_covariance_field_unchanged=True, alpha1_byte_identity=True)
        protected[str(target)] = sha(target)
    save(ROOT/'manifest.json', dict(protected=protected, inputs=inputs, environment=ENV,
         binary_sha256=EXPECTED, cutoff_ns=old['cutoff_ns'], protocol_sha256=sha(HERE/'PROTOCOL.txt'),
         repository_head=subprocess.check_output(['git','-C','/home/yeranis/repos/gnssFGO','rev-parse','HEAD'], text=True).strip()))
    print(json.dumps(inputs, indent=2), flush=True)

def run(mode, repeat=False):
    protect_check()
    label = mode + ('_repeat' if repeat else '_x2')
    out = ROOT/label
    assert not out.exists(), 'refuse duplicate launch/overwrite'
    assert not (ROOT/f'{label}_started.json').exists(), 'refuse automatic retry'
    if repeat:
        assert json.loads((ROOT/f'{mode}_validation.json').read_text())['passed']
    command = [str(BINARY), '--receiver-model','source_wnoa','--mode','sources',
        '--timeline','record','--solver','isam2','--support-policy','uniform_imu',
        '--support-period-ns','100000000','--receiver-startup-samples','59',
        '--event-stream',str(ROOT/'inputs'/f'{mode}_kimera_covariance_x2.events'),'--out',str(out)]
    save(ROOT/f'{label}_started.json',dict(command=command, environment=ENV, pid=os.getpid(), unix_time=time.time()))
    start = time.monotonic()
    with (ROOT/f'{label}.log').open('x') as log:
        result = subprocess.run(command, env=dict(os.environ, **ENV), stdout=log, stderr=subprocess.STDOUT)
    save(ROOT/f'{label}_command.json',dict(command=command, environment=ENV,
        returncode=result.returncode, seconds=time.monotonic()-start))
    assert result.returncode == 0, f'{label} failed: inspect preserved log; no retry'
    print(label, 'completed', time.monotonic()-start, flush=True)

def validate(mode):
    protected_count = protect_check()
    case = ROOT/(mode+'_x2')
    base = BASE/(mode+'_wnoa')
    s = json.loads((case/'summary.json').read_text())
    old = json.loads((base/'summary.json').read_text())
    for key in ('receiver_model','support_policy','kimera_channel','kimera_covariance','streaming',
      'pending_at_end','support_period_ns','initial_timestamp_ns','mode','imu_support','motion_prior',
      'timeline','solver','smoother','lag_seconds','support_states','beliefs_accepted','revision_messages_loaded',
      'revision_insertions','revision_removals','revision_duplicates','revision_stale','source_model_noise',
      'beliefs_skipped_no_support','factor_kinds','experimental_qc_diagonal','source_wnoa_initial_sigmas',
      'imu_factor_count','bias_variable_count','wnoa_factor_count','acceleration_variable_count',
      'initial_prior_count','relative_factor_count','active_factor_count'):
        assert s[key] == old[key], (key,s[key],old[key])
    identities = {}
    for name in ('support_clock.csv','source_accounting.csv','causal_dispatch.csv',
                 'unsupported_beliefs.csv','model_covariance.csv','measurement_events.csv'):
        identities[name] = sha(case/name) == sha(base/name)
        assert identities[name], name
    conv = rows(case/'publication_convergence.csv')
    assert len(conv) == s['support_states']
    assert all(r['status']=='stationary' and float(r['gradient_after'])<=1e-5 for r in conv)
    assert s['maximum_factor_covariance_abs_difference'] < 1e-12
    actual = rows(case/'source_model_covariance.csv')
    original = rows(base/'source_model_covariance.csv')
    assert len(actual) == len(original) == s['beliefs_accepted']
    counts = Counter()
    maximum = 0.
    for a,b in zip(actual,original):
        for key in ('source','session','from_index','to_index','revision','rotation_sigma_rad','translation_sigma_m'):
            assert a[key] == b[key]
        alpha = 2 if a['source']=='kimera_front' else 1
        counts[a['source']] += 1
        for i in range(6):
            for j in range(6):
                for prefix in ('reported_', 'effective_'):
                    key = prefix+str(i)+str(j)
                    difference = abs(float(a[key])-alpha*float(b[key]))
                    maximum = max(maximum, difference)
                    assert difference == 0, (key,difference)
    for name in ('online_trajectory.tum','trajectory.tum'):
        a = np.loadtxt(case/name); b = np.loadtxt(base/name)
        assert a.shape == b.shape and np.array_equal(a[:,0],b[:,0]) and np.isfinite(a).all()
    report = dict(passed=True, mode=mode, protected_hashes_verified=protected_count,
      unchanged_accounting_and_model=identities, accepted_by_source=dict(counts),
      supports=s['support_states'], versions=s['revision_messages_loaded'], replacements=s['revision_removals'],
      unsupported=s['beliefs_skipped_no_support'], maximum_covariance_scaling_difference=maximum,
      maximum_factor_covariance_difference=s['maximum_factor_covariance_abs_difference'],
      max_publication_gradient=max(float(r['gradient_after']) for r in conv))
    save(ROOT/f'{mode}_validation.json', report)
    print(json.dumps(report,indent=2),flush=True)

def repeat_check(mode):
    protect_check()
    repeated = ROOT/(mode+'_repeat')
    convergence = rows(repeated/'publication_convergence.csv')
    assert len(convergence)==1193
    assert all(r['status']=='stationary' and float(r['gradient_after'])<=1e-5 for r in convergence)
    identities = {}
    for name in ('trajectory.tum','states.csv','online_trajectory.tum','online_states.csv','support_clock.csv',
                 'source_model_covariance.csv','model_covariance.csv','causal_dispatch.csv','measurement_events.csv',
                 'source_accounting.csv','unsupported_beliefs.csv'):
        a = ROOT/(mode+'_x2')/name
        b = ROOT/(mode+'_repeat')/name
        assert sha(a) == sha(b), ('determinism failure', mode, name)
        identities[name] = sha(a)
    snapshot_count=0
    for name in ('incremental_states.csv','ledger.csv','factor_payload_probe.txt'):
        original_files=sorted((ROOT/(mode+'_x2')).glob('snapshot_*/'+name))
        repeated_files=sorted(repeated.glob('snapshot_*/'+name))
        assert [p.parent.name for p in original_files]==[p.parent.name for p in repeated_files]
        for a,b in zip(original_files,repeated_files):
            assert sha(a)==sha(b), ('snapshot determinism failure',mode,a)
        snapshot_count+=len(original_files)
    save(ROOT/f'{mode}_determinism.json',dict(passed=True,exact_sha256=identities,
        exact_snapshot_state_ledger_payload_files=snapshot_count,
        max_repeat_publication_gradient=max(float(r['gradient_after']) for r in convergence)))
    print(mode, 'exact deterministic repeat PASS',flush=True)

if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('action',choices=['prepare','run','validate','repeat','check-repeat','protect'])
    parser.add_argument('mode',nargs='?',choices=['off','on'])
    args=parser.parse_args()
    if args.action=='prepare': prepare()
    elif args.action=='protect': print(protect_check())
    else:
        assert args.mode
        if args.action in ('run','repeat'): run(args.mode,args.action=='repeat')
        elif args.action=='validate': validate(args.mode)
        else: repeat_check(args.mode)
