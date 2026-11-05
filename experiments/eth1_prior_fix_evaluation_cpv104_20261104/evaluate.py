"""CP104 saved-only Method1 scoring; imports helpers, never archived mains."""
import copy
import datetime
import importlib.metadata
import importlib.util
import json
import sys
from pathlib import Path

sys.dont_write_bytecode = True
import numpy as np

H = Path(__file__).resolve().parent
E = H.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
NEW = E/'eth1_prior_fix_live_cpv103_20261103'
OLD = E/'eth1_split_imu_live_pair_cpv95_20261103_continuation1'
EARLY = E/'eth1_split_imu_persistent_on_cpv91_20261102_attempt1'
P = W/'runs'/NEW.name/'prepared'
P95 = W/'runs/eth1_split_imu_live_pair_cpv95_20261103_attempt3/prepared'
P91 = W/'runs'/EARLY.name/'prepared'
O = H/'evaluation'
spec = importlib.util.spec_from_file_location('cp104_saved_helpers', E/'eth1_alpha_bias_rw_off60_cpv88_20261102/evaluate.py')
b = importlib.util.module_from_spec(spec)
spec.loader.exec_module(b)
d = b.d
a = d.a
audit = d.read(P/'input_audit.json')
epoch = audit['epoch_ns']
duration = audit['duration_s']
d.epoch = epoch
d.d39.epoch = epoch


def verify_manifest(entries):
    for path, digest in entries.items():
        assert d.sha(Path(path)) == digest, path
    return len(entries)


def physical(curve, body):
    ts, pos, rot = curve
    transform = np.asarray(audit['T_'+body+'_prism'])
    return ts, pos + np.einsum('nij,j->ni', rot, transform[:3, 3]), rot @ transform[:3, :3]


def refs(folder, ts, pos, rot, cuts):
    folder.mkdir()
    ids = np.concatenate(cuts)
    path = folder/'reference.tum'
    ref = d.export(path, d.curve(ts[ids], pos[ids], rot[ids]))
    segments, pairs, info = d.reference_segments(folder, ts, pos, rot, cuts)
    return path, ref, segments, pairs, info


def score_full(folder, name, curve, reference):
    ts, pos, rot = curve
    gp, gr, good = a.sample(reference, ts, 50_000_000)
    assert good.all(), name
    cuts = np.split(np.arange(len(ts)), np.flatnonzero(np.diff(ts)>500_000_000)+1)
    rp, rt, rfs, pairs, info = refs(folder, ts, gp, gr, cuts)
    result = d.score_series(folder, name, ts, pos, rot, gp, gr, cuts, rfs, rp, rt)
    support = sum(float((ts[x[-1]]-ts[x[0]])*1e-9) for x in cuts)
    result.update(first_s=float((ts[0]-epoch)*1e-9), last_s=float((ts[-1]-epoch)*1e-9),
                  supported_s=support, coverage_percent=100*support/duration,
                  span_s=float((ts[-1]-ts[0])*1e-9), maximum_gap_s=float(np.diff(ts).max()*1e-9),
                  gaps=len(cuts)-1, segments=info,
                  native_polyline_including_gap_chords_m=d.curve(ts,pos,rot).path_length,
                  reference_polyline_including_gap_chords_m=rt.path_length,
                  newly_scored=True)
    return result


def metric_values(row):
    return [row['ATE_RMSE_m'], row['RPE']['translation_m']['statistics']['rmse'],
            row['RPE']['rotation_deg']['statistics']['rmse'], row['path_length_m'],
            row['reference_path_length_m']]


def main():
    assert not O.exists() and not (H/'results.json').exists()
    assert importlib.metadata.version('evo') == '1.36.3' and duration == 346.32
    assert d.read(NEW/'post_closeout.json')['all_owned_jobs_exited']
    sealed = {}
    for root in (NEW, OLD, EARLY):
        record = d.read(root/'complete.json')
        assert record['passed']
        sealed[str(root)] = verify_manifest(record['output_sha256'])
    protected = d.read(P/'protected_manifest.json')
    assert verify_manifest(protected) == 8120
    q = d.read(NEW/'qualification.json')
    assert q['passed'] and q['no_evo_scoring'] and q['prior_failure_archives'] == 0
    assert d.read(NEW/'on_candidate_validation.json')['passed']
    assert d.read(NEW/'on_local_provenance.json')['passed']
    assert d.read(NEW/'frame_on.json')['passed']
    assert d.read(NEW/'receiver_validation/on/validation.json')['passed']
    for path in (P95/'input_audit.json', P91/'input_audit.json'):
        oldaudit = d.read(path)
        for key in ('epoch_ns', 'duration_s', 'T_alphasense_prism', 'T_cpt7_prism'):
            assert oldaudit[key] == audit[key], (path, key)
    for mod in (b,d,d.m,d.m.c,d.m.c.b,a,a.g,d.u,d.d39,d.d39.v):
        d.pin(mod.__file__)
    import evo
    for path in Path(evo.__file__).parent.rglob('*.py'):
        d.pin(path)
    for path in (Path(__file__), H/'PROTOCOL.txt', H/'verify.py',
                 NEW/'MORNING_REPORT.txt', NEW/'copy_parity.json',
                 Path('/home/yeranis/.local/bin/evo_ape').resolve(),
                 Path('/home/yeranis/.local/bin/evo_rpe').resolve()):
        d.pin(path)
    for root in (OLD, EARLY):
        assert d.read(root/'report_verification.json')['passed']
    old95 = d.read(OLD/'results.json')
    old91 = d.read(EARLY/'results.json')
    record = next(r for r in d.m.c.b.ROWS if r[0]=='hesai_eth1')
    _, _, raw = d.m.c.b.load(record)
    reference = physical((raw.times, np.asarray(raw.poses)[:,:3,3], np.asarray(raw.poses)[:,:3,:3]), 'cpt7')
    curves = {}
    inputs = {}
    for prefix, source, receiver in (
        ('CP95_OFF', P95/'full_off_source', P95/'full_off_receiver/receiver'),
        ('CP95_ON', P95/'full_on_source', P95/'full_on_receiver/receiver'),
        ('CP91_ON', P91/'on_source', None),
        ('CP103_ON', P/'full_on_source', P/'full_on_receiver/receiver')):
        assert d.read(source/'source_coverage.json')['passed']
        assert d.read(source/'effective_mode_check.json')['passed']
        curves[prefix+'_glim'] = physical(a.load_curve(source/'glim_odometry.csv'), 'cpt7')
        curves[prefix+'_kimera'] = physical(b.kimera(source,'alphasense'), 'alphasense')
        for path in (source/'glim_odometry.csv', source/'kimera_odometry.csv', source/'estimators.log'):
            d.pin(path)
        if receiver:
            for role, file in (('fusion_online','online_trajectory.tum'), ('fusion_delayed','finalized_trajectory.tum')):
                curves[prefix+'_'+role] = physical(a.load_curve(receiver/file), 'cpt7')
                d.pin(receiver/file)
        inputs[prefix] = dict(source=str(source), receiver=str(receiver) if receiver else None)
    primary = [key for key in curves if not key.endswith('delayed')]
    assert len(primary)==11 and len(curves)==14
    first = max(curves[key][0][0] for key in primary)
    last = min(curves[key][0][-1] for key in primary)
    grid = np.arange(((int(first)+99_999_999)//100_000_000)*100_000_000, int(last)+1, 100_000_000,dtype=np.int64)
    gp, gr, valid = a.sample(reference,grid,50_000_000)
    samples = {}
    supports = {}
    for key, curve in curves.items():
        pos, rot, good = a.sample(curve, grid, 500_000_000)
        samples[key] = pos, rot
        supports[key] = good
        if key in primary:
            valid &= good
    assert all(supports[key][valid].all() for key in curves)
    ids = np.flatnonzero(valid)
    assert len(ids)>100
    cuts = np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
    O.mkdir()
    d.PINS.update(a.HASHES)
    d.save(H/'admission.json', dict(passed=True,created=datetime.datetime.now().astimezone().isoformat(),
        preserved_terminal_counts=sealed,protected_files=len(protected),primary_curves=primary,
        curves=list(curves),common_rows=len(ids),same_physical_transforms=True,
        source_runs=0,receiver_replays=0,no_new_matched_OFF_ON=True))
    folder = O/'common'
    rp, rt, rfs, pairs, info = refs(folder,grid,gp,gr,cuts)
    common = {}
    arrays = dict(timestamp_ns=grid, common_valid=valid, reference_position=gp, reference_rotation=gr)
    for key,(pos,rot) in samples.items():
        common[key] = d.score_series(folder,key,grid,pos,rot,gp,gr,cuts,rfs,rp,rt)
        arrays[key+'_position'] = pos
        arrays[key+'_rotation'] = rot
        arrays[key+'_support'] = supports[key]
    full = {}
    for key,curve in curves.items():
        if key.startswith('CP103'):
            full[key] = score_full(O/('full_'+key),key,curve,reference)
        else:
            if key.startswith('CP95'):
                oldkey = key.removeprefix('CP95_').lower()
                source, row = OLD/'results.json', old95['full_saved'][oldkey]
            else:
                oldkey = 'ON_GLIM' if key.endswith('glim') else 'ON_Kimera'
                source, row = EARLY/'results.json', old91['full_saved'][oldkey]
            full[key] = copy.deepcopy(row)
            full[key].update(newly_scored=False,reused_from=str(source),reused_key=oldkey)
            for kind in ('translation_m','rotation_deg'):
                d.pin(full[key]['RPE'][kind]['archive'])
    reproduction = {}
    oldnp = np.load(d.pin(OLD/'evaluation/comparison.npz'))
    same95 = np.array_equal(grid[valid],oldnp['timestamp_ns'][oldnp['common_valid']])
    if same95:
        for key,row in common.items():
            if key.startswith('CP95'):
                oldkey = key.removeprefix('CP95_').lower()
                delta = float(np.max(abs(np.asarray(metric_values(row))-metric_values(old95['common'][oldkey]))))
                assert delta < 1e-8,(key,delta)
                reproduction[key] = delta
    old91ref = d.file_interface.read_tum_trajectory_file(d.pin(EARLY/'evaluation/common/reference.tum'))
    same91 = np.array_equal((grid[valid]-epoch)*1e-9,old91ref.timestamps)
    if same91:
        for role,oldkey in (('glim','ON_GLIM'),('kimera','ON_Kimera')):
            key = 'CP91_ON_'+role
            delta = float(np.max(abs(np.asarray(metric_values(common[key]))-metric_values(old91['common'][oldkey]))))
            assert delta < 1e-8,(key,delta)
            reproduction[key] = delta
    coverage = d.m.c.coverage(grid,valid,epoch,duration)
    comparisons = {}
    for prefix in ('CP95_ON','CP95_OFF','CP91_ON'):
        comparisons[prefix+'_to_CP103_ON'] = {}
        for role in ('glim','kimera','fusion_online','fusion_delayed'):
            oldkey = prefix+'_'+role
            if oldkey not in common:
                continue
            oldvalue, newvalue = common[oldkey]['ATE_RMSE_m'], common['CP103_ON_'+role]['ATE_RMSE_m']
            comparisons[prefix+'_to_CP103_ON'][role] = dict(ATE_delta_m=newvalue-oldvalue,
                ATE_reduction_percent=100*(oldvalue-newvalue)/oldvalue,
                causal_effect_claim=False,fresh_same_binary_control=False)
    result = dict(passed=True,task='CP-V104',evo_version='1.36.3',capture_s=duration,
        common=common,full_saved=full,common_coverage=coverage,common_RPE_pairs=len(pairs),common_segments=info,
        primary_curves=primary,inputs=inputs,comparisons=comparisons,
        same_CP95_common_mask=same95,same_CP91_common_mask=same91,
        historical_metric_reproduction_max_errors=reproduction,verification=d.checks,
        new_evo_CLI_calls=len(d.commands),source_runs=0,receiver_replays=0,new_matched_OFF_ON=False,
        CP103_changed_backend=True,CP91_has_no_fusion=True,
        method='Method1: GLIM first-published absolute; Kimera newest absolute backend at exact native header stamps, NOT incremental ROS',
        reference='IE-TC at calibrated AP20 prism point; shared CPT7, not independent Leica/prism GT',
        alignment='whole-mask SE3 per curve, no scale; RPE consecutive reference-distance0.5m within each segment',
        omissions=dict(source_interpolation_max_s=.5,reference_interpolation_max_s=.05,
            missing_right_image_relative_s=203.898841154,common_mask_gaps_preserved=True,
            delayed='7-second fixed-lag-finalized, not ONLINE/full-history',
            CP103_receiver_before_init=25,CP103_expired_versions=134,CP103_unsupported_EOF=0,
            failed_CP99_ON_excluded=True),
        CP103_engineering_evidence=str(NEW/'qualification.json'),
        limits=['Changed backend and separate live realization: not a controlled causal effect of the correction',
                'No proof of randomness, calibrated covariance, independent peer beliefs or nonlinear stationarity',
                'Estimated sample polylines are not physical travel; full-saved and common coverage differ'])
    d.save(O/'commands.json',d.commands)
    np.savez_compressed(O/'comparison.npz',**arrays)
    d.PINS.update(a.HASHES)
    assert verify_manifest(d.PINS)==len(d.PINS)
    assert verify_manifest(protected)==8120
    for root in (NEW,OLD,EARLY):
        verify_manifest(json.loads((root/'complete.json').read_text())['output_sha256'])
    d.save(O/'input_sha256.json',d.PINS)
    d.save(H/'results.json',result)
    lines = ['CP-V104 — SAVED CP103 CORRECTED ON vs CP95 LIVE OFF/ON and CP91 SOURCE-ONLY ON',
        result['method'],result['reference'],result['alignment'],
        'No new capture/replay; CP103 changed backend, NOT a newly matched or same-binary repeat pair.',
        f"COMMON100ms: {coverage['valid_queries']} rows, {coverage['supported_s']:.6f}s/{duration}s ({coverage['supported_capture_percent']:.3f}%), {len(pairs)} reference-distance0.5m pairs.",
        f"Same original CP95 mask={same95}; same CP91 mask={same91}. Delayed curves do not shrink primary mask.",
        'Run/product | ATE m | translation RPE m | rotation RPE deg | segmented estimated/reference path m']
    for key,row in common.items():
        v = metric_values(row)
        lines.append('%s | %.6f | %.6f | %.6f | %.4f / %.4f'%(key,*v))
    lines += ['', 'FULL-SAVED cadence: new CP103 scores; verified CP95/91 results reused unchanged.',
        'Product | ATE m | tRPE m | rRPE deg | segmented path/reference m | rows | supported s | gaps | new score']
    for key,row in full.items():
        v = metric_values(row)
        lines.append('%s | %.6f | %.6f | %.6f | %.4f / %.4f | %d | %.6f | %d | %s'%(key,*v,row['rows'],row['supported_s'],row['gaps'],row['newly_scored']))
    lines += ['', 'ATE reductions versus each historical baseline (positive=lower error, not causal attribution):',
              json.dumps(comparisons,indent=2), '', *result['limits'],
              'Actual gaps/missing right image/25 pre-init and134expired CP103 versions retained; no gap filling.',
              'All4 CP103 runtime gates passed; uncertainty calibration and mixed-prior feedback remain unqualified.',
              'DELAYED7s-finalized is separate; CP91 fusion does not exist. Failed CP99 ON is excluded.',
              f'Direct EVO CLI calls: {len(d.commands)}; parity checks: {d.checks}.']
    with (H/'RESULTS_REPORT.txt').open('x') as f:
        f.write('\n'.join(lines)+'\n')
    print('\n'.join(lines[:22]),flush=True)


if __name__=='__main__':
    main()
