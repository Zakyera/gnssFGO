"""CP-V27 native absolute poses, CP-V32 evo fits, identical CP-V30 context."""
import importlib.util
import hashlib
import json
import sys
from pathlib import Path
sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial.transform import Rotation
from evo.core import metrics
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface
import evo

H = Path(__file__).resolve().parent
E = H.parent
H27 = E/'newest_absolute_comparison_cpv27_20261028'
H30 = E/'all_method2_rerun_cpv30_20261029'
H32 = E/'all_evo_tables_cpv32_20261029'
spec = importlib.util.spec_from_file_location('cpv27', H27/'analyze.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
a = m.a
PINS = {}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        while data := f.read(8*1024*1024): h.update(data)
    return h.hexdigest()


def pin(path, expected=None):
    path = Path(path).resolve()
    digest = sha(path)
    if expected is not None: assert digest == expected, path
    if str(path) in PINS: assert digest == PINS[str(path)], path
    PINS[str(path)] = digest
    return path


def load(path, expected=None):
    return json.loads(pin(path, expected).read_text())


def save(path, obj):
    with Path(path).open('x') as f: json.dump(obj, f, indent=2)


def traj(t, p, r):
    q = Rotation.from_matrix(r).as_quat()[:, [3,0,1,2]]
    return PoseTrajectory3D(positions_xyz=p, orientations_quat_wxyz=q,
                            timestamps=(t-t[0]).astype(float)*1e-9)


def prepare(case, pins27, pins32):
    target = H/(case+'.npz')
    assert not target.exists() and not target.with_suffix('.json').exists()
    dpath, zpath = H27/(case+'.json'), H27/(case+'.npz')
    d = load(dpath, pins27[str(dpath)])
    z = np.load(pin(zpath, pins27[str(zpath)]))
    metricpath = H32/(case+'.json')
    table = load(metricpath, pins32[str(metricpath)])
    context = load(H30/(case+'.json'))
    ctx = np.load(pin(H30/(case+'.npz'), context['trajectory_npz_sha256']))
    cloudpath = pin(H30/(case+'_cloud.npz'), context['cloud_npz_sha256'])
    assert d['policy'] == 'newest_absolute_graph_pose_at_output_update'
    assert d['original_receiver_Qc_multiplier'] == context['original_receiver_Qc_multiplier'] == 1.0
    assert d['common_coverage'] == context['common_coverage'] == table['common_coverage']
    assert d['reference'] == context['reference']
    epoch = context['epoch_ns']
    origin = np.asarray(context['origin_enu_m'])
    arrays = dict(reference_t_ns=ctx['reference_t_ns'], reference_p=ctx['reference_p'])
    grid, valid = z['timestamp_ns'], z['common_valid']
    gt = traj(grid[valid], z['reference_position'][valid], z['reference_rotation'][valid])
    products = {}
    for mode in ('off','on'):
        for role, oldrole in [('glim','glim_initial'),('kimera','kimera_absolute'),('fusion','fusion_online')]:
            key, oldkey = mode+'_'+role, mode+'_'+oldrole
            if role == 'kimera':
                curve = (z[mode+'_kimera_timestamp_ns'],
                         z[mode+'_kimera_absolute_body_position'],
                         z[mode+'_kimera_absolute_body_rotation'])
                source = dict(path=str(zpath), sha256=PINS[str(zpath)],
                              product='recovered absolute backend, NOT native incremental ROS')
            else:
                record = d['old_input_products'][oldkey]
                path = pin(record['path'], record['sha256'])
                curve = a.load_curve(path)
                source = dict(path=str(path), sha256=PINS[str(path)], product=oldrole)
            ts, bodyp, bodyr = curve
            assert np.all(np.diff(ts)>0)
            native, nr = a.prism(bodyp, bodyr)
            sp, sr, good = a.sample(curve, grid, 500_000_000)
            sp, sr = a.prism(sp, sr)
            assert good[valid].all()
            diff = float(np.max(abs(sp[valid]-z[oldkey+'_position'][valid])))
            assert diff < 1e-9
            entry = table['results']['method1_'+key]
            resultpath = Path(entry['ATE_evo_archive'])
            result = file_interface.load_res_file(pin(resultpath, pins32[str(resultpath)]))
            T = result.np_arrays['alignment_transformation_sim3']
            assert np.allclose(T[3], [0,0,0,1], atol=1e-14)
            rot, tr = T[:3,:3], T[:3,3]
            assert np.max(abs(rot.T@rot-np.eye(3))) < 1e-12
            assert abs(np.linalg.det(rot)-1) < 1e-12
            estimate = traj(grid[valid], sp[valid], sr[valid])
            estimate.transform(T)
            ape = metrics.APE(metrics.PoseRelation.translation_part)
            ape.process_data((gt, estimate))
            rmse = float(ape.get_statistic(metrics.StatisticsType.rmse))
            assert abs(rmse-entry['ATE_RMSE_m']) < 1e-9
            error_diff = float(np.max(abs(ape.error-result.np_arrays['error_array'])))
            assert error_diff < 1e-9
            aligned = native@rot.T+tr-origin
            full = d['full_saved'][oldkey]
            assert len(ts) == full['rows']
            native_traj = traj(ts, native, nr)
            native_path = float(native_traj.path_length)
            assert abs(native_path-full['native_polyline_length_m']) < 1e-8
            gaps = np.diff(ts)>500_000_000
            assert int(gaps.sum()) == full['gap_chord_count']
            groups = np.split(np.arange(len(ts)), np.flatnonzero(gaps)+1)
            drawn_path = float(sum(traj(ts[ix], native[ix], nr[ix]).path_length for ix in groups if len(ix)>1))
            # Geometry invariant, not an alternative accuracy metric.
            before, after = np.diff(native,axis=0), np.diff(aligned,axis=0)
            assert np.max(abs(after-before@rot.T)) < 1e-10
            quant = float(np.max(abs(aligned-aligned.astype(np.float32))))
            assert quant < 1e-4 and np.isfinite(aligned).all()
            arrays[key+'_p'], arrays[key+'_t_ns'] = aligned, ts-epoch
            products[key] = dict(**source, rows=len(ts),
                alignment_evo_archive=str(resultpath), alignment_SE3=T.tolist(),
                common_ATE_m=rmse, common_path_m=entry['path_length_m'],
                common_RPE_translation_m=entry['RPE']['translation_m']['statistics']['rmse'],
                common_RPE_rotation_deg=entry['RPE']['rotation_deg']['statistics']['rmse'],
                common_RPE_pairs=entry['RPE']['translation_m']['pairs'],
                path_with_gap_chords_m=native_path, displayed_gap_aware_path_m=drawn_path,
                gaps_over_500ms=int(gaps.sum()), max_float32_error_m=quant,
                common_pose_max_difference_m=diff, evo_error_max_difference_m=error_diff,
                first_s=float((ts[0]-epoch)*1e-9), last_s=float((ts[-1]-epoch)*1e-9))
    np.savez_compressed(target, **arrays)
    meta = dict(case=case, sensor=d['sensor'], sequence=d['sequence'], method=1,
        original_receiver_Qc_multiplier=1.0, epoch_ns=epoch, duration_s=d['capture_s'],
        origin_enu_m=origin.tolist(), reference=d['reference'],
        reference_vertices=len(arrays['reference_p']), products=products,
        common_coverage=d['common_coverage'], terminal_omissions=d['original_terminal_omissions'],
        kimera_reconstruction_checks=d['kimera_reconstruction_checks'],
        cloud_path=str(cloudpath), cloud_points=context['cloud_points'],
        submaps=context['submaps'], cloud_alignment=context['cloud_alignment'],
        trajectory_npz_sha256=sha(target), cloud_npz_sha256=context['cloud_npz_sha256'],
        reference_and_cloud_identical_to_method2=True, evo_version=evo.__version__)
    save(target.with_suffix('.json'), meta)
    print(json.dumps(dict(case=case, native_estimator_points=sum(p['rows'] for p in products.values()),
                         cloud_points=meta['cloud_points'], checks_passed=True)), flush=True)


if __name__ == '__main__':
    pin(__file__); pin(H/'PROTOCOL.txt')
    for module in (m,m.c,m.c.b,a,a.g): pin(module.__file__)
    c27, c30, c32 = (load(p/'closeout.json') for p in (H27,H30,H32))
    assert c27['passed'] and c30['passed'] and c32['passed']
    for path, digest in c30['outputs'].items(): pin(path, digest)
    cases = [row[0] for row in m.c.b.ROWS]
    for case in cases: prepare(case, c27['output_sha256'], c32['output_sha256'])
    assert all(sha(p)==digest for p,digest in PINS.items())
    save(H/'input_checks.json', dict(passed=True, cases=cases, input_sha256=PINS,
         method2_outputs=c30['outputs'], original_inputs_unchanged=True,
         source_or_receiver_runs=0, evo_version=evo.__version__))
