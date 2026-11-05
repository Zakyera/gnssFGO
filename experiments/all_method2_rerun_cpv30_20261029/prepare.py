"""Prepare immutable CP-V28 method2 trajectories and saved map context only."""
import importlib.util
import json
import sys
from pathlib import Path
sys.dont_write_bytecode = True
import numpy as np
from evo.core.geometry import umeyama_alignment

H = Path(__file__).resolve().parent
E = H.parent
OLD = E/'all_incremental_comparison_cpv28_20261028'
spec = importlib.util.spec_from_file_location('cpv28', OLD/'analyze.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
a = m.a
PINS = {}
REFS = {}
HISTORICAL_OUTPUTS = {}


def pin(path):
    path = Path(path).resolve()
    if str(path) not in PINS:
        PINS[str(path)] = a.g.sha(path)
    return path


def load(path):
    return json.loads(pin(path).read_text())


def fit(ep, gp, valid):
    rot, tr, scale = umeyama_alignment(ep[valid].T, gp[valid].T, with_scale=False)
    assert scale == 1 and abs(np.linalg.det(rot)-1) < 1e-10
    residual = ep[valid]@rot.T+tr-gp[valid]
    rmse = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
    return rot, tr, rmse


def prepare(case):
    target = H/(case+'.json')
    assert not target.exists() and not (H/(case+'.npz')).exists()
    d = load(OLD/(case+'.json'))
    z = np.load(pin(OLD/(case+'.npz')))
    for path in (OLD/(case+'.json'), OLD/(case+'.npz')):
        assert PINS[str(path)] == HISTORICAL_OUTPUTS[str(path)]
    assert d['method'] == 2 and d['original_receiver_Qc_multiplier'] == 1.0
    epoch, duration = d['epoch_ns'], d['capture_s']
    gp, valid = z['reference_position'], z['common_valid']
    gtpath = pin(m.c.b.local(d['reference']['path']))
    assert PINS[str(gtpath)] == d['reference']['sha256']
    if str(gtpath) not in REFS:
        ref = a.g.Reference(gtpath)
        poses = np.asarray(ref.poses)
        rp, _ = a.prism(poses[:, :3, 3], poses[:, :3, :3])
        REFS[str(gtpath)] = np.asarray(ref.times, np.int64), rp
    rt, rp = REFS[str(gtpath)]
    keep = (rt >= epoch) & (rt <= epoch+round(duration*1e9))
    origin = rp[np.flatnonzero(keep)[0]].copy()
    arrays = dict(reference_t_ns=rt[keep]-epoch, reference_p=rp[keep]-origin)
    products = {}
    for key, pathname in d['trajectories'].items():
        path = pin(pathname)
        expected = d['input_sha256'].get(str(path), HISTORICAL_OUTPUTS.get(str(path)))
        assert expected is not None and PINS[str(path)] == expected
        curve = a.load_curve(path)
        ts, p, r = curve
        native, _ = a.prism(p, r)
        sp, sr, ok = a.sample(curve, z['timestamp_ns'], 500_000_000)
        sp, _ = a.prism(sp, sr)
        assert ok[valid].all()
        assert np.max(abs(sp[valid]-z[key+'_position'][valid])) < 1e-9
        rot, tr, rmse = fit(sp, gp, valid)
        assert abs(rmse-d['common'][key]['ATE_RMSE_m']) < 1e-9
        aligned = native@rot.T+tr-origin
        steps = np.linalg.norm(np.diff(native, axis=0), axis=1)
        checksteps = np.linalg.norm(np.diff(aligned, axis=0), axis=1)
        assert np.max(abs(steps-checksteps)) < 1e-10
        full = d['full_saved'][key]
        assert len(ts) == full['rows']
        assert abs(float(steps.sum())-full['native_polyline_length_m']) < 1e-8
        quant = float(np.max(abs(aligned-aligned.astype(np.float32))))
        assert quant < 1e-4
        gaps = np.diff(ts) > 500_000_000
        assert int(gaps.sum()) == full['gap_chord_count']
        arrays[key+'_p'], arrays[key+'_t_ns'] = aligned, ts-epoch
        products[key] = dict(path=str(path), sha256=PINS[str(path)], rows=len(ts),
            alignment_rotation=rot.tolist(), alignment_translation_m=tr.tolist(),
            common_ATE_m=rmse, full_saved_ATE_m=full['ATE_RMSE_m'],
            path_with_gap_chords_m=float(steps.sum()),
            displayed_gap_aware_path_m=float(steps[~gaps].sum()),
            gaps_over_500ms=int(gaps.sum()), max_float32_error_m=quant)

    # Context map remains in corrected GLIM world, independently placed in ENU.
    # Its alignment must NOT be replaced by the drifting method2 chain's fit.
    h10 = E/'first_published_comparison_cpv10_20261026/outputs'
    old = load(h10/(case+'.json'))
    zz = np.load(pin(h10/(case+'.npz')))
    assert np.array_equal(valid, zz['common_valid'])
    maprot, maptr, maperr = fit(zz['on_glim_initial_position'],
                                zz['reference_position'], zz['common_valid'])
    assert abs(maperr-old['common']['on_glim_initial']['ATE_RMSE_m']) < 1e-9
    source = Path(d['trajectories']['on_kimera']).parent
    odom = np.loadtxt(pin(source/'odom_imu.txt'))
    clouds, submaps = [], []
    for path in sorted((source/'glim_dump').glob('[0-9]*/points_compact.bin')):
        lines = pin(path.with_name('data.txt')).read_text().splitlines()
        def matrix(label):
            idx = next(i for i,s in enumerate(lines) if s.strip() == label+':')
            mat = np.asarray([[float(v) for v in s.split()] for s in lines[idx+1:idx+5]])
            assert np.allclose(mat[3], [0,0,0,1])
            assert np.max(abs(mat[:3,:3].T@mat[:3,:3]-np.eye(3))) < 3e-6
            return mat
        world = matrix('T_world_origin')
        stamps = np.asarray([float(s.split()[1]) for s in lines if s.startswith('stamp: ')])
        idx = int(np.argmin(abs(odom[:,0]-stamps[0])))
        assert abs(odom[idx,0]-stamps[0]) < 1e-4
        imu = matrix('T_world_lidar')@matrix('T_lidar_imu')
        err = float(np.linalg.norm(imu[:3,3]-odom[idx,1:4]))
        assert err < 1e-3, (case, path, err)
        raw = np.fromfile(pin(path), dtype='<f4').reshape(-1,3).astype(np.float64)
        assert np.isfinite(raw).all()
        cloud = (raw@world[:3,:3].T+world[:3,3])@maprot.T+maptr-origin
        clouds.append(cloud.astype(np.float32))
        submaps.append(dict(id=path.parent.name, points=len(cloud), frame_check_m=err,
            first_s=float(stamps[0]-epoch*1e-9), last_s=float(stamps[-1]-epoch*1e-9)))
    assert clouds, (case, 'no saved cloud')
    cloud = np.concatenate(clouds)
    assert np.isfinite(cloud).all()
    np.savez_compressed(H/(case+'.npz'), **arrays)
    np.savez_compressed(H/(case+'_cloud.npz'), position=cloud)
    meta = dict(case=case, sensor=d['sensor'], sequence=d['sequence'], method=2,
        original_receiver_Qc_multiplier=1.0, epoch_ns=epoch, duration_s=duration,
        origin_enu_m=origin.tolist(), reference=d['reference'],
        reference_vertices=len(arrays['reference_p']), products=products,
        common_coverage=d['common_coverage'], terminal_omissions=d['original_terminal_omissions'],
        cloud_points=len(cloud), submaps=submaps,
        cloud_alignment=dict(source='CP-V10 ON first-published fit, not method2 fit',
            rotation=maprot.tolist(), translation_m=maptr.tolist()),
        trajectory_npz_sha256=a.g.sha(H/(case+'.npz')),
        cloud_npz_sha256=a.g.sha(H/(case+'_cloud.npz')))
    a.save(target, meta)
    print(json.dumps(dict(case=case, trajectory_points=sum(p['rows'] for p in products.values()),
        cloud_points=len(cloud), all_reported_ATE_and_path_checks=True)), flush=True)


if __name__ == '__main__':
    pin(Path(__file__)); pin(H/'PROTOCOL.txt')
    closeout = load(OLD/'closeout.json')
    assert closeout['pass_']
    HISTORICAL_OUTPUTS.update(closeout['output_sha256'])
    for mod in (m,m.base,m.fullmod,m.c,m.c.b,a,a.g): pin(Path(mod.__file__))
    for row in m.c.b.ROWS: prepare(row[0])
    assert all(a.g.sha(Path(p)) == sha for p,sha in PINS.items())
    a.save(H/'input_checks.json', dict(passed=True, input_sha256=PINS,
        cases=[row[0] for row in m.c.b.ROWS], original_inputs_unchanged=True))
