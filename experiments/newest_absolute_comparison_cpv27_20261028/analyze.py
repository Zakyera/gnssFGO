"""CP-V27: saved newest absolute outputs; no estimator or receiver execution."""
import csv
import importlib.util
import json
import re
import sys
from pathlib import Path

sys.dont_write_bytecode = True
import numpy as np
from scipy.spatial.transform import Rotation as R

H = Path(__file__).resolve().parent
H10 = H.parent/'first_published_comparison_cpv10_20261026'
spec = importlib.util.spec_from_file_location('cpv10', H10/'analyze.py')
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
a = c.a
SOURCE = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/fixed_lag_current_chart_fl4q_20261016/VioBackend.cpp')
ROLES = ('glim_initial', 'kimera_absolute', 'fusion_online')
ANSI = re.compile(r'\x1b\[[0-9;]*m')


def load(path):
    return json.loads(a.pin(path).read_text())


def rounding_bound(values):
    """Half one last-place unit of default C++ six-significant-digit logging."""
    values = np.asarray(values)
    return np.where(values == 0, 0., .5*10.**(np.floor(np.log10(np.maximum(abs(values), 1e-300)))-5))


def kimera(source):
    path = a.pin(source/'estimators.log')
    selected = []
    with path.open(errors='strict') as f:
        for line in f:
            if 'KIMERA_BACKEND_POSE_PATH_ROW,' in line:
                fields = ANSI.sub('', line.split('KIMERA_BACKEND_POSE_PATH_ROW,', 1)[1]).strip().split(',')
                assert len(fields) == 24, (path, fields)
                selected.append(fields)
    with a.pin(source/'kimera_odometry.csv').open() as f:
        pubs = list(csv.DictReader(f))
    assert len(selected) == len(pubs) > 100
    ids = [int(x[1]) for x in selected]
    seq = [int(x['field.header.seq']) for x in pubs]
    assert ids == seq == list(range(len(pubs))), (path, ids[:4], seq[:4])
    stamps = np.asarray([int(x['field.header.stamp']) for x in pubs], np.int64)
    assert np.all(np.diff(stamps) > 0)
    assert {x['field.header.frame_id'] for x in pubs} == {'odom'}
    assert {x['field.child_frame_id'] for x in pubs} == {'cpt7_imu'}
    data = np.asarray(selected, float)
    assert np.isfinite(data).all() and np.all(data[:, 23] == 0)
    # Logged epoch stamps lose subsecond precision. IDs plus EVERY published
    # incremental pose identify the corresponding exact CSV header timestamp.
    native = np.asarray([[float(p['field.pose.pose.position.'+k]) for k in 'xyz']+
                         [float(p['field.pose.pose.orientation.'+k]) for k in 'wxyz'] for p in pubs])
    logged_inc = data[:, 9:16]
    # Quaternion sign equivalence, not an alternative frame or body transform.
    sign = np.where(np.sum(native[:, 3:]*logged_inc[:, 3:], axis=1) < 0, -1., 1.)
    native[:, 3:] *= sign[:, None]
    bounds = rounding_bound(logged_inc)
    delta = abs(native-logged_inc)
    assert np.all(delta <= bounds+2e-12), (path, float(np.max(delta-bounds)))
    raw = data[:, 2:9]
    graph_q = raw[:, [4,5,6,3]]
    norm_error = float(np.max(abs(np.linalg.norm(graph_q, axis=1)-1)))
    assert norm_error < 2e-6
    graph_r = R.from_quat(graph_q).as_matrix()
    inc_r = R.from_quat(logged_inc[:, [4,5,6,3]]).as_matrix()
    raw_bound = rounding_bound(raw)
    q_bound = np.linalg.norm(raw_bound[:, 3:], axis=1)
    # Conservative bound for normalized quaternion rotation and prism position.
    angle_bound = 4*np.arcsin(np.minimum(1., q_bound/(np.linalg.norm(graph_q,axis=1)-q_bound)))
    prism_bound = np.linalg.norm(raw_bound[:, :3], axis=1)+2*np.linalg.norm(a.TP)*np.sin(angle_bound/2)
    maximum_prism_bound = float(prism_bound.max())
    assert maximum_prism_bound < .001
    checks = dict(rows=len(pubs), first_keyframe=ids[0], last_keyframe=ids[-1],
                  every_id_unique_contiguous_and_matches_csv=True,
                  exact_header_stamps_used=True, rounded_log_timestamps_not_used=True,
                  every_increment_pose_within_log_rounding_bounds=True,
                  increment_position_max_difference_m=float(np.linalg.norm(delta[:, :3], axis=1).max()),
                  increment_quaternion_max_component_difference=float(delta[:, 3:].max()),
                  graph_quaternion_norm_max_error=norm_error,
                  graph_prism_position_rounding_bound_m=maximum_prism_bound,
                  graph_rotation_rounding_bound_rad=float(angle_bound.max()),
                  logged_no_incremental_pose=False,
                  graph_vs_increment_position_max_m=float(np.linalg.norm(raw[:, :3]-logged_inc[:, :3], axis=1).max()),
                  source_log=str(path))
    return (stamps, raw[:, :3], graph_r), (stamps, logged_inc[:, :3], inc_r), checks


def full_metric(curve, reference, epoch, duration):
    ts,p,r = curve
    gp,gr,good = a.sample(reference,ts,50_000_000)
    assert good.all()
    ep,_ = a.prism(p,r)
    gp,_ = a.prism(gp,gr)
    av,_,independent = c.ate(ep,gp)
    return dict(ATE_RMSE_m=av, rows=len(ts), reference_supported_rows=int(good.sum()),
                first_s=float((ts[0]-epoch)*1e-9), last_s=float((ts[-1]-epoch)*1e-9),
                span_s=float((ts[-1]-ts[0])*1e-9), tail_gap_s=float(duration-(ts[-1]-epoch)*1e-9),
                native_polyline_length_m=float(np.linalg.norm(np.diff(ep,axis=0),axis=1).sum()),
                reference_native_polyline_length_m=float(np.linalg.norm(np.diff(gp,axis=0),axis=1).sum()),
                gap_chord_count=int(np.sum(np.diff(ts)>500_000_000)), evo_alignment_difference_m=independent)


def analyze(row):
    case,sensor,sequence,_,_,duration,epoch = row
    target = H/(case+'.json')
    assert not target.exists() and not (H/(case+'.npz')).exists(), 'Refuse overwrite'
    a.HASHES.clear()
    for p in (Path(__file__), H/'PROTOCOL.txt', Path(c.__file__), Path(c.b.__file__), Path(a.__file__), Path(a.g.__file__), SOURCE):
        a.pin(p)
    old = load(H10/'outputs'/(case+'.json'))
    saved = np.load(a.pin(H10/'outputs'/(case+'.npz')))
    grid,valid = saved['timestamp_ns'],saved['common_valid']
    _,_,ref = c.b.load(row)
    reference = ref.times, np.asarray(ref.poses)[:,:3,3], np.asarray(ref.poses)[:,:3,:3]
    gp,gr,gtok = a.sample(reference,grid,50_000_000)
    gp,gr = a.prism(gp,gr)
    assert np.array_equal(gp,saved['reference_position'])
    assert np.array_equal(gr,saved['reference_rotation'])
    curves,checks,arrays = {},{},dict(timestamp_ns=grid,common_valid=valid,
                                    reference_position=gp,reference_rotation=gr)
    for mode in ('off','on'):
        src = Path(old['input_products'][mode+'_kimera']['path']).parent
        manifest = load(src.parent/'protected_manifest.json')
        assert manifest[str(SOURCE)] == a.HASHES[str(SOURCE)]
        settings = load(src/'effective_mode_check.json')
        assert settings['passed'] and '--no_incremental_pose=false' in json.dumps(settings)
        graph,rounded_inc,checks[mode] = kimera(src)
        curves[mode+'_kimera_absolute'] = graph
        curves[mode+'_kimera_rounded_incremental_control'] = rounded_inc
        arrays[mode+'_kimera_timestamp_ns'] = graph[0]
        arrays[mode+'_kimera_absolute_body_position'] = graph[1]
        arrays[mode+'_kimera_absolute_body_rotation'] = graph[2]
        for role in ('glim_initial','kimera','fusion_online'):
            key=mode+'_'+role
            record=old['input_products'][key]
            p=a.pin(Path(record['path']))
            assert a.HASHES[str(p)] == record['sha256']
            curves[key] = a.load_curve(p)
    common,full,old_checks = {},{},{}
    current_valid=gtok.copy()
    primary=[m+'_'+r for m in ('off','on') for r in ROLES]
    for key,curve in curves.items():
        p,r,good=a.sample(curve,grid,500_000_000)
        if key in primary: current_valid &= good
        assert np.all(good[valid])
        ep,er=a.prism(p,r)
        common[key],_=c.metrics(ep,er,gp,gr,valid,grid)
        full[key]=full_metric(curve,reference,epoch,duration)
        if key in old['common']:
            dc=abs(common[key]['ATE_RMSE_m']-old['common'][key]['ATE_RMSE_m'])
            df=abs(full[key]['ATE_RMSE_m']-old['full_saved'][key]['ATE_RMSE_m'])
            dp=abs(common[key]['path_length_m']-old['common'][key]['path_length_m'])
            assert max(dc,df,dp)<1e-10, (key,dc,df,dp)
            old_checks[key]=dict(common_ATE_difference_m=dc,full_ATE_difference_m=df,common_path_difference_m=dp)
        if key in primary:
            arrays[key+'_position']=ep
            arrays[key+'_rotation']=er
    assert np.array_equal(current_valid,valid), 'No mask change permitted'
    for mode in ('off','on'):
        original=mode+'_kimera'; rounded=mode+'_kimera_rounded_incremental_control'
        checks[mode]['rounded_incremental_common_ATE_difference_m']=abs(common[rounded]['ATE_RMSE_m']-common[original]['ATE_RMSE_m'])
        checks[mode]['rounded_incremental_full_ATE_difference_m']=abs(full[rounded]['ATE_RMSE_m']-full[original]['ATE_RMSE_m'])
        assert checks[mode]['rounded_incremental_common_ATE_difference_m']<.001
        assert checks[mode]['rounded_incremental_full_ATE_difference_m']<.001
    data=dict(case=case,sensor=sensor,sequence=sequence,capture_s=duration,
              policy='newest_absolute_graph_pose_at_output_update', original_receiver_Qc_multiplier=1.,
              reference=old['reference'], common_coverage=old['common_coverage'], mask_identical_to_CPV10=True,
              common=common,full_saved=full,kimera_reconstruction_checks=checks,
              unchanged_product_checks=old_checks,synthetic_geometry_tests=a.tests(),
              old_input_products=old['input_products'],
              original_terminal_omissions=old['original_terminal_omissions'],
              off_on={r:c.improve(common['off_'+r]['ATE_RMSE_m'],common['on_'+r]['ATE_RMSE_m']) for r in ROLES})
    assert all(a.g.sha(Path(p))==sha for p,sha in a.HASHES.items())
    data['input_sha256']=dict(a.HASHES);data['input_hashes_unchanged']=True
    np.savez_compressed(H/(case+'.npz'),**arrays)
    a.save(target,data)
    print(json.dumps(dict(case=case,ATE={k:common[k]['ATE_RMSE_m'] for k in primary},checks=checks)),flush=True)


if __name__=='__main__':
    selected=[r for r in c.b.ROWS if sys.argv[1] in ('all',r[0])]
    assert selected
    for row in selected: analyze(row)
