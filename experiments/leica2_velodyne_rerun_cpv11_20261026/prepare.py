"""Export immutable native-rate CP-V4 trajectories using CP-V10 rigid fits."""
import importlib.util
import json
import sys
from pathlib import Path
sys.dont_write_bytecode = True
import numpy as np
from evo.core.geometry import umeyama_alignment

H = Path(__file__).resolve().parent
OLD = H.parent / 'first_published_comparison_cpv10_20261026'
spec = importlib.util.spec_from_file_location('cpv10', OLD / 'analyze.py')
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
a = c.a


def main():
    assert not (H/'trajectories.npz').exists() and not (H/'data_checks.json').exists()
    a.HASHES.clear()
    for f in (Path(__file__), H/'PROTOCOL.txt', OLD/'analyze.py',
              Path(c.b.__file__), Path(a.__file__), Path(a.g.__file__)):
        a.pin(f)
    metadata_path = a.pin(OLD/'outputs/velodyne_leica2.json')
    mask_path = a.pin(OLD/'outputs/velodyne_leica2.npz')
    old = json.loads(metadata_path.read_text())
    z = np.load(mask_path)
    epoch, duration = old['epoch_ns'], old['capture_s']
    gtpath = c.b.local(old['reference']['path'])
    assert a.g.sha(a.pin(gtpath)) == old['reference']['sha256']
    ref = a.g.Reference(gtpath)
    rt = np.asarray(ref.times, np.int64)
    poses = np.asarray(ref.poses)
    refcurve = rt, poses[:, :3, 3], poses[:, :3, :3]
    rp, _ = a.prism(refcurve[1], refcurve[2])
    keep = (rt >= epoch) & (rt <= epoch+round(duration*1e9))
    origin = rp[np.flatnonzero(keep)[0]].copy()
    arrays = {'reference_t_ns': rt[keep]-epoch, 'reference_p': rp[keep]-origin}
    checks = {'case':'CP-V4 LEICA-2 Velodyne baseline', 'epoch_ns':epoch,
              'duration_s':duration,'origin_reference_enu_m':origin.tolist(),
              'reference':old['reference'], 'common_coverage':old['common_coverage'],
              'products':{},'alignment':'single per-product SE(3), no scale, CP-V10 common mask',
              'source_rerun':False, 'trajectory_resampling_or_smoothing':False}
    valid = z['common_valid']
    for mode in ('off','on'):
        for role in c.ROLES:
            key = mode+'_'+role
            record = old['input_products'][key]
            path = a.pin(Path(record['path']))
            assert a.g.sha(path) == record['sha256']
            ts, p, r = a.load_curve(path)
            assert len(ts) == record['rows']
            p, _ = a.prism(p,r)
            rot, tr, scale = umeyama_alignment(z[key+'_position'][valid].T,
                                               z['reference_position'][valid].T, with_scale=False)
            assert scale == 1.
            residual = z[key+'_position'][valid]@rot.T+tr-z['reference_position'][valid]
            rmse = float(np.sqrt(np.mean(np.sum(residual**2,axis=1))))
            assert abs(rmse-old['common'][key]['ATE_RMSE_m']) < 1e-9
            aligned = p@rot.T+tr-origin
            # A rigid display transform must preserve every native edge length.
            rawsteps = np.linalg.norm(np.diff(p,axis=0),axis=1)
            steps = np.linalg.norm(np.diff(aligned,axis=0),axis=1)
            assert np.max(np.abs(rawsteps-steps)) < 1e-10
            dt = np.diff(ts)
            joined = dt <= 500_000_000
            quant = float(np.max(np.abs(aligned-aligned.astype(np.float32))))
            assert quant < 1e-4
            arrays[key+'_t_ns'] = ts-epoch
            arrays[key+'_p'] = aligned
            product = dict(record, alignment_rotation=rot.tolist(), alignment_translation_m=tr.tolist(),
                           common_ATE_check_m=rmse, native_vertices=len(p),
                           native_path_including_gap_chords_m=float(steps.sum()),
                           displayed_gap_aware_path_m=float(steps[joined].sum()),
                           gaps_over_500ms=int((~joined).sum()),
                           max_float32_coordinate_error_m=quant)
            if role == 'fusion_online':
                gp, gr, gok = a.sample(refcurve,ts,50_000_000)
                assert gok.all() and np.all(dt==100_000_000)
                gp,_ = a.prism(gp,gr)
                gtsteps = np.linalg.norm(np.diff(gp,axis=0),axis=1)
                arrays[mode+'_step_t_ns'] = ts[1:]-epoch
                arrays[mode+'_fusion_step_m'] = steps
                arrays[mode+'_reference_step_m'] = gtsteps
                product['reference_at_same_native_timestamps_path_m'] = float(gtsteps.sum())
                product['max_100ms_step_m'] = float(steps.max())
                product['p95_100ms_step_m'] = float(np.percentile(steps,95))
            checks['products'][key] = product
    windows=[]
    # Choose a reproducible inspection example, not a representative-window claim.
    for start in range(1,381):
        cm = (z['time_s']>=start)&(z['time_s']<=start+10)
        if cm.sum()<99 or not valid[cm].all():
            continue
        t = arrays['off_step_t_ns']*1e-9
        sel = (t>start)&(t<=start+10)
        if sel.sum()!=100:
            continue
        excess = float((arrays['off_fusion_step_m'][sel]-arrays['off_reference_step_m'][sel]).sum())
        windows.append((excess,start))
    excess,start = max(windows)
    checks['selected_inspection_window'] = {'start_s':start,'end_s':start+10,
        'selection':'largest OFF fusion-minus-reference 10s path excess, integer-second starts, fully CP-V10-common-supported',
        'off_path_excess_m':excess, 'not_representative':True}
    checks['reference_native_vertices'] = len(arrays['reference_p'])
    checks['original_inputs_unchanged'] = all(a.g.sha(Path(p))==sha for p,sha in a.HASHES.items())
    assert checks['original_inputs_unchanged']
    checks['input_sha256'] = dict(a.HASHES)
    np.savez_compressed(H/'trajectories.npz',**arrays)
    checks['npz_sha256'] = a.g.sha(H/'trajectories.npz')
    a.save(H/'data_checks.json',checks)
    print(json.dumps({k:checks[k] for k in ('selected_inspection_window','reference_native_vertices','original_inputs_unchanged')},indent=2))
    for k,p in checks['products'].items():
        print(k, p['native_vertices'],p['displayed_gap_aware_path_m'],p.get('max_100ms_step_m'))


if __name__ == '__main__':
    main()
