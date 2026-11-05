"""Saved EIG-2 peer-factor audit. Never starts an estimator or optimizer."""
from pathlib import Path
import importlib.util
import json
import re
import sys
import numpy as np

sys.dont_write_bytecode = True
H = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('fl4z', H.parent/'fixed_lag_covariance_fl4z_20261017/analyze.py')
a = importlib.util.module_from_spec(spec); spec.loader.exec_module(a)
g = a.geo
W = a.W
ROOT = W/'runs/gnssfgo_eig2_local_cov60_on_cpr3_20261019/prepared'
SRC = ROOT/'live60_on_source'


def keyed(path, tag, direction, decode):
    out = {}
    for _, f in g.rows(path, [tag]):
        if direction is not None and f[0] != direction:
            continue
        key, value = decode(f)
        assert key not in out, (tag, key)
        out[key] = value
    return out


def main():
    assert not (H/'results.json').exists()
    inputs = {}
    def protect(p):
        inputs[str(p)] = g.sha(p)
        return p
    for p in (Path(__file__), H/'PROTOCOL.txt', Path(a.__file__), Path(g.__file__)):
        protect(p)
    manifest = json.loads(protect(ROOT/'protected_manifest.json').read_text())
    code = [W/'runs/local_revision_covariance_cpi1_20261019/cbs_bridge.cpp',
            W/'runs/fixed_lag_current_chart_fl4q_20261016/VioBackend.cpp',
            W/'src/glim_ros1/include/glim_ros/cbs_native_imu_mean_bridge.hpp',
            W/'src/Kimera-VIO/include/kimera-vio/imu-frontend/CbsNativeImuMeanBridge.h',
            W/'src/cbs/src/bpsam/bpsam.cpp']
    code_checks = {}
    for p in code:
        protect(p)
        pinned = str(p) in manifest
        if pinned:
            assert inputs[str(p)] == manifest[str(p)], p
        code_checks[str(p)] = dict(pinned_to_run_manifest=pinned, sha256=inputs[str(p)])
    for name in ('local_provenance.json', 'candidate_validation.json', 'effective_mode_check.json'):
        p = protect(SRC/name); d = json.loads(p.read_text()); assert d['passed']
        if name == 'effective_mode_check.json':
            assert d['glim_revision_covariance'] == 'native_local'
    protect(ROOT/'live60_on_control/complete.json')
    cp1 = H.parent/'local_covariance_error_cpa1_20261019/eig2_results.json'
    previous = json.loads(protect(cp1).read_text())
    cfg = previous['config']
    versions, first, source_checks, _ = a.read_events(ROOT, 'live60', 'on', cfg, inputs)
    reference = g.Reference(protect(Path(previous['reference']['path'])))
    injections, checks = a.injection_records(ROOT, cfg, reference, first, versions, inputs)
    logs = {'G2K': next(SRC.glob('ros_logs/*/kimera_vio_ros-*-stdout.log')),
            'K2G': next(SRC.glob('ros_logs/*/glim_rosnode-*-stdout.log'))}
    frozen = keyed(protect(SRC/'estimators.log'), 'FL4M_FROZEN_FACTOR_ROW', None,
                   lambda f: ((int(f[3]), int(f[4])), (g.matrix(f[5]).ravel(), g.matrix(f[6]))))
    source_lookup = {(x['source'], x['source_i'], x['source_j']): key for key, x in first.items()}
    results = {}
    def idx(x):
        assert x[0] == 'x'
        return int(x[1:])
    for direction, group in injections.items():
        path = protect(logs[direction])
        matrices = keyed(path, 'CBS_ODOM_FACTOR_MATRIX_ROW', direction,
                         lambda f: ((idx(f[1]), idx(f[2])), (f[3], g.matrix(f[4]))))
        if direction == 'G2K':
            slots = {}
            for _, f in g.rows(path, ['CBS_BPSAM_ACTIVE_WINDOW_RECEIVER_EDGE_ROW']):
                if f[3] == 'active_window_slot_tracked':
                    key = tuple(map(idx, f[2].split('->')))
                    assert key not in slots; slots[key] = f[4:]
        else:
            slots = {}
            for _, f in g.rows(path, ['GLIM_CBS_ACTIVE_WINDOW_FACTOR_ROW']):
                if f[0] == 'slot_tracked':
                    key = idx(f[1]), idx(f[2])
                    assert key not in slots; slots[key] = f[3:]
        rr = group['records']
        assert len(rr) == len(matrices) == len(slots)
        assert checks[direction]['matches_exported_version'] == len(rr)
        keys = set(); max_c = max_mean = 0.; rounded_ns = 0
        for r in rr:
            key = r['receiver_i'], r['receiver_j']
            assert key not in keys; keys.add(key); assert key in slots
            action, installed = matrices[key]
            assert action == ('active_window_temporary_odom_applied' if direction == 'G2K'
                              else 'active_window_temporary_graph_queued')
            C = np.asarray(r['cov']); mu = np.asarray(r['aligned_mu']); Z = g.exp(mu)
            max_c = max(max_c, float(np.max(np.abs(C-installed))))
            if direction == 'G2K':
                fm, fc = frozen[key]
                max_mean = max(max_mean, float(np.max(np.abs(Z-g.exp(fm)))))
                max_c = max(max_c, float(np.max(np.abs(fc-installed))))
            source = 'glim_hesai' if direction == 'G2K' else 'kimera_front'
            sk = source_lookup[source, r['source_i'], r['source_j']]
            candidates = [v for v in versions[sk].values()
                          if np.allclose(v['cov'], C, rtol=1e-10, atol=1e-18)
                          and np.allclose(v['mu'], r['raw_mu'], rtol=1e-10, atol=1e-12)]
            assert len(candidates) == 1, (direction, key, len(candidates))
            v = candidates[0]
            delta_ns = max(abs(v['a']-r['source_from_ns']), abs(v['b']-r['source_to_ns']))
            rounded_ns = max(rounded_ns, delta_ns); assert delta_ns <= 512
            assert max(r['source_to_ns'], r['receiver_to_ns']) <= r['causal_limit_ns']
            r['source_version'] = {k:z for k,z in v.items() if k not in ('mu','cov')}
            r['factor_action'] = action; r['installed_slot_log'] = slots[key]
            r['unaligned_on_receiver'] = a.measure(dict(a=r['receiver_from_ns'], b=r['receiver_to_ns'],
                                                       mu=np.asarray(r['raw_mu']), cov=installed), reference)
            r['raw_exact_source_timestamps'] = a.measure(v, reference)
            delta = g.log(g.inv(g.exp(r['raw_mu']))@Z)
            r['alignment_delta_rot_deg'] = float(np.linalg.norm(delta[:3])*180/np.pi)
            r['alignment_delta_rho_mm'] = float(np.linalg.norm(delta[3:])*1000)
            r['from_shift_ms'] = (r['receiver_from_ns']-r['source_from_ns'])/1e6
            r['to_shift_ms'] = (r['receiver_to_ns']-r['source_to_ns'])/1e6
        assert keys == set(slots) == set(matrices)
        if direction == 'G2K': assert keys == set(frozen)
        assert max_c == 0 and max_mean == 0
        pre = keyed(path, 'CBS_ODOM_PREINJECTION_RESIDUAL_ROW', direction,
                    lambda f: ((idx(f[3]), idx(f[4])), np.asarray(f[7:],float)))
        bykey = {(r['receiver_i'],r['receiver_j']):r for r in rr}
        sample_n = 0; max_q_error = 0.; max_info_error = 0.
        for _,f in g.rows(path,['CBS_COVARIANCE_EXCHANGE_MATRIX_ROW']):
            if f[0] != direction: continue
            key=idx(f[5]),idx(f[6]); assert key in bykey
            C=np.asarray(bykey[key]['cov'])
            if f[8:10] == ['receiver_preinjection_state','relative_pose_matrix']:
                e=g.log(g.inv(g.exp(bykey[key]['aligned_mu']))@g.matrix(f[12]))
                q=float(e@np.linalg.solve(C,e));sample_n+=1
                max_q_error=max(max_q_error,abs(q-pre[key][7])/(1+q))
            elif f[8:10] == ['receiver_inserted_factor','information_omega']:
                info=g.matrix(f[12]);max_info_error=max(max_info_error,float(np.max(np.abs(C@info-np.eye(6)))))
        assert max_q_error<1e-8 and max_info_error<1e-8
        stages=['raw','raw_exact_source_timestamps','aligned','unaligned_on_receiver']
        summaries={s:a.summarize([r[s] for r in rr]) for s in stages}
        extra={k:a.stat([r[k] for r in rr]) for k in ['from_shift_ms','to_shift_ms',
                'alignment_delta_rot_deg','alignment_delta_rho_mm']}
        extra['aligned_better_than_unaligned_receiver_fraction']={
            k:float(np.mean([r['aligned'][k]<r['unaligned_on_receiver'][k] for r in rr]))
            for k in ('rho_error_mm','rotation_error_deg')}
        results[direction]=dict(count=len(rr), checks=dict(source_join=checks[direction],
            max_installed_covariance_difference=max_c, max_frozen_mean_difference=max_mean,
            max_source_timestamp_rounding_ns=rounded_ns, all_slots_tracked=True,
            sampled_preinjection_checks=sample_n,max_pre_q_relative_error=max_q_error,
            max_covariance_times_logged_information_error=max_info_error),
            coverage=dict(receiver_from_ns=min(r['receiver_from_ns'] for r in rr),
                          receiver_to_ns=max(r['receiver_to_ns'] for r in rr)),
            summaries=summaries, alignment=extra, records=rr)
    assert all(g.sha(Path(p))==h for p,h in inputs.items())
    result=dict(sequence='eig2',run=str(ROOT),source_checks=source_checks,
                reference=dict(path=previous['reference']['path'],max_bracket_ns=reference.max_bracket_ns),
                directions=results,code_checks=code_checks,input_hashes=inputs,inputs_unchanged=True)
    a.write(H/'results.json',result)
    compact={**result,'directions':{k:{l:v for l,v in r.items() if l!='records'} for k,r in results.items()}}
    a.write(H/'summary.json',compact)
    print(json.dumps({k:r['checks'] for k,r in results.items()},indent=2))


if __name__ == '__main__':
    g.self_test(); main()
