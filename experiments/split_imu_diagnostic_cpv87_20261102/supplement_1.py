"""Read-only raw-range/onset supplement; no estimator or trajectory scoring."""
import json
from pathlib import Path
import numpy as np
from analyze import H, P85, P86, pin, load, raw_imu, save, PINS, sha

assert not (H/'supplement_1.json').exists()
audit = load(P85/'input_audit.json')
epoch = audit['epoch_ns']
events = load(H/'revision_events.json')
A = raw_imu(audit['bag_files']['alphasense_imu']['path'], epoch)
C = raw_imu(audit['bag_files']['cpt7_imu']['path'], epoch)
out = {'scope': 'saved first 60s, no calibration application or new scoring'}
# Test a candidate signed-16-bit +/-4g encoding with g=9.8. This is an inference
# from released numbers, not a retrieved hardware register/firmware setting.
unit = 39.2 / 32768
counts = A[:,4:] / unit
rounderr = np.abs(counts - np.rint(counts))
edge = np.any(np.isclose(counts, -32768, atol=1e-7, rtol=0) |
              np.isclose(counts, 32767, atol=1e-7, rtol=0), axis=1)
out['candidate_4g_encoding'] = {
    'quantum_m_s2': unit,
    'maximum_fractional_count_residual': float(rounderr.max()),
    'fraction_within_1e_7_of_integer': float((rounderr < 1e-7).mean()),
    'component_min_counts': np.rint(counts.min(axis=0)).astype(int).tolist(),
    'component_max_counts': np.rint(counts.max(axis=0)).astype(int).tolist(),
    'full_scale_samples': [{'time_s': float(a[0]), 'acc_m_s2': a[4:].tolist()}
                           for a in A[edge]],
    'interpretation': 'consistent with 4g signed16 quantization; actual firmware/range not independently recovered'
}
out['largest_imu_gaps'] = {}
for name, data in [('Alphasense', A), ('CPT7', C)]:
    gap = np.diff(data[:,0]); ids = np.argsort(gap)[-5:]
    out['largest_imu_gaps'][name] = [dict(start_s=float(data[i,0]),
        end_s=float(data[i+1,0]), duration_s=float(gap[i])) for i in ids]
out['around_raw_acceleration_events'] = []
for time in (15.750043353, 18.7922292, 20.655503537, 51.635929086):
    ea = [r for r in events['Alphasense'] if time-.5 <= r['time_s'] <= time+.6]
    ec = [r for r in events['CPT7'] if time-.5 <= r['time_s'] <= time+.6]
    out['around_raw_acceleration_events'].append(dict(raw_event_s=time, Alpha=ea, CPT7=ec))
out['first_postsolve_imu_cost_crossings'] = {}
for name, p in [('Alphasense',P85),('CPT7',P86)]:
    cost=np.loadtxt(pin(p/'off_source/objective.csv'), delimiter=',')
    # Reuse event timestamps to avoid the default-precision objective timestamp.
    rows=events[name]
    out['first_postsolve_imu_cost_crossings'][name]={str(th):next((r for r in rows
        if r['postsolve_imu_cost'] > th),None) for th in (10,100,1000,10000)}
for f in ['RetirementAudit.h','include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h']:
    pin(Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/persistent_compatibility_cpv76_20261101')/f)
pin(Path(__file__))
assert all(sha(p)==s for p,s in PINS.items())
save(H/'supplement_1_inputs.json',PINS)
save(H/'supplement_1.json',out)
print(json.dumps({k:v for k,v in out.items() if k!='around_raw_acceleration_events'},indent=2))
