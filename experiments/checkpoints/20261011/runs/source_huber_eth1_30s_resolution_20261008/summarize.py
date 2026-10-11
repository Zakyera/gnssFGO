"""Collect existing evaluator outputs and preserve their units/masks explicitly."""
import csv
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parent
CAPTURE=ROOT.parent/'source_huber_eth1_30s_opencv42_20261008'

def read(p):return json.loads(p.read_text())

def main():
    score=ROOT/'matched30_common_evaluation'
    result=dict(common_mask=read(score/'validated_mask.json'),
      raw_interval=read(ROOT/'matched30_evaluation/common_interval.json'),
      cases={},GT_conventions=dict(ATE='AP20 prism position; CPT7-to-prism lever arm; SE3 Kabsch no scale',
        RPE='1 s relative SE3 vs IE-TC in prism frame; no global alignment',
        priors='GT is not used in estimation'))
    for mode in ['off_gaussian','on_gaussian','on_huber']:
        metrics={}
        for method in ['native_glim','native_kimera','fused_online','fused_final']:
            label=mode+'_'+method
            a=read(score/(label+'_prism.json'));b=read(score/(label+'_rpe.json'))
            raw=np.loadtxt(ROOT/'matched30_evaluation'/(label+'.tum'))
            matched=np.loadtxt(score/(label+'.tum'))
            metrics[method]=dict(ATE_rmse_m=a['ape_rmse_m'],t_RPE_1s_rmse_m=b['translation_error_m']['rmse'],
              r_RPE_1s_rmse_deg=b['rotation_error_deg']['rmse'],
              full_raw_body_path_m=float(np.linalg.norm(np.diff(raw[:,1:4],axis=0),axis=1).sum()),
              common_grid_body_path_m=float(np.linalg.norm(np.diff(matched[:,1:4],axis=0),axis=1).sum()),
              common_grid_prism_path_m=a['estimate_full_path_m'],
              GT_overlap_prism_estimate_path_m=a['matched_estimate_path_m'],
              GT_overlap_prism_reference_path_m=a['matched_reference_path_m'],
              GT_coverage_s=a['matched_elapsed_coverage_s'],ATE_poses=a['matched_poses'],RPE_pairs=b['valid_pairs'],
              raw_rows=len(raw),raw_start=float(raw[0,0]),raw_end=float(raw[-1,0]),
              raw_trajectory_sha256=hashlib.sha256((ROOT/'matched30_evaluation'/(label+'.tum')).read_bytes()).hexdigest())
        v=read(ROOT/(mode+'_validation.json'))
        stats=read(CAPTURE/(mode+'_receiver/transport_status.json'))
        result['cases'][mode]=dict(metrics=metrics,versions=v['versions'],replacements=v['replacements'],
          active_source_ids=v['active_source_ids'],imu_samples=v['imu_samples'],unsupported=len(v['unsupported']),
          max_publication_gradient=v['max_publication_gradient'],live_output_age_s=v['live_output_age_s'],
          transport=stats,original_exact_repeat_passed=v['original_byte_gate_passed'])
    a=result['cases']['on_gaussian']['metrics'];b=result['cases']['on_huber']['metrics']
    result['huber_change_vs_on_gaussian_percent']={method:{k:100*(b[method][k]/a[method][k]-1)
      for k in ['ATE_rmse_m','t_RPE_1s_rmse_m','r_RPE_1s_rmse_deg']} for method in a}
    # Explain the common-mask exclusion using actual native sampling, not GT.
    result['native_kimera_initial_times_and_gaps']={}
    for mode in result['cases']:
        ts=np.loadtxt(ROOT/'matched30_evaluation'/(mode+'_native_kimera.tum'))[:,0]
        result['native_kimera_initial_times_and_gaps'][mode]=dict(first_timestamps=list(ts[:5]),first_deltas_s=list(np.diff(ts[:5])))
    with (ROOT/'results.json').open('x') as f:json.dump(result,f,indent=2)
    print(json.dumps(result['huber_change_vs_on_gaussian_percent'],indent=2))

if __name__=='__main__':main()
