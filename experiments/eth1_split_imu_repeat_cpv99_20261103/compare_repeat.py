"""Saved-only EVO comparison of two live pairs, on exactly the same timestamps."""
import json
from pathlib import Path
import numpy as np
import evaluate as q
from paths import H,BASE_H
d=q.d
def common_data(old,new):
    ts,i,j=np.intersect1d(old['timestamp_ns'],new['timestamp_ns'],return_indices=True)
    valid=old['common_valid'][i]&new['common_valid'][j]
    assert len(ts)>100 and np.all(np.diff(ts)==100_000_000)
    assert np.allclose(old['reference_position'][i][valid],new['reference_position'][j][valid],atol=1e-12,rtol=0)
    assert np.allclose(old['reference_rotation'][i][valid],new['reference_rotation'][j][valid],atol=1e-12,rtol=0)
    ids=np.flatnonzero(valid)
    assert len(ids)>100
    cuts=np.split(ids,np.flatnonzero(np.diff(ids)>1)+1)
    return ts,i,j,valid,cuts
def main():
    assert d.read(H/'report_verification.json')['passed']
    O=H/'repeat_evaluation';assert not O.exists();O.mkdir()
    for f in (Path(__file__),BASE_H/'results.json',H/'results.json'):
        d.pin(f)
    old=np.load(d.pin(BASE_H/'evaluation/comparison.npz'))
    new=np.load(d.pin(H/'evaluation/comparison.npz'))
    ts,i,j,valid,cuts=common_data(old,new)
    gp=new['reference_position'][j];gr=new['reference_rotation'][j]
    folder=O/'common';rp,rt,refs,pairs,info=q.reference_files(folder,ts,gp,gr,cuts)
    scores={}
    roles=('glim','kimera','fusion_online','fusion_delayed')
    for run,data,indices in [('original',old,i),('repeat',new,j)]:
        for mode in ('off','on'):
            for role in roles:
                key=mode+'_'+role;p=data[key+'_position'][indices];r=data[key+'_rotation'][indices]
                name=run+'_'+key
                scores[name]=d.score_series(folder,name,ts,p,r,gp,gr,cuts,refs,rp,rt)
    assert all(c['exit']==0 for c in d.commands)
    for x in scores.values():
        assert x['rows']==int(valid.sum())
        for kind in ('translation_m','rotation_deg'):
            z=x['RPE'][kind];archive=d.file_interface.load_res_file(z['archive'])
            assert archive.stats==z['statistics'] and len(archive.np_arrays['error_array'])==len(pairs)
    coverage=d.m.c.coverage(ts,valid,q.epoch,q.duration)
    out=dict(passed=True,task='CP-V99 repeatability',original=str(BASE_H),repeat=str(H),scores=scores,
             common_coverage=coverage,reference_RPE_pairs=len(pairs),segments=info,
             same_original_pair_mask=bool(np.array_equal(ts[valid],old['timestamp_ns'][old['common_valid']])),
             same_repeat_pair_mask=bool(np.array_equal(ts[valid],new['timestamp_ns'][new['common_valid']])),
             EVO_CLI_calls=len(d.commands),verification=d.checks,
             limitations='Two realizations only; not a variance estimate or proof of randomness/no bug. IE-TC shares CPT7. Delayed fixed-lag separate from ONLINE; paths not physical travel.')
    text=['CP-V99 versus CP95: identical-settings repeated live pair',
          f'Cross-repeat common100ms mask: {valid.sum()}rows; {coverage["supported_s"]:.3f}s; {len(pairs)}reference-distance0.5m pairs.',
          'Mode/estimator | original/repeat ATE m | tRPE m | rRPE deg | path m | reference m']
    for mode in ('off','on'):
        for role in roles:
            x=scores['original_'+mode+'_'+role];y=scores['repeat_'+mode+'_'+role]
            text.append('%s/%s | %.6f / %.6f | %.6f / %.6f | %.6f / %.6f | %.4f / %.4f | %.4f'%(mode,role,x['ATE_RMSE_m'],y['ATE_RMSE_m'],
                x['RPE']['translation_m']['statistics']['rmse'],y['RPE']['translation_m']['statistics']['rmse'],
                x['RPE']['rotation_deg']['statistics']['rmse'],y['RPE']['rotation_deg']['statistics']['rmse'],
                x['path_length_m'],y['path_length_m'],x['reference_path_length_m']))
    text.append(out['limitations'])
    d.save(O/'commands.json',d.commands);d.save(O/'input_sha256.json',d.PINS)
    assert all(d.sha(Path(p))==v for p,v in d.PINS.items())
    d.save(H/'repeat_comparison.json',out)
    with (H/'REPEAT_COMPARISON.txt').open('x') as f:f.write('\n'.join(text)+'\n')
    print('\n'.join(text),flush=True)
if __name__=='__main__':main()
