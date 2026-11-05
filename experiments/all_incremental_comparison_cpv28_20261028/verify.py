"""Read-back and independent product checks for the complete METHOD2 table."""
import json,sys
from pathlib import Path
sys.dont_write_bytecode=True
import numpy as np
import analyze as m

H=Path(__file__).resolve().parent
assert not (H/'verification.json').exists()
pins={};cases={};readbacks={};total=0
for row in m.c.b.ROWS:
    name=row[0];d=json.loads((H/(name+'.json')).read_text());assert d['method']==2 and d['original_receiver_Qc_multiplier']==1
    z=np.load(H/(name+'.npz'));old=np.load(m.H10/(name+'.npz'))
    assert np.array_equal(z['timestamp_ns'],old['timestamp_ns']) and np.array_equal(z['common_valid'],old['common_valid'])
    for p,v in d['input_sha256'].items():
        if p in pins:assert pins[p]==v
        pins[p]=v
    for mode in ('off','on'):
        for role in ('glim','fusion'):
            curve=m.a.load_curve(Path(d['trajectories'][mode+'_'+role]))
            if name=='velodyne_leica2':
                root=m.H26 if role=='glim' else m.H23
                raw=np.load(root/'publication_products.npz')
            else:raw=np.load(H/(name+'_'+role+'_products.npz'))
            t=raw[mode+'_timestamp_ns'];o=raw[mode+'_chained_body_T']
            assert np.array_equal(t,curve[0])
            err=max(float(np.max(abs(o[:,:3,3]-curve[1]))),float(np.max(abs(o[:,:3,:3]-curve[2]))))
            assert err<1e-9
            p,r,ok=m.a.sample(curve,z['timestamp_ns'],500000000);p,r=m.a.prism(p,r)
            # TUM serialization normalizes the quaternion representation of a
            # long matrix-product chain; require numerical rather than bitwise
            # equality for that representation roundtrip, exact timestamps.
            dp=float(np.nanmax(abs(p-z[mode+'_'+role+'_position'])))
            dr=float(np.nanmax(abs(r-z[mode+'_'+role+'_rotation'])))
            assert np.allclose(p,z[mode+'_'+role+'_position'],atol=1e-9,rtol=0,equal_nan=True)
            assert np.allclose(r,z[mode+'_'+role+'_rotation'],atol=1e-9,rtol=0,equal_nan=True)
            readbacks[name+'_'+mode+'_'+role]=dict(rows=len(t),pose_roundtrip_max=err,
                common_position_roundtrip_max_m=dp,common_rotation_roundtrip_max=dr,timestamps_exact=True)
            total+=len(t)
    cases[name]=d
# Reused files are checked against their original completed closeout, not just
# compared with scores that could hide an orientation-only difference.
prior=json.loads((m.H26/'closeout.json').read_text())
for p,v in prior['output_sha256'].items():assert m.sha(Path(p))==v
for p,v in pins.items():assert m.sha(Path(p))==v,p
cross={}
for seq in ('spx3','eig2','leica2'):
    h=Path(cases['hesai_'+seq]['trajectories']['off_kimera'])
    v=Path(cases['velodyne_'+seq]['trajectories']['off_kimera'])
    cross[seq]=m.sha(h)==m.sha(v);assert cross[seq]
replay=json.loads((H/'replay_complete.json').read_text());assert replay['pass_'] and len(replay['records'])==12
for rec in replay['records']:
    name=rec['case']+'_'+rec['mode']
    assert m.sha(H/'replay_evidence'/(name+'.tar.gz'))==rec['archive']['sha256']
    assert m.sha(H/'replay_evidence'/(name+'_publications.csv'))==rec['publication_sha256']
    assert json.loads((H/(name+'_parity.json')).read_text())['pass_']
out=dict(passed=True,method=2,cases=len(cases),scored_products=42,replayed_receivers=12,reused_receivers=2,
         recovered_glim_and_fusion_poses_read_back=total,readbacks=readbacks,
         verified_host_inputs=len(pins),input_sha256=pins,all_input_hashes_unchanged=True,
         Kimera_OFF_same_data_between_sensors=cross,
         evo_alignment_max_difference_m=max(x['evo_alignment_difference_m'] for d in cases.values() for block in ('common','full_saved') for x in d[block].values()),
         matrix_RPE_max_difference=max(x['direct_RPE_check_max'] for d in cases.values() for x in d['common'].values()),
         synthetic_checks=m.base.synthetic_checks())
m.a.save(H/'verification.json',out)
print(json.dumps({k:v for k,v in out.items() if k not in ('input_sha256','readbacks')},indent=2))
