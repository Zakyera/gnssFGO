"""Score frozen/raw and mean-only aligned beliefs; never optimize a graph."""
import csv
import hashlib
import json
import struct
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from prepare import ROOT, AUDIT, ON, sha, write_json
sys.path.insert(0,str(AUDIT))
import analyze as prior_audit


def pose(values):
    x,y,z,w,qx,qy,qz=values
    result=np.eye(4)
    result[:3,:3]=Rotation.from_quat([qx,qy,qz,w]).as_matrix()
    result[:3,3]=[x,y,z]
    return result


def covariance_bytes(values):
    return struct.pack('<36d',*values)


def main():
    assert not (ROOT/'summary.json').exists()
    metadata=json.loads((ROOT/'metadata.json').read_text())
    results=np.loadtxt(ROOT/'aligned.txt')
    repeated=np.loadtxt(ROOT/'aligned_repeat.txt')
    assert np.array_equal(results,repeated)
    assert results.shape==(175,25)
    states, times, _, _=prior_audit.parse_snapshots(ON/'pilot_on_source/estimators.log')
    ref=np.load(AUDIT/'reference.npz'); t0=prior_audit.T0
    ts=ref['times']-t0
    slerp=Slerp(ts,Rotation.from_quat(ref['quaternions']))
    def reference(ns):
        t=ns*1e-9-t0
        k=np.searchsorted(ts,t)
        assert 0<k<len(ts) and ts[k]-ts[k-1]<.006
        alpha=(t-ts[k-1])/(ts[k]-ts[k-1])
        out=np.eye(4);out[:3,:3]=slerp([t]).as_matrix()[0]
        out[:3,3]=(1-alpha)*ref['positions'][k-1]+alpha*ref['positions'][k]
        return out
    exported=[];rows=[]
    inv=prior_audit.inverse;log=prior_audit.log_se3;exp=prior_audit.exp_se3
    def errors(transform,truth):
        return np.linalg.norm(transform[:3,3]-truth[:3,3]),np.degrees(np.linalg.norm(log(inv(truth)@transform)[:3]))
    for m,out in zip(metadata,results,strict=True):
        assert int(out[0])==m['ordinal']
        u,i,j=m['update'],m['receiver_i'],m['receiver_j']
        aligned,left,right=pose(out[4:11]),pose(out[11:18]),pose(out[18:25])
        raw=exp(np.asarray(m['original']['mu']))
        assert np.max(np.abs(aligned-left@raw@right))<1e-12
        rc,rd=reference(m['receiver_start_ns']),reference(m['receiver_end_ns'])
        ra,rb=reference(m['source_start_ns']),reference(m['source_end_ns'])
        truth=inv(rc)@rd
        before=inv(states[u-1][i])@states[u-1][j]
        record=dict(ordinal=m['ordinal'],update=u,time_s=times[u]-t0,
                    receiver_i=i,receiver_j=j,available_imu_samples=int(out[1]),
                    last_available_imu_ns=int(out[2]),prefix_max_difference=float(out[3]))
        for name,value,target in [('raw',raw,truth),('aligned',aligned,truth),
                                 ('native_pre',before,truth),('source_own',raw,inv(ra)@rb),
                                 ('left_bridge',left,inv(rc)@ra),('right_bridge',right,inv(rb)@rd)]:
            et,er=errors(value,target);record[name+'_translation_m']=float(et);record[name+'_rotation_deg']=float(er)
        covariance=np.array(m['original']['covariance']).reshape(6,6)
        for name,value in [('raw',raw),('aligned',aligned)]:
            residual=log(inv(value)@before)
            record[name+'_frozen_pre_rotation_residual_deg']=float(np.degrees(np.linalg.norm(residual[:3])))
            record[name+'_frozen_pre_rho_residual_m']=float(np.linalg.norm(residual[3:]))
            record[name+'_frozen_pre_whitened_squared']=float(residual@np.linalg.solve(covariance,residual))
        candidate=dict(m, aligned_mu=log(aligned).tolist(),
                       covariance=list(m['original']['covariance']),
                       note='Aligned to receiver interval; original source timestamps retained as provenance, not an on-wire relabel.')
        original_bytes=covariance_bytes(m['original']['covariance'])
        assert covariance_bytes(candidate['covariance'])==original_bytes
        assert hashlib.sha256(original_bytes).hexdigest()==m['covariance_sha256']
        rows.append(record);exported.append(candidate)
    def summarize(selected):
        values={key:prior_audit.summary([r[key] for r in selected])
                for key in rows[0] if key.endswith(('_translation_m','_rotation_deg'))}
        return dict(count=len(selected),statistics=values,
                    translation_improved=sum(r['aligned_translation_m']<r['raw_translation_m'] for r in selected),
                    rotation_improved=sum(r['aligned_rotation_deg']<r['raw_rotation_deg'] for r in selected))
    protected=json.loads((ROOT/'manifest.json').read_text())['inputs']
    unchanged={p:sha(Path(p))==h for p,h in protected.items()}
    assert all(unchanged.values())
    summary=dict(
        kind='Retained-data mean-only timing ablation. NOT a new native or fused trajectory.',
        reference='CPT7 IE-TC, shares native IMU; not independent prism GT.',
        bridge='Existing pure open-loop IMU Stage-3 history with fixed prior-pilot bootstrap; not native Kimera state reconstruction.',
        first_8s=summarize([r for r in rows if r['time_s']<=8]),
        all_175=summarize(rows),first_factor=rows[0],
        deterministic_byte_identical=sha(ROOT/'aligned.txt')==sha(ROOT/'aligned_repeat.txt'),
        all_175_covariance_payloads_bit_identical=True,
        maximum_prefix_difference=float(results[:,3].max()),
        protected_inputs_unchanged=unchanged,
        limitations=['Measurement covariance intentionally NOT transported; statistical consistency not claimed.',
                     'No new optimizer update; frozen-prior residual changes cannot establish changed corrections or ATE.',
                     'Pure-IMU bridge has open-loop initial velocity/bias sensitivity; do not deploy as native bridge without validation.',
                     'Conditional on retained ON wire means and prior snapshots; not a new closed-loop exchange.'])
    write_json(ROOT/'aligned_payloads.json',exported)
    # Reload serialized payload to test persisted doubles, not just memory copy.
    loaded=json.loads((ROOT/'aligned_payloads.json').read_text())
    assert all(covariance_bytes(a['covariance'])==covariance_bytes(b['original']['covariance'])
               for a,b in zip(loaded,metadata,strict=True))
    with (ROOT/'per_edge.csv').open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    write_json(ROOT/'summary.json',summary)
    print(json.dumps({k:v for k,v in summary.items() if k!='protected_inputs_unchanged'},indent=2))


if __name__=='__main__': main()
