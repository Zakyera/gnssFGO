"""ARC-3 input-aware normalized scan coverage. No optimizer/model changes."""
import json
from pathlib import Path
import numpy as np


def check_available(t, audit):
    t = np.asarray(t, float)
    raw = np.asarray([c['normalized_stamp_s'] for c in audit['clouds']])
    assert np.isfinite(t).all() and len(t)>10 and np.all(np.diff(t)>0)
    j = np.searchsorted(raw, t)
    hi = np.clip(j, 0, len(raw)-1); lo = np.maximum(hi-1, 0)
    idx = np.where(abs(raw[hi]-t)<abs(raw[lo]-t), hi, lo)
    error = abs(raw[idx]-t)
    assert error.max()<=5e-6, ('output not an audited scan',float(error.max()))
    assert np.all(np.diff(idx)==1), ('unexpected skipped/duplicate available scans',idx[np.flatnonzero(np.diff(idx)!=1)].tolist())
    assert 0<idx[0]<=20 and idx[-1]>=len(raw)-3, ('startup/tail loss',int(idx[0]),int(idx[-1]))
    assert all(audit['clouds'][i]['captured_imu_bracketed'] for i in idx)
    return dict(passed=True,available_raw_scans=len(raw),published_scans=len(t),
        first_raw_index=int(idx[0]),last_raw_index=int(idx[-1]),
        unexpected_interior_scan_losses=0,max_timestamp_error_s=float(error.max()),
        recorded_gaps_over_1s=sum(g['gap_s']>1 for g in audit['known_raw_gaps']),
        contract='Every available scan between first and last publication, no extra interior loss; raw gaps retained')


def check_from_prepared(t):
    return check_available(t,json.loads((Path(__file__).parent/'input_audit.json').read_text()))


def self_test():
    raw=np.r_[np.arange(30)*.1,4+np.arange(30)*.1]
    audit=dict(clouds=[dict(normalized_stamp_s=float(t),captured_imu_bracketed=True) for t in raw],known_raw_gaps=[dict(gap_s=1.1)])
    good=raw[2:];check_available(good,audit)
    bad=[np.delete(good,15),np.r_[good[:15],good[14],good[15:]],good+.01,
         np.r_[good[:-1],np.nan],good[:-4]]
    for value in bad:
        try:check_available(value,audit)
        except AssertionError:pass
        else:raise AssertionError('negative test accepted')
    return dict(passed=True,raw_gap_accepted=1,injected_losses_duplicates_shift_nonfinite_tail_rejected=5)


if __name__=='__main__':print(json.dumps(self_test()))
