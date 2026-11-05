"""Quantify the predetermined input changes; no optimization or outcome selection."""
import csv,importlib.util,sys
import numpy as np
from common import *
sys.dont_write_bytecode=True
assert not (H/'intervention_audit.json').exists()
spec=importlib.util.spec_from_file_location('g',E/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py');g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g)
anchors=read(H/'anchors.json');epoch=read(P/'input_audit.json')['epoch_ns'];rows=[]
with (R/'events.csv').open() as f:
    for row in csv.reader(f):
        if row[0]!='B' or row[2]!='kimera_front':continue
        key='|'.join([row[2],row[3],row[6],row[7]]);a=anchors.get(key)
        if not a or int(row[1])-int(row[9])<=2_500_000_000:continue
        old=np.array(row[16:],float).reshape(6,6);held=np.array(a['input_covariance'],float).reshape(6,6)
        assert np.linalg.eigvalsh(held).min()>0
        e=g.log(g.inv(g.exp(np.array(row[10:16],float)))@g.exp(np.array(a['input_mean'],float)))
        rows.append([(int(row[9])-epoch)*1e-9,np.trace(held[3:,3:])/np.trace(old[3:,3:]),np.linalg.norm(e[3:]),np.linalg.norm(e[:3])*180/np.pi])
rows=np.array(rows);assert len(rows)==3393
def stats(x):return dict(mean=float(np.mean(x)),median=float(np.median(x)),p95=float(np.percentile(x,95)),maximum=float(np.max(x)),minimum=float(np.min(x)))
out=[]
for lo,hi in WINDOWS:
    x=rows[(rows[:,0]>=lo)&(rows[:,0]<=hi)]
    out.append(dict(start_s=lo,end_s=hi,late_rows=len(x),held_over_original_translation_trace=stats(x[:,1]),fraction_trace_increases=float(np.mean(x[:,1]>1)),held_vs_original_mean_translation_m=stats(x[:,2]),held_vs_original_mean_rotation_deg=stats(x[:,3])))
check(read(H/'input_sha256.json'));save('intervention_audit.json',dict(passed=True,windows=out,changed_rows=3393,no_scoring_or_selection=True))
print(json.dumps(out,indent=2))
