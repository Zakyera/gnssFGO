from pathlib import Path
import csv, hashlib, json
import numpy as np
from scipy.spatial.transform import Rotation
H=Path(__file__).resolve().parent
B=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/glim_incremental_publisher_cpv29_20261028')
source=H.parent/'all_incremental_comparison_cpv28_20261028/hesai_eth1_glim_products.npz'
x=np.load(source)
ts=x['off_timestamp_ns']
names=['graph_body_T','previous_same_update_body_T','chained_body_T']
arrays=[x['off_'+n].copy() for n in names]
# No previous state exists at initialization; the accumulator seeds with X0.
arrays[1][0]=arrays[0][0]
assert all(np.isfinite(a).all() for a in arrays)
out=B/'eth1_off_fixture.csv'
with out.open('x') as f:
 w=csv.writer(f);w.writerow(['index','stamp_ns']+[n+'_'+q for n in names for q in ['x','y','z','qx','qy','qz','qw']])
 for k,t in enumerate(ts):
  row=[k,int(t)]
  for a in arrays:row.extend([*a[k,:3,3],*Rotation.from_matrix(a[k,:3,:3]).as_quat()])
  w.writerow(row)
with (H/'fixture.json').open('x') as f:
 json.dump(dict(rows=len(ts),source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
               fixture=str(out),sha256=hashlib.sha256(out.read_bytes()).hexdigest()),f,indent=2)
print('Full ETH1 fixture:',len(ts))
