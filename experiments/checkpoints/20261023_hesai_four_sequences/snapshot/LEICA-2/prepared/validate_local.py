"""Additional provenance gate; does not replace frozen capture validators."""
from pathlib import Path
import collections,csv,json,re,sys
import numpy as np
P=Path(__file__).resolve().parent
sys.path.insert(0,str(P/'transport_tools'))
from live_receiver_protocol import parse_belief
mode=sys.argv[1];assert mode in ('off','on')
S=P/f'full_{mode}_source';R=P/f'full_{mode}_receiver'
assert (P/f'full_{mode}_control/complete.json').exists()
assert not (P/f'full_{mode}_control/failed.txt').exists()
effective=json.loads((S/'effective_mode_check.json').read_text())
assert effective['glim_revision_covariance']=='native_local'
expected={};result={}
for kind,source,glob in [('glim','glim_hesai','*/glim_rosnode-*-stdout.log'),('kimera','kimera_front','*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log')]:
 paths=list((S/'ros_logs').glob(glob));assert len(paths)==1,paths
 provider=None;pending=None;first=revisions=queries=0;tmp=0;provenance=set()
 histories=collections.defaultdict(list)
 payload_log=paths[0] if kind=='glim' else S/'estimators.log'
 if kind=='kimera':
  for telemetry in paths[0].open():
   if 'CBS_JOINT_MARGINAL_INFORMATION_TIMING_ROW,k,' in telemetry:
    fields=telemetry.split('CBS_JOINT_MARGINAL_INFORMATION_TIMING_ROW,k,',1)[1].split(',')
    assert fields[0]=='tmp_marginals',('MAIN fallback',telemetry);tmp+=1
 for raw in payload_log.open():
  line=re.sub(r'\x1b\[[0-9;]*m','',raw).strip()
  if 'SHARED_NATIVE_PROVIDER_FAILURE' in line:raise AssertionError(line)
  if 'SHARED_NATIVE_PROVIDER_QUERY ' in line:
   f=dict(x.split('=',1) for x in line.split('SHARED_NATIVE_PROVIDER_QUERY ',1)[1].split())
   assert int(f['resets'])==0
   assert int(f['admitted'])==sum(int(f[x]) for x in ('active','absorbed','removed'))
   provider=(int(f['from'])&((1<<56)-1),int(f['to'])&((1<<56)-1),np.array([[float(v) for v in row.split(':')] for row in f['matrix'].split(';')]).reshape(-1));queries+=1
  if 'GLIM_CBS_REVISION_COVARIANCE_PROVENANCE_ROW,' in line:
   f=line.split('GLIM_CBS_REVISION_COVARIANCE_PROVENANCE_ROW,',1)[1].split(',')
   assert len(f)==5 and f[-1]=='native_local';pending=tuple(map(int,f[:4]));assert pending not in provenance;provenance.add(pending)
  b=parse_belief(kind,line)
  if b is None:continue
  if kind=='glim':
   if b['source_revision']:
    assert pending==(b['i'],b['j'],b['source_revision'],int(b['source_update']));revisions+=1;pending=None
   else:
    assert provider is not None and provider[:2]==(b['i'],b['j'])
    assert np.array_equal(provider[2],b['covariance']), 'first provider/export matrix changed'
    first+=1
  payload=tuple([b['ti'],b['tj'],*b['mean'],*b['covariance']])
  values=histories[b['i'],b['j']]
  if not values or values[-1]!=payload:values.append(payload)
 for (i,j),values in histories.items():expected[source,str(i),str(j)]=values
 if kind=='glim':assert first>=4267 and revisions>0 and len(provenance)==revisions and pending is None
 else:assert tmp>0
 result[kind]=dict(ids=len(histories),versions=sum(map(len,histories.values())),first=first,revised=revisions,provider_queries=queries,local_joint_queries=tmp)
actual=collections.defaultdict(list)
with (R/'events.csv').open() as f:
 for b in csv.reader(f):
  if b[0]=='B':actual[b[2],b[6],b[7]].append(tuple([int(b[8]),int(b[9]),*map(float,b[10:])]))
assert expected==dict(actual),'source->receiver payload/order conservation'
report=dict(passed=True,mode=mode,sources=result,all_payloads_exact=True,
 revision_provenance='per-version native_local row plus pinned/tested provider route; no separate repeated matrix dump',
 limits='native explicit information at corrected charts; not independent/calibrated likelihood')
with (S/'local_provenance.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps(report,indent=2))
