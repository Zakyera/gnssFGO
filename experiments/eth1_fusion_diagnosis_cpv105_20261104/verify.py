"""Independent diagnostics checks and receiver-code identity review; no replay."""
import copy
import datetime
import hashlib
import json
from pathlib import Path
import numpy as np

H=Path(__file__).resolve().parent;E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs')
Ps=[W/'eth1_split_imu_live_pair_cpv95_20261103_attempt3/prepared',
    W/'eth1_prior_fix_live_cpv103_20261103/prepared']
PINS={}
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def pin(p):p=Path(p);PINS[str(p)]=sha(p);return p
def read(p):return json.loads(pin(p).read_text())
def save(n,v):
    with (H/n).open('x') as f:json.dump(v,f,indent=2,allow_nan=False)

loc=read(H/'localization.json');bel=read(H/'belief_analysis.json')
arrays=np.load(pin(H/'belief_cohorts.npz'))
max_q=0.;count=0
for role,z in bel['sources'].items():
    for mode in ('OFF','ON'):
        for stage in ('first','age2.5','age4.5','terminal'):
            stem=role+'_'+mode+'_'+stage
            C=arrays[stem+'_covariance'];err=arrays[stem+'_residual']
            ev,V=np.linalg.eigh(C);assert ev.min()>0
            q=np.sum((np.einsum('nji,nj->ni',V,err)**2)/ev,axis=1)
            want=z['summary'][mode][stage]['full6_normalized_reference_discrepancy']
            delta=abs(float(np.median(q))-want['median'])/(1+want['median'])
            max_q=max(max_q,delta);assert delta<1e-8
            for name,sl in (('translation',slice(3,6)),('rotation',slice(0,3))):
                target=z['summary'][mode][stage][name]
                assert abs(np.trace(C[:,sl,sl],axis1=1,axis2=2).mean()/target['trace']['mean']-1)<1e-12
                assert abs(np.mean(np.sum(err[:,sl]**2,axis=1))/target['mean_squared_tangent_reference_discrepancy']-1)<1e-12
            count+=len(C)
    c=bel['cohorts'][role]['all_stage_same_edge_cohort']
    assert all(z['summary'][m][s]['n']==c for m in ('OFF','ON') for s in ('first','age2.5','age4.5','terminal'))
errors=np.load(pin(H/'localized_errors.npz'))
for key,value in loc['curves'].items():
    err=errors[key+'_error']
    assert abs(np.sqrt(np.mean(np.sum(err**2,axis=1)))-value['saved_ATE_RMSE_m'])<1e-8

bases=[p/('full_'+m+'_receiver') for p,m in zip(Ps,('off','on'))]
configs=[read(p/'resolved_config.json') for p in bases]
normalized=[]
for P,cfg in zip(Ps,configs):
    cc=copy.deepcopy(cfg)
    for k in ('sequence','ros_master_uri','receiver_binary'):cc.pop(k)
    for source in cc['sources']:
        for k in ('session','log_glob'):source.pop(k)
    normalized.append(cc)
assert normalized[0]==normalized[1]
assert pin(Ps[0]/'receiver_cpk19.sh').read_bytes()==pin(Ps[1]/'receiver_cpk19.sh').read_bytes()
manifests=[read(p/'manifest.json') for p in bases]
for base,m in zip(bases,manifests):
    for name,digest in m['transport_sha256'].items():
        assert sha(pin(base/'transport_code'/name))==digest
for name in manifests[0]['transport_sha256']:
    aa=(bases[0]/'transport_code'/name).read_text().replace(str(Ps[0]),'<PREPARED>')
    bb=(bases[1]/'transport_code'/name).read_text().replace(str(Ps[1]),'<PREPARED>')
    assert aa==bb,name
build=read(Path('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/build_live.json'))
source=E/'receiver_finalization_cpk19_20261021'
for name in ('Receiver.h','Live.cpp'):
    assert sha(pin(source/name))==build['sources'][name]
binary=Path('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/live')
assert sha(pin(binary))==build['binary_sha256']=='4b08a7f5ba97f5e4ec22959df23367df79eebf5d644dc19d939747dcd4566d6e'
for root,mode in ((E/'eth1_split_imu_live_pair_cpv95_20261103_continuation1','off'),
                  (E/'eth1_prior_fix_live_cpv103_20261103','on')):
    gate=read(root/'receiver_validation'/mode/'validation.json')
    assert gate['passed'] and gate['covariance_payload_error']==0
    assert gate['summary']['duplicate']==gate['summary']['stale']==0
    assert gate['summary']['unsupported_end']==gate['summary']['pending_at_end']==0
    assert read(root/('frame_'+mode+'.json'))['passed']
for group in (loc,bel):
    for path,digest in group['input_sha256'].items():assert sha(path)==digest,path
for path,digest in PINS.items():assert sha(path)==digest,path
save('verification.json',dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),
     matrices_verified=count,independent_eigen_whitening_relative_error=max_q,
     same_physical_edge_cohorts=True,existing_EVO_error_norms_reproduced=True,
     receiver_scientific_config_equal=True,wrapper_bytes_equal=True,
     transport_only_difference='Expected private wrapper absolute path; all adapter mathematics byte-identical',
     receiver_binary_sha256=build['binary_sha256'],compiled_receiver_source_hashes_pass=True,
     both_frame_payload_lifecycle_gates_pass=True,input_sha256=PINS,
     hypothesis='Mature Kimera covariance tightening without matching residual improvement, plus unmodeled dependence; causal intervention still required',
     causal_proof=False,no_source_or_receiver_replay=True))
print(json.dumps(dict(passed=True,matrices=count,eigen_whitening_relative_error=max_q,
                     receiver_scientific_config_equal=True,adapter_math_equal=True),indent=2))
