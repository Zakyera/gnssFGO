"""Saved-data input-only preparation; no optimizer, source or container launch."""
import collections,csv,importlib.util,os,shutil,subprocess,sys
import numpy as np
from common import *
sys.dont_write_bytecode=True
assert not (H/'prepared.json').exists() and not (H/'variant_events.csv').exists()
cfg=read(R/'resolved_config.json');pins={}
for root,count in [('eth1_prior_fix_live_cpv103_20261103',62),
                   ('eth1_prior_fix_evaluation_cpv104_20261104',318),
                   ('eth1_fusion_diagnosis_cpv105_20261104',12)]:
    rec=read(E/root/'complete.json');assert rec['passed']
    assert check(rec['output_sha256'])==count
    pins[str(E/root/'complete.json')]=sha(E/root/'complete.json')
assert check(read(P/'protected_manifest.json'))==8120
assert sha(B)=='4b08a7f5ba97f5e4ec22959df23367df79eebf5d644dc19d939747dcd4566d6e'
assert cfg['receiver_fixed_lag_seconds']==7 and cfg['receiver_model']=='source_wnoa'
assert cfg['imu_frame']=='cpt7_imu'
assert all(s['belief_body_frame']=='cpt7_imu' for s in cfg['sources'])
assert shutil.disk_usage(H).free>15*2**30
env=dict(os.environ,**cfg['receiver_environment'])
link=subprocess.run(['ldd',str(B)],env=env,text=True,capture_output=True,check=True)
with (H/'host_ldd.txt').open('x') as f:f.write(link.stdout)
assert 'libboost_serialization.so.1.74.0 => not found' in link.stdout
save('host_admission_rejection.json',dict(receiver_started=False,container_started=False,
    cause='Missing host Boost1.74; original receiver ran in existing Humble container',
    resolution='Use original container runtime; no installation/build/settings change'))
for p in [B,P/'receiver_cpk19.sh',R/'resolved_config.json',R/'events.csv',P/'input_audit.json',
          E/'receiver_finalization_cpk19_20261021/Receiver.h',
          E/'receiver_finalization_cpk19_20261021/Live.cpp',
          E/'receiver_endpoint_reuse_e2b_20261013/SourceWnoaModel.h']:
    pins[str(p)]=sha(p)
first={};insertions=collections.Counter()
commits=sorted((R/'receiver/archive/commits').iterdir(),key=lambda p:int(p.name))
for directory in commits:
    with (directory/'records.txt').open() as f:
        for row in csv.reader(f):
            if row[:2]!=['belief','insert']:continue
            assert len(row)==55,len(row)
            insertions[row[2]]+=1
            if row[2]!='kimera_front':continue
            key=tuple(row[2:6]);assert key not in first
            first[key]=dict(revision=int(row[6]),ordinal=int(row[7]),start=row[8],end=row[9],
                           arrival=row[10],insertion=row[11],covariance=row[19:],commit=directory.name)
assert sum(insertions.values())==4924 and len(first)==1472,insertions
# Right-local tangent convention test: fixed coordinates in CPT7 body basis,
# not an adjoint conversion between sensors or transport of an old posterior.
geometry=E/'fixed_lag_eig2_g2k_reference_fl4y_20261017/analyze.py'
spec=importlib.util.spec_from_file_location('cp106_geometry',geometry)
g=importlib.util.module_from_spec(spec);spec.loader.exec_module(g);pins[str(geometry)]=sha(geometry)
max_retract=0.;mineig=float('inf');maxsym=0.
for i,item in enumerate(first.values()):
    C=np.array(item['covariance'],float).reshape(6,6)
    mineig=min(mineig,float(np.linalg.eigvalsh(C).min()));maxsym=max(maxsym,float(abs(C-C.T).max()))
    assert np.count_nonzero(C-np.diag(np.diag(C)))>0
    if i<20:
        Z=g.exp(np.array([.07,-.03,.02,.1,.2,-.15]))
        Znew=g.exp(np.array([.09,-.02,.01,.12,.18,-.13]))
        for k in range(6):
            eps=np.linalg.cholesky(C)[:,k]
            for center in (Z,Znew):
                max_retract=max(max_retract,float(abs(g.log(g.inv(center)@(center@g.exp(eps)))-eps).max()))
assert mineig>0 and maxsym<1e-12 and max_retract<1e-10
counts=collections.Counter();matched=set();changed=0;modified_ids=set();previous=-1
out=H/'variant_events.csv'
with (R/'events.csv').open() as src,out.open('x') as dst:
    for line in src:
        row=line.rstrip('\n').split(',');counts[row[0]]+=1
        assert int(row[1])>=previous;previous=int(row[1])
        replacement=line
        if row[0]=='B':
            assert len(row)==52
            counts[row[2]]+=1
            key=(row[2],row[3],row[6],row[7]);item=first.get(key)
            if item and int(row[5])>=item['revision']:
                assert row[8:10]==[item['start'],item['end']]
                if int(row[5])==item['revision']:
                    assert int(row[4])==item['ordinal'] and row[1]==item['arrival']
                    assert np.array_equal(np.array(row[16:],float),np.array(item['covariance'],float))
                    matched.add(key)
                    # Preserve original input numeric tokens, not commit formatting.
                    item['input_covariance_tokens']=row[16:]
                assert 'input_covariance_tokens' in item
                replacement=','.join(row[:16]+item['input_covariance_tokens'])+'\n'
                if replacement!=line:changed+=1;modified_ids.add(key)
        dst.write(replacement)
assert len(matched)==len(first) and counts['E']==1 and sum(counts[k] for k in ('I','B','E'))==237614
assert counts['B']==202995 and counts['I']==34618 and changed>30000
with (R/'events.csv').open() as src,out.open() as dst:
    compared=0
    for a,b in zip(src,dst,strict=True):
        aa=a.rstrip('\n').split(',');bb=b.rstrip('\n').split(',');compared+=1
        if a!=b:
            assert aa[:16]==bb[:16] and aa[0]=='B' and aa[2]=='kimera_front'
            key=(aa[2],aa[3],aa[6],aa[7]);assert bb[16:]==first[key]['input_covariance_tokens']
        else:assert aa==bb
oldtree=tree(R/'receiver/archive')
save('original_archive_sha256.json',oldtree)
save('first_accepted_covariances.json',{'|'.join(k):v for k,v in first.items()})
save('input_sha256.json',pins)
save('prepared.json',dict(passed=True,created=now(),events=237614,counts=dict(counts),
    initial_covariances=len(first),changed_covariance_rows=changed,changed_ids=len(modified_ids),
    original_archive_files=len(oldtree),original_archive_digest=tree_digest(oldtree),
    original_events_sha256=sha(R/'events.csv'),variant_events_sha256=sha(out),
    exact_preserved_non_covariance_tokens=True,full_6x6_cross_terms_preserved=True,
    minimum_initial_covariance_eigenvalue=mineig,maximum_asymmetry=maxsym,
    right_local_retraction_error=max_retract,free_bytes=shutil.disk_usage(H).free,
    receiver_binary_sha256=sha(B),baseline_required_before_variant=True,
    altered_source_means=False,source_runs=0))
print(json.dumps(read(H/'prepared.json'),indent=2))
