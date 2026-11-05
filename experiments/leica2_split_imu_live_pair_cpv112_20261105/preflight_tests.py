"""Saved-data/schema/geometry/synthetic-EVO tests only; no source/receiver run."""
import hashlib,json,subprocess,sys
from pathlib import Path
import numpy as np
from paths import *
def read(p):return json.loads(p.read_text())
def save(name,v):
    with (H/name).open('x') as f:json.dump(v,f,indent=2,allow_nan=False)
from injection_schema import validate_injections
bad=[('off',{}),('off',{'total':0}),('off',False),('off',0.),('off','0'),('off',1),
 ('on',0),('on',{}),('on',{'G2K_queued':1,'G2K_persistent':1}),
 ('on',{'G2K_queued':1,'G2K_persistent':1,'K2G_persistent':0}),
 ('on',{'G2K_queued':1,'G2K_persistent':1,'K2G_persistent':True}),
 ('on',{'G2K_queued':1,'G2K_persistent':1,'K2G_persistent':'1'}),
 ('on',{'G2K_queued':2,'G2K_persistent':1,'K2G_persistent':1}),
 ('on',{'G2K_queued':1,'G2K_persistent':1,'K2G_persistent':1,'extra':1})]
for mode,val in bad:
    try:validate_injections(mode,val)
    except AssertionError:pass
    else:raise AssertionError((mode,val))
for mode,source in [('off',BASE_P/'full_off_source'),('on',QUAL_P/'full_on_source')]:
    validate_injections(mode,read(source/'source_coverage.json')['external_injections'])
sys.path.insert(0,str(P))
from eof_boundary import test_boundaries
assert test_boundaries()==dict(valid=1,rejected=4)
save('validator_test.json',dict(passed=True,negative_schemas=14,real_saved_schemas=2,eof_controls=5,
    inherited_numerical_gate=str(QUAL_H/'on_candidate_validation.json'),
    changes='LEICA2 raw duration/stereo/scan count bounds only; same corrected DSO/fatal/finite objective gates'))

host=[]
for cmd in (['python3','-B',str(P/'transport_tools/qualify_runtime.py')],
            ['python3','-B',str(P/'transport_tools/launch_grandtour_live_receiver.py'),'--help'],
            ['python3','-B',str(P/'run_case_on.py'),'--help']):
    r=subprocess.run(cmd,capture_output=True,text=True);assert r.returncode==0,(cmd,r.stderr)
    host.append(dict(args=cmd,exit=r.returncode,stdout=r.stdout,stderr=r.stderr))
save('host_runtime_preflight.json',dict(passed=True,checks=host,Noetic_fixture_still_required=True))

sys.path.insert(0,str(P/'transport_tools'))
from frame_adapter import adjoint,native_to_cpt7
# Same independent scipy geometry used by the prior live frame qualifications.
import importlib.util
spec=importlib.util.spec_from_file_location('geometry108',E/'eig2_hesai_onset_cpv50_20261030/analyze.py')
geo=importlib.util.module_from_spec(spec);spec.loader.exec_module(geo)
audit=read(P/'input_audit.json');X=np.asarray(audit['T_alphasense_cpt7']);Y=np.linalg.inv(X);J=np.array(adjoint(Y))
rng=np.random.default_rng(108);errors=[];jac=[]
for i in range(40):
    mu=rng.normal(size=6)*.3;Z=rng.normal(size=(6,6));C=Z@Z.T+np.eye(6)*.1
    b=dict(mean=mu.tolist(),covariance=C.reshape(-1).tolist(),i=i,j=i+1,ti=100,tj=200,source_revision=4)
    out=native_to_cpt7('kimera',b,X)
    expected=Y@geo.exp(mu)@X
    errors.extend([float(np.max(abs(geo.exp(out['mean'])-expected))),float(np.max(abs(np.array(out['covariance']).reshape(6,6)-J@C@J.T)))])
    assert all(out[k]==b[k] for k in b if k not in ('mean','covariance'))
    assert native_to_cpt7('glim',b,X)==b
    N=geo.exp(out['mean'])
    numerical=np.column_stack([geo.log(geo.inv(N)@(Y@(geo.exp(mu)@geo.exp(np.eye(6)[j]*1e-7))@X))/1e-7 for j in range(6)])
    jac.append(float(np.max(abs(numerical-J))))
assert max(errors)<1e-10 and max(jac)<1e-7
save('live_adapter_test.json',dict(passed=True,random_cases=40,geometry_covariance_error_max=max(errors),
    right_tangent_jacobian_error_max=max(jac),metadata_preserved=True,glim_identity=True,
    same_qualified_stdlib_adapter=True,exact_Noetic_fixture_mandatory_before_playback=True))

import evaluate as q
d=q.d;a=q.a
reference=q.load_reference()
grid=np.arange(q.epoch,q.epoch+428750000000,100000000,dtype=np.int64)
gp,gr,valid=a.sample(reference,grid,50000000)
assert valid.all() and len(reference[0])>30000
native=q.b.kimera(QUAL_P/'full_on_source','alphasense')
assert len(native[0])==1492 and np.isfinite(native[1]).all()
old=W/'runs/gnssfgo_leica2_full_cpk29_20261023/prepared/full_on_source'
oldglim=a.load_curve(old/'glim_odometry.csv')
assert a.sample(reference,oldglim[0],50000000)[2].all()
O=H/'report_contract_test';O.mkdir()
ts=q.epoch+np.arange(601,dtype=np.int64)*100000000
t=np.linspace(0,1,len(ts));xyz=np.column_stack([210*t,3*np.sin(6*t),2*np.cos(5*t)])
rot=np.tile(np.eye(3),(len(ts),1,1));cuts=[np.arange(0,300),np.arange(305,601)]
rp,rt,refs,pairs,info=q.reference_files(O/'fixture',ts,xyz,rot,cuts)
result=d.score_series(O/'fixture','synthetic_contract_only',ts,xyz,rot,xyz,rot,cuts,refs,rp,rt)
assert result['ATE_RMSE_m']<1e-8 and len(pairs)>100
for kind in ('translation_m','rotation_deg'):
    z=d.file_interface.load_res_file(result['RPE'][kind]['archive'])
    assert z.stats['rmse']<1e-8 and len(z.np_arrays['error_array'])==len(pairs)
assert all(c['exit']==0 for c in d.commands)
d.save(O/'commands.json',d.commands);d.PINS.update(a.HASHES);d.save(O/'input_sha256.json',d.PINS)
save('report_preflight.json',dict(passed=True,mission_reference_rows=len(reference[0]),full_reference_grid_rows=len(grid),
    reference_all_queries_supported=True,old_LEICA2_GLIM_reference_supported=True,CP103_Kimera_Method1_rows=len(native[0]),
    synthetic_EVO_CLI_calls=len(d.commands),checks=d.checks,new_scientific_results=False,source_runs=0,receiver_replays=0))
print('PASS schema, EOF, host import/fixture, mission frame/Jacobian, reference/parser and synthetic EVO tests')
