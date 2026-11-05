"""Run unchanged numerical/source gates on real CP89/91 and reject bad schemas."""
import hashlib,json,subprocess
from paths import H,E,W,P,PY
from injection_schema import validate_injections
T=H/'validator_tests';T.mkdir(exist_ok=True)
assert not any(T.iterdir()),'Do not overwrite test evidence'
bad=[('off',{}),('off',{'total':0}),('off',False),('off',0.),('off','0'),('off',1),
     ('on',0),('on',{}),('on',{'G2K_queued':1,'G2K_persistent':1}),
     ('on',{'G2K_queued':1,'G2K_persistent':1,'K2G_persistent':0}),
     ('on',{'G2K_queued':1,'G2K_persistent':1,'K2G_persistent':True}),
     ('on',{'G2K_queued':1,'G2K_persistent':1,'K2G_persistent':'1'}),
     ('on',{'G2K_queued':2,'G2K_persistent':1,'K2G_persistent':1}),
     ('on',{'G2K_queued':1,'G2K_persistent':1,'K2G_persistent':1,'extra':1})]
for mode,value in bad:
    try:validate_injections(mode,value)
    except AssertionError:pass
    else:raise AssertionError(('malformed schema admitted',mode,value))
trials=[]
for mode,name in [('off','eth1_alpha_bias_rw_full_off_cpv89_20261102'),
                  ('on','eth1_split_imu_persistent_on_cpv91_20261102_attempt1')]:
    root=W/'runs'/name/'prepared';source=root/f'{mode}_source'
    profile=root
    assert profile.is_dir(),profile
    command=[PY,'-B',str(H/'validate_source.py'),mode,'--source',str(source),
             '--profile-root',str(profile),'--output',str(T/(mode+'.json'))]
    with (T/(mode+'.log')).open('x') as log:
        result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT)
    assert result.returncode==0,(mode,'see validation log')
    trials.append(dict(mode=mode,source=str(source),command=command,exit=result.returncode,
                       coverage_sha256=hashlib.sha256((source/'source_coverage.json').read_bytes()).hexdigest()))
with (H/'validator_test.json').open('x') as f:
    json.dump(dict(passed=True,real_saved_trials=trials,negative_schema_tests=len(bad),
                   all_original_numerical_checks_retained=True,source_runs=0),f,indent=2)
print('PASS: both real saved schemas and full source gates; 14 malformed/invalid cases rejected')
