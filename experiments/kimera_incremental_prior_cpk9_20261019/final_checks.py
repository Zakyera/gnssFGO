"""Read-only integrity checks plus exclusive evidence receipt, after owned jobs end."""
import hashlib
import json
from pathlib import Path
import subprocess

H=Path(__file__).resolve().parent
G=H.parents[1]
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
R=W/'runs/kimera_incremental_prior_cpk9_20261019'
BASE=W/'runs/kimera_failed_factor_cpk1_20261019'
def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
    return h.hexdigest()
def main():
    protected=json.loads((R/'protected_before.json').read_text())
    bad=[p for p,h in protected.items() if not Path(p).is_file() or sha(p)!=h]
    assert not bad,bad
    for name in ('OriginalChartPrior.h','VioBackend.cpp','CurrentChartFallback.h','RetirementAudit.h','LinearParityAudit.h','private_build/libcbs.so'):
        assert sha(R/name)==sha(BASE/name),name
    for name in ('NativePriorRefresh.h','test.cpp','build.py','PROTOCOL.txt'):
        assert sha(R/name)==sha(H/name),name
    for mode in ('release','san','backend'):
        assert json.loads((R/(mode+'_initial/passed.json')).read_text())['passed']
    assert (R/'release_initial/run0.log').read_bytes()==(R/'release_initial/run1.log').read_bytes()
    assert 'not found' not in (R/'release_initial/dependencies.log').read_text()
    assert 'not found' not in (R/'backend_initial/dependencies.log').read_text()
    checks=json.loads(subprocess.check_output(['python3',str(G/'experiments/next_experiment_settings_cpk8_20261019/verify_settings.py')],text=True))
    assert checks['failures']==[] and checks['launch_allowed'] is False
    states=subprocess.check_output(['docker','inspect','--format','{{.Name}} {{.State.Status}}',
                                   'cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
    assert all(x.endswith(' exited') for x in states),states
    metrics={}
    for row in (R/'release_initial/run0.log').read_text().splitlines():
        if '=' in row:
            k,v=row.split('=',1);metrics[k]=float(v)
    result={'passed':True,'protected_files':len(protected),'settings_hashes':checks['files_checked'],
            'full_run_admission':False,'dataset_replay':False,'containers':states,
            'release_repeat_identical':True,'focused_asan_ubsan_pass':True,
            'vendor_libraries_instrumented':False,'metrics':metrics,
            'candidate_dso_sha256':sha(R/'private_build/libkimera_vio.so'),
            'inputs_current':{str(p):sha(p) for p in sorted(R.rglob('*'))
                              if p.is_file() and p.suffix in ('.h','.cpp','.py')}}
    with (H/'validation.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='inputs_current'},indent=2))
if __name__=='__main__':main()
