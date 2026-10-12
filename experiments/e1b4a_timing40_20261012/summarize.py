"""Read-only timing comparison and small evidence archive after all run gates."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess
from analyze_profile import analyze, legacy_timers, summary

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD=WS/'runs/gnssfgo_e1b_profile40_mapguard_20261012'
RUN=WS/'runs/gnssfgo_e1b4a_timing40_20261012'
ROOT=RUN/'prepared'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for block in iter(lambda:f.read(1048576),b''):h.update(block)
    return h.hexdigest()
def read(p):return json.loads(p.read_text())
def write(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2);f.write('\n')
def extra(run):
    source=run/'prepared/profile40_on_source'
    data=[json.loads(l) for l in (source/'e1b_profile.jsonl').open()]
    log,=[p for p in (source/'ros_logs').glob('*/glim_rosnode-*-stdout.log') if p.parent.name!='latest']
    scans,_=legacy_timers(log)
    first=scans[1]['timestamp']
    out={}
    for key,start,end in [('early_0_10s',0,10),('middle_10_30s',10,30),('late_30_40s',30,40),('all',0,40)]:
        rows=[v for v in scans.values() if start<=v['timestamp']-first<end]
        out[key]={field:summary(v[field] for v in rows) for field in ['total_ms','main_ms','pre_ms']}
    out['context_root_ms']={str(c):summary(r['owner_wall']['root'][1]/1e6 for r in data if r['context']==c) for c in sorted({r['context'] for r in data})}
    return out
def main():
    validation=read(ROOT/'profile40_on_validation.json')
    assert validation['passed']
    assert all(v['byte_identical'] for v in validation['repeat_files'].values())
    manifest=read(ROOT/'protected_manifest.json')
    for p,h in manifest.items():assert sha(p)==h,p
    repos=read(HERE.parent/'e1b_local_profile_20261012/validation.json')['source_git_unchanged']
    for p,v in repos.items():
        assert subprocess.check_output(['git','-C',p,'rev-parse','HEAD'],text=True).strip()==v['head']
        assert not subprocess.check_output(['git','-C',p,'status','--porcelain'],text=True).strip()
    baseline=analyze(OLD); candidate=analyze(RUN)
    ratios={metric:candidate['all_local_root_ms'][metric]/baseline['all_local_root_ms'][metric] for metric in ['mean','median','p95','max']}
    b_extra=extra(OLD);c_extra=extra(RUN)
    info=dict(functional_pass=True,protected_hashes=len(manifest),source_repositories= repos,
       local_timing_gate_pass=all(ratios[k]<=.8 for k in ['median','p95']),
       first40s_scan_headroom_pass=candidate['legacy']['scan_total_ms']['p95']<800,
       local_candidate_over_baseline=ratios,
       local_total_speedup=baseline['local_root_wall_s']/candidate['local_root_wall_s'],
       baseline_run=str(OLD),candidate_run=str(RUN),
       baseline_controller=read(OLD/'prepared/profile40_on_control/complete.json'),
       candidate_controller=read(ROOT/'profile40_on_control/complete.json'),
       baseline_extra=b_extra,candidate_extra=c_extra,
       baseline=baseline,candidate=candidate,
       caveats=['Separate live captures: same configuration, not identical factor Values/interleavings.',
        'Timing is host elapsed, not isolated GPU kernel time. No GPU power/clock control.',
        'No independent ordinary reference followed every update in this timing capture.',
        'Functional validation and prior exact-input gates do not prove long-run throughput.',
        'No mathematical change or default promotion. MAIN/direct-loop scope is unchanged.'])
    write(HERE/'comparison.json',info)
    evidence=HERE/'evidence';evidence.mkdir(exist_ok=False)
    for name in ['profile40_on_validation.json','profile40_on_native_validation.json',
                 'profile40_on_lifecycle_validation.json','profile40_on_diagnostics_validation.json',
                 'protected_manifest.json','profile40_on.json']:
        shutil.copy2(ROOT/name,evidence/name)
    for name in ['preflight/validation.json','preflight/regression.xml',
                 'profile40_on_control/complete.json','profile40_on_source/effective_mode_check.json',
                 'profile40_on_source/source_coverage.json','profile40_on_receiver/transport_status.json']:
        dest=evidence/name;dest.parent.mkdir(exist_ok=True,parents=True);shutil.copy2(ROOT/name,dest)
    harness=evidence/'harness';harness.mkdir()
    for name in ['experiment.py','run_case.py','source.env','source_runner.sh','verify_effective.py',
                 'glim_profile.launch','receiver_e2.sh','map_guard.py']:
        shutil.copy2(ROOT/name,harness/name)
    write(evidence/'hashes.json',{str(p.relative_to(evidence)):sha(p) for p in sorted(evidence.rglob('*')) if p.is_file()})
    print(json.dumps({k:v for k,v in info.items() if k not in ['baseline','candidate','baseline_extra','candidate_extra','source_repositories']},indent=2))
if __name__=='__main__':main()
