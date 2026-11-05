"""Second explicit batch: old non-GrandTour visualizations and debugging outputs."""
from pathlib import Path
import json, os, sys, stat, subprocess
import cleanup as c
H=c.H

def make_plan():
    dirs=[]; single=[]
    for p in sorted((c.W/'runs/kimera_bisect').rglob('manifest.json')):
        tag=c.classify(json.loads(p.read_text()))
        if tag: dirs.append((p.parent,tag,str(p)))
    dirs.extend([
      (c.W/'runs/comparisons/kimera_original_ocv42_vs_cbs_on_20260608','S3E','reviewed comparison source paths'),
      (c.W/'runs/comparisons/s3e_alpha_cbs_off_vs_on_video_20260608','S3E','explicit S3E comparison'),
      (c.W/'runs/live_debug_logs','S3E','every log explicitly opens S3E Square 1'),
      (c.V/'rerun_recordings/m3dgr','M3DGR','dataset-specific recordings directory')])
    nc=c.W/'src/cbsms/benchmark_results/newer_college_official_benchmark'
    for n in ('aggregate','pilots','sequences'):dirs.append((nc/n,'Newer College','official benchmark result slots; README reviewed'))
    for n in ('cov_debug_latest.rrd','cov_ellipsoid_fixed.rrd','imu_cbs_latest.rrd',
              'glim_kimera_recording_20260527-014919.rrd',
              'glim_kimera_recording_20260527-014919_no_uncertainty.rrd',
              'glim_kimera_recording_20260527-014919_trajectories_only.rrd',
              'tmp_quad_easy_alignment_overlay.rrd','tmp_quad_easy_alignment_overlay.json'):
        single.append((c.W/n,'S3E / Newer College','old named visualization artifact'))
    single.append((c.V/'rerun_recordings/s3e_alpha_odom_between.rrd','S3E','dataset-named recording'))
    dr=c.W/'stage2b_implementation/deterministic_receiver'
    # These are generated diagnostic records, not original sensor bags. The
    # reviewed recording script names the odometry/belief-only output topics.
    for p in sorted((dr/'runs').glob('*/outputs.bag')):
        single.append((p,'M3DGR','run_kimera_shadow_replay.py:169 records outputs'))
    # Reviewed manifests identify these four as derived 20-second fixtures.
    for n in ('input_a.bag','input_b.bag','raw_input_a.bag','raw_input_b.bag'):
        single.append((dr/n,'M3DGR','corresponding *_manifest.json: generated diagnostic/sensor window'))
    pins=json.loads((H/'protected_before.json').read_text())
    protected=set()
    for p,r in pins.items():
        for q in (Path(p),Path(r['resolved'])):protected.add(q);protected.update(q.parents)
    entries={}
    for root,tag,evidence in dirs:
        assert root not in protected,root
        for p in c.scan(root):
            # Keep source scripts and any unexpected raw-sensor bag.
            if p.suffix in c.CODE|c.DATA:continue
            assert p not in protected,p
            entries[str(p)]=dict(path=str(p),dataset=tag,evidence=evidence,**c.identity(p))
    for p,tag,evidence in single:
        assert p.exists() and not p.is_symlink() and p.resolve()==p and p not in protected,p
        entries[str(p)]=dict(path=str(p),dataset=tag,evidence=evidence,**c.identity(p))
    rows=list(entries.values())
    empty=[c.W/'runs'/n for n in ('20260511-134551_duplicate-identity-60s-cbs-on',
        '20260615-120420_glim_only_cbs_off_observe_cov_240s',
        '20260615-120520_glim_only_cbs_off_observe_cov_240s')]
    assert all(not list(c.scan(p)) for p in empty)
    plan=dict(time=c.now(),entries=rows,dirs=[str(x[0]) for x in dirs]+list(map(str,empty)),
              allocated_bytes=sum(r['blocks'] for r in rows),files=len(rows))
    c.save('additional_plan.json',plan)
    print(json.dumps({k:v for k,v in plan.items() if k not in ('entries','dirs')},indent=2))

def apply():
    assert json.loads((H/'completion.json').read_text())['passed']
    plan=json.loads((H/'additional_plan.json').read_text())
    pins=json.loads((H/'protected_before.json').read_text())
    assert not (H/'additional_started.json').exists()
    for p,r in pins.items():assert c.digest(p)==r['sha256'],p
    def check(r):
        p=Path(r['path'])
        assert p.parent.resolve()==p.parent and c.within(p,c.V) and p not in (c.V,c.W),p
        assert c.identity(p)=={k:r[k] for k in c.identity(p)},p
        assert os.access(p.parent,os.W_OK|os.X_OK),p
    for r in plan['entries']:check(r)
    before=c.free();c.save('additional_started.json',dict(time=c.now(),free_bytes=before))
    with (H/'additional_deleted.jsonl').open('x') as f:
        for r in plan['entries']:
            check(r);Path(r['path']).unlink();f.write(json.dumps(r)+'\n')
    empty=0
    for root in plan['dirs']:
        for directory,dirs,files in os.walk(root,topdown=False,followlinks=False):
            p=Path(directory)
            if p.is_symlink():continue
            try:p.rmdir();empty+=1
            except OSError:pass
    for r in plan['entries']:assert not os.path.lexists(r['path'])
    for p,r in pins.items():assert c.digest(p)==r['sha256'] and str(Path(p).resolve())==r['resolved'],p
    result=dict(passed=True,time=c.now(),files=len(plan['entries']),empty_directories_removed=empty,
                freed_bytes=c.free()-before,free_bytes=c.free(),protected_files=len(pins))
    c.save('additional_completion.json',result);print(json.dumps(result,indent=2))

if __name__=='__main__':
    if sys.argv[1]=='prepare':make_plan()
    elif sys.argv[1]=='apply':apply()
    else:raise ValueError(sys.argv[1])
