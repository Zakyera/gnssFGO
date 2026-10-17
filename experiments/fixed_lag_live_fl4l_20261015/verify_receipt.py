"""Host-side identity/coverage receipt; read-only with respect to all inputs."""
import hashlib,json,subprocess
from pathlib import Path
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
root=W/'runs/fixed_lag_same_graph_fl4l_20261015'
frozen=W/'runs/gnssfgo_eth1_fixed_lag_live120_fl4g_20261015/prepared'
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
    return h.hexdigest()
protected=json.loads((frozen/'protected_manifest.json').read_text())
changed=[p for p,h in protected.items() if not Path(p).is_file() or sha(Path(p))!=h]
assert not changed,changed
source=root/'on_source_capture'
profiles={}
for name in ['glim_config','kimera_profile']:
    actual={str(p.relative_to(source/name)):sha(p) for p in (source/name).rglob('*') if p.is_file()}
    baseline={str(p.relative_to(frozen/'pilot120_on_source'/name)):sha(p) for p in (frozen/'pilot120_on_source'/name).rglob('*') if p.is_file()}
    assert actual and actual==baseline,name
    profiles[name]={'files':len(actual),'exact_match':True}
settings=json.loads((source/'effective_mode_check.json').read_text())
outputs={str(p.relative_to(root)):sha(p) for p in (root/'capture/update_438').glob('*') if p.is_file()}
report={'protected_files_verified':len(protected),'protected_files_changed':changed,
        'profiles':profiles,'effective_mode_check_passed':settings['passed'],
        'private_kimera_sha256':sha(root/'private_build/libkimera_vio.so'),
        'production_kimera_sha256':sha(W/'build/kimera_vio/libkimera_vio.so'),
        'capture_files':outputs,
        'git_heads':{str(p):subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip()
            for p in [Path('/home/yeranis/repos/gnssFGO'),W/'src/cbs',W/'src/Kimera-VIO']},
        'source_coverage':json.loads((source/'source_coverage.json').read_text()) if (source/'source_coverage.json').exists() else None,
        'capture_complete':(root/'control_capture/complete.json').exists()}
with (root/'receipt.json').open('x') as f:json.dump(report,f,indent=2)
print(json.dumps({k:v for k,v in report.items() if k!='capture_files'},indent=2))
