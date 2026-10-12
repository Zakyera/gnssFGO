"""Verify frozen baseline and private batching inputs; never edit them."""
from pathlib import Path
import hashlib
import json
import subprocess
from prepare import WS,ROOT,sha

def main():
    batch=WS/'runs/e1b4a_local_update_batching_20261012'
    protected=json.loads((WS/'runs/gnssfgo_e1b_profile40_mapguard_20261012/prepared/protected_manifest.json').read_text())
    baseline=len(protected)
    protected.update(json.loads((ROOT/'protected.json').read_text()))
    for path,h in protected.items():
        p=Path(path.replace('/workspace/cbs_gtsam4.3',str(WS)))
        assert sha(p)==h,path
    build={}
    for p in batch.glob('*/*_protected.json'):build.update(json.loads(p.read_text()))
    code=('import hashlib,json,sys;from pathlib import Path;d=json.loads(sys.argv[1]);'
          'bad=[p for p,h in d.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h];'
          'assert not bad,bad;print(len(d))')
    subprocess.run(['docker','exec','cbsms_ws_gpu','python3','-c',code,json.dumps(build)],check=True)
    repos=json.loads((ROOT/'source_git.json').read_text())
    for path,info in repos.items():
        assert subprocess.check_output(['git','-C',path,'rev-parse','HEAD'],text=True).strip()==info['head']
        assert not subprocess.check_output(['git','-C',path,'status','--porcelain'],text=True).strip(),path
    # Verify overlays differ from pinned originals ONLY by a friend declaration.
    for overlay in (ROOT/'include').rglob('*'):
        if not overlay.is_file():continue
        suffix=str(overlay.relative_to(ROOT/'include'))
        candidates=[Path(k) for k in protected if k.endswith(suffix)]
        assert len(candidates)==1,(suffix,candidates)
        clean=overlay.read_text().replace('  friend class ExactGpuSnapshot;\n','').replace('  friend class LocalSnapshotAccess;\n','')
        assert clean==candidates[0].read_text(),suffix
    out=dict(passed=True,baseline_hashes=baseline,total_host_hashes=len(protected),
             container_build_inputs=len(build),source_repos=repos,friend_only_headers=5)
    with (ROOT/'integrity_final.json').open('x') as f:json.dump(out,f,indent=2)
    print(json.dumps({k:v for k,v in out.items() if k!='source_repos'},indent=2))
if __name__=='__main__':main()
