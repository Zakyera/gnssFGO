#!/usr/bin/env python3
"""Host-side verification and small evidence archive; never copies binaries."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import xml.etree.ElementTree as ET

HERE=Path(__file__).resolve().parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
RUN=WS/'runs/e1b_local_profile_20261012'

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()

def host(path):
    return Path(str(path).replace('/workspace/cbs_gtsam4.3',str(WS),1))

def main():
    evidence=HERE/'evidence'
    evidence.mkdir(exist_ok=True)
    def retain(src,dest):
        # Resume a partial evidence copy only when existing bytes are identical.
        if dest.exists():assert sha(src)==sha(dest),'refuse overwrite '+str(dest)
        else:shutil.copy2(src,dest)
    manifest=HERE.parent/'e1_e2_live160_20261011/evidence/prepared/protected_manifest.json'
    frozen=json.loads(manifest.read_text())
    assert all(sha(p)==h for p,h in frozen.items()),'frozen manifest changed'
    sources=json.loads((RUN/'source_git.json').read_text())
    for path,info in sources.items():
        assert subprocess.check_output(['git','-C',path,'rev-parse','HEAD'],text=True).strip()==info['head']
        assert not subprocess.check_output(['git','-C',path,'status','--porcelain'],text=True).strip()
    assert all(sha(p)==h for p,h in json.loads((RUN/'source_manifest.json').read_text()).items())
    artifacts={};totals={};protected={}
    container_check=json.loads((RUN/'protected_verified.json').read_text())
    for variant in ('release_2','ubsan_1'):
        build=RUN/variant
        protected[variant]=json.loads((build/'protected.json').read_text())
        assert container_check[variant]['manifest_sha256']==sha(build/'protected.json')
        assert not container_check[variant]['mismatches']
        assert all(sha(host(p))==h for p,h in protected[variant].items()
                   if p.startswith('/workspace/cbs_gtsam4.3/')),'original workspace build input changed'
        complete=json.loads((build/'complete.json').read_text())
        assert all(sha(build/p)==h for p,h in complete['artifacts'].items()),'candidate build artifact changed'
        artifacts[variant]=complete['artifacts']
        assert json.loads((build/'tests1/passed.json').read_text())['passed']
        tests=failures=errors=disabled=0
        for p in (build/'tests1').glob('*.xml'):
            root=ET.parse(p).getroot()
            tests+=int(root.attrib['tests'])-int(root.attrib['disabled'])
            disabled+=int(root.attrib['disabled'])
            failures+=int(root.attrib['failures']);errors+=int(root.attrib['errors'])
        assert tests==29 and failures==errors==0
        totals[variant]=dict(executed=tests,disabled=disabled,failures=failures,errors=errors)
    extra=json.loads((RUN/'release_2/extra1/passed.json').read_text());assert extra['passed']
    io=json.loads((RUN/'release_2/io1/passed.json').read_text());assert io['rows']==8750
    asan=ET.parse(RUN/'release_2/extra1/asan_ubsan.xml').getroot()
    assert asan.attrib['tests']=='8' and asan.attrib['failures']=='0' and asan.attrib['errors']=='0'
    config=WS/'build/gtsam/gtsam/config.h'
    assert '/* #undef GTSAM_USE_TBB */' in config.read_text()
    files={};large={}
    for p in RUN.rglob('*'):
        if not p.is_file() or p.is_symlink():continue
        if p.suffix not in ('.json','.log','.xml','.diff'):continue
        if p.stat().st_size>1000000:
            large[str(p)]=dict(bytes=p.stat().st_size,sha256=sha(p));continue
        relative=p.relative_to(RUN);dest=evidence/relative;dest.parent.mkdir(parents=True,exist_ok=True)
        retain(p,dest);files[str(relative)]=sha(p)
    retain(config,evidence/'gtsam_config.h')
    files['gtsam_config.h']=sha(config)
    for name in ('RuntimeProfile.h','RuntimeProfile.cpp','test_profile.cpp','test_solver.cpp','test_gpu.cpp','build.py','test.py','bench.cpp','validate_extra.py','check_overhead.py'):
        assert sha(HERE/name)==sha(RUN/name),'authored/staged mismatch '+name
    summary=dict(protected_manifest_path=str(manifest),protected_manifest_sha256=sha(manifest),
                 protected_files_unchanged=len(frozen),original_relink_inputs_unchanged={k:len(v) for k,v in protected.items()},
                 source_git_unchanged=sources,tests=totals,profiler_asan_ubsan_tests=8,
                 original_and_repeat_comparison=extra,io_overhead=io,gtsam_internal_tbb_enabled=False,
                 candidate_artifacts=artifacts,evidence_manifest=files,large_evidence_retained_locally=large)
    (HERE/'validation.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({k:v for k,v in summary.items() if k not in ('candidate_artifacts','evidence_manifest','source_git_unchanged')},indent=2))

if __name__=='__main__':main()
