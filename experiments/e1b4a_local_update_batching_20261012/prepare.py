"""Create private overlays only; refuse to overwrite a previous preparation."""
from pathlib import Path
import hashlib
import json
import shutil
import subprocess

HERE = Path(__file__).resolve().parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT = WS/'runs/e1b4a_local_update_batching_20261012'
OLD = WS/'runs/e1b_local_profile_20261012'

def replace_once(text, before, after):
    assert text.count(before) == 1, before
    return text.replace(before, after)

def main():
    expected = json.loads((OLD/'source_git.json').read_text())
    for repo, info in expected.items():
        assert subprocess.check_output(['git','-C',repo,'rev-parse','HEAD'],text=True).strip() == info['head']
        assert not subprocess.check_output(['git','-C',repo,'status','--porcelain'],text=True).strip()
    ROOT.mkdir(exist_ok=False)
    shutil.copytree(HERE, ROOT/'authoring')
    overlay = ROOT/'overlay'; overlay.mkdir()
    src = OLD/'overlay/NonlinearFactorGraph.cpp'
    graph = '#include "ScopedGraphLinearizer.h"\n' + src.read_text()
    graph = replace_once(graph, '  // create an empty linear FG',
        '  e1b4a::GraphCall dispatch_guard;\n'
        '  if (auto batched = dispatch_guard.dispatch(*this, linearizationPoint)) return batched;\n\n'
        '  // create an empty linear FG')
    (overlay/'NonlinearFactorGraph.cpp').write_text(graph)
    src = OLD/'overlay/cbs_bridge.cpp'
    bridge = '#include "LocalUpdateBatching.h"\n' + src.read_text()
    bridge = replace_once(bridge, '    sidecar_->update(new_factors, new_values, new_stamps);',
        '    gtsam::e1b4a::ScopedLinearizer batch_scope(\n'
        '        glim::e1b4a::enabled() ? &glim::e1b4a::linearize : nullptr);\n'
        '    sidecar_->update(new_factors, new_values, new_stamps);')
    (overlay/'cbs_bridge.cpp').write_text(bridge)
    manifest = {str(p):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [OLD/'overlay/NonlinearFactorGraph.cpp', OLD/'overlay/cbs_bridge.cpp',
                  WS/'runs/e1b_startup_fix_20261012/fixed_release/libglim_ros.so']}
    (ROOT/'protected_preparation.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (ROOT/'source_git.json').write_text(json.dumps(expected,indent=2)+'\n')
    print(ROOT)

if __name__ == '__main__': main()
