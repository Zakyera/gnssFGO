"""Mechanically clone the validated timing harness into a fresh namespace."""
from pathlib import Path
import hashlib
import json
import shutil

HERE = Path(__file__).resolve().parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
OLD = WS / 'runs/gnssfgo_e1b4a_timing40_20261012'
RUN = WS / 'runs/gnssfgo_shared_linearization_b0_20261012'
ROOT = RUN / 'prepared'
PREVIOUS = HERE.parent / 'e1b4a_timing40_20261012'

def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1048576), b''):
            h.update(block)
    return h.hexdigest()

def transform(s):
    return (s.replace(OLD.name, RUN.name).replace('11414', '11418')
            .replace('eth1_e1b4a_timing40_on_20261012',
                     'eth1_shared_linearization_b0_on_20261012'))

def main():
    assert not RUN.exists(), 'Refuse existing run root'
    protected = json.loads((OLD/'prepared/protected_manifest.json').read_text())
    assert len(protected) == 677
    for path, expected in protected.items():
        assert sha(Path(path)) == expected, path
    s1 = json.loads((HERE.parent/'shared_linearization_s1_20261012/INTEGRITY.json').read_text())
    for path, record in s1['inspected_evidence'].items():
        assert sha(WS/path) == record['sha256'], path
    ROOT.mkdir(parents=True)
    shutil.copy2(HERE/'PROTOCOL.txt', RUN/'PROTOCOL.txt')
    records = []
    def copy_text(src, dst):
        original = src.read_text()
        transformed = transform(original)
        with dst.open('x') as f:
            f.write(transformed)
        records.append(dict(source=str(src), destination=str(dst),
                            source_sha256=sha(src), destination_sha256=sha(dst),
                            byte_identical=original == transformed))
    names = ['experiment.py','run_case.py','source.env','source_runner.sh',
             'active_window.launch','glim_profile.launch','grandtour_eth1_kimera_stereo.launch',
             'kimera_vio_ros.launch','kimera_vio_ros_newer_college_2021_stereo.launch',
             'receiver_e2.sh','coverage_checks.py','validate_numerical_case.py',
             'numerical_equivalence.py','profile40_on.json','verify_effective.py','map_guard.py']
    for name in names:
        copy_text(OLD/'prepared'/name, ROOT/name)
    (ROOT/'receiver_e2.sh').chmod(0o755)
    shutil.copytree(OLD/'prepared/private_build', ROOT/'private_build', symlinks=True)
    for name in ['validate_native.py','validate_lifecycle.py','validate_diagnostics.py',
                 'admission_contract.py','lifecycle_contract.py']:
        copy_text(OLD/name, RUN/name)
    for name in ['preflight.py','validate.py','summarize.py','analyze_profile.py']:
        copy_text(PREVIOUS/name, HERE/name)
    with (HERE/'preparation.json').open('x') as f:
        json.dump(dict(inherited_protected_hashes=len(protected),
                       s1_evidence_hashes=len(s1['inspected_evidence']),
                       allowed_substitutions=[OLD.name+' -> '+RUN.name,
                         '11414 -> 11418','ETH1 session namespace'],files=records), f, indent=2)
    print(ROOT)

if __name__ == '__main__':
    main()
