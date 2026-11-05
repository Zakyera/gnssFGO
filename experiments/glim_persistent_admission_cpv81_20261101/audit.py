"""Static admission evidence and read-only integrity checks; no ROS/build/run."""
import datetime
import hashlib
import json
import mmap
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

H = Path(__file__).resolve().parent
R = H.parents[1]
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
H80 = R / 'experiments/eig2_persistent_live_cpv80_20261101'
H78 = R / 'experiments/persistent_native_full_cpv78_20261101'
I1 = W / 'runs/local_revision_covariance_cpi1_20261019'
S4 = W / 'runs/shared_linearization_s4_provider_20261012/release_2/source'


def read(path):
    return json.loads(path.read_text())


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def verify(entries):
    errors = []
    for name, expected in entries.items():
        try:
            if sha(Path(name)) != expected:
                errors.append(name)
        except OSError as exc:
            errors.append({'path': name, 'error': str(exc)})
    return {'checked': len(entries), 'passed': not errors, 'errors': errors}


assert not (H / 'audit.json').exists(), 'Never overwrite audit evidence'
P = Path(read(H80 / 'prepared.json')['prepared'])
effective = read(P / 'full_on_source/effective_mode_check.json')
source = I1 / 'cbs_bridge.cpp'
code = source.read_text()
guard = '''if (native_provider && (mode_ != Mode::ActiveWindowTemporary ||
      outgoing_marginal_source_ != OutgoingMarginalSource::Direct || live_revision_enable_ ||
      outgoing_odom_covariance_mode_ != "schur_relative_between"))'''
error = 'shared-native provider unsupported CBS configuration'
env = {item.attrib['name']: item.attrib['value']
       for item in ET.parse(P / 'glim_profile_on.launch').iter('env')}
build = [row for row in read(I1 / 'release_commands.json')
         if row['name'] in ('bridge_compile', 'bridge_link')]
with (I1 / 'libglim_ros.so').open('rb') as stream:
    with mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as binary:
        binary_has_guard_error = binary.find(error.encode()) >= 0
paths = [source, I1 / 'libglim_ros.so', I1 / 'RevisionCovariance.h',
         I1 / 'release_commands.json', S4 / 'ShadowObserver.cpp', S4 / 'test_lifecycle.cpp',
         W / 'runs/main_gpu_batching_cpk16_20261020/release_1/libgtsam_points.so.1.2.1',
         P / 'glim_profile_on.launch', P / 'full_on_source/effective_mode_check.json']
out = {
    'observed': datetime.datetime.now().astimezone().isoformat(),
    'scope': 'Static frozen-code/configuration admission review; NOT a new runtime test',
    'inputs_sha256': {str(path): sha(path) for path in paths},
    'source_guard_exact': guard in code,
    'source_guard_line': code[:code.index(guard)].count('\n') + 1 if guard in code else None,
    'binary_contains_guard_error': binary_has_guard_error,
    'bridge_build_receipts': build,
    'cp80_modes': {'glim': effective['glim']['glim_cbs_mode'],
                   'kimera': effective['kimera']['cbs_odom_factor_mode']},
    'frozen_env': {key: env.get(key) for key in (
        'GLIM_SHARED_NATIVE_PROVIDER', 'GLIM_REVISION_COVARIANCE_POLICY',
        'CPK16_MAIN_GPU_BATCHING')},
    'proposed_glim_mode': 'InjectPersistent',
    'proposed_mode_rejected_by_current_source_guard': guard in code and
        env.get('GLIM_SHARED_NATIVE_PROVIDER') == 'shared_native',
    'new_runtime_test_performed': False,
    'new_capture_launched': False,
    'protected': verify(read(P / 'protected_manifest.json')),
    'cp80_terminal': verify(read(H80 / 'complete.json')['output_sha256']),
    'cp78_terminal': verify(read(H78 / 'complete.json')['output_sha256']),
}
out['containers'] = subprocess.check_output([
    'docker', 'inspect', '-f', '{{.Name}} {{.State.Status}}',
    'cbsms_ws_gpu', 'ubuntu-jammy-humble-nvidia'], text=True).splitlines()
out['containers_restored'] = out['containers'] == read(H80 / 'start.json')['before']
out['hesai_checkpoint'] = subprocess.check_output([
    'python3', str(R / 'experiments/checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),
    '--originals'], text=True).strip()
out['audit_evidence_verified'] = (
    out['source_guard_exact'] and out['binary_contains_guard_error']
    and len(build) == 2 and all(row['returncode'] == 0 for row in build)
    and out['frozen_env'] == {'GLIM_SHARED_NATIVE_PROVIDER': 'shared_native',
                            'GLIM_REVISION_COVARIANCE_POLICY': 'native_local',
                            'CPK16_MAIN_GPU_BATCHING': '1'}
    and all(out[key]['passed'] for key in ('protected', 'cp80_terminal', 'cp78_terminal'))
    and out['containers_restored']
    and out['hesai_checkpoint'] == 'PASS: 4 cases, 667 saved files; originals=True')
out['live_admission_passed'] = False
with (H / 'audit.json').open('x') as stream:
    json.dump(out, stream, indent=2, allow_nan=False)
print(json.dumps({key: out[key] for key in ('observed', 'audit_evidence_verified',
    'live_admission_passed', 'source_guard_line', 'frozen_env', 'protected',
    'cp80_terminal', 'cp78_terminal', 'containers', 'hesai_checkpoint')}, indent=2))
assert out['audit_evidence_verified'], 'Evidence/integrity failed; inspect without overwriting'
