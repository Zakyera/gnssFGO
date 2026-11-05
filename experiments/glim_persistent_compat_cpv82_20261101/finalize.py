"""Saved-only CP82 test closeout; never builds or runs an estimator."""
import datetime,difflib,hashlib,json,subprocess
from pathlib import Path
import xml.etree.ElementTree as ET
H=Path(__file__).resolve().parent;E=H.parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3');R=W/'runs'/H.name
def read(p):return json.loads(p.read_text())
def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,v):
    with p.open('x') as f:json.dump(v,f,indent=2)
assert not (H/'complete.json').exists()
tests={}
for label in ('release_4','ubsan_1'):
    p=R/label;v=read(p/'passed.json');assert v['passed']
    tree=ET.parse(p/'tests.xml').getroot()
    assert tree.attrib['tests']=='29' and tree.attrib['failures']=='0' and tree.attrib['errors']=='0'
    assert read(p/'parity.json')['passed']
    gpu=read(p/'gpu_equivalence.log');assert gpu['passed']
    assert all(v==0 for v in gpu['max_absolute_differences'].values())
    assert 'retired_peers=13' in (p/'gpu_1.log').read_text()
    c=read(H/label/'closeout.json');assert c['code']==0 and not c['protected_changed']
    tests[label]=dict(tests=29,bridge_sha256=v['bridge_sha256'],temporary_exact_parity=read(p/'parity.json'),
        gpu_records=gpu['records'],gpu_numeric_entries=gpu['numeric_entries'],gpu_max_differences=gpu['max_absolute_differences'],
        peer_retirements=13,main_updates=72,covariance_pairs=555,closeout=c)
original=W/'runs/local_revision_covariance_cpi1_20261019/cbs_bridge.cpp'
diff=''.join(difflib.unified_diff(original.read_text().splitlines(True),(R/'cbs_bridge.cpp').read_text().splitlines(True),fromfile=str(original),tofile=str(R/'cbs_bridge.cpp')))
with (H/'bridge.diff').open('x') as f:f.write(diff)
assert diff.count('@@')==4, 'Only include and admission/diagnostic hunks expected'
pins={}
for old in ('eig2_persistent_live_cpv80_20261101','persistent_native_full_cpv78_20261101'):
    for p,d in read(E/old/'complete.json')['output_sha256'].items():assert sha(p)==d,p
    pins[old]=len(read(E/old/'complete.json')['output_sha256'])
frozen=read(W/'runs/eig2_persistent_live_cpv80_20261101/prepared/protected_manifest.json')
assert all(sha(p)==d for p,d in frozen.items())
containers=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True)
assert containers=='/cbsms_ws_gpu exited\n/ubuntu-jammy-humble-nvidia exited\n'
hesai=subprocess.check_output(['python3',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True)
verification=dict(passed=True,tests=tests,protected=len(frozen),terminal_outputs=pins,containers=containers,hesai=hesai,
    no_sensor_capture=True,production_change='Opt-in admission of existing persistent path; diagnostic only otherwise',
    covariance_math_changed=False,weights_changed=False,default_promoted=False,
    sanitizer_scope='UBSan private bridge/test and existing instrumented K16 overlay, not all vendor libraries; no ASan claim')
save(H/'verification.json',verification)
report='''CP-V82 GLIM PERSISTENT COMPATIBILITY — PRIVATE TESTS PASS
The current all-LOCAL covariance provider previously rejected persistent mode
at startup. The existing persistent algorithm was still present. Private opt-in
CPV82_GLIM_PERSISTENT_COMPAT=1 admits that combination; all other restrictions,
defaults, weights, matching/alignment and covariance formulas remain unchanged.
Candidate is release_4/libglim_ros.so; SHA256 aecc729f4bf51548950b8cb997c3dcd474e30cabe23785c4011e6b22f7dbe918.

Release and focused UBSan: 29/29 tests pass each. Actual bridge insertion and
duplicate rejection; retained factor identities until retirement; native-only
positive admission/conservation; first/revision LOCAL covariance equal. Disabled
and enabled opt-in in temporary mode match original frozen DSO exactly in states,
factor slots, Gaussian cache, native marginal history and covariance output.
Actual GPU/IMU wrapper: 72 updates, 13 peer marginal retirements, 555 covariance
queries, slot reuse. Batching OFF/ON has 51,343 equal records / 2,590,537 numeric
entries, including native marginal history, states, device snapshot bytes and
exported covariance: maximum difference ZERO in release and UBSan. No extra GPU
covariance work or MAIN covariance fallback. No provider reset.
These are compatibility tests, not a sensor accuracy result, covariance
calibration proof, or nonlinear stationarity certificate. Exact-endpoint bridge
fixture is not a new raw IMU calibration audit; unchanged alignment is inherited.
Vendor libraries are not all sanitizer-instrumented. No ASan claim.

Three failed harness attempts retained: (1) executable linked default rather than
private dependency overlays; (2) ROS node lifetime ended between test cases;
(3) synthetic fixture lacked received IMU samples. Repairs confined to harness;
no estimator numerical retry, guard relaxation, or sensor capture occurred.
Both containers restored stopped, CP80/78 terminal outputs and 1617 protected
hashes unchanged, original Hesai667 verifies. Original libraries not replaced.

Authorized next stage: one fresh 60s EIG-2 Hesai live BOTH-persistent integration,
all three estimators, unchanged CP80 sensor/receiver/weight setup. CP83 gets an
exclusive new root and independent admission gates. No full/cross-sequence run
or default promotion authorized. Kimera's accepted mixed-prior provenance limit
remains explicit; restored persistent support does NOT repair peer correlation.
'''
with (H/'REPORT.txt').open('x') as f:f.write(report)
outputs={}
for root in (H,R):
    for p in root.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:outputs[str(p)]=sha(p)
save(H/'complete.json',dict(passed=True,finished=datetime.datetime.now().astimezone().isoformat(),output_sha256=outputs,
    private_compatibility_only=True,live_capture_started=False,conditional60_authorized=True))
print(json.dumps(verification,indent=2))
