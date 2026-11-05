"""Static compatibility audit only; never imports or executes experiment code."""
from datetime import datetime
from pathlib import Path
import hashlib
import json

H = Path(__file__).resolve().parent
R = H.parent.parent
W = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
K = W / 'runs/retention_readiness_cpv73_20261031'
H73 = R / 'experiments/retention_readiness_cpv73_20261031'
OLD = W / 'runs/grandtour_eig2_glim_kimera/eig2_stereo_shared_cpt7_cbs_on_bidirectional_full305_start5p7_rate010_20260921/grandtour_launch_used.launch'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

inputs = {}
def read(p):
    p = Path(p)
    inputs[str(p)] = sha(p)
    return p.read_text()

def js(p):
    return json.loads(read(p))

def save(name, value):
    with (H / name).open('x') as f:
        json.dump(value, f, indent=2)
        f.write('\n')

for name in ('audit.json', 'input_sha256.json', 'verification.json'):
    assert not (H / name).exists(), 'Preserve an existing audit; use a fresh attempt.'

complete = js(H73 / 'complete.json')
pins = complete['output_sha256']
for p, expected in pins.items():
    assert sha(p) == expected, p

sources = js(K / 'backend_initial/sources.json')
for p, expected in sources.items():
    host = p.replace('/workspace/cbs_gtsam4.3', str(W), 1)
    assert sha(host) == expected, host

cpp = read(W / 'src/cbs/src/bpsam/bpsam.cpp')
core = read(K / 'include/cbs/bpsam/bpsam.h')
smoother = read(K / 'include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h')
refresh = read(K / 'BpsamOnRefresh.inc')
prior = read(K / 'NativePriorRefresh.h')
on = read(K / 'OnRefresh.h')
compat = read(K / 'RetainCompatibility.h')
backend = read(K / 'VioBackend.cpp')
test = read(K / 'test.cpp')
on_test = read(K / 'on_tests.inc')
off_test = read(K / 'off_tests.inc')
params = read(W / 'runs/kimera_incremental_prior_cpk9_20261019/test.cpp')
old_test = read(W / 'runs/fixed_lag_retirement_fl4m_20261015/test.cpp')
old_launch = read(OLD)
freeze_script = read(W / 'runs/retention_full_frozen_cpv71_20261031/replay/prepare_inputs.py')
frozen = js(K / 'replay/frozen_inputs.json')
observed = js(K / 'replay/retain_capture/preflight_observed.json')
protected = js(H73 / 'build_initial/protected_before.json')
assert inputs[str(W / 'src/cbs/src/bpsam/bpsam.cpp')] == protected[str(W / 'src/cbs/src/bpsam/bpsam.cpp')]

checks = []
def check(name, condition, evidence):
    assert condition, name
    checks.append(dict(name=name, passed=True, evidence=evidence, kind='static_or_saved_receipt'))

check('old explicit factor mode, not sidecar',
      '--cbs_odom_factor_mode=persistent' in old_launch and
      '--cbs_use_persistent_bpsam_for_main_backend=false' in old_launch,
      f'{OLD}:237,244')
check('persistent parser retained',
      'if (mode == "persistent" || mode == "inject_persistent")' in backend,
      f'{K}/VioBackend.cpp:525')
check('corrected core accepts active-window only',
      'params_.cbs_odom_factor_mode!=CbsOdomFactorMode::ActiveWindowTemporary)\n      throw std::runtime_error("CPK9 incompatible native core mode")' in core,
      f'{K}/include/cbs/bpsam/bpsam.h:96')
check('mode validation before ordinary solve',
      smoother.index('if (cpv40_enabled) isam_.cpv40ValidateMode();') < smoother.index('return fl4qUpdateImpl(') and
      'void cpv40ValidateMode() const {\n  cpk9ValidateMode();' in refresh,
      f'{K}/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h:124')
check('CP40 cannot be kept while disabling CP-K9',
      '!cpk9::enabled()' in on,
      f'{K}/OnRefresh.h:9')
check('post-refresh ownership requires active-window maps',
      'active_cbs_odom_slots_.size()!=active_window_cbs_odom_factors_.size()' in refresh and
      'active_window_cbs_odom_keys_by_receiver_.size()!=active_window_cbs_odom_factors_.size()' in refresh,
      f'{K}/BpsamOnRefresh.inc:34')
check('persistent registers slots but not active-window staged records',
      'active_cbs_odom_slots_.insert_or_assign(' in cpp and
      'if (staged_it != staged_active_window_odom.end())' in cpp and
      'active_window_cbs_odom_factors_.insert_or_assign(' in cpp,
      f'{W}/src/cbs/src/bpsam/bpsam.cpp:4037,4529,4534')
check('persistent pending replacement path retained',
      'pending_cbs_odom_beliefs_.insert_or_assign(odom_key, odom_belief);' in cpp,
      f'{W}/src/cbs/src/bpsam/bpsam.cpp:2933')
check('persistent active replacement path retained',
      'replaced_persistent_cbs_odom_keys.insert(active_it->first);' in cpp,
      f'{W}/src/cbs/src/bpsam/bpsam.cpp:4068')
check('persistent cross-update receiver-match policy retained',
      'receiver_edge_retained_better_persistent_match' in backend and
      'if (same_sender_edge || strictly_better_alternative)' in backend,
      f'{K}/VioBackend.cpp:3080')
check('retirement receipts apply to active-window mode',
      'if (isam_.usesActiveWindowTemporaryCbsOdomFactors())' in smoother and
      'if(cpv69::retain())' in smoother,
      f'{K}/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h:190,496')
check('corrected native extra step preserves prior representation',
      'cpk9PromotePriorCharts();' in refresh and
      'BaseSAM::update({}, {}, refresh)' in refresh and
      'std::make_shared<fl4l::OriginalChartPrior>(*p,theta)' in prior,
      f'{K}/BpsamOnRefresh.inc:77,82 and NativePriorRefresh.h:52')
check('whole-update rollback remains enabled under CP40',
      'return fl4q::transaction(*this, newFactors' in smoother,
      f'{K}/include/cbs/bpsam/incremental_fixed_lag_bpsam_smoother.h:135')
check('native mode-specific recovery must be tested',
      'if (active_window_mode && !staged_active_window_odom.empty())' in cpp and
      'nearbyVariable is a robot key, removing its factors' in cpp,
      f'{W}/src/cbs/src/bpsam/bpsam.cpp:4336')
check('corrected suite inherits active-window-only params',
      'p.cbs_odom_factor_mode=cbs::BPSAM::CbsOdomFactorMode::ActiveWindowTemporary;' in params and
      'CbsOdomFactorMode::Persistent' not in test + on_test + off_test,
      f'{W}/runs/kimera_incremental_prior_cpk9_20261019/test.cpp:16; {K}/test.cpp,on_tests.inc,off_tests.inc')
check('old isolated fixture tests persistent, before corrected backend',
      'CbsOdomFactorMode::Persistent' in old_test and
      'CPV40_REFRESH' not in old_test and 'CPK9_PRIOR_REFRESH' not in old_test,
      f'{W}/runs/fixed_lag_retirement_fl4m_20261015/test.cpp:8')
check('CP73 uses retained active-window, not persistent',
      '--cbs_odom_factor_mode=active_window_temporary' in observed['argv'] and
      observed['candidate_env']['CPV69_RETAIN_COMPAT'] == '1' and
      observed['candidate_env']['CPV40_REFRESH'] == '1',
      f'{K}/replay/retain_capture/preflight_observed.json')
check('frozen replay excludes original unaccepted input',
      frozen['accepted_rows'] == 1743 and frozen['original_wire_beliefs'] == 3541 and
      len(frozen['omitted_original_unaccepted']) == 1798,
      f'{K}/replay/frozen_inputs.json')
check('frozen replay changes raw means/endpoints to accepted aligned values',
      "n.relative_mu=r['aligned']" in freeze_script and "n.from_stamp_sec=r['times'][2]*1e-9" in freeze_script,
      f'{W}/runs/retention_full_frozen_cpv71_20261031/replay/prepare_inputs.py:57')

libraries = {}
for name in ('libcbs.so', 'libkimera_vio.so'):
    p = K / 'private_build' / name
    libraries[name] = sha(p)
    inputs[str(p)] = libraries[name]
    assert '/workspace/cbs_gtsam4.3/runs/retention_readiness_cpv73_20261031/private_build/' + name in observed['maps']
assert libraries['libkimera_vio.so'] == '141474ecf226f863c81e5f8b70df0843b80420aa2e78f74fb48729a1958169d1'

for p, expected in frozen['source_hashes'].items():
    host = p.replace('/workspace/cbs_gtsam4.3', str(W), 1)
    assert sha(host) == expected, host
    inputs[host] = expected

result = dict(
    audit_complete=True,
    drop_in_compatible=False,
    mathematical_incompatibility_proven=False,
    runtime_test_executed=False,
    native_solver_executed=False,
    build_executed=False,
    replay_launched=False,
    source_changed=False,
    checks=checks,
    libraries=libraries,
    verified_CP73_build_sources=len(sources),
    original_input_inventory=dict(wire_beliefs=3541, accepted_frozen=1743, omitted_from_frozen=1798),
    next_work='Private general-persistent mode-aware compatibility extension and native synthetic tests; separate authorization needed. No capture until those pass.',
    caveats=[
        'Static source evidence is not a numerical execution or performance result.',
        'Current persistent implementation is not certified byte/event-identical to September binaries.',
        'Changing weights, disabling corrections, or deleting the mode assertion is not an acceptable compatibility repair.',
        'One-way saved replay is conditional on its original feedback history; no new peer loop.',
        'CP67 mixed-prior LOCAL export limitation remains; no live bidirectional certification.',
    ],
)
for p, expected in {**pins, **inputs}.items():
    assert sha(p) == expected, p
save('audit.json', result)
save('input_sha256.json', inputs)
save('verification.json', dict(
    audit_passed=True, compatibility_passed=False, static_checks=len(checks),
    CP73_outputs_unchanged=len(pins), audited_inputs_unchanged=len(inputs),
    completed=datetime.now().astimezone().isoformat(),
))
print(json.dumps(dict(checks=len(checks), compatible=False, sources_verified=len(sources),
                     inputs_unchanged=len(inputs), CP73_outputs_unchanged=len(pins),
                     libraries=libraries), indent=2))
