"""One fresh diagnostic harness; mechanically reuse CP-K5 with CP-K9 pinned."""
import ast,hashlib,json,shutil,xml.etree.ElementTree as ET
from pathlib import Path
H=Path(__file__).resolve().parent
W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
O=W/'runs/kimera_incremental_on_cpk5_20261019'
C=W/'runs/kimera_incremental_prior_cpk9_20261019'
R=W/'runs/kimera_incremental_prior_live_cpk10_20261019'
OLDH=H.parent/'kimera_incremental_on_cpk5_20261019'
def sha(p):
    h=hashlib.sha256()
    with open(p,'rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def main():
    assert not R.exists(),'Refuse duplicate preparation'
    v=json.loads((H.parent/'kimera_incremental_prior_cpk9_20261019/validation.json').read_text())
    assert v['passed'] and v['candidate_dso_sha256']==sha(C/'private_build/libkimera_vio.so')
    protected=json.loads((C/'protected_before.json').read_text())
    assert all(sha(p)==h for p,h in protected.items()),'Old protected evidence changed'
    for root in (O,C):
        for p in root.rglob('*'):
            if p.is_file():protected[str(p)]=sha(p)
    assert shutil.disk_usage(W).free>8*1024**3
    R.mkdir()
    shutil.copytree(O/'kimera_profile',R/'kimera_profile')
    for n in ('grandtour_eth1_kimera_stereo.launch','kimera_vio_ros.launch',
              'kimera_vio_ros_newer_college_2021_stereo.launch','raw_glim_inputs.bag','input_provenance.json'):
        shutil.copy2(O/n,R/n)
    p=R/'kimera_vio_ros.launch';s=p.read_text()
    old='<env name="FL4Q_ENABLE" value="0"/>'
    assert s.count(old)==1
    p.write_text(s.replace(old,old+'\n    <env name="CPK9_PRIOR_REFRESH" value="1"/>'))
    for p in R.glob('*.launch'):ET.parse(p)
    c=(O/'run_case.py').read_text()
    def edit(a,b):
        nonlocal c
        assert a in c,a
        c=c.replace(a,b)
    edit('CP-K5','CP-K10');edit('cpk5_preflight','cpk10_preflight');edit('11457','11458')
    edit("K=W/'runs/kimera_failed_factor_cpk1_20261019';D=R/'on_capture'",
         "K=W/'runs/kimera_failed_factor_cpk1_20261019';D=R/'on_capture'\nC=W/'runs/kimera_incremental_prior_cpk9_20261019'\nO=W/'runs/kimera_incremental_on_cpk5_20261019'")
    edit("FL4Q_ENABLE='0',FL4Q_LOG=", "CPK9_PRIOR_REFRESH='1',FL4Q_ENABLE='0',FL4Q_LOG=")
    edit("str(K/'private_build')","str(C/'private_build')")
    edit("str(K/'private_build'/lib)","str(C/'private_build'/lib)")
    edit("assert env['FL4Q_ENABLE']=='0' and env['FL4M_RETIREMENT']=='remove'",
         "assert env['FL4Q_ENABLE']=='0' and env['FL4M_RETIREMENT']=='remove' and env['CPK9_PRIOR_REFRESH']=='1'")
    edit("for k in ('FL4Q_ENABLE','FL4Q_LOG'", "for k in ('CPK9_PRIOR_REFRESH','FL4Q_ENABLE','FL4Q_LOG'")
    edit("resolved=run(['rosparam','get',node]);params=yaml.safe_load(resolved)",
         """baseline_argv=json.loads((O/'on_capture/preflight.json').read_text())['argv']
    def normalize(args,root):
        return [a.replace(str(root),'<RUN>') for a in args if not a.startswith('__log:=')]
    assert normalize(argv,R)==normalize(baseline_argv,O),'Full argv mismatch vs CP-K5'
    resolved=run(['rosparam','get',node]);params=yaml.safe_load(resolved)""")
    edit("('FL4Q no decreasing','CPK1'", "('CPK9 ','FL4Q no decreasing','CPK1'")
    edit('ON SAVED-INPUT DIAGNOSTIC: FL4Q=0','ON SAVED-INPUT DIAGNOSTIC: CPK9=1, FL4Q=0')
    ast.parse(c);(R/'run_case.py').write_text(c)
    shutil.copy2(H/'PROTOCOL.txt',R/'PROTOCOL.txt')
    for name in ('validate_score.py','inspect_failure.py','final_integrity.py'):
        s=(OLDH/name).read_text().replace(O.name,R.name).replace('ON_saved_inputs_ordinary','ON_saved_inputs_CPK9')
        if name!='final_integrity.py':
            needle="assert pre['selected_env']['FL4Q_ENABLE']=='0' and pre['selected_env']['FL4M_RETIREMENT']=='remove'"
            assert needle in s
            s=s.replace(needle,needle+" and pre['selected_env']['CPK9_PRIOR_REFRESH']=='1'")
        ast.parse(s);(R/name).write_text(s)
    # Static preparation checks, distinct from completed CPK9 numerical tests.
    tests=dict(passed=True,xml_parsed=True,python_ast_parsed=True,
               exact_profile=all(sha(p)==sha(O/'kimera_profile'/p.relative_to(R/'kimera_profile'))
                                 for p in (R/'kimera_profile').rglob('*') if p.is_file()),
               unchanged_input_bag=sha(R/'raw_glim_inputs.bag')==sha(O/'raw_glim_inputs.bag'),
               candidate_unit_gate=str(H.parent/'kimera_incremental_prior_cpk9_20261019/validation.json'))
    assert tests['exact_profile'] and tests['unchanged_input_bag']
    (R/'preflight_tests.json').write_text(json.dumps(tests,indent=2))
    (R/'protected_before.json').write_text(json.dumps(protected,indent=2))
    manifest={str(p):sha(p) for p in R.rglob('*') if p.is_file()}
    for p in (C/'private_build').iterdir():
        if p.is_file():manifest[str(p)]=sha(p)
    (R/'pre_capture_receipt.json').write_text(json.dumps(dict(manifest=manifest,one_playback=True,profile_unchanged=True),indent=2))
    print(json.dumps(dict(prepared=str(R),protected=len(protected),manifest=len(manifest),source_launches=0)))
if __name__=='__main__':main()
