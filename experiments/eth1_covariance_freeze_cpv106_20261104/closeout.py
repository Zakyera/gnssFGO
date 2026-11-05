"""Final integrity/owned-exit checks and new immutable supplemental seal."""
import os,subprocess
from common import *
assert not (H/'complete.json').exists() and not (H/'closeout.json').exists()
assert read(H/'report_verification.json')['passed'] and (H/'REPORT.txt').is_file()
assert read(H/'replay_complete.json')['passed'] and read(H/'container_restore.json')['passed']
inputs=check(read(H/'input_sha256.json'));evaluation=check(read(H/'evaluation/input_sha256.json'))
assert sha(H/'variant_events.csv')==read(H/'prepared.json')['variant_events_sha256']
protected=check(read(P/'protected_manifest.json'));assert protected==8120
old={}
for name,want in [('eth1_prior_fix_live_cpv103_20261103',62),
                  ('eth1_prior_fix_evaluation_cpv104_20261104',318),
                  ('eth1_fusion_diagnosis_cpv105_20261104',12)]:
    old[name]=check(read(E/name/'complete.json')['output_sha256']);assert old[name]==want
original=tree(R/'receiver/archive');assert original==read(H/'original_archive_sha256.json')
archives={}
for mode in ('baseline','frozen_kimera'):
    entries=tree(H/mode/'archive')
    assert entries.keys()==original.keys()
    if mode=='baseline':assert entries==original
    save(mode+'_archive_sha256.json',entries)
    archives[mode]=dict(files=len(entries),digest=tree_digest(entries),manifest=str(H/(mode+'_archive_sha256.json')))
    assert read(H/(mode+'_exit.json'))['exit']==0
identities=[]
for record in [read(H/'replay_start.json')['identity']]+[read(H/(mode+'_loaded.json'))['host_identity'] for mode in ('baseline','frozen_kimera')]:
    p=Path('/proc')/str(record['pid'])
    if p.exists():
        start=int((p/'stat').read_text().rsplit(')',1)[1].split()[19])
        assert start!=record['proc_start'],record
    identities.append(dict(**record,original_identity_exited=True))
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert states==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited']
checkpoint=E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'
hesai=subprocess.check_output(['/usr/bin/python3','-B',str(checkpoint),'--originals'],text=True,timeout=180)
assert '667 saved files; originals=True' in hesai
save('closeout.json',dict(passed=True,finished=now(),input_pins=inputs,evaluation_pins=evaluation,
    protected_files=protected,old_terminal_outputs=old,original_archive_files=len(original),
    output_archives=archives,owned_processes=identities,containers=states,checkpoint=hesai.strip(),
    source_runs=0,receiver_replays=2,new_EVO_calls=24,no_global_changes=True))
outputs={}
for p in sorted(H.rglob('*')):
    if 'archive' in p.relative_to(H).parts:continue
    mode=p.lstat().st_mode
    assert stat.S_ISREG(mode) or stat.S_ISDIR(mode),p
    if stat.S_ISREG(mode):outputs[str(p)]=sha(p)
save('complete.json',dict(passed=True,task='CP-V106',finished=now(),source_runs=0,receiver_replays=2,
    output_sha256=outputs,archive_manifests=archives,old_evidence_preserved=True,
    scope='Conditional covariance-revision intervention on saved CP103 ON; not a new live OFF/ON or production fix'))
print(json.dumps(dict(passed=True,protected=protected,outputs=len(outputs),archives=archives,
                     checkpoint=hesai.strip(),containers=states),indent=2))
