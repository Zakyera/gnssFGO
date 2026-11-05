"""Host-visible owned exits, original integrity and supplemental seal."""
import subprocess
from common import *
assert not (H/'complete.json').exists() and not (H/'closeout.json').exists()
assert read(H/'report_verification.json')['passed'] and (H/'REPORT.txt').is_file()
assert read(H/'replay_complete.json')['passed'] and read(H/'container_restore.json')['passed']
inputs=check(read(H/'input_sha256.json'));evaluation=check(read(H/'evaluation/input_sha256.json'))
for m in MODES[1:]:assert sha(H/(m+'_events.csv'))==read(H/'prepared.json')['variant_sha256'][m]
protected=check(read(P/'protected_manifest.json'));assert protected==8873
old={str(root):check(read(root/'complete.json')['output_sha256']) for root in (S,D)};assert old[str(S)]==383
original=tree(R/'receiver/archive');assert original==read(H/'original_archive_sha256.json')
archives={}
for mode in MODES:
    entries=tree(H/mode/'archive');assert entries.keys()==original.keys()
    if mode=='baseline':assert entries==original
    save(mode+'_archive_sha256.json',entries)
    archives[mode]=dict(files=len(entries),digest=tree_digest(entries),manifest=str(H/(mode+'_archive_sha256.json')))
    assert read(H/(mode+'_exit.json'))['exit']==0
identities=[]
records=[read(H/'replay_start.json')['identity']]+[read(H/(mode+'_loaded.json'))['host_identity'] for mode in MODES]+[read(H/(mode+'_process.json'))['identity'] for mode in MODES]
for rec in records:
    p=Path('/proc')/str(rec['pid'])
    if p.exists():assert int((p/'stat').read_text().rsplit(')',1)[1].split()[19])!=rec['proc_start'],rec
    identities.append(dict(**rec,original_identity_exited=True))
states=subprocess.check_output(['docker','inspect','-f','{{.Name}} {{.State.Status}}','cbsms_ws_gpu','ubuntu-jammy-humble-nvidia'],text=True).splitlines()
assert states==['/cbsms_ws_gpu exited','/ubuntu-jammy-humble-nvidia exited']
hesai=subprocess.check_output(['/usr/bin/python3','-B',str(E/'checkpoints/20261023_hesai_four_sequences/verify_checkpoint.py'),'--originals'],text=True,timeout=180)
assert '667 saved files; originals=True' in hesai
save('closeout.json',dict(passed=True,at=now(),input_pins=inputs,evaluation_pins=evaluation,protected_files=protected,old_terminal_outputs=old,original_archive_files=len(original),output_archives=archives,owned_processes=identities,containers=states,checkpoint=hesai.strip(),source_runs=0,receiver_replays=4,new_EVO_calls=48,no_global_changes=True))
outputs={}
for p in sorted(H.rglob('*')):
    if 'archive' in p.relative_to(H).parts:continue
    mode=p.lstat().st_mode;assert stat.S_ISREG(mode) or stat.S_ISDIR(mode),p
    if stat.S_ISREG(mode):outputs[str(p)]=sha(p)
save('complete.json',dict(passed=True,task='CP-V110',at=now(),source_runs=0,receiver_replays=4,output_sha256=outputs,archive_manifests=archives,old_evidence_preserved=True,scope='Regional late mean/covariance ablations on saved CP108 ON; not new live data or a production fix'))
print(json.dumps(dict(passed=True,protected=protected,outputs=len(outputs),archives=archives,checkpoint=hesai.strip(),containers=states),indent=2))
