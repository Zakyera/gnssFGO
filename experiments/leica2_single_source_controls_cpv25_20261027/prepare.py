"""Create exact source-subsequence journals in fresh diagnostic roots."""
import hashlib, itertools, json, shutil
from collections import Counter
from pathlib import Path

H=Path(__file__).resolve().parent
R=Path('/home/yeranis/results/gnssfgo_leica2_single_source_controls_cpv25_20261027')
P=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_leica2_velodyne_full_cpv4_20261024/prepared')
B=Path('/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/live')
LIB=Path('/home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime/lib')
CASES={'glim_only':'glim_velodyne','kimera_only':'kimera_front'}

def sha(p):
    with Path(p).open('rb') as f:return hashlib.file_digest(f,'sha256').hexdigest()
def save(p,x):
    with p.open('x') as f:json.dump(x,f,indent=2)
def chosen(line,source):
    return not line.startswith(b'B,') or line.split(b',',3)[2]==source.encode()

assert not R.exists()
assert shutil.disk_usage(R.parent).free>8*2**30
assert sha(B)=='4b08a7f5ba97f5e4ec22959df23367df79eebf5d644dc19d939747dcd4566d6e'
R.mkdir()
oldpins=json.loads((H.parent/'leica2_incremental_full_cpv23_20261027/input_pins.json').read_text())
pins={};checked=0
selected=[B,P/'receiver_cpk19.sh',P/'protected_manifest.json',P/'full_off_receiver/events.csv',
          P/'full_off_receiver/resolved_config.json',H/'PROTOCOL.txt',Path(__file__),H/'run.py',
          H.parent/'receiver_finalization_cpk19_20261021/Receiver.h',H.parent/'receiver_finalization_cpk19_20261021/Live.cpp',
          B.parent/'build_live.json']
selected += [P/'full_off_receiver/receiver'/n for n in ('online_states.csv','finalized_states.csv','online_trajectory.tum','finalized_trajectory.tum','summary.json','stream_progress.csv')]
selected += [P/'full_off_source'/n for n in ('glim_odometry.csv','kimera_odometry.csv')]
selected += sorted(p for p in LIB.iterdir() if p.is_file())
for p in selected:
    digest=sha(p)
    if str(p) in oldpins:
        assert digest==oldpins[str(p)],str(p)
        checked+=1
    pins[str(p)]=digest
save(H/'input_pins.json',pins)
sourcefile=P/'full_off_receiver/events.csv'
report={'original_events_sha256':pins[str(sourcefile)],'original_binary_sha256':pins[str(B)],
        'baseline_selected_pin_checks':checked,'fresh_root':str(R),'cases':{}}
for case,source in CASES.items():
    path=R/(case+'_events.csv');mapping=[];counts=Counter();omitted=0
    digest_clock=hashlib.sha256();digest_belief=hashlib.sha256()
    with sourcefile.open('rb') as src,path.open('xb') as dst:
        for i,line in enumerate(src,1):
            assert line[:2] in (b'I,',b'B,',b'E,')
            if chosen(line,source):
                dst.write(line);mapping.append(i);counts[line[:1].decode()]+=1
                (digest_belief if line.startswith(b'B,') else digest_clock).update(line)
            else:omitted+=1
    with sourcefile.open('rb') as src,path.open('rb') as dst:
        for x,y in itertools.zip_longest((line for line in src if chosen(line,source)),dst):assert x==y
    save(R/(case+'_original_lines.json'),mapping)
    assert counts['E']==1
    report['cases'][case]={'source':source,'events':len(mapping),'counts':dict(counts),'removed_other_source_versions':omitted,
        'selected_subsequence_byte_equality':True,'retained_clock_end_sha256':digest_clock.hexdigest(),
        'retained_beliefs_sha256':digest_belief.hexdigest(),'filtered_sha256':sha(path),
        'mapping_sha256':sha(R/(case+'_original_lines.json'))}
assert report['cases']['glim_only']['retained_clock_end_sha256']==report['cases']['kimera_only']['retained_clock_end_sha256']
for p,digest in pins.items():assert sha(p)==digest,p
save(H/'prepared.json',report)
print(json.dumps(report,indent=2))
