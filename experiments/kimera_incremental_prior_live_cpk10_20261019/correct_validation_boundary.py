"""Preserve the failed checker; generate a post-only exact-query-window checker."""
import hashlib
import json
from pathlib import Path

H = Path(__file__).resolve().parent
R = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/kimera_incremental_prior_live_cpk10_20261019')
old = R / 'validate_score.py'
new = R / 'validate_score_actual_window.py'
assert not new.exists() and not (H / 'postprocessing_correction.json').exists()
assert (R / 'on_capture/capture_complete.json').exists()
assert not (R / 'on_capture/failed.txt').exists()
assert not (R / 'on_capture/validation.json').exists()

text = old.read_text()
replacements = [
    ('expected=[];source={};scheduled={};received=[];arrival_errors=[]', '''# Exact selected-topic rosbag query start, not first stereo-pair/pose time.
play=json.loads((D/'started.json').read_text())['play_args']
topics=play[play.index('--topics')+1:]
first_records=[]
for bag_path in [x for x in play if x.endswith('.bag')]:
    with rosbag.Bag(bag_path) as bag:
        topic,_,stamp=next(bag.read_messages(topics=topics))
        first_records.append(dict(path=bag_path,topic=topic,record_ns=stamp.to_nsec()))
query_start_ns=min(x['record_ns'] for x in first_records)
duration_ns=int(float(play[play.index('--duration=111')].split('=')[1])*1e9)
assert duration_ns==111000000000
query_end_ns=query_start_ns+duration_ns
assert query_start_ns==1727775007855396309
expected=[];source={};scheduled={};received=[];arrival_errors=[]'''),
    ('if t.to_sec()>=1727775118.955396:continue',
     'if not query_start_ns<=t.to_nsec()<query_end_ns:continue'),
    ("assert len(expected)==1100 and received==expected,'Wire payload loss/reorder/mutation'",
     "assert len(expected)==1099 and received==expected,'Wire payload loss/reorder/mutation'"),
    ('expected_wire=1100,received_wire=len(received)',
     'expected_wire=len(expected),received_wire=len(received)'),
    ('nominal_playback_start_s=1727775007.955396,nominal_playback_end_s=1727775118.955396,',
     'nominal_playback_start_s=query_start_ns*1e-9,nominal_playback_end_s=query_end_ns*1e-9,\n'
     '       exact_query_start_ns=query_start_ns,exact_query_end_ns=query_end_ns,first_selected_records=first_records,\n'
     "       original_validator_failure='Expected1100 from a100ms-late assumed start; actual query schedules1099',"),
    ('native_start_gap_s=on[0][0]*1e-9-1727775007.955396,native_end_gap_s=1727775118.955396-on[-1][0]*1e-9,',
     'native_start_gap_s=(on[0][0]-query_start_ns)*1e-9,native_end_gap_s=(query_end_ns-on[-1][0])*1e-9,'),
]
for before, after in replacements:
    assert text.count(before)==1, before
    text=text.replace(before,after)
compile(text,str(new),'exec')
with new.open('x') as f:
    f.write(text)
receipt=dict(original_checker=str(old),original_sha256=hashlib.sha256(old.read_bytes()).hexdigest(),
             corrected_checker=str(new),corrected_sha256=hashlib.sha256(new.read_bytes()).hexdigest(),
             original_check_passed=False,
             original_failure="AssertionError: Wire payload loss/reorder/mutation (expected1100,recorded1099)",
             evidence='All1099 actual query-window messages are an exact ordered source prefix. '
                      'The1100th message is31.869858ms after the true cutoff. '
                      'Selected front-right image record starts100ms before the assumed timestamp.',
             correction='Postprocessing query boundary only; no numerical/estimator safety gate changed.',
             source_rerun=False,protected_checker_modified=False,capture_modified=False)
with (H/'postprocessing_correction.json').open('x') as f:
    json.dump(receipt,f,indent=2)
print(json.dumps(receipt,indent=2))
