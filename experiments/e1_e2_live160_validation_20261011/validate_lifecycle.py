"""Replace only Kimera's incomplete log state machine; retain GLIM checks."""
import fcntl
from pathlib import Path
import sys
from lifecycle_contract import check_kimera
CAPTURE=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live160_20261011')
OUT=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_e1_e2_live160_validation_20261011')
lock=(CAPTURE/'prepared/controller.lock').open('a');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
path=CAPTURE/'check_lifecycle.py';original=path.read_text()
start=original.index('active = {}');end=original.index('gactive = set()')
replacement='''kresult=check_kimera(klog.open())
tracked=kresult['tracked'];removed=kresult['removed_by_update']
active={str(i):None for i in range(kresult['remaining'])}
rollbacks=kresult['rollbacks'];removals=kresult['removals']
guards=Counter()
for line in klog.open():
    for token in ['drop_before_stage','rollback_and_retry_without_staged_cbs',
                  'rollback_remove_active_and_retry','skip_duplicate_receiver_edge']:
        if token in line:guards[token]+=1
'''
code=original[:start]+replacement+original[end:]
code=code.replace("ROOT/(tag+'_lifecycle_validation.json')","OUT/(tag+'_lifecycle_validation.json')")
with (OUT/'lifecycle_corrected.py').open('x') as f:f.write(code)
sys.argv=[str(path),'live160_on']
exec(compile(code,str(path),'exec'),dict(__name__='__main__',__file__=str(path),OUT=OUT,check_kimera=check_kimera))
