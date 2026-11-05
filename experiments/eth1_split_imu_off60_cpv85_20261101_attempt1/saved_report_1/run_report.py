"""One saved-only evaluator invocation with immutable process/exit receipts."""
import datetime,json,os,subprocess
from pathlib import Path
Q=Path(__file__).resolve().parent
assert json.loads((Q/'source_coverage.json').read_text())['passed']
assert not (Q/'evaluation').exists() and not (Q/'evaluate.log').exists()
cmd=['/home/yeranis/.local/share/pipx/venvs/evo/bin/python','-B',str(Q/'evaluate_saved.py')]
with (Q/'evaluate.log').open('x') as f:
    p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1'))
    with (Q/'evaluate_process.json').open('x') as out:json.dump(dict(pid=p.pid,proc_start=Path(f'/proc/{p.pid}/stat').read_text().split()[21],command=cmd),out,indent=2)
    rc=p.wait(timeout=240)
with (Q/'evaluate_receipt.json').open('x') as f:json.dump(dict(exit=rc,finished=datetime.datetime.now().astimezone().isoformat()),f,indent=2)
print('Evaluator exit',rc)
raise SystemExit(rc)
