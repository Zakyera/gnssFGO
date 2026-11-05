"""Mechanical report-path rebase only; no original artifacts edited."""
from pathlib import Path
Q=Path(__file__).resolve().parent;H=Q.parent
t=(H/'evaluate.py').read_text()
t=t.replace('H=Path(__file__).resolve().parent;E=H.parent','H=Path(__file__).resolve().parent;E=H.parent.parent')
t=t.replace("/H.name/'prepared';S=P/'off_source'","/H.parent.name/'prepared';S=P/'off_source'")
t=t.replace("S/'source_coverage.json'","H/'source_coverage.json'")
t=t.replace("result=dict(passed=True,task='CP-V85'","result=dict(passed=True,original_strict_guard_passed=False,supplemental_endpoint_audit=True,task='CP-V85'")
t=t.replace("'No fusion or CBS ON. Not a full-sequence result or shared-vs-split control comparison.',","'No fusion or CBS ON. Not a full-sequence result or shared-vs-split control comparison.',\n           'Original strict frontend-count check failed; exact saved endpoint audit reconciles601=3startup+597processed+1unsupportedEOF. No source retry.',")
with (Q/'evaluate_saved.py').open('x') as f:f.write(t)
print(Q/'evaluate_saved.py')
