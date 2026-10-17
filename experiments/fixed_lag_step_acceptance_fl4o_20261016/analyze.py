"""Reuse FL4m's exact full-coverage interval diagnostic, no new GT convention."""
from pathlib import Path
import json
R=Path('/workspace/cbs_gtsam4.3/runs/fixed_lag_step_acceptance_fl4o_20261016')
OLD=Path('/workspace/cbs_gtsam4.3/runs/fixed_lag_linear_parity_fl4n_20261015')
source=Path('/workspace/cbs_gtsam4.3/runs/fixed_lag_retirement_fl4m_20261015/analyze.py').read_text()
def once(a,b):
    global source
    assert source.count(a)==1,(a,source.count(a));source=source.replace(a,b)
once("R=Path(__file__).resolve().parent",'R=Path('+repr(str(R))+')')
once("for mode in ['remove','retain']:","for mode in ['baseline','candidate']:")
once("    D=R/(mode+'_capture')",'    D=(Path('+repr(str(OLD))+') if mode=="baseline" else R)/"remove_capture"')
source=source.replace("native_cases['remove']","native_cases['baseline']").replace("native_cases['retain']","native_cases['candidate']")
source=source.replace("report['cases']['remove']","report['cases']['baseline']")
namespace={'__file__':str(R/'analyze.py')}
exec(compile(source,'FL4m_reused_analysis','exec'),namespace)
report=namespace['report']
stats=namespace['stats']
summary={'reference':report['reference'],'cases':{}}
for name,c in report['cases'].items():
    rows=[r for r in c['rows'] if 'native_increment_error_m' in r]
    summary['cases'][name]={
        'all_native_increment_translation_m':stats([r['native_increment_error_m'] for r in rows]),
        'all_native_increment_rotation_deg':stats([r['native_increment_error_deg'] for r in rows]),
        'bins':c['bins'],'cost_increase_count':c['cost_increase_count'],
        'update438':c['update438'],'guards':c['guards']}
with (R/'comparison_summary.json').open('x') as f:json.dump(summary,f,indent=2,allow_nan=False)
