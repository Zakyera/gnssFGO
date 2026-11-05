"""Read-only forecast for a future full matched pair; never launches or deletes."""
import json,math,shutil,subprocess
from pathlib import Path
H=Path(__file__).resolve().parent;W=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
old=W/'runs/gnssfgo_eig2_velodyne_full_cpv4_20261024/prepared';gib=1024**3
sizes={}
for mode in ('off','on'):
    paths=[old/f'full_{mode}_{part}' for part in ('source','receiver','control')]
    output=subprocess.check_output(['du','-s','-B1',*map(str,paths)],text=True)
    sizes[mode]={line.split('\t',1)[1]:int(line.split('\t',1)[0]) for line in output.splitlines()}
totals={mode:sum(v.values()) for mode,v in sizes.items()}
budgets={mode:math.ceil(size*1.25) for mode,size in totals.items()}
free=shutil.disk_usage(W).free
minimum_no_margin=max(8*gib+totals['off'],5*gib+sum(totals.values()))
required=max(8*gib+budgets['off'],5*gib+sum(budgets.values()))
result=dict(passed=free>=required,current_free_bytes=free,original_actual_allocated_bytes=sizes,
    original_mode_totals=totals,forecast_mode_budgets_25_percent_margin=budgets,
    minimum_without_growth_margin=minimum_no_margin,required_free_bytes=required,
    additional_bytes_required=max(0,required-free),predicted_free_before_ON_without_growth_margin=free-totals['off'],
    launch_reserve_bytes=8*gib,runtime_reserve_bytes=5*gib,
    decision='Full matched pair NOT launched unless forecast passes. No files deleted or moved.')
with (H/'full_pair_storage_forecast.json').open('x') as f:json.dump(result,f,indent=2)
print(json.dumps(result,indent=2))
