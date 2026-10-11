"""Independent evo APE verification of all full/common evaluation products."""
import json
from pathlib import Path
import numpy as np
from evo.core import metrics
from evo.tools import file_interface

root = Path(__file__).resolve().parent
report = json.loads((root / 'summary.json').read_text())
results = {}
for name, evaluation in report['results'].items():
    for short, key in [('full', 'full_saved_output'), ('common', 'identical_time_comparison')]:
        estimate = file_interface.read_tum_trajectory_file(root / (name+'_'+short+'_estimate_prism.tum'))
        reference = file_interface.read_tum_trajectory_file(root / (name+'_'+short+'_reference_prism.tum'))
        assert np.array_equal(estimate.timestamps, reference.timestamps)
        assert estimate.num_poses == evaluation[key]['rows']
        _, _, scale = estimate.align(reference, correct_scale=False)
        assert scale == 1.0
        metric = metrics.APE(metrics.PoseRelation.translation_part)
        metric.process_data((reference, estimate))
        rmse = float(metric.get_statistic(metrics.StatisticsType.rmse))
        difference = abs(rmse-evaluation[key]['ATE_RMSE_m'])
        assert difference < 1e-9, (name, short, difference)
        results[name+'_'+short] = dict(evo_ATE_RMSE_m=rmse, difference_m=difference, rows=estimate.num_poses)
out = dict(passed=True, cases=len(results), maximum_difference_m=max(v['difference_m'] for v in results.values()), results=results)
with (root / 'evo_verification.json').open('x') as stream:
    json.dump(out, stream, indent=2)
    stream.write('\n')
print(json.dumps(out, indent=2))
