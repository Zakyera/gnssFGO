"""Read-only reuse of the validated nested-rollback lifecycle checker."""
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       'e1_e2_live160_validation_20261011'))
from lifecycle_contract import check_kimera

ROOT = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/'
            'gnssfgo_eth1_fixed_lag_live120_fl4g_20261015')
log = ROOT / ('prepared/pilot120_on_source/ros_logs/'
              'b72a69ce-c816-11f1-a6aa-267c7c544b29/'
              'kimera_vio_ros-kimera_vio_ros_node-4-stdout.log')
result = check_kimera(log.open())
with (ROOT/'audit_fl4h/lifecycle_validation.json').open('x') as f:
    json.dump(result, f, indent=2)
print({k: v for k, v in result.items() if k not in ['rollbacks', 'removals']})
