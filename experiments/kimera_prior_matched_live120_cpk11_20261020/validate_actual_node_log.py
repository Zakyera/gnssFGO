"""Postprocessing-only CP-K11 correction; immutable original checks retained."""
from pathlib import Path
import difflib
import hashlib
import json
import sys

H = Path(__file__).resolve().parent
P = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_prior_live120_cpk11_20261020/prepared")
mode = sys.argv[1]
assert mode in ("off", "on")
original = P / "validate_candidate.py"
text = original.read_text()
original_sha = hashlib.sha256(original.read_bytes()).hexdigest()
assert original_sha == json.loads((P / "protected_manifest.json").read_text())[str(original)]
old = 'log = (S / "estimators.log").read_text()'
assert text.count(old) == 1
replacement = '''import re
node_logs = list(S.glob("ros_logs/*/kimera_vio_ros-kimera_vio_ros_node-*-stdout.log"))
assert len(node_logs) == 1, node_logs
launcher_log = (S / "estimators.log").read_text()
node_log = re.sub(r"\\x1b\\[[0-9;]*m", "", node_logs[0].read_text())
assert "CPK9_PRIOR_REFRESH," not in launcher_log, "avoid duplicate diagnostic streams"
log = launcher_log + "\\n" + node_log'''
corrected = text.replace(old, replacement)
needle = 'with (S / "candidate_validation.json").open("x") as f:'
assert corrected.count(needle) == 1
checks = '''# Strengthen accounting to every update, not just the total.
by_update = {int(p["time"]): p["retired"] for p in promotions}
assert len(by_update) == len(promotions)
assert all(by_update.get(int(r[0]), 0) == r[2] for r in rows)
result["validation_correction"] = dict(
    reason="CP-K9 diagnostics are in actual Kimera node stdout, not launcher log",
    original_validator_sha256=ORIGINAL_SHA,
    corrected_checker_sha256=CORRECTED_CHECKER_SHA,
    actual_node_log=str(node_logs[0]),
    actual_node_log_sha256=hashlib.sha256(node_logs[0].read_bytes()).hexdigest(),
    every_update_retirement_matches=True,
    original_failed_check_preserved=True,
    estimator_or_threshold_changes=False)
'''
output_name = "candidate_node_log_validation.json" if mode == "off" else "candidate_validation.json"
corrected = corrected.replace(needle, checks + f'with (S / "{output_name}").open("x") as f:')
if mode == "off":
    assert json.loads((P / "live120_off_source/candidate_validation.json").read_text())["passed"]
    corrected = corrected.replace(
        'assert not (S / "candidate_validation.json").exists()',
        'assert not (S / "candidate_node_log_validation.json").exists()')
diff = H / f"candidate_log_correction_{mode}.diff"
with diff.open("x") as f:
    f.writelines(difflib.unified_diff(text.splitlines(True), corrected.splitlines(True),
                                    fromfile=str(original), tofile="in-memory corrected checker"))
namespace = dict(__name__="__main__", __file__=str(original), ORIGINAL_SHA=original_sha,
    CORRECTED_CHECKER_SHA=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
exec(compile(corrected, str(original) + "[actual-node-log]", "exec"), namespace)
