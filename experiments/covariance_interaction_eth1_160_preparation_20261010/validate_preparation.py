#!/usr/bin/env python3
"""Validate preparation only; never starts a source, fuser or playback."""
import ast
import json
import subprocess
import sys
from pathlib import Path

from inspect_capture import sha

ROOT = Path(__file__).resolve().parent
BASE = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/gnssfgo_eth1_native_mean_alignment_20261010/gpu_restored")
GLIM_ROS = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/src/glim_ros1")


def main():
    out = ROOT / "preparation_validation.json"
    if out.exists():
        raise ValueError("refuse overwrite")
    manifest_checks = {}
    for name in ["protected_manifest.json", "acceptance_manifest.json"]:
        manifest = json.loads((BASE / name).read_text())
        for path, expected in manifest.items():
            if sha(Path(path)) != expected:
                raise ValueError("frozen input changed: " + path)
        manifest_checks[name] = len(manifest)
    for path in ROOT.glob("*.py"):
        ast.parse(path.read_text(), filename=str(path))
    contract = json.loads((ROOT / "capture_contract.json").read_text())
    if contract["status"] != "PREPARATION_ONLY_NOT_AUTHORIZED_TO_LAUNCH":
        raise ValueError("preparation must not claim launch readiness")
    check = subprocess.run(["git", "apply", "--check", str(ROOT / "glim_active_window_audit_hook.patch")],
                           cwd=GLIM_ROS, text=True, capture_output=True, check=True)
    tests = subprocess.run([sys.executable, "-m", "unittest", "-v", "test_inspect_capture.py"],
                           cwd=ROOT, text=True, capture_output=True, check=True)
    inventory = json.loads((ROOT / "retained40_inventory.json").read_text())
    for path, expected in inventory["input_sha256"].items():
        if sha(Path(path)) != expected:
            raise ValueError("retained log changed: " + path)
    with out.open("x") as f:
        json.dump(dict(
            status="PREPARATION_CHECKS_PASS_RUNTIME_GATE_PENDING",
            protected_hash_checks=manifest_checks,
            retained40_input_hashes_unchanged=True,
            tests=tests.stdout + tests.stderr,
            patch_check=check.stdout + check.stderr,
            patch_applied=False, estimator_code_changed=False,
            estimator_or_fusion_run_started=False,
            prepared_files_sha256={p.name: sha(p) for p in sorted(ROOT.iterdir()) if p.is_file() and p != out},
            gnssfgo_head=subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        ), f, indent=2, allow_nan=False)
        f.write("\n")
    print(json.dumps(dict(status="PREPARATION_CHECKS_PASS_RUNTIME_GATE_PENDING", hash_checks=manifest_checks)))


if __name__ == "__main__":
    main()
