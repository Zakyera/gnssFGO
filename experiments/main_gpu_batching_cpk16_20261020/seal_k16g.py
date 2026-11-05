"""Seal the prelaunch fixture after correcting a diagnostic count expectation."""
from pathlib import Path
import difflib
import hashlib
import json
H = Path(__file__).resolve().parent
P = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/main_gpu_batching_cpk16_20261020/k16g_source_1")
assert not (P / "build_1").exists() and not (P / "paired").exists()
assert not (P / "prelaunch_count_note.txt").exists()
manifest = json.loads((P / "prepared.json").read_text())
changes = json.loads((P / "extension_diff.json").read_text())
old = (H / "validate_k16f.py").read_text()
new = (P / "validate_k16g.py").read_text()
changes["validate_k16g.py"] = "".join(difflib.unified_diff(
    old.splitlines(True), new.splitlines(True),
    fromfile="K16f/validate_k16g.py", tofile="K16g/validate_k16g.py"))
(P / "extension_diff.json").write_text(json.dumps(changes, indent=2))
for name in ("extension_diff.json", "validate_k16g.py", "PROTOCOL_K16G.txt"):
    manifest["files"][name] = hashlib.sha256((P / name).read_bytes()).hexdigest()
(P / "prepared.json").write_text(json.dumps(manifest, indent=2))
with (P / "prelaunch_count_note.txt").open("x") as f:
    f.write("Before any build/playback, corrected expected retirement updates >900 to >500.\n"
            "One retirement per new scan after the ~50-pose window, not per wrapper call.\n"
            "K16f has143wrapper calls but20retirement updates over71published poses.\n"
            "No observed60s result or numerical tolerance changed; no launch yet.\n")
print("Prelaunch count expectation corrected and manifest sealed")
