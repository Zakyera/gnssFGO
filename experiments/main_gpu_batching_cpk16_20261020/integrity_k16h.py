"""Write-once before/after receipts for the authorized K16g diagnostic."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys

W = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
R = W / "runs/main_gpu_batching_cpk16_20261020"
P = R / "k16h_source_1"
phase = sys.argv[1]
assert phase in ("before", "after")
assert not (P / ("integrity_" + phase + ".json")).exists()

def sha(path):
    with Path(path).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()

result = dict(phase=phase, files={}, repositories={})
for name in ("protected", "input_hashes"):
    hashes = json.loads((R / (name + ".json")).read_text())
    changed = [p for p, h in hashes.items() if sha(p) != h]
    assert not changed, changed
    result["files"][name] = dict(count=len(hashes), changed=changed)
for name, old in json.loads((R / "source_git.json").read_text()).items():
    repo = W / "src" / name
    current = dict(
        head=subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip(),
        status=subprocess.check_output(["git", "-C", str(repo), "status", "--porcelain"], text=True),
        diff_sha256=hashlib.sha256(subprocess.check_output(
            ["git", "-C", str(repo), "diff", "HEAD", "--binary"])).hexdigest())
    assert current == old, name
    result["repositories"][name] = dict(unchanged=True)
files = json.loads((P / "prepared.json").read_text())["files"]
for path, digest in files.items():
    assert sha(P / path) == digest, path
assert sha(R / "release_1/libgtsam_points.so.1.2.1") == \
    "6f71292d29d974a0555abd9fda6b2370fdab81ca634a4e1343c216d2a762d2ca"
assert json.loads((R / "k16g_source_1/smoke/result.json").read_text())["passed"]
observer = sha(R / "k16g_source_1/build_1/probe.so")
if phase == "before":
    assert not (P / "paired").exists(), "refuse duplicate source launch"
else:
    assert observer == json.loads((P / "integrity_before.json").read_text())["observer_sha256"]
result.update(prepared_files=len(files), observer_sha256=observer, passed=True)
with (P / ("integrity_" + phase + ".json")).open("x") as f:
    json.dump(result, f, indent=2)
print(json.dumps(result, indent=2))

