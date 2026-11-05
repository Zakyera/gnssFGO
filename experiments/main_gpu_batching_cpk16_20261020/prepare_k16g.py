"""Mechanical extension of the passed observer/harness, in a fresh root only."""
from pathlib import Path
import difflib
import hashlib
import json
import shutil
import subprocess

H = Path(__file__).resolve().parent
W = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
R = W / "runs/main_gpu_batching_cpk16_20261020"
F = R / "k16f_source_2"
P = R / "k16g_source_1"

def sha(path):
    with Path(path).open("rb") as f:
        return hashlib.file_digest(f, "sha256").hexdigest()

assert not P.exists(), "refuse overwrite"
assert json.loads((H / "RESULT_K16F.json").read_text())["passed"]
assert json.loads((F / "integrity_after.json").read_text())["passed"]
for name in ("protected", "input_hashes"):
    for path, digest in json.loads((R / (name + ".json")).read_text()).items():
        assert sha(path) == digest, path
for name, old in json.loads((R / "source_git.json").read_text()).items():
    repo = W / "src" / name
    current = dict(
        head=subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip(),
        status=subprocess.check_output(["git", "-C", str(repo), "status", "--porcelain"], text=True),
        diff_sha256=hashlib.sha256(subprocess.check_output(
            ["git", "-C", str(repo), "diff", "HEAD", "--binary"])).hexdigest())
    assert current == old, name
original = json.loads((F / "prepared.json").read_text())["files"]
for path, digest in original.items():
    assert sha(F / path) == digest, path
P.mkdir()
for path in original:
    dest = P / path
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(F / path, dest)

changes = {}
def replace(name, substitutions):
    path = P / name
    old = path.read_text()
    new = old
    for before, after in substitutions:
        assert new.count(before) == 1, (name, before)
        new = new.replace(before, after)
    path.write_text(new)
    changes[name] = "".join(difflib.unified_diff(
        old.splitlines(True), new.splitlines(True), fromfile="K16f/" + name,
        tofile="K16g/" + name))

replace("probe_k16f.cpp", [
    ('p.updates<180,"8s diagnostic update limit"', 'p.updates<1400,"60s diagnostic update limit"'),
    ('t-p.first_stamp<8.1,', 't-p.first_stamp<60.1,')])
replace("input_k16f.py", [("8_000_000_000", "60_000_000_000")])
replace("run_source_k16f.py", [
    ("port = 11465", "port = 11466"),
    ('"cpk16f_" + mode', '"cpk16g_" + mode'),
    ("deadline = time.monotonic() + 90", "deadline = time.monotonic() + 900"),
    ("requested_window_s=8", "requested_window_s=60")])
shutil.copy2(H / "validate_k16f.py", P / "validate_k16g.py")
replace("validate_k16g.py", [
    ('P=R/"k16f_source_2"', 'P=R/"k16g_source_1"'),
    ('requested_window_s"]==8', 'requested_window_s"]==60'),
    ('last["retirement_updates"]>0', 'last["retirement_updates"]>500'),
    ('last["pair_queries"]>500', 'last["pair_queries"]>40000'),
    ('last["stamp"]-rows[0]["stamp"]>6.5', 'last["stamp"]-rows[0]["stamp"]>58.5'),
    ('len(native)>=70', 'len(native)>=590'),
    ('(times[-1]-times[0])/1e9>6.8', '(times[-1]-times[0])/1e9>58.8'),
    ('"/k16f_source_2/build_1/probe.so"', '"/k16g_source_1/build_1/probe.so"'),
    ('H/"RESULT_K16F.json"', 'H/"RESULT_K16G.json"')])
shutil.copy2(H / "PROTOCOL_K16G.txt", P / "PROTOCOL_K16G.txt")
with (P / "extension_diff.json").open("x") as f:
    json.dump(changes, f, indent=2)
files = {str(p.relative_to(P)): sha(p) for p in P.rglob("*") if p.is_file()}
with (P / "prepared.json").open("x") as f:
    json.dump(dict(files=files, inherited_root=str(F),
                   scope="K16g duration-only diagnostic extension"), f, indent=2)
print(json.dumps(dict(root=str(P), files=len(files), changes=changes), indent=2))
