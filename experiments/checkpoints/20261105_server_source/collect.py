"""Archive reviewed text inputs without executing any historical experiment.

Run once with ``collect``, then ``verify``. Never imports archived source modules.
This is a source checkpoint, not a claim of a self-contained clean build.
"""
import ast
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import subprocess
import sys

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
EXPERIMENTS = REPO / "experiments"
WORKSPACE = Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3")
CODE = {".py", ".sh", ".cpp", ".cc", ".c", ".h", ".hh", ".hpp", ".inc", ".cu", ".cuh", ".cmake", ".patch", ".diff"}
CONFIG = {".yaml", ".yml", ".flags", ".launch", ".repos", ".toml"}
MAX_BYTES = 8 * 1024 * 1024
SENSITIVE = [
    ("private_key", re.compile(rb"-----BEGIN (?:RSA |EC |OPENSSH |DSA )?PRIVATE KEY-----")),
    ("github_token", re.compile(rb"(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{40,})")),
    ("aws_access_key", re.compile(rb"\b(?:AKIA|ASIA)[A-Z0-9]{16}\b")),
    ("credential_in_url", re.compile(rb"https?://[^\s/:\"']+:[^\s/@\"']+@")),
    ("slack_token", re.compile(rb"xox[baprs]-[A-Za-z0-9-]{20,}")),
    ("openai_key", re.compile(rb"\bsk-(?:proj-)?[A-Za-z0-9_-]{32,}")),
]


def git(*args):
    return subprocess.check_output(["git", "-C", str(REPO), *args])


def digest(data):
    return hashlib.sha256(data).hexdigest()


def checked_bytes(path):
    mode = path.lstat().st_mode
    if not stat.S_ISREG(mode):
        raise ValueError("not a regular file: " + str(path))
    if path.stat().st_size > MAX_BYTES:
        raise ValueError("oversize: " + str(path))
    data = path.read_bytes()
    if b"\0" in data:
        raise ValueError("binary: " + str(path))
    data.decode("utf-8")
    for label, pattern in SENSITIVE:
        if pattern.search(data):
            raise ValueError("potential secret (" + label + "): " + str(path))
    return data


def save_new(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as handle:
        handle.write(data)


def collect():
    assert not (HERE / "selection.json").exists(), "checkpoint already collected"
    assert not git("diff", "--cached", "--name-only").strip(), "existing staged changes"
    tracked = set(git("ls-files", "-z").decode().split("\0"))
    patterns = ["*" + suffix for suffix in sorted(CODE | CONFIG | {".md"})]
    candidates = set(git("ls-files", "--others", "--exclude-standard", "-z", "--", *patterns).decode().split("\0")) - {""}
    candidates |= set(git("diff", "--name-only", "--diff-filter=AM", "-z").decode().split("\0")) - {""}
    for item in REPO.iterdir():
        if item.is_file() and item.suffix in {".txt", ".md"} and item.name not in tracked:
            candidates.add(item.name)
    # Compact top-level protocols, configurations, results and receipts. Do not
    # walk journals or add the million-file raw output trees to Git.
    omitted = []
    for experiment in sorted(EXPERIMENTS.iterdir()):
        if not experiment.is_dir() or experiment.is_symlink() or experiment.name == "checkpoints":
            continue
        for path in sorted(experiment.iterdir()):
            if path.is_symlink() or not path.is_file():
                continue
            if path.suffix not in {".txt", ".json"}:
                continue
            if re.search(r"(?:inputs|protected|pins|preserved|inventory|targets|deleted|archive_sha256|expected_headers|edge_events|window_events|belief_events)", path.name, re.I):
                omitted.append({"path": str(path.relative_to(REPO)), "reason": "bulk provenance/data; retained locally"})
                continue
            if path.stat().st_size > 1024 * 1024:
                omitted.append({"path": str(path.relative_to(REPO)), "reason": "large generated record; retained locally"})
                continue
            if str(path.relative_to(REPO)) not in tracked:
                candidates.add(str(path.relative_to(REPO)))

    archived = []

    def archive(path, destination):
        data = checked_bytes(path)
        target = HERE / "snapshot" / destination
        save_new(target, data)
        archived.append({"source": str(path), "path": str(target.relative_to(REPO)), "sha256": digest(data), "bytes": len(data)})
        candidates.add(str(target.relative_to(REPO)))

    def archive_tree(root, label, extra=set()):
        for directory, subdirs, files in os.walk(root, followlinks=False):
            subdirs[:] = sorted(d for d in subdirs if d not in {".git", "__pycache__"} and not (Path(directory) / d).is_symlink())
            for name in sorted(files):
                path = Path(directory) / name
                if path.is_symlink():
                    continue
                if path.suffix in CODE | CONFIG or name in extra:
                    archive(path, Path(label) / path.relative_to(root))

    # Qualified private source copies; do not change canonical package sources.
    for name, subdir in [
        ("prior_anchor_candidate_cpv101_20261103", "candidate"),
        ("prior_anchor_integration_cpv102_20261103", "candidate"),
    ]:
        archive_tree(WORKSPACE / "runs" / name / subdir, Path("overlays") / name / subdir)
    for name in ["persistent_compatibility_cpv76_20261101", "glim_persistent_compat_cpv82_20261101"]:
        root = WORKSPACE / "runs" / name
        for path in sorted(root.iterdir()):
            if not path.is_symlink() and path.is_file() and path.suffix in CODE:
                archive(path, Path("overlays") / name / path.name)
        if (root / "include").exists():
            archive_tree(root / "include", Path("overlays") / name / "include")
    integration = WORKSPACE / "runs/prior_anchor_integration_cpv102_20261103"
    archive(integration / "archive_integration.cpp", Path("overlays") / integration.name / "archive_integration.cpp")
    for name in ["VioBackend.d", "PriorFailureArchive.d"]:
        archive(integration / "private_build" / name, Path("build_provenance") / integration.name / name)
    for name in ["baseline", "candidate", "tests_v2"]:
        for path in sorted((EXPERIMENTS / integration.name / name).glob("*command.json")):
            archive(path, Path("build_provenance") / integration.name / name / path.name)
    compatibility = WORKSPACE / "runs/persistent_compatibility_cpv76_20261101"
    for relative in ["backend_fixture1/commands.json", "release_fixture1/compatibility_compile_command.json", "release_fixture1/compatibility_link_command.json"]:
        archive(compatibility / relative, Path("build_provenance") / compatibility.name / relative)
    glim = WORKSPACE / "runs/glim_persistent_compat_cpv82_20261101"
    for path in sorted(glim.glob("release_*/commands.json")):
        archive(path, Path("build_provenance") / glim.name / path.relative_to(glim))
    receiver = Path("/home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021")
    for name in ["build_live.json", "build_focused.json", "build_legacy.json", "validation.json"]:
        archive(receiver / name, Path("build_provenance/receiver_cpk19") / name)

    # Actual prepared launch/profile bytes for the latest three matched pairs
    # and corrected ETH1 ON. Source/receiver output directories are excluded.
    run_names = [
        "eth1_prior_fix_live_cpv103_20261103",
        "eig2_split_imu_live_pair_cpv108_20261104",
        "spx3_split_imu_live_pair_cpv111_20261104",
        "leica2_split_imu_live_pair_cpv112_20261105_attempt1",
    ]
    for name in run_names:
        root = WORKSPACE / "runs" / name / "prepared"
        for path in sorted(root.iterdir()):
            if path.is_file() and not path.is_symlink() and (path.suffix in CODE | CONFIG or path.name in {"full_on.json", "full_off.json", "playback_window.json", "input_audit.json", "preflight.json"}):
                archive(path, Path("prepared") / name / path.name)
        for dirname in ["off", "on", "transport_tools"]:
            if (root / dirname).is_dir():
                archive_tree(root / dirname, Path("prepared") / name / dirname, {"config.json", "BackendParams.yaml", "ImuParams.yaml"})
                # GLIM configuration is JSON, not code.
                for path in sorted((root / dirname).rglob("*.json")):
                    target = HERE / "snapshot/prepared" / name / path.relative_to(root)
                    if not path.is_symlink() and not target.exists() and path.stat().st_size <= 1024 * 1024:
                        archive(path, Path("prepared") / name / path.relative_to(root))

    selected = []
    for relative in sorted(candidates):
        path = REPO / relative
        if path.is_symlink() or not path.is_file():
            omitted.append({"path": relative, "reason": "non-regular; retained locally"})
            continue
        if path.stat().st_size > MAX_BYTES:
            omitted.append({"path": relative, "reason": "oversize generated diff; retained locally"})
            continue
        data = checked_bytes(path)
        selected.append({"path": relative, "sha256": digest(data), "bytes": len(data)})
    record = dict(schema=1, scope="source/configuration/compact evidence checkpoint; no clean-build claim", files=selected, archived_originals=archived, omitted=omitted)
    save_new(HERE / "selection.json", (json.dumps(record, indent=2) + "\n").encode())
    print(json.dumps(dict(files=len(selected), bytes=sum(x["bytes"] for x in selected), archived_originals=len(archived), omitted=len(omitted))))


def verify():
    record = json.loads((HERE / "selection.json").read_text())
    python_count = 0
    json_count = 0
    syntax_notes = []
    for entry in record["files"]:
        path = REPO / entry["path"]
        data = checked_bytes(path)
        assert digest(data) == entry["sha256"], entry["path"]
        if path.suffix == ".py":
            try:
                ast.parse(data, filename=str(path))
            except SyntaxError as error:
                # Historical rejected/preparation sources stay unchanged. Report
                # invalid syntax rather than silently repairing old evidence.
                syntax_notes.append({"path": entry["path"], "line": error.lineno, "message": error.msg})
            python_count += 1
        if path.suffix == ".json":
            try:
                json.loads(data)
            except (ValueError, UnicodeError) as error:
                syntax_notes.append({"path": entry["path"], "message": str(error)})
            json_count += 1
    for entry in record["archived_originals"]:
        assert digest(checked_bytes(Path(entry["source"]))) == entry["sha256"], entry["source"]
    result = dict(hash_parity=True, secret_pattern_scan="no matches; limited pattern scan, not a comprehensive security audit", files=len(record["files"]), python_parsed=python_count, json_parsed=json_count, syntax_notes=syntax_notes, builds_or_experiments_executed=0)
    print(json.dumps(result, indent=2))
    if not (HERE / "verification.json").exists():
        save_new(HERE / "verification.json", (json.dumps(result, indent=2) + "\n").encode())


if __name__ == "__main__":
    assert len(sys.argv) == 2 and sys.argv[1] in {"collect", "verify"}
    {"collect": collect, "verify": verify}[sys.argv[1]]()
