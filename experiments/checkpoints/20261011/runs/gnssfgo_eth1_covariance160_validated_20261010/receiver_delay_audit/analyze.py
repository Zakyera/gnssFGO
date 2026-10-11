"""Read saved timing logs only; does not run any estimator or optimizer."""
import csv
import hashlib
import json
import statistics
from collections import Counter, defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUN = HERE.parent
T0 = 1727775008003475000
CASES = {
    "off": RUN.parent / "gnssfgo_eth1_covariance160_20261010/prepared/full160_off_receiver/receiver",
    "on": RUN / "prepared/full160_on_receiver/receiver",
}


def read(path):
    with path.open() as stream:
        return list(csv.DictReader(stream))


def stats(values):
    values = sorted(float(x) for x in values)
    if not values:
        return {"n": 0}
    k = (len(values) - 1) * .95
    i = int(k)
    p95 = values[i] + (values[min(i + 1, len(values) - 1)] - values[i]) * (k - i)
    return dict(n=len(values), total=sum(values), mean=statistics.mean(values),
                median=statistics.median(values), p95=p95, maximum=values[-1])


protected = json.loads((RUN / "prepared/protected_manifest.json").read_text())
protected.update(json.loads((RUN / "additional_protected_evidence.json").read_text()))
for path, digest in protected.items():
    assert hashlib.sha256(Path(path).read_bytes()).hexdigest() == digest, path

result = {"scope": "read-only saved-log/code investigation; no optimization or source rerun",
          "protected_entries_unchanged": len(protected), "cases": {}, "input_sha256": {}}
for mode, root in CASES.items():
    data = {}
    for name in ["publication_convergence.csv", "publication_steps.csv", "revision_events.csv", "causal_dispatch.csv"]:
        data[name] = read(root / name)
        result["input_sha256"][str(root / name)] = hashlib.sha256((root / name).read_bytes()).hexdigest()
    convergence = [r for r in data["publication_convergence.csv"] if r["phase"] == "support"]
    by_index = defaultdict(list)
    for row in data["publication_steps.csv"]:
        if row["phase"] == "support":
            by_index[int(row["index"])].append(row)
    revisions, dispatch = data["revision_events.csv"], data["causal_dispatch.csv"]
    assert len(revisions) == len(dispatch)
    for a, b in zip(revisions, dispatch):
        assert all(a[k] == b[k] for k in ["from_index", "to_index", "revision", "outcome"])
    windows = []
    for lo, hi in [(0, 60), (60, 100), (100, 115), (115, 120), (120, 125), (125, 127.8)]:
        c = [r for r in convergence if lo <= (int(r["physical_ns"]) - T0) / 1e9 < hi]
        rd = [(r, d) for r, d in zip(revisions, dispatch)
              if lo <= (int(r["record_timestamp_ns"]) - T0) / 1e9 < hi]
        steps = [s for r in c for s in by_index[int(r["index"])]]
        windows.append(dict(
            seconds=[lo, hi], refinement_wall_s=stats(r["seconds"] for r in c),
            attempts=stats(r["attempted"] for r in c),
            recovery_supports=sum(any(s["recovery"] == "1" for s in by_index[int(r["index"])]) for r in c),
            qr_rebuilds=sum(s["recovery"] == "1" for s in steps),
            private_polish_passes=sum(s["local_prediction"] == "1" and s["recovery"] == "1" for s in steps),
            replacement_wall_s=stats(r["update_seconds"] for r, _ in rd),
            source_versions=dict(Counter(d["source"] for _, d in rd))))
    recovery = [r for r in convergence if any(s["recovery"] == "1" for s in by_index[int(r["index"])])]
    polish = [r for r in convergence if any(s["local_prediction"] == "1" for s in by_index[int(r["index"])])]
    result["cases"][mode] = dict(
        windows=windows, first_recovery=recovery[0] if recovery else None,
        first_private_polish=polish[0] if polish else None,
        slowest_supports=sorted(convergence, key=lambda r: float(r["seconds"]), reverse=True)[:10],
        recorded_versions=len(revisions), inserted_ids=sum(r["outcome"] == "insert" for r in revisions),
        maximum_logged_active_factors=max(int(r["active_factors"]) for r in revisions))
    if mode == "on":
        failure = [r for r in convergence if r["status"] != "stationary"]
        assert len(failure) == 1
        steps = by_index[int(failure[0]["index"])]
        normal = [r for r in steps if r["recovery"] == "0"]
        private = [r for r in steps if r["recovery"] == "1" and r["local_prediction"] == "1"]
        assert len(normal) == 20 and len(private) == 15
        assert all(r["qr_disagreement"] == "0" for r in private)
        assert all(r["relinearized"] == "3819" for r in normal)
        result["failure"] = dict(
            convergence=failure[0], steps=steps, estimated_support_count=1273,
            reported_relinearized_keys=3819, logged_active_factors=3155,
            # Counts follow the retained FullPublicationRefinement.h branches;
            # they are NOT subphase wall-time measurements.
            code_path_counts=dict(solver_copies=20, full_qr_rebuilds=16,
                                  normalized_gradient_calls=37, gaussian_cost_change_calls=62,
                                  explicit_full_graph_linearizations_outside_solver=99),
            minimum_trial_gradient=min(float(s["gradient_after"]) for s in steps),
            private_qr_disagreement_max=max(float(s["qr_disagreement"]) for s in private),
            final_trajectory_bytes=(root / "trajectory.tum").stat().st_size,
            full_state_rows=max(0, len((root / "states.csv").read_text().splitlines()) - 1),
            snapshot_directories=[str(p) for p in root.glob("snapshot_*") if p.is_dir()])

result["window_semantics"] = (
    "Publication timings binned by physical support time; replacement timings by recorded arrival time. "
    "These compare the same labelled time sections, not an exact exclusive CPU-time partition. "
    "Replacement timer excludes factor construction, pose queries, logging; publication timer excludes log writes."
)
with (HERE / "evidence.json").open("x") as stream:
    json.dump(result, stream, indent=2)
print(json.dumps({"protected": len(protected), "failure_counts": result["failure"]["code_path_counts"],
                  "first_polish": {k: v["first_private_polish"] for k, v in result["cases"].items()}}, indent=2))
