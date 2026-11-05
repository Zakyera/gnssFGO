"""Read successful gate receipts; save bounded claims, not performance claims."""
from pathlib import Path
import hashlib
import json
H=Path(__file__).resolve().parent
R=Path("/home/yeranis/repos/V4RL/cbs_gtsam4.3/runs/main_gpu_batching_cpk16_20261020")
def sha(p):
    with Path(p).open("rb") as f:return hashlib.file_digest(f,"sha256").hexdigest()
receipts={str(p):json.loads(p.read_text()) for p in [
  R/"k16b_checks_4/passed.json",R/"k16b_checks_5/passed.json",
  H/"INTEGRITY_K16B_FINAL.json",R/"abi_load.json"]}
assert all(x["passed"] for x in receipts.values())
for gate in ["k16b_checks_4","k16b_checks_5"]:
    receipt=receipts[str(R/gate/"passed.json")]
    assert len({m["geometry_sha256"] for m in receipt["modes"].values()})==1
    for mode in ["disabled","enabled","ubsan"]:
        comp=receipt["modes"][mode]["comparison"]
        assert comp["passed"] and all(x==0 for x in comp["max_absolute_differences"].values())
        for group,c in comp["candidate_counters"].items():
            ref=comp["reference_counters"][group]
            assert c[0]==ref[0],"different number of GPU derivative requests"
            if mode=="disabled" or group=="unattached":assert c==ref
            else:assert c[1]==0 and c[7]==ref[1],"batch consumption mismatch"
main=json.loads((R/"k16b_checks_4/enabled_comparison.json").read_text())
extra=json.loads((R/"k16b_checks_5/enabled_comparison.json").read_text())
assert "PARTIAL sites 1 0 1 1" in (R/"k16b_checks_4/enabled.log").read_text()
assert "selected_cached=1 selected_fresh=4 prepared_hits=4" in (R/"k16b_checks_5/enabled.log").read_text()
assert "selected_cached=3 selected_fresh=0 prepared_hits=0" in (R/"k16b_checks_5/enabled.log").read_text()
result=dict(passed=True,gate="CP-K16b",candidate_sha256=sha(R/"release_1/libgtsam_points.so.1.2.1"),
  actual_wrapper_updates=77,synthetic_arrivals=24,retirement_updates=19,
  slot_reuse_occurrences=37,relative_pair_samples=278,
  first_pair_samples=23,later_pair_samples=255,
  ordinary_and_revision_helpers_agree_exactly=True,
  records_per_comparison=main["records"]+extra["records"],
  numeric_entries_per_comparison=main["numeric_entries"]+extra["numeric_entries"],
  exact_gpu_snapshots_per_comparison=main["counts"]["bytes"]+extra["counts"]["bytes"],
  max_state_matrix_covariance_difference=0,
  wrapper_gpu_derivative_requests=564,wrapper_reference_sync_calls=564,
  wrapper_candidate_sync_calls=0,wrapper_candidate_prepared_hits=564,
  cache_mixed_check=dict(reused=1,fresh=4),cache_only_check=dict(reused=3,fresh=0),
  no_extra_gpu_evaluations_from_local_queries=True,
  timing_performed=False,dataset_run=False,promoted=False,
  limitations=[
    "Bounded synthetic actual-solver histories, not a live GrandTour acceptance run.",
    "Reused GPU kernels and provider/vendor objects are not fully sanitizer-instrumented.",
    "No CUDA-device sanitizer or end-to-end speedup claim.",
    "Source revision-window versus receiver retention mismatch is still separate.",
    "Three invalid pre-validation harness attempts and one compile error preserved; no enabled-candidate numerical failure.",
    "No production source changes or candidate-library rebuild in K16b."],
  receipts=receipts,
  authored_files={p.name:sha(p) for p in H.glob("*") if p.is_file() and p.suffix in (".cpp",".py",".txt")})
with (H/"RESULT_K16B.json").open("x") as f:json.dump(result,f,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ("receipts","authored_files")},indent=2))
