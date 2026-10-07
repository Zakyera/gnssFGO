# Authorized bounded 120-second CBS-ON check

User authorization: "Ok let’s try also 120sec of cbs on and check".

Use ETH1 mission 2024-10-01-11-29-55, bag offset 0, duration 120 s, playback 0.1x. Reuse the exact source.env, protected_manifest_v2.json, native profiles, WNOA executable/runtime and strict nominal publication settings from the current experiment. Only capture duration, run labels and source session identifiers change. Classic source CBS ON; downstream stable-ID revision replacement. No central IMU factors, tuning, covariance changes, graph inspectors or visualization in the critical path.

The previous full OFF trial remains a stopped throughput-failed experiment. The previous automation remains paused. Do not start full ON or another OFF run. Store this capture under bounded120_on_source, bounded120_on_receiver and bounded120_on_control; refuse existing directories.

run_bounded120_case.py is a scoped copy of run_case.py: phase/mode choices are restricted to bounded120/on, the existing smoke checks remain mandatory and capture duration is 120 s. It does not change estimators or receiver transport. After capture use the original validate_case.py bounded120_on: frozen settings, payload conservation, causal accounting, stationary publications, unchanged throughput thresholds and exact journal replay must pass.

Use the copied score_bounded120_case.py and score_bounded120_common.py with phase bounded120. Their only changes are the phase selector, selecting ON only, and four expected trajectories instead of eight. Evaluate original/native GLIM, original/native Kimera ROS odometry, fused online and fused final on the same physical interval/mask using unchanged existing evaluators. Keep final retrospective output separate from live output. Do not compare the old 227-second final OFF solution directly with this 120-second final ON solution: their information horizons differ.

Stop and preserve artifacts on a failed gate; no automatic retry or parameter changes. Report measured throughput and evaluation limits without claiming 1x capacity or full-sequence validation.
