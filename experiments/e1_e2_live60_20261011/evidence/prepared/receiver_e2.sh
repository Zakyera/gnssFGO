#!/bin/bash
set -eu
export E2_EXACT_LINEARIZATION_CACHE=1
exec /home/yeranis/results/gnssfgo_e1_e2_correctness_audit_20261011/candidate/replay "$@"
