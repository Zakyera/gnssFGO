#!/bin/bash
set -eu
unset WNOJ_STRICT_PUBLICATION WNOJ_INCREMENTAL_PROFILE
export E2_EXACT_LINEARIZATION_CACHE=0 E2B_ENDPOINT_CACHE=0 OMP_NUM_THREADS=1
exec taskset -c 2 /home/yeranis/results/gnssfgo_fixed_lag_failure_fl4e_20261014/candidate_live "$@"
