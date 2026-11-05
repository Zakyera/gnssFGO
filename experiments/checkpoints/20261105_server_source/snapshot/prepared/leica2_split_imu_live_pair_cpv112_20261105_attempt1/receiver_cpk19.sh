#!/bin/bash
# Experimental only. Use via bash in a newly authorized harness; not a default.
set -eu
unset WNOJ_STRICT_PUBLICATION WNOJ_INCREMENTAL_PROFILE
export E2_EXACT_LINEARIZATION_CACHE=0 E2B_ENDPOINT_CACHE=0 OMP_NUM_THREADS=1
exec taskset -c 2 /home/yeranis/results/gnssfgo_receiver_finalization_cpk19_20261021/live \
  --absorbed-revision-policy seal_at_absorption "$@"
