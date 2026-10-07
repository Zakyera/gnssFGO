#!/usr/bin/env bash
set -euo pipefail
docker run --rm --network none --read-only --user 1000:1000 --tmpfs /tmp:rw,exec,size=2g \
 -e LD_LIBRARY_PATH=/runtime/lib -e OMP_NUM_THREADS=1 \
 -v /home/yeranis/results/gnssfgo_eth1_wnoa_prediction_audit_20261007:/audit \
 -v /home/yeranis/results/gnssfgo_eth1_local_prediction_20261007:/previous:ro \
 -v /home/yeranis/results/gnssfgo_synthetic_uncertainty_20261004/baseline/runtime:/runtime:ro \
 -v /home/yeranis/repos/gnssFGO:/source:ro \
 sha256:6a664509d183e504230b22edfd747dfe1ca5bca8ae6fc34c76154db8ffd1fc8d python3 /audit/run.py "$@"
