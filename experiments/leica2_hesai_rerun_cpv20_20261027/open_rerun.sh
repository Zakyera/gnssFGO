#!/bin/bash
set -euo pipefail
exec /home/yeranis/.local/bin/rerun "/home/yeranis/repos/gnssFGO/experiments/leica2_hesai_rerun_cpv20_20261027/leica2_hesai_full_scene.rrd" --bind 127.0.0.1 --port auto --memory-limit 2GB --window-size 1600x1100
