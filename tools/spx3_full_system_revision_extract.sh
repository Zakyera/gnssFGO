#!/usr/bin/env bash
set -euo pipefail

# Run inside the existing Noetic container after both matched source runs finish.
products="${1:?usage: extract PRODUCTS_DIRECTORY}"
requested_mode="${2:-both}"
workspace=/workspace/cbs_gtsam4.3
exporter="${workspace}/runs/_eig2_glim_endpoint_revision_audit_20260923/export_cbs_unique_relative_beliefs.py"
export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
source /opt/ros/noetic/setup.bash
source "${workspace}/devel/setup.bash"

case "${requested_mode}" in
  both) modes=(off on) ;;
  off|on) modes=("${requested_mode}") ;;
  *) echo "mode must be off, on, or both" >&2; exit 2 ;;
esac
for mode in "${modes[@]}"; do
  run="${workspace}/runs/spx3_full_system_revision_diagnostics_20260927/spx3_${mode}_classic_revisions_nodelog"
  [[ ( -e "${run}/complete.marker" || -e "${run}/validated_complete.marker" ) && -s "${run}/recordings/cbs.bag" ]] || {
    echo "Incomplete source run: ${run}" >&2; exit 2;
  }
  mkdir -p "${products}/${mode}/glim" "${products}/${mode}/kimera"
  python3 "${exporter}" --bag "${run}/recordings/cbs.bag" \
    --belief-topic /kimera/cbs/odom_belief_in --source-agent g \
    --output-beliefs "${products}/${mode}/glim/published.csv" \
    --output-audit "${products}/${mode}/glim/bag_audit.json"
  python3 "${exporter}" --bag "${run}/recordings/cbs.bag" \
    --belief-topic /kimera/cbs/odom_belief_out --source-agent k \
    --output-beliefs "${products}/${mode}/kimera/published.csv" \
    --output-audit "${products}/${mode}/kimera/bag_audit.json"
  python3 /home/yeranis/repos/gnssFGO/tools/spx3_kimera_odom_record_times.py \
    --bag "${run}/recordings/cbs.bag" \
    --out "${products}/${mode}/kimera/odom_record_times.csv"
done
