"""Generate a private mechanical overlay; never edit frozen source/build files."""
from pathlib import Path
import hashlib
import json
import shutil

HERE = Path(__file__).resolve().parent
WS = Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
BASE = WS / 'runs/shared_linearization_s4_provider_20261012/release_2'
OUT = WS / 'runs/local_revision_covariance_cpi1_20261019'

def swap(text, old, new):
    assert text.count(old) == 1, (old, text.count(old))
    return text.replace(old, new)

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    assert not OUT.exists(), 'refuse overwrite'
    protected = [p for p in (BASE/'source').rglob('*') if p.is_file()]
    protected += [BASE/'overlay/cbs_bridge.cpp', BASE/'libglim_ros.so',
                  BASE/'libgtsam_points.so.1.2.1']
    snapshot = {str(p): sha(p) for p in protected}
    bridge = (BASE/'overlay/cbs_bridge.cpp').read_text()
    bridge = swap(bridge, '#include "Provider.h"', '#include "RevisionCovariance.h"')
    bridge = swap(bridge, '  const bool native_provider = shared_native::providerEnabled();',
        '  const bool native_provider = shared_native::providerEnabled();\n'
        '  shared_native::validateRevisionCovariance(shared_native::revisionCovariancePolicy(), native_provider);')
    bridge = swap(bridge, '      << " revision_covariance=unchanged_main");',
        '      << " revision_covariance=" << shared_native::revisionCovarianceName(shared_native::revisionCovariancePolicy()));')
    old = '''          const auto joint = smoother.jointMarginalInformation(
              gtsam::KeyVector{from_key, to_key});
          const auto result = cbs::relative_pose_covariance::compute(
              joint, from_key, to_key, from_pose, to_pose, mean,
              outgoing_odom_covariance_mode_);'''
    new = '''          const auto policy = shared_native::revisionCovariancePolicy();
          const auto result = shared_native::revisionCovariance(
              policy, &smoother, from_key, to_key, from_pose, to_pose, [&] {
                const auto joint = smoother.jointMarginalInformation(
                    gtsam::KeyVector{from_key, to_key});
                return cbs::relative_pose_covariance::compute(
                    joint, from_key, to_key, from_pose, to_pose, mean,
                    outgoing_odom_covariance_mode_);
              });'''
    bridge = swap(bridge, old, new)
    bridge = swap(bridge, '            ++it->second.revision;\n            if (export_endpoint_audit_enable_) {',
        '''            ++it->second.revision;
            if (export_endpoint_audit_enable_) {
              ROS_INFO_STREAM("GLIM_CBS_REVISION_COVARIANCE_PROVENANCE_ROW,"
                  << from_index << "," << to_index << "," << it->second.revision
                  << "," << update_index << ","
                  << shared_native::revisionCovarianceName(policy));
            }
            if (export_endpoint_audit_enable_) {''')
    # The normal/classic publication method must remain literally unchanged.
    marker = 'GlimCbsBridge::getOdometryBeliefsDirectForPairs'
    original = (BASE/'overlay/cbs_bridge.cpp').read_text()
    assert bridge[bridge.index(marker):] == original[original.index(marker):]
    OUT.mkdir()
    (OUT/'cbs_bridge.cpp').write_text(bridge)
    for name in ['RevisionCovariance.h','test_revision.cpp','build.py','test_kimera.cpp']:
        shutil.copy2(HERE/name, OUT/name)
    (OUT/'protected.json').write_text(json.dumps(snapshot,indent=2)+'\n')
    print(OUT)

if __name__ == '__main__':
    main()
