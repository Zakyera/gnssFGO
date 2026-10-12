"""Private S4d mechanical overlay; inherited source/build artifacts read-only."""
from pathlib import Path
import shutil,sys

HERE=Path(__file__).resolve().parent
WS=Path('/workspace/cbs_gtsam4.3')
ROOT=WS/'runs/shared_linearization_s4_provider_20261012'
S3=WS/'runs/shared_linearization_s3_capture_20261012'
S4=WS/'runs/shared_linearization_s4_certificate_20261012/driver'
def swap(s,a,b):
    assert s.count(a)==1,(a,s.count(a));return s.replace(a,b)

def bridge_overlay(s):
    s=swap(s,'#include "ShadowObserver.h"','#include "Provider.h"')
    s=swap(s,'  if (sidecar_required) {','''  const bool native_provider = shared_native::providerEnabled();
  if (native_provider && (mode_ != Mode::ActiveWindowTemporary ||
      outgoing_marginal_source_ != OutgoingMarginalSource::Direct || live_revision_enable_ ||
      outgoing_odom_covariance_mode_ != "schur_relative_between"))
    throw std::runtime_error("shared-native provider unsupported CBS configuration");
  ROS_INFO_STREAM("SHARED_NATIVE_PROVIDER_SETUP enabled=" << native_provider
      << " independent_local=" << (sidecar_required && !native_provider)
      << " revision_covariance=unchanged_main");
  if (sidecar_required && !native_provider) {''')
    s=swap(s,'      shared_native::queryForLog(&smoother, from_key, to_key, from_pose, to_pose);',
      '      if (!shared_native::providerEnabled()) shared_native::queryForLog(&smoother, from_key, to_key, from_pose, to_pose);')
    a=s.index('      const auto joint_info_start = TimingClock::now();',s.index('GlimCbsBridge::getOdometryBeliefsDirectForPairs'))
    b=s.index('      const double from_pose_trace =',a)
    old=s[a:b]
    old=swap(old,'      const auto covariance_result =','      covariance_result =')
    old=swap(old,'      const gtsam::Matrix from_pose_covariance =','      from_pose_covariance =')
    old=swap(old,'      const gtsam::Matrix to_pose_covariance =','      to_pose_covariance =')
    # Original timer was introduced inside the old block but consumed afterwards.
    old=swap(old,'      const auto covariance_start = TimingClock::now();','')
    new='''      const auto covariance_start = TimingClock::now();
      cbs::relative_pose_covariance::Result covariance_result;
      gtsam::Matrix from_pose_covariance, to_pose_covariance;
      if (shared_native::providerEnabled()) {
        shared_native::PublishedCovariance provider;
        const auto status=shared_native::publicationCovariance(&smoother,from_key,to_key,from_pose,to_pose,provider);
        if (!status.ok) {
          ROS_ERROR_STREAM("SHARED_NATIVE_PROVIDER_FAILURE " << status.message);
          throw std::runtime_error(status.message);
        }
        covariance_result=provider.relative;
        from_pose_covariance=provider.endpoints.joint_current.topLeftCorner(6,6);
        to_pose_covariance=provider.endpoints.joint_current.bottomRightCorner(6,6);
        ++n_marginal_pose_pairs;
        const auto observer=shared_native::lookup(&smoother);
        ROS_INFO_STREAM("SHARED_NATIVE_PROVIDER_QUERY epoch=" << observer->history().epoch()
          << " from=" << from_key << " to=" << to_key
          << " admitted=" << observer->history().admittedCount()
          << " active=" << observer->history().activeCount()
          << " absorbed=" << observer->history().absorbedCount()
          << " removed=" << observer->history().removedCount()
          << " resets=" << observer->resets()
          << " query_ms=" << timing_elapsed_ms(covariance_start)
          << " matrix=" << cbs::relative_pose_covariance::matrixToken(covariance_result.covariance));
      } else {
'''+old+'      }\n'
    s=s[:a]+new+s[b:]
    # Guard frozen revision implementation by literal identity.
    reference=(S3/'release_1/overlay/cbs_bridge.cpp').read_text()
    start='  if (export_endpoint_audit_enable_ || live_revision_enable_) {'
    end='void GlimCbsBridge::'
    def revision(t):
        i=t.index(start,t.index('void GlimCbsBridge::onSmootherUpdateFinish'))
        return t[i:t.index(end,i)]
    assert revision(s)==revision(reference),'revision path changed'
    return s

def main():
    author=ROOT/'authoring'
    if not author.exists():
        shutil.copytree(S3/'release_1/source',author)
        shutil.copy2(S4/'candidate.cpp',author/'NativeGaussianHistory.cpp')
        for name in ['RankCertificate.h','Timing.h']:shutil.copy2(S4/name,author/name)
        shutil.copy2(HERE/'Provider.h',author/'Provider.h')
        s=(author/'ShadowObserver.cpp').read_text()
        s='#include "Provider.h"\n'+s
        s=swap(s,'  if (!flag || std::strcmp(flag,"1")!=0) return;',
          '  if ((!flag || std::strcmp(flag,"1")!=0) && !providerEnabled()) return;')
        (author/'ShadowObserver.cpp').write_text(s)
        s='#include "Provider.h"\n'+(author/'test_lifecycle.cpp').read_text()+(HERE/'provider_tests.inc').read_text()
        (author/'test_lifecycle.cpp').write_text(s)
    base=(S3/'driver/base_build.py').read_text()
    base=swap(base,"ROOT=WS/'runs/shared_linearization_s3_20261012'",'ROOT=Path('+repr(str(ROOT))+')')
    base=swap(base,"    (overlay/'cbs_bridge.cpp').write_text(bridge)","    bridge=bridge_overlay(bridge)\n    (overlay/'cbs_bridge.cpp').write_text(bridge)")
    scope={'__name__':'s4d_base','bridge_overlay':bridge_overlay}
    exec(compile(base,str(S3/'driver/base_build.py'),'exec'),scope)
    scope['main']()
if __name__=='__main__':main()
