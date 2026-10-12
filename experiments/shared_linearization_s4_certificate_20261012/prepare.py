"""Stage immutable S4b inputs; mechanically insert the private check wrapper."""
from pathlib import Path
import difflib
import hashlib
import json
import shutil
HERE=Path(__file__).resolve().parent
REPO=HERE.parent.parent
WS=Path('/home/yeranis/repos/V4RL/cbs_gtsam4.3')
ROOT=WS/'runs/shared_linearization_s4_certificate_20261012'
OLD=WS/'runs/shared_linearization_s4_eigenvalues_20261012'

def main():
    ROOT.mkdir(exist_ok=False);driver=ROOT/'driver';driver.mkdir()
    for n in ['NativeGaussianHistory.h','test_native_gaussian.cpp','rank_tests.cpp','replay.cpp','Timing.h']:
        shutil.copy2(OLD/'driver'/n,driver/n)
    for n in ['RankCertificate.h','certificate_tests.cpp','build.py','run.py','environment.py']:
        shutil.copy2(HERE/n,driver/n)
    original=(OLD/'driver/candidate.cpp').read_text()
    shutil.copy2(OLD/'driver/candidate.cpp',driver/'original.cpp')
    wrapper='''
void certifiedRankCheck(const gtsam::Matrix& h) {
  finiteSymmetric(h);
  require(h.rows()>0,"empty native information");
  rank_certificate::Result r;
  const auto start=diagnostic_timing::Clock::now();
  try { r=rank_certificate::inspect(h); }
  catch (...) { r.certified=false;r.reason=rank_certificate::Reason::Bound; }
  auto* log=diagnostic_timing::output;
  if(log) {
    (*log)["rank.certificate_s"]=std::chrono::duration<double>(diagnostic_timing::Clock::now()-start).count();
    (*log)["rank.certified"]=r.certified; (*log)["rank.fallback"]=!r.certified;
    (*log)["rank.reason"]=int(r.reason); (*log)["rank.dimension"]=h.rows();
    (*log)["rank.factor_nnz"]=r.factor_nnz;
    // Only finite diagnostics are serialized as JSON numbers.
    if(std::isfinite(r.lower)) (*log)["rank.lower"]=r.lower;
    if(std::isfinite(r.target)) (*log)["rank.target"]=r.target;
    if(std::isfinite(r.error)) (*log)["rank.error"]=r.error;
  }
  if(!r.certified) {
    const auto fallback_start=diagnostic_timing::Clock::now();
    positiveDefinite(h); // exact S4b code, same failure reason
    if(log) (*log)["rank.fallback_s"]=std::chrono::duration<double>(diagnostic_timing::Clock::now()-fallback_start).count();
  }
}
'''
    assert original.count('bool sameChart(')==1
    assert original.count('  positiveDefinite(h);')==1
    candidate=original.replace('  positiveDefinite(h);','  certifiedRankCheck(h);')
    candidate=candidate.replace('bool sameChart(',wrapper+'bool sameChart(')
    candidate='#include "RankCertificate.h"\n#include "Timing.h"\n'+candidate
    (driver/'candidate.cpp').write_text(candidate)
    (HERE/'candidate.patch').write_text(''.join(difflib.unified_diff(original.splitlines(True),candidate.splitlines(True),fromfile='S4b/candidate.cpp',tofile='S4c/candidate.cpp')))
    helpers=original[original.index('void require('):original.index('bool sameChart(')]
    access='#include "candidate.cpp"\nnamespace shared_native {\nvoid probeRank(const gtsam::Matrix& h) { certifiedRankCheck(h); }\n}\n'
    access+='namespace original_rank {\n'+helpers+'}\n'
    (driver/'rank_access.cpp').write_text(access)
    pins={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in driver.iterdir()}
    (ROOT/'input_hashes.json').write_text(json.dumps(pins,indent=2))
    print(ROOT)
if __name__=='__main__':main()
