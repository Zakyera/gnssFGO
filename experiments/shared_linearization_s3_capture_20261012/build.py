"""Private mechanical recorder overlay; original S2/S3 inputs stay untouched."""
from pathlib import Path
import importlib.util
import shutil
import sys

HERE=Path(__file__).resolve().parent
WS=Path('/workspace/cbs_gtsam4.3')
ROOT=WS/'runs/shared_linearization_s3_capture_20261012'

def replace(s,a,b):
    assert s.count(a)==1,(a,s.count(a))
    return s.replace(a,b)

def main():
    # This script and sibling sources are copied once to ROOT/driver by host.
    old=WS/'runs/shared_linearization_s3_20261012/release_3/source'
    author=ROOT/'authoring'
    if not author.exists():
        shutil.copytree(old,author)
        shutil.copy2(HERE/'Recording.h',author/'Recording.h')
        h=(author/'ShadowObserver.h').read_text()
        h=replace(h,'  bool valid() const;', '  void setRecording(const char*) noexcept;\n  bool logDue() const;\n  void recordQueryInput(gtsam::Key,gtsam::Key,const gtsam::Pose3&,const gtsam::Pose3&);\n  void recordQueryOutput(const EndpointCovariance&,double);\n  bool valid() const;')
        (author/'ShadowObserver.h').write_text(h)
        s=(author/'ShadowObserver.cpp').read_text()
        s='#include "Recording.h"\n'+s
        s=replace(s,'  NativeGaussianHistory history{Mode::Shadow};','  std::unique_ptr<Recording> recording;\n  NativeGaussianHistory history{Mode::Shadow};')
        s=replace(s,'  p_->valid=false;', '  p_->valid=false;\n  if(p_->recording)p_->recording->failure(p_->history.epoch(),reason);')
        s=replace(s,'    if (p_->audit) p_->audit(s);','    if (p_->recording) p_->recording->stage(s);\n    if (p_->audit) p_->audit(s);')
        s=replace(s,'    p_->begun=false; p_->ready=false;', '    p_->begun=false; p_->ready=false;\n    if(p_->recording)p_->recording->commit(p_->history);')
        s=replace(s,'      p=attach(owner);','      p=attach(owner);\n      if(const char* path=std::getenv("GLIM_SHARED_NATIVE_JOURNAL"))p->setRecording(path);')
        s=replace(s,'  auto p=lookup(owner); if (!p || !p->valid()) return;', '  auto p=lookup(owner); if (!p || !p->valid() || !p->logDue()) return;')
        s=replace(s,'    EndpointCovariance out;\n    const auto result=p->query(from,to,pf,pt,out);','    p->recordQueryInput(from,to,pf,pt);\n    const auto started=std::chrono::steady_clock::now();\n    EndpointCovariance out;\n    const auto result=p->query(from,to,pf,pt,out);')
        s=replace(s,'    if (!result.ok) return;', '    if (!result.ok) return;\n    p->recordQueryOutput(out,Recording::seconds(started));')
        s=replace(s,'ShadowObserver::~ShadowObserver() = default;', '''ShadowObserver::~ShadowObserver() = default;
void ShadowObserver::setRecording(const char* path) noexcept {
  try {
    check(!p_->recording && !p_->begun && !p_->captures,"recording must start before initialization");
    p_->recording=std::make_unique<Recording>(path);
  } catch(const std::exception& e){fail(e.what());}catch(...){fail("recording setup failure");}
}
bool ShadowObserver::logDue() const { return !p_->recording || p_->recording->due(p_->history.epoch()); }
void ShadowObserver::recordQueryInput(gtsam::Key a,gtsam::Key b,const gtsam::Pose3& pa,const gtsam::Pose3& pb) {
  if(p_->recording)p_->recording->queryInput(p_->history,a,b,pa,pb);
}
void ShadowObserver::recordQueryOutput(const EndpointCovariance& c,double seconds) {
  if(p_->recording)p_->recording->queryOutput(p_->history.epoch(),c,seconds);
}''')
        (author/'ShadowObserver.cpp').write_text(s)
        t=(author/'test_lifecycle.cpp').read_text()
        t=replace(t,'  observer->setAudit([&](const Snapshot& s){oracle.capture(s);});',
          '  if(const char* p=std::getenv("SHARED_NATIVE_TEST_JOURNAL"))observer->setRecording(p);\n  observer->setAudit([&](const Snapshot& s){oracle.capture(s);});')
        t=replace(t,'    ASSERT_TRUE(result.ok)<<result.message;\n    EXPECT_EQ(c->linear,before.linear);',
          '    ASSERT_TRUE(result.ok)<<result.message;\n    observer->recordQueryInput(x(i-1),x(i),main.calculateEstimate<Pose3>(x(i-1)),main.calculateEstimate<Pose3>(x(i)));\n    observer->recordQueryOutput(output,0.);\n    EXPECT_EQ(c->linear,before.linear);')
        t+='''
TEST(Recording, FailureContainedAndNoExistingFileOverwritten) {
  Main main(2,params()),baseline(2,params());auto o=attach(&main);
  o->setRecording("/dev/null");EXPECT_FALSE(o->valid());
  initial(main,o);initial(baseline);exactMain(main,baseline);EXPECT_FALSE(main.fallbackHappened());
}
TEST(Recording, NoLateBootstrap) {
  Main main(2,params());auto o=attach(&main);initial(main,o);
  o->setRecording("/path/that/does/not/exist/journal");EXPECT_FALSE(o->valid());
  EXPECT_NE(std::string(o->error()).find("before initialization"),std::string::npos);
}
'''
        (author/'test_lifecycle.cpp').write_text(t)
    spec=importlib.util.spec_from_file_location('base',HERE/'base_build.py')
    base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)
    base.ROOT=ROOT;base.main()

if __name__=='__main__':main()
