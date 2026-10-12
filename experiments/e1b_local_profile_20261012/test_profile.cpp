#include "RuntimeProfile.h"
#include <gtest/gtest.h>
#include <atomic>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace e1b;
namespace {
const Cell& cell(const Record& r,Stage s,bool worker=false) {
  return (worker?r.worker:r.owner)[static_cast<unsigned>(s)];
}
void accounting(const Record& r) {
  uint64_t owner=0,worker=0;
  for(auto& c:r.owner){ASSERT_LE(c.exclusive_ns,c.inclusive_ns);owner+=c.exclusive_ns;}
  for(auto& c:r.worker){ASSERT_LE(c.exclusive_ns,c.inclusive_ns);worker+=c.exclusive_ns;}
  EXPECT_EQ(owner,cell(r,Stage::Root).inclusive_ns);
  EXPECT_EQ(worker,cell(r,Stage::WorkerRoot,true).inclusive_ns);
}
TEST(Profile, DisabledNoContextOrCalls) {
  ASSERT_FALSE(active()); Session session(1,Context::Local);
  EXPECT_FALSE(active()); Scope scope(Stage::Backup); EXPECT_FALSE(capture().collector);
}
TEST(Profile, NestedDisjointAccounting) {
  Record record;
  {Session s(1,Context::Local,&record); Scope a(Stage::BaseUpdate);
   {Scope b(Stage::GpuError); Scope c(Stage::GpuLinearize);}
   metric(Metric::GpuLinearizeHits,2);}
  EXPECT_EQ(cell(record,Stage::GpuLinearize).calls,1u); accounting(record);
  EXPECT_EQ(record.metrics[static_cast<unsigned>(Metric::GpuLinearizeHits)],2u);
  EXPECT_FALSE(active());
}
TEST(Profile, StopIdempotentAndNoExtraFunctionCall) {
  Record record; int calls=0;
  {Session s(0,Context::Local,&record); Scope a(Stage::Backup); a.stop();a.stop();
   EXPECT_EQ(measure(Stage::Estimate,[&]{++calls;return 4;}),4);}
  EXPECT_EQ(calls,1); EXPECT_EQ(cell(record,Stage::Backup).calls,1u); accounting(record);
}
TEST(Profile, ExceptionRestoresContext) {
  Record record;
  try {Session s(0,Context::Local,&record);Scope a(Stage::Prepare);throw std::runtime_error("test");}
  catch(const std::runtime_error&){}
  EXPECT_TRUE(record.aborted); EXPECT_FALSE(active());
  EXPECT_EQ(cell(record,Stage::Prepare).exceptions,1u);accounting(record);
}
TEST(Profile, NestedSessionRestoresOuter) {
  Record outer,inner;
  {Session s(0,Context::Main,&outer); auto token=capture();
   {Session child(0,Context::Local,&inner); EXPECT_NE(token.collector,capture().collector);}
   EXPECT_EQ(token.collector,capture().collector); Scope a(Stage::Backup);}
  EXPECT_EQ(cell(inner,Stage::Backup).calls,0u); EXPECT_EQ(cell(outer,Stage::Backup).calls,1u);
  accounting(outer);accounting(inner);
}
TEST(Profile, WorkerIsolationAndCrossThreadMerge) {
  Record r; constexpr int threads=4, repeats=100;
  {Session s(0,Context::Local,&r);const auto token=capture();
   std::vector<std::thread> workers;
   for(int n=0;n<threads;++n)workers.emplace_back([=]{
     EXPECT_FALSE(active());
     {Attach attached(token); for(int i=0;i<repeats;++i){Scope a(Stage::GpuLinearize);metric(Metric::GpuLinearizeHits,1);}}
     EXPECT_FALSE(active());
   });
   for(int i=0;i<repeats;++i){Scope a(Stage::GpuLinearize);metric(Metric::GpuLinearizeHits,1);}
   for(auto& t:workers)t.join();}
  EXPECT_EQ(cell(r,Stage::GpuLinearize).calls,repeats);
  EXPECT_EQ(cell(r,Stage::GpuLinearize,true).calls,threads*repeats);
  EXPECT_EQ(r.metrics[static_cast<unsigned>(Metric::GpuLinearizeHits)],(threads+1)*repeats);
  accounting(r);
}
TEST(Profile, SameThreadAttachmentDoesNotDuplicateTree) {
  Record r;
  {Session s(0,Context::Local,&r);Attach a(capture());Scope b(Stage::Backup);}
  EXPECT_EQ(cell(r,Stage::WorkerRoot,true).calls,0u);accounting(r);
}
TEST(Profile, WorkerExceptionRestoresContext) {
  Record r;
  {Session s(0,Context::Local,&r);auto token=capture();
   std::thread worker([&]{try{Attach a(token);Scope b(Stage::GpuError);throw std::runtime_error("test");}
     catch(const std::runtime_error&){} EXPECT_FALSE(active());});worker.join();}
  EXPECT_EQ(cell(r,Stage::GpuError,true).exceptions,1u);accounting(r);
}
}  // namespace
