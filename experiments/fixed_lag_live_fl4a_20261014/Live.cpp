// I/O adapter only. The tested fixed-lag receiver and parser are unchanged.
#define main FL4A_UNUSED_MAIN
#include "../receiver_endpoint_reuse_e2b_20261013/DiagnosticReplay.cpp"
#undef main
#include "../fixed_lag_slots_fl3i_20261014/Receiver.h"
#include <sys/resource.h>
#include <chrono>
namespace fs=std::filesystem;
using Steady=std::chrono::steady_clock;
static std::vector<std::string> Fields(const std::string& s){std::vector<std::string> v;std::istringstream in(s);std::string x;while(std::getline(in,x,','))v.push_back(x);return v;}
static std::ofstream Fresh(const fs::path& p){if(fs::exists(p))throw std::runtime_error("refuse output overwrite");std::ofstream f(p);f.exceptions(std::ios::failbit|std::ios::badbit);f<<std::setprecision(17);return f;}
int main(int argc,char**argv){
 const tbb::global_control serial(tbb::global_control::max_allowed_parallelism,1);
 try{
  std::map<std::string,std::string> args;for(int i=1;i<argc;i+=2){if(i+1>=argc||!args.emplace(argv[i],argv[i+1]).second)throw std::runtime_error("invalid arguments");}
  for(auto [k,v]:std::map<std::string,std::string>{{"--mode","sources"},{"--timeline","record"},{"--solver","isam2"},{"--support-policy","uniform_imu"},{"--support-period-ns","100000000"},{"--receiver-startup-samples","59"},{"--receiver-model","source_wnoa"}})
   if(args.at(k)!=v)throw std::runtime_error("unsupported frozen setting: "+k);
  if(args.size()!=9)throw std::runtime_error("unexpected args");
  const fs::path out=args.at("--out");if(!fs::create_directory(out))throw std::runtime_error("fresh output required");
  auto online=Fresh(out/"online_states.csv"),final=Fresh(out/"finalized_states.csv");
  const char* header="state_index,timestamp_ns,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,availability_ns\n";
  online<<header<<std::flush;final<<header<<std::flush;
  auto otum=Fresh(out/"online_trajectory.tum"),ftum=Fresh(out/"finalized_trajectory.tum");
  auto timing=Fresh(out/"stream_progress.csv");
  timing<<"event,kind,arrival_ns,processed_wall_ns,service_seconds,publications,newest,oldest,active,pending,supports,slots,nonempty,rss_kib\n"<<std::flush;
  fl3i::Receiver receiver(out/"archive");EventStream input(args.at("--event-stream"));size_t eventno=0;
  for(;;){auto event=input.Next();auto started=Steady::now();++eventno;
   if(event.kind=='E')receiver.end(event.arrival_ns);
   else if(event.kind=='I')receiver.imuClock(event.imu.timestamp_ns,event.arrival_ns);
   else{const auto&r=event.belief;fl3i::Belief b;b.source=r.source_agent=="g"?"glim":r.source_agent=="k"?"kimera":r.source_agent;b.session=r.session_id;b.from=r.from_index;b.to=r.to_index;b.ordinal=r.ordinal;b.revision=r.revision;b.start=r.from_timestamp_ns;b.end=r.to_timestamp_ns;b.arrival=r.record_timestamp_ns;b.mean=r.measurement;b.covariance=r.covariance;receiver.belief(b);}
   const auto&s=receiver.state();const auto commit=out/"archive/commits"/std::to_string(s.event)/"records.txt";
   if(fs::exists(commit)){std::ifstream in(commit);std::string line;while(std::getline(in,line)){
    const auto v=Fields(line);if(v[0]!="online_incremental"&&v[0]!="finalized_incremental")continue;
    if(v.size()!=17)throw std::runtime_error("unexpected committed state schema");
    auto& csv=v[0]=="online_incremental"?online:final;auto& tum=v[0]=="online_incremental"?otum:ftum;
    csv<<v[1]<<','<<v[2];for(size_t j=4;j<v.size();++j)csv<<','<<v[j];csv<<','<<v[3]<<'\n';
    const auto stamp=std::stoll(v[2]);tum<<stamp/1000000000<<'.'<<std::setw(9)<<std::setfill('0')<<stamp%1000000000;
    for(size_t j=4;j<11;++j)tum<<' '<<v[j];tum<<'\n';
   }}
   online.flush();final.flush();otum.flush();ftum.flush();
   size_t nonempty=0;for(const auto& f:s.solver.getFactors())nonempty+=bool(f);
   rusage usage{};getrusage(RUSAGE_SELF,&usage);
   const double elapsed=std::chrono::duration<double>(Steady::now()-started).count();
   const auto wall=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
   timing<<eventno<<','<<event.kind<<','<<event.arrival_ns<<','<<wall<<','<<elapsed<<','<<s.counts.publications<<','<<s.newest<<','<<s.oldest<<','<<s.active.size()<<','<<s.pending.size()+s.buffered.size()<<','<<s.solver.timestamps().size()/3<<','<<s.solver.getFactors().size()<<','<<nonempty<<','<<usage.ru_maxrss<<'\n'<<std::flush;
   if(event.kind=='E'){
    auto summary=Fresh(out/"summary.json");summary<<"{\"complete\":true,\"model\":\"source_wnoa_fixed_lag\",\"lag_seconds\":7,\"stationarity\":\"not_checked_incremental_policy\",\"events\":"<<eventno<<",\"received\":"<<s.counts.received<<",\"inserted\":"<<s.counts.inserted<<",\"replaced\":"<<s.counts.replaced<<",\"excluded\":"<<s.counts.excluded<<",\"unsupported_end\":"<<s.counts.unsupportedEnd<<",\"absorbed\":"<<s.counts.absorbed<<",\"active\":"<<s.active.size()<<",\"pending_at_end\":"<<s.pending.size()+s.buffered.size()<<",\"publications\":"<<s.counts.publications<<",\"finalized\":"<<s.counts.finalized<<",\"retained_supports\":"<<s.solver.timestamps().size()/3<<",\"peak_slots\":"<<s.counts.peakSlots<<"}\n";summary.close();break;
   }
  }return 0;
 }catch(const std::exception&e){std::cerr<<"FL4A_STREAM_FAILURE "<<e.what()<<'\n';return 1;}
}
