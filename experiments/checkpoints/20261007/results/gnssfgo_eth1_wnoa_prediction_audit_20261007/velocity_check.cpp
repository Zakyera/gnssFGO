#define main saved_audit_main
#include "/audit/audit.cpp"
#undef main
int main(int argc,char**argv){try{
 Check(argc==2,"velocity_check output.csv");Check(!std::filesystem::exists(argv[1]),"refuse overwrite");
 std::ofstream out(argv[1]);out<<std::setprecision(17)<<"mode,snapshot,interval,endpoint,epsilon_s,velocity_error_mps,omega_error_radps\n";
 for(auto&r:Read("/audit/requests.csv")){auto mode=r[0],label=r[1];auto d=Load("/previous/"+mode+"_wnoa/snapshot_"+label+"/incremental_states.csv");size_t n=d.times.size()-1;
  for(size_t i=n>5?n-5:0;i<n;++i)for(double eps:{1e-5,1e-6})for(bool end:{false,true}){
   int64_t t=end?d.times[i+1]:d.times[i],offset=llround(eps*1e9);auto s=ReadSourceWnoaState(d.values,end?i+1:i);auto p0=Endpoint(d,end?t-offset:t).pose(d.values),p1=Endpoint(d,end?t:t+offset).pose(d.values);
   auto v=((p1.translation()-p0.translation())/eps).eval();auto w=(gtsam::Rot3::Logmap(p0.rotation().between(p1.rotation()))/eps).eval();
   Check(v.allFinite()&&w.allFinite(),"derivative nonfinite");out<<mode<<','<<label<<','<<i<<','<<(end?"end":"start")<<','<<eps<<','<<(v-s.velocity).norm()<<','<<(w-s.omega).norm()<<'\n';
  }
 }std::cout<<"completed read-only endpoint derivative comparison\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
