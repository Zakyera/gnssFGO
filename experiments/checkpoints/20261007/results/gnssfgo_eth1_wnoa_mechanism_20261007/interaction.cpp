// Read-only reconstruction and linearization of captured WNOA snapshots.
// Reuse the validated model, reconstruction and payload check implementation.
#define main previous_audit_main
#include "/old_audit/audit.cpp"
#undef main
int main(int argc,char**argv){try{
  tbb::global_control threads(tbb::global_control::max_allowed_parallelism,1);
  Check(argc==2,"request CSV required");
  for(const auto&r:Read(argv[1])){
    std::string mode=r[0],label=r[1],dir="/audit/"+mode+"_wnoa/snapshot_"+label;
    std::string prefix="/audit/interaction/"+mode+"_"+label;
    Check(!std::filesystem::exists(prefix+"_factors.csv"),"refuse overwrite");
    auto d=Load(dir+"/incremental_states.csv");double costerr;
    auto g=Build(d,dir,"/audit/payloads/"+mode+"_"+label+".csv",costerr);
    auto[ce,re]=Probe(g,d,dir+"/factor_payload_probe.txt");
    std::map<size_t,std::string> kinds;for(const auto&e:Read(dir+"/incremental_errors.csv"))kinds[std::stoull(e[0])]=e[1];
    std::ofstream fac(prefix+"_factors.csv"),res(prefix+"_r.bin",std::ios::binary),jac(prefix+"_jac.bin",std::ios::binary),raw(prefix+"_raw.csv");
    fac<<std::setprecision(17)<<"slot,family,row,dimension,cost\n";
    raw<<std::setprecision(17)<<"slot,family,rotation_norm,rho_norm,cost\n";
    int64_t row=0;size_t n=0;
    for(size_t slot=0;slot<g.size();++slot)if(g[slot]){
      auto f=boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(g[slot]);std::vector<M> h(f->keys().size());
      auto e=f->unwhitenedError(d.values,h);auto w=e;
      f->noiseModel()->WhitenSystem(h,w);
      Check(std::abs(.5*w.squaredNorm()-f->error(d.values))<1e-8*(1+f->error(d.values)),"whitening error");
      fac<<slot<<','<<kinds.at(slot)<<','<<row<<','<<w.size()<<','<<f->error(d.values)<<'\n';
      if(e.size()==6)raw<<slot<<','<<kinds.at(slot)<<','<<e.head(3).norm()<<','<<e.tail(3).norm()<<','<<f->error(d.values)<<'\n';
      res.write(reinterpret_cast<const char*>(w.data()),w.size()*sizeof(double));
      for(size_t k=0;k<f->keys().size();++k){gtsam::Symbol key(f->keys()[k]);
        int offset=key.chr()=='x'?0:key.chr()=='v'?6:key.chr()=='w'?9:-1;Check(offset>=0,"unexpected key");
        for(int i=0;i<h[k].rows();++i)for(int j=0;j<h[k].cols();++j){double v=h[k](i,j);if(v==0)continue;
          int64_t a=row+i,b=12*key.index()+offset+j;jac.write(reinterpret_cast<const char*>(&a),8);jac.write(reinterpret_cast<const char*>(&b),8);jac.write(reinterpret_cast<const char*>(&v),8);}
      }row+=w.size();++n;
    }
    std::ofstream check(prefix+"_check.csv");check<<std::setprecision(17)<<"factors,states,cost_relative_error,probe_cov_error,probe_residual_error\n"<<n<<','<<d.times.size()<<','<<costerr<<','<<ce<<','<<re<<'\n';
    std::cout<<mode<<' '<<label<<" checked "<<n<<" factors\n"<<std::flush;
  }
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
