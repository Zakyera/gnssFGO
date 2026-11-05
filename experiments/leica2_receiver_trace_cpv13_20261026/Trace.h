// CP-V13 observer only. Included inside namespace fl3i after State definition.
// No solver update, factor insertion, covariance edit or relinearization here.
inline void Trace(const State& s,const char* stage,const Belief* b=nullptr,
                  const char* action="",const NonlinearFactor::shared_ptr& incoming={}) {
  constexpr int64_t epoch=1732549034915071665LL;
  if(!s.initialized||s.lastArrival<epoch+379900000000LL||
     s.lastArrival>epoch+390100000000LL)return;
  const char* directory=std::getenv("CPV13_TRACE_ROOT");
  Require(directory&&*directory,"CPV13 trace directory required");
  struct Files {
    std::ofstream meta,states,costs;size_t snapshot=0;
    explicit Files(const fs::path& root) {
      for(const char* name:{"trace_meta.csv","trace_states.csv","trace_costs.csv"})
        Require(!fs::exists(root/name),"refuse trace overwrite");
      meta.open(root/"trace_meta.csv");states.open(root/"trace_states.csv");costs.open(root/"trace_costs.csv");
      for(auto* f:{&meta,&states,&costs}){f->exceptions(std::ios::failbit|std::ios::badbit);*f<<std::setprecision(17);}
      meta<<"snapshot,event,stage,arrival_ns,newest,oldest,source,action,from,to,revision,start_ns,end_ns,incoming_error,erx,ery,erz,etx,ety,etz\n";
      states<<"snapshot,index,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz\n";
      costs<<"snapshot,category,count,error\n";
    }
  };
  static Files files(directory);
  const size_t serial=++files.snapshot;
  // calculateEstimate can update mutable delta caches: query a COPY only.
  ISAM2 copy(s.solver.isam());const Values values=copy.calculateEstimate();
  Vector residual=Vector::Zero(6);double error=0.;
  if(incoming) {
    auto f=boost::dynamic_pointer_cast<WnoaRelativeFactor>(incoming->clone());
    Require(bool(f),"incoming trace factor type");
    residual=f->unwhitenedError(values);error=f->error(values);
  }
  files.meta<<serial<<','<<s.event<<','<<stage<<','<<s.lastArrival<<','<<s.newest<<','<<s.oldest
    <<','<<(b?b->source:"")<<','<<action<<','<<(b?b->from:0)<<','<<(b?b->to:0)
    <<','<<(b?b->revision:0)<<','<<(b?b->start:0)<<','<<(b?b->end:0)<<','<<error;
  for(double x:residual)files.meta<<','<<x;files.meta<<'\n';
  for(const auto& [key,tick]:s.solver.timestamps()) {
    const Symbol sym(key);if(sym.chr()!='x')continue;
    const size_t index=sym.index();const auto v=ReadSourceWnoaState(values,index);
    const auto q=v.pose.rotation().toQuaternion();
    files.states<<serial<<','<<index<<','<<v.pose.x()<<','<<v.pose.y()<<','<<v.pose.z()
      <<','<<q.x()<<','<<q.y()<<','<<q.z()<<','<<q.w();
    for(double x:v.velocity)files.states<<','<<x;
    for(double x:v.omega)files.states<<','<<x;files.states<<'\n';
  }
  if(std::string(stage)=="publication") {
    std::map<size_t,std::string> sourceSlots;
    for(const auto& [id,e]:s.active)sourceSlots[e.slot]=e.latest.source;
    std::map<std::string,std::pair<size_t,double>> totals;
    const auto& graph=s.solver.getFactors();
    for(size_t i=0;i<graph.size();++i) {
      if(!graph[i])continue;
      std::string category;
      NonlinearFactor::shared_ptr diagnostic;
      if(const auto* f=dynamic_cast<const LinearContainerFactor*>(graph[i].get())) {
        category="marginal_original_chart";
        diagnostic=boost::make_shared<fl2::ChartMarginal>(f->factor(),s.solver.marginals.at(i).anchor);
      } else {
        diagnostic=graph[i]->clone();
        if(sourceSlots.count(i))category="source_"+sourceSlots.at(i);
        else if(dynamic_cast<const fgo::factor::GPWNOAPrior*>(graph[i].get()))category="WNOA";
        else category="initial_prior";
      }
      auto& total=totals[category];++total.first;total.second+=diagnostic->error(values);
    }
    for(const auto& [category,total]:totals)
      files.costs<<serial<<','<<category<<','<<total.first<<','<<total.second<<'\n';
  }
  files.meta.flush();files.states.flush();files.costs.flush();
}
