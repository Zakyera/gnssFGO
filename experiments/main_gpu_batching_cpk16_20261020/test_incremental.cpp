// Reuse the immutable, completed K16b fixture. No extra solver implementation.
#define main archived_k16b_main
#include "/workspace/cbs_gtsam4.3/runs/main_gpu_batching_cpk16_20261020/k16b_build_5/test_main_lifecycle.cpp"
#undef main

int main(int argc,char** argv) {
  try {
    require(argc==4,"usage: incremental output enabled frozen_geometry");
    const bool enabled=std::string(argv[2])=="1";
    Output o(argv[1]);Geometry geometry(argv[3]);
    o.prefix="input";o.gpu("geometry",*geometry.factor(X(0),X(1)));
    Solver main(params());
#ifdef K16_CANDIDATE
    cpk16::Stats stats;cpk16::Scope scope(&main,enabled,&stats);
#endif
    NonlinearFactorGraph graph;Values values;
    for(int k=0;k<24;++k) {
      values.insert(X(k),guess(k));
      if(!k)graph.emplace_shared<PriorFactor<Pose3>>(X(0),Pose3(),noise());
      else {
        graph.push_back(geometry.factor(X(k-1),X(k)));
        graph.emplace_shared<BetweenFactor<Pose3>>(X(k-1),X(k),Pose3(),noise());
        if(k>1)graph.push_back(geometry.factor(X(k-2),X(k)));
      }
    }
    main.update(graph,values);main.calculateEstimate();
    for(int phase=0;phase<2;++phase) {
      const auto old_cache=main.getLinearFactorsUnsafe();
      ISAM2UpdateParams p;p.noRelinKeys=FastList<Key>();p.extraReelimKeys=FastList<Key>();
      // Fine-grained diagnostic requests, equal in both branches: force the
      // real incremental path without changing any production parameters.
      for(int k=0;k<(phase?24:22);++k)p.noRelinKeys->push_back(X(k));
      p.extraReelimKeys->push_back(X(22));p.extraReelimKeys->push_back(X(23));
      p.force_relinearize=true;
      NonlinearFactorGraph incoming;
      if(!phase)incoming.emplace_shared<PriorFactor<Pose3>>(X(23),
        main.calculateEstimate<Pose3>(X(23)).retract(Vector6::Constant(.0002)),noise(.2));
#ifdef K16_CANDIDATE
      const auto before=stats;
#endif
      ISAM2Result result;e1b::Record rec;
      {e1b::Session session(phase,e1b::Context::Main,&rec);result=main.update(incoming,{},p);}
      o.prefix="incremental/"+std::to_string(phase);
      inspect(o,main,result);o.profile(rec);
      size_t selected_cached=0,selected_fresh=0;
      for(size_t slot=0;slot<old_cache.size();++slot) {
        const bool reused=old_cache[slot]==main.getLinearFactorsUnsafe()[slot];
        o.counts("reused/"+std::to_string(slot),{reused});
        if(!dynamic_cast<GPU*>(main.getFactorsUnsafe()[slot].get()))continue;
        bool selected=true;
        for(auto k:main.getFactorsUnsafe()[slot]->keys()) {
          auto it=result.detail->variableStatus.find(k);
          selected &= it!=result.detail->variableStatus.end()&&it->second.isReeliminated;
        }
        if(selected) {if(reused)++selected_cached;else ++selected_fresh;}
      }
      o.counts("selected_gpu",{selected_cached,selected_fresh});
      require(selected_cached>0,"no selected cached GPU coverage");
      if(!phase)require(selected_fresh>0,"no selected fresh GPU coverage");
      else require(selected_fresh==0&&rec.owner[static_cast<size_t>(e1b::Stage::GpuLinearize)].calls==0,
                   "cached-only update freshly linearized a GPU factor");
#ifdef K16_CANDIDATE
      if(enabled) {
        require(stats.sites[static_cast<size_t>(cpk16::Site::FullGraph)]==
                before.sites[static_cast<size_t>(cpk16::Site::FullGraph)],"unexpected batch path");
        require(stats.sites[static_cast<size_t>(cpk16::Site::Affected)]>
                before.sites[static_cast<size_t>(cpk16::Site::Affected)],"affected path not exercised");
        if(!phase)require(stats.work.prepared_factors>before.work.prepared_factors,
                          "fresh affected GPU factors not prepared");
      }
#endif
      std::cout<<"INCREMENTAL phase="<<phase<<" selected_cached="<<selected_cached
        <<" selected_fresh="<<selected_fresh<<" prepared_hits="
        <<rec.metrics[static_cast<size_t>(e1b::Metric::GpuLinearizeHits)]<<"\n";
    }
    o.f.flush();require(bool(o.f),"output failure");
    std::cout<<"K16B_INCREMENTAL_COMPLETED\n";return 0;
  } catch(const std::exception& e) {
    std::cerr<<"K16B_INCREMENTAL_FAILURE "<<e.what()<<"\n";return 1;
  }
}
