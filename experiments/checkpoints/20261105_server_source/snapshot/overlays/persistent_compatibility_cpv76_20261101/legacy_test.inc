#include "on_tests.inc"
#include "Checkpoint.h"

void retainedBound() {
  onRefresh();setenv("CPV69_RETAIN_COMPAT","1",1);setenv("FL4M_RETIREMENT","retain",1);
  Fixture f;cbs::BPSAM b(params());NonlinearFactorGraph g;Values v;
  for(int i=0;i<2;++i){g.push_back(f.factors(i));v.insert(f.initial(i));}
  b.update(g,v);b.addOdometryBeliefsDetailed({f.belief(0,1)});
  auto bounds=b.cpv40PreBounds({});b.update({},{});b.cpv40Refresh(bounds,{});
  auto removals=b.activeWindowTemporaryCbsOdomFactorRemovalsOutsideKeys({X(1),V(1),B(1)});
  require(removals.size()==1,"retained negative fixture missing");
  b.cpv40ValidateActive(bounds,removals);
  bounds[0].maximum=-1;rejects([&]{b.cpv40ValidateActive(bounds,removals);});
  auto broken=removals;broken[0].factor_index+=100;rejects([&]{b.cpv40ValidateActive({},broken);});
  metric("retained_residual_and_slot_negative_checks",2);
}

void boundaryChecks(const std::filesystem::path& root) {
  double schurMax=0,boundaryMax=0,priorMax=0;size_t wrappers=0;
  for(int k=4;k<13;++k){
    auto d=root/("update_"+std::to_string(k));
    auto a=cpv36::load(d/"after_refresh/checkpoint.bin");
    auto b=cpv36::load(d/"after_marginalization/checkpoint.bin");
    std::vector<size_t> gone,stay;size_t cursor=0;
    for(Key key:a.theta.keys()){
      for(size_t j=0;j<a.theta.at(key).dim();++j)(b.theta.exists(key)?stay:gone).push_back(cursor+j);
      cursor+=a.theta.at(key).dim();
      if(b.theta.exists(key)){
        require(a.theta.at(key).equals_(b.theta.at(key),0),"boundary chart changed");
        boundaryMax=std::max(boundaryMax,a.estimate.at(key).localCoordinates_(b.estimate.at(key)).norm());
      }
    }
    require(gone.size()==15,"one NavState must retire");
    Matrix Haa(gone.size(),gone.size()),Hab(gone.size(),stay.size()),Hbb(stay.size(),stay.size());
    Vector ga(gone.size()),gb(stay.size());
    for(size_t i=0;i<gone.size();++i){ga[i]=a.cached(gone[i],cursor);
      for(size_t j=0;j<gone.size();++j)Haa(i,j)=a.cached(gone[i],gone[j]);
      for(size_t j=0;j<stay.size();++j)Hab(i,j)=a.cached(gone[i],stay[j]);}
    for(size_t i=0;i<stay.size();++i){gb[i]=a.cached(stay[i],cursor);
      for(size_t j=0;j<stay.size();++j)Hbb(i,j)=a.cached(stay[i],stay[j]);}
    Eigen::LDLT<Matrix> llt(Haa);require(llt.info()==Eigen::Success&&llt.isPositive(),"marginal block SPD");
    Matrix H=Hbb-Hab.transpose()*llt.solve(Hab);Vector rhs=gb-Hab.transpose()*llt.solve(ga);
    const size_t n=stay.size();
    schurMax=std::max({schurMax,rel(b.cached.topLeftCorner(n,n),H),rel(b.cached.block(0,n,n,1),rhs)});
    require(schurMax<=1e-7 && boundaryMax<=1e-7,"retained Schur/boundary gate");
    auto graph=b.graph;auto cache=graph.clone().linearize(b.theta);KeySet fixed(b.fixed.begin(),b.fixed.end());
    auto p=cpk9::promote(graph,b.theta,*cache,fixed);require(p.priors>0&&fixed.empty(),"retained prior promotion");
    Values displaced=b.estimate;Vector6 shift;shift<<.1,-.07,.04,.2,-.1,.05;
    for(Key key:displaced.keys())if(Symbol(key).chr()=='x')displaced.update(key,displaced.at<Pose3>(key).retract(shift));
    for(size_t i=0;i<b.graph.size();++i)if(auto prior=dynamic_cast<LinearContainerFactor*>(b.graph[i].get())){
      double old=cbs::fixedLagFactorErrorInChart(*prior,displaced,b.theta);
      priorMax=std::max(priorMax,std::abs(graph[i]->error(displaced)-old)/(1+std::abs(old)));++wrappers;}
    require(priorMax<=1e-10,"retained prior function changed");
  }
  require(wrappers>=9,"repeated mixed priors missing");
  metric("retain_dense_Schur_max",schurMax);metric("retain_boundary_max",boundaryMax);
  metric("retain_prior_function_max",priorMax);metric("retain_boundaries",9);
}

int cpv69_saved_main(int argc,char**argv)try {
  require(argc==2,"snapshot root required");
  std::cout<<std::unitbuf;spdlog::set_level(spdlog::level::err);
  unsetenv("CPV69_RETAIN_COMPAT");unsetenv("CPV66_AUDIT_DIR");
  require(cpv40_saved_main()==0,"legacy CP40 suite");
  onRefresh();setenv("FL4M_RETIREMENT","retain",1);rejects([]{cpk9::enabled();});
  setenv("CPV69_RETAIN_COMPAT","1",1);require(cpk9::enabled()&&cpv40::enabled(),"retain admission");
  setenv("CPV40_REFRESH","0",1);rejects([]{cpk9::enabled();});
  onRefresh();setenv("CPV69_RETAIN_COMPAT","typo",1);rejects([]{cpv69::retain();});
  retainedBound();
  auto remove=onLifecycle(false,false),retain=onLifecycle(false,true);
  setenv("CPV66_AUDIT_DIR",argv[1],1);setenv("CPV66_TEST_ALL","1",1);setenv("FL4M_AUDIT","/dev/null",1);
  auto observed=onLifecycle(false,true);unsetenv("CPV66_AUDIT_DIR");unsetenv("FL4M_AUDIT");
  auto rollback=onLifecycle(true,true);equal(retain,observed);equal(retain,rollback);
  for(int k=0;k<4;++k){require(remove.latest.at<Pose3>(X(k)).equals(retain.latest.at<Pose3>(X(k)),0),"pre-retirement state parity");
    require((remove.cov[k]-retain.cov[k]).norm()==0,"pre-retirement covariance parity");}
  boundaryChecks(argv[1]);
  metric("retain_post_retirement_fixture_effect",remove.latest.localCoordinates(retain.latest).norm());
  std::cout<<"PASS CP69 compatibility. Retained LOCAL is mixed-history, not native-only. No real-data accuracy claim.\n";
  return 0;
}catch(const std::exception& e){std::cerr<<"FAIL CP69 "<<e.what()<<'\n';return 1;}
