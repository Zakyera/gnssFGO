// Non-estimator fixture for the saved-factor analyzer and Schur accounting.
#define main snapshot_main
#include "smart_snapshot.cpp"
#undef main
int main(){try{
  Saved s;s.params.retriangulationThreshold=.001;s.params.degeneracyMode=ZERO_ON_DEGENERACY;
  auto k=std::make_shared<Cal3_S2Stereo>(500,500,0,320,240,.2);
  Pose3 a,b(Rot3(),Point3(1,0,0));Point3 landmark(.2,.3,5);
  s.keys={1,2};s.calibrations={k,k};
  auto z0=StereoCamera(a,k).project(landmark),z1=StereoCamera(b,k).project(landmark);
  s.measurements={StereoPoint2(z0.uL(),z0.uR(),z0.v()+.5),StereoPoint2(z1.uL()+2,z1.uR()-1,z1.v())};
  s.values.insert(1,a);s.values.insert(2,b);
  RestoredSmart original(s);original.error(s.values);s.result=original.point();s.cached={a,b};
  s.values.update(2,b.retract((Vector6()<<0,0,0,.0001,0,0).finished()));
  s.direction.insert(1,Vector6::Zero());s.direction.insert(2,(Vector6()<<0,0,0,.001,0,0).finished());
  RestoredSmart reconstructed(s);auto c=slopes(reconstructed,s);
  const double fd=numerical(reconstructed,s,.01,false);
  if(std::abs(fd-c.raw)>1e-7 || std::abs(c.analytic-c.raw-c.schur_correction)>1e-9 || std::abs(c.schur_correction)<1e-6)
    throw std::runtime_error("Schur/held-point fixture accounting failed");
  std::cout<<std::setprecision(17)<<"PASS cached-point FD="<<fd<<" raw="<<c.raw<<" Schur="<<c.analytic<<" correction="<<c.schur_correction<<'\n';
  return 0;
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
