  // Diagnostic-only accessors: no data members/virtuals, no estimator use.
  void cpk1ClearCache() { cameraPosesTriangulation_.clear(); }
  void cpk1Dump(std::ostream& out) const {
    out<<"params,"<<params_.linearizationMode<<','<<params_.degeneracyMode<<','
       <<params_.retriangulationThreshold<<','<<params_.triangulation.rankTolerance<<','
       <<params_.triangulation.enableEPI<<','<<params_.triangulation.landmarkDistanceThreshold<<','
       <<params_.triangulation.dynamicOutlierRejectionThreshold<<','<<params_.triangulation.useLOST<<','
       <<params_.throwCheirality<<','<<params_.verboseCheirality<<','<<noiseModel_->sigma()<<'\n';
    const auto body=body_P_sensor().matrix();out<<"body";
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<body(i,j);out<<'\n';
    out<<"cache_status,"<<result_.status;
    if(result_)out<<','<<(*result_).x()<<','<<(*result_).y()<<','<<(*result_).z();out<<'\n';
    for(size_t i=0;i<keys_.size();++i){const auto& z=measured_[i];const auto& k=*K_all_[i];
      out<<"measurement,"<<keys_[i]<<','<<z.uL()<<','<<z.uR()<<','<<z.v()<<','
         <<k.fx()<<','<<k.fy()<<','<<k.skew()<<','<<k.px()<<','<<k.py()<<','<<k.baseline()<<'\n';}
    for(size_t n=0;n<cameraPosesTriangulation_.size();++n){auto p=cameraPosesTriangulation_[n].matrix();
      out<<"cached_pose,"<<n;for(int i=0;i<4;++i)for(int j=0;j<4;++j)out<<','<<p(i,j);out<<'\n';}
  }
