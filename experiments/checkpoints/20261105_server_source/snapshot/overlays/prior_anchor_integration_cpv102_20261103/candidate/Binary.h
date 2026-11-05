#pragma once
// Local versioned native-byte archive, explicit fields; no Boost polymorphic IO.
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <fstream>
#include <type_traits>
namespace cpv36 {
using namespace gtsam;
struct BinaryOut{std::ostream& stream;explicit BinaryOut(std::ostream& s):stream(s){}template<class T>BinaryOut& operator<<(const T& v);};
struct BinaryIn{std::istream& stream;explicit BinaryIn(std::istream& s):stream(s){}template<class T>BinaryIn& operator>>(T& v);};
template<class T,std::enable_if_t<std::is_arithmetic<T>::value||std::is_enum<T>::value,int> =0>
void pack(BinaryOut& a,const T& v){a.stream.write(reinterpret_cast<const char*>(&v),sizeof(v));}
template<class T,std::enable_if_t<std::is_arithmetic<T>::value||std::is_enum<T>::value,int> =0>
void unpack(BinaryIn& a,T& v){a.stream.read(reinterpret_cast<char*>(&v),sizeof(v));}
template<class D>void pack(BinaryOut& a,const Eigen::MatrixBase<D>& v){size_t r=v.rows(),c=v.cols();a<<r<<c;for(size_t j=0;j<c;++j)for(size_t i=0;i<r;++i)a<<v(i,j);}
template<class D>void unpack(BinaryIn& a,Eigen::MatrixBase<D>& v){size_t r,c;a>>r>>c;if(r>10000||c>10000||r*c>100000000)throw std::runtime_error("archive matrix size");if((D::RowsAtCompileTime!=Eigen::Dynamic&&r!=D::RowsAtCompileTime)||(D::ColsAtCompileTime!=Eigen::Dynamic&&c!=D::ColsAtCompileTime))throw std::runtime_error("archive matrix shape");v.derived().resize(r,c);for(size_t j=0;j<c;++j)for(size_t i=0;i<r;++i)a>>v(i,j);}
inline void pack(BinaryOut&a,const std::string&v){size_t n=v.size();a<<n;a.stream.write(v.data(),n);}
inline void unpack(BinaryIn&a,std::string&v){size_t n;a>>n;if(n>1000000)throw std::runtime_error("archive string");v.resize(n);a.stream.read(v.data(),n);}
template<class T>void pack(BinaryOut&a,const std::vector<T>&v){size_t n=v.size();a<<n;for(const auto&x:v)a<<x;}
template<class T>void unpack(BinaryIn&a,std::vector<T>&v){size_t n;a>>n;if(n>1000000)throw std::runtime_error("archive vector");v.resize(n);for(auto&x:v)a>>x;}
template<class T>void pack(BinaryOut&a,const std::optional<T>&v){bool yes=v.has_value();a<<yes;if(yes)a<<*v;}
template<class T>void unpack(BinaryIn&a,std::optional<T>&v){bool yes;a>>yes;if(yes){T x;a>>x;v=x;}else v.reset();}
inline void pack(BinaryOut&a,const Pose3&v){a<<v.rotation().matrix()<<v.translation();}
inline void unpack(BinaryIn&a,Pose3&v){Matrix3 r;Vector3 t;a>>r>>t;v=Pose3(Rot3(r),t);}
inline void pack(BinaryOut&a,const imuBias::ConstantBias&v){a<<v.accelerometer()<<v.gyroscope();}
inline void unpack(BinaryIn&a,imuBias::ConstantBias&v){Vector3 x,y;a>>x>>y;v=imuBias::ConstantBias(x,y);}
inline void pack(BinaryOut&a,const StereoPoint2&v){a<<v.uL()<<v.uR()<<v.v();}
inline void unpack(BinaryIn&a,StereoPoint2&v){double x,y,z;a>>x>>y>>z;v=StereoPoint2(x,y,z);}
inline void pack(BinaryOut&a,const Cal3_S2Stereo&v){a<<v.fx()<<v.fy()<<v.skew()<<v.px()<<v.py()<<v.baseline();}
inline void unpack(BinaryIn&a,Cal3_S2Stereo&v){double f,g,s,x,y,b;a>>f>>g>>s>>x>>y>>b;v=Cal3_S2Stereo(f,g,s,x,y,b);}
inline void pack(BinaryOut&a,const SmartStereoProjectionParams&v){a<<v.linearizationMode<<v.degeneracyMode<<v.retriangulationThreshold<<v.throwCheirality<<v.verboseCheirality<<v.triangulation.rankTolerance<<v.triangulation.enableEPI<<v.triangulation.landmarkDistanceThreshold<<v.triangulation.dynamicOutlierRejectionThreshold<<v.triangulation.useLOST;}
inline void unpack(BinaryIn&a,SmartStereoProjectionParams&v){a>>v.linearizationMode>>v.degeneracyMode>>v.retriangulationThreshold>>v.throwCheirality>>v.verboseCheirality>>v.triangulation.rankTolerance>>v.triangulation.enableEPI>>v.triangulation.landmarkDistanceThreshold>>v.triangulation.dynamicOutlierRejectionThreshold>>v.triangulation.useLOST;}
inline void pack(BinaryOut&a,const PreintegrationParams&v){a<<v.gyroscopeCovariance<<v.omegaCoriolis<<v.body_P_sensor<<v.accelerometerCovariance<<v.integrationCovariance<<v.use2ndOrderCoriolis<<v.n_gravity;}
inline void unpack(BinaryIn&a,PreintegrationParams&v){a>>v.gyroscopeCovariance>>v.omegaCoriolis>>v.body_P_sensor>>v.accelerometerCovariance>>v.integrationCovariance>>v.use2ndOrderCoriolis>>v.n_gravity;}
struct RestorePim:PreintegratedImuMeasurements{
  void read(BinaryIn&a){p_=std::make_shared<PreintegrationParams>();Matrix3 r;Vector3 p,v;
    a>>*p_>>biasHat_>>deltaTij_>>r>>p>>v>>delRdelBiasOmega_>>delPdelBiasAcc_>>delPdelBiasOmega_>>delVdelBiasAcc_>>delVdelBiasOmega_>>preintMeasCov_;
    deltaXij_=NavState(Rot3(r),p,v);}
};
inline void pack(BinaryOut&a,const PreintegratedImuMeasurements&v){a<<v.p()<<v.biasHat()<<v.deltaTij()<<v.deltaRij().matrix()<<v.deltaPij()<<v.deltaVij()<<v.delRdelBiasOmega()<<v.delPdelBiasAcc()<<v.delPdelBiasOmega()<<v.delVdelBiasAcc()<<v.delVdelBiasOmega()<<v.preintMeasCov();}
inline void unpack(BinaryIn&a,PreintegratedImuMeasurements&v){RestorePim r;r.read(a);v=r;}
template<class T>BinaryOut& BinaryOut::operator<<(const T&v){pack(*this,v);if(!stream)throw std::runtime_error("archive write failed");return *this;}
template<class T>BinaryIn& BinaryIn::operator>>(T&v){unpack(*this,v);if(!stream)throw std::runtime_error("archive read failed");return *this;}
}
