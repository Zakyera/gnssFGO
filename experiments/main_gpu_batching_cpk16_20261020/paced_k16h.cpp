// Diagnostic input adapter only. Reuses the unchanged GlimROS bag interfaces.
#include <glim_ros/glim_ros.hpp>
#include <glim/odometry/callbacks.hpp>
#include <glim/util/config.hpp>
#include <glim/util/time_keeper.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Imu.h>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <deque>
#include <iomanip>
#include <limits>
#include <optional>
#include <thread>

namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
void require(bool ok,const std::string& why) {if(!ok) throw std::runtime_error(why);}
ros::Time from_ns(uint64_t n) {ros::Time t;t.fromNSec(n);return t;}
int main(int argc,char** argv) {
  ros::init(argc,argv,"glim_rosnode");
  ros::V_string args;ros::removeROSArgs(argc,argv,args);
  require(args.size()==9,"mode config imu_bag lidar_bag start_ns end_ns output ready_timeout");
  const bool inspect=args[1]=="inspect";
  require(inspect||args[1]=="run","invalid mode");
  const fs::path out=args[7];
  const uint64_t begin=std::stoull(args[5]),end=std::stoull(args[6]);
  const std::string imu_topic="/boxi/cpt7/imu",lidar_topic="/boxi/hesai/points";
  std::atomic<size_t> entered{0};
  std::atomic<double> entered_stamp{0};
  std::unique_ptr<glim::GlimROS> source;
  std::unique_ptr<glim::TimeKeeper> inspector_time;
  std::unique_ptr<ros::NodeHandle> nh;
  ros::Publisher imu_pub,lidar_pub,clock_pub;
  std::unique_ptr<ros::AsyncSpinner> spinner;
  int callback_id=-1;
  try {
    if(inspect) {
      glim::GlobalConfig::instance(args[2]);
      inspector_time=std::make_unique<glim::TimeKeeper>();
    } else {
      nh=std::make_unique<ros::NodeHandle>("~");
      source=std::make_unique<glim::GlimROS>(*nh);
      // This callback fires only once the async worker owns the frame. Waiting
      // for it before wait_for_odometry_idle excludes the queue-transfer race.
      callback_id=glim::OdometryEstimationCallbacks::on_insert_frame.add(
        [&](const glim::PreprocessedFrame::Ptr& f){
          entered_stamp.store(f->stamp);entered.fetch_add(1);
        });
      imu_pub=nh->advertise<sensor_msgs::Imu>(imu_topic,1000);
      lidar_pub=nh->advertise<sensor_msgs::PointCloud2>(lidar_topic,100);
      clock_pub=nh->advertise<rosgraph_msgs::Clock>("/clock",100);
      spinner=std::make_unique<ros::AsyncSpinner>(1);spinner->start();
      require(!fs::exists(out/"driver_ready"),"duplicate driver");
      std::ofstream(out/"driver_ready")<<"ready\n";
      const auto deadline=Clock::now()+std::chrono::seconds(std::stoi(args[8]));
      while(!fs::exists(out/"go")) {
        require(ros::ok()&&Clock::now()<deadline,"controller-ready timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    const fs::path logfile=out/(inspect?"coverage.csv":"pace.csv");
    require(!fs::exists(logfile),"refuse log overwrite");
    std::ofstream log(logfile);
    log<<"scan,seq,record_ns,header_ns,normalized_stamp,scan_end,covering_imu,entered,wall_wait_s,native_eos\n"<<std::setprecision(17);
    rosbag::Bag imu_bag(args[3],rosbag::bagmode::Read),lidar_bag(args[4],rosbag::bagmode::Read);
    rosbag::View view;
    view.addQuery(imu_bag,rosbag::TopicQuery({imu_topic}),from_ns(begin),from_ns(end-1));
    view.addQuery(lidar_bag,rosbag::TopicQuery({lidar_topic}),from_ns(begin),from_ns(end-1));
    size_t scans=0,imus=0,acks=0,events=0,dispatch_index=0,received_scans=0,max_buffer=0;
    std::ofstream dispatch(out/(inspect?"expected_dispatch.csv":"dispatch.csv"));
    dispatch<<"event_index,event_type,message_stamp_ns,bag_record_stamp_ns,source_sequence\n";
    auto dispatched=[&](const char* type,uint64_t header,uint64_t record,uint32_t seq){
      dispatch<<dispatch_index++<<','<<type<<','<<header<<','<<record<<','<<seq<<'\n';dispatch.flush();
    };
    struct Buffered {sensor_msgs::PointCloud2::ConstPtr msg;uint64_t record;};
    std::deque<Buffered> waiting;
    double last_imu=-std::numeric_limits<double>::infinity();
    struct Pending {uint32_t seq;uint64_t record,header;double stamp,end;};
    std::optional<Pending> pending;
    uint64_t last_record=0;
    auto activate=[&] {
      if(pending||waiting.empty())return;
      auto b=waiting.front();waiting.pop_front();const auto& msg=b.msg;
      auto raw=glim::extract_raw_points(msg);require(bool(raw)&&!raw->times.empty(),"invalid point fields");
      if(inspect) require(inspector_time->process(raw),"native time normalization rejected scan");
      else source->insert_frame(raw,msg->header.seq,b.record,msg->header.stamp.toNSec());
      dispatched("PointCloud2",msg->header.stamp.toNSec(),b.record,msg->header.seq);
      const double scan_end=raw->stamp+*std::max_element(raw->times.begin(),raw->times.end());
      require(scan_end>=raw->stamp&&scan_end-raw->stamp<.2,"unexpected normalized scan duration");
      ++scans;pending=Pending{msg->header.seq,b.record,msg->header.stamp.toNSec(),raw->stamp,scan_end};
    };
    auto flush=[&] {
      activate();
      while(pending && last_imu>=pending->end) {
      const auto started=Clock::now();
      if(!inspect) {
        const auto deadline=started+std::chrono::seconds(180);
        while(entered.load()<scans) {
          require(ros::ok()&&Clock::now()<deadline,"scan-entry watchdog");
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        require(entered.load()==scans,"unaccounted scan callback");
        require(entered_stamp.load()==pending->stamp,"scan callback stamp mismatch");
        const double remaining=std::chrono::duration<double>(deadline-Clock::now()).count();
        require(remaining>0 && source->wait_for_odometry_idle(remaining),"scan-completion watchdog");
      }
      log<<scans<<','<<pending->seq<<','<<pending->record<<','<<pending->header<<','
         <<pending->stamp<<','<<pending->end<<','<<last_imu<<','
         <<(inspect?0:entered.load())<<','
         <<std::chrono::duration<double>(Clock::now()-started).count()<<",0\n";
      log.flush();++acks;pending.reset();activate();
      }
    };
    for(const rosbag::MessageInstance& m:view) {
      const uint64_t record=m.getTime().toNSec();
      require(record>=begin&&record<end&&record>=last_record,"record order/window");
      last_record=record;
      if(!inspect) {
        require(ros::ok(),"ROS stopped during input");
        rosgraph_msgs::Clock c;c.clock=m.getTime();clock_pub.publish(c);
        const auto clock_deadline=Clock::now()+std::chrono::seconds(2);
        while(ros::Time::now()!=m.getTime()) {
          require(ros::ok()&&Clock::now()<clock_deadline,"clock acknowledgement timeout");
          std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
      }
      if(m.getTopic()==imu_topic) {
        const auto msg=m.instantiate<sensor_msgs::Imu>();require(bool(msg),"bad IMU payload");
        const double stamp=msg->header.stamp.toSec();
        require(stamp>=last_imu,"IMU timestamp rewind");last_imu=stamp;
        if(!inspect) {
          const auto& a=msg->linear_acceleration;const auto& w=msg->angular_velocity;
          source->insert_imu(stamp,{a.x,a.y,a.z},{w.x,w.y,w.z},msg->header.seq,record,msg->header.stamp.toNSec());
          imu_pub.publish(msg);
        }
        dispatched("IMU",msg->header.stamp.toNSec(),record,msg->header.seq);
        ++imus;flush();
      } else {
        require(m.getTopic()==lidar_topic,"unexpected topic");
        const auto msg=m.instantiate<sensor_msgs::PointCloud2>();require(bool(msg),"bad scan payload");
        waiting.push_back({msg,record});++received_scans;
        max_buffer=std::max(max_buffer,waiting.size());
        require(max_buffer<=10,"diagnostic raw scan buffer bound");
        if(!inspect)lidar_pub.publish(msg);
        flush();
      }
      ++events;
    }
    bool joined=false;
    // Match glim_rosnode's existing end-of-bag wait(true): the async worker
    // sets end_of_sequence and processes the terminal scan with exactly the
    // selected IMU. No appended samples or changed integration equations.
    if(pending) {
      require(scans==598&&received_scans==598&&waiting.empty(),"unexpected unfinished nonterminal scan");
      require(pending->stamp<=last_imu&&pending->end-last_imu>0&&pending->end-last_imu<.1,
              "unexpected terminal IMU coverage gap");
      const auto started=Clock::now();
      if(!inspect) {
        source->wait(true);joined=true;
        require(entered.load()==scans&&entered_stamp.load()==pending->stamp,"terminal scan acknowledgement");
      }
      log<<scans<<','<<pending->seq<<','<<pending->record<<','<<pending->header<<','
         <<pending->stamp<<','<<pending->end<<','<<last_imu<<','
         <<(inspect?0:entered.load())<<','
         <<std::chrono::duration<double>(Clock::now()-started).count()<<",1\n";
      log.flush();++acks;pending.reset();
    }
    require(scans==598&&received_scans==598&&imus==5986&&acks==scans,"incomplete exact input manifest");
    if(!inspect) {
      require(entered.load()==scans,"missing processed scans");
      if(!joined)source->wait(true);
      source->save((out/"glim_dump").string());
      glim::OdometryEstimationCallbacks::on_insert_frame.remove(callback_id);
      source.reset();
      // Allow queued native publications to drain before process teardown.
      std::this_thread::sleep_for(std::chrono::seconds(1));
      spinner->stop();
    }
    const fs::path result=out/(inspect?"coverage_complete.json":"driver_complete.json");
    require(!fs::exists(result),"refuse completion overwrite");
    std::ofstream(result)<<"{\"passed\":true,\"scans\":"<<scans<<",\"imus\":"<<imus
      <<",\"acks\":"<<acks<<",\"events\":"<<events<<",\"max_raw_scan_buffer\":"<<max_buffer<<"}\n";
    return 0;
  } catch(const std::exception& e) {
    std::cerr<<"CPK16H_DRIVER_FAILURE "<<e.what()<<std::endl;
    // Do not get stuck destroying a worker whose prerequisite failed; owned
    // controller preserves evidence and performs cleanup, with no retry.
    std::ofstream(out/"driver_failure.txt")<<e.what()<<'\n';
    std::_Exit(92);
  }
}
