// AS2 SDK process: no ROS dependency, stdin commands, stdout JSON telemetry.
#include <unitree/robot/as2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <unistd.h>

using Clock = std::chrono::steady_clock;
std::atomic<bool> running{true};
void signal_handler(int) { running = false; }
int main(int argc, char** argv) {
  if (argc < 2 || argc > 3 || (argc == 3 && std::string(argv[2]) != "--enable")) {
    std::cerr << "usage: as2_agent interface [--enable]\n"; return 2;
  }
  const bool enabled = argc == 3;
  signal(SIGINT, signal_handler); signal(SIGTERM, signal_handler);
  signal(SIGPIPE, signal_handler);
  // Prevent stalled SSH stdout from blocking the watchdog/control loop.
  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
  fcntl(STDOUT_FILENO, F_SETFL, O_NONBLOCK);
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
  unitree::robot::as2::SportClient client;
  if (enabled) { client.SetTimeout(0.1f); client.Init(); }
  std::mutex mutex;
  double x=0, y=0, yaw=0;
  bool have_state=false;
  int64_t last_stamp=-1;
  uint32_t error_code=0;
  auto state_time=Clock::now()-std::chrono::seconds(10);
  unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> sub("rt/lf/sportmodestate");
  sub.InitChannel([&](const void* message) {
    auto& s = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);
    std::lock_guard<std::mutex> lock(mutex);
    int64_t stamp=static_cast<int64_t>(s.stamp().sec())*1000000000LL+s.stamp().nanosec();
    if (stamp!=0 && stamp<=last_stamp) return;
    last_stamp=stamp; have_state=true; error_code=s.error_code();
    x=s.position()[0]; y=s.position()[1]; yaw=s.imu_state().rpy()[2];
    state_time=Clock::now();
  }, 1);
  std::string buffer;
  long long token=0, last_command_token=-1;
  auto command_time=Clock::now()-std::chrono::seconds(10);
  double vx=0, vy=0, wz=0;
  bool fault=false;
  int rc=0;
  while (running) {
    auto start=Clock::now();
    char bytes[4096];
    // Bounded input work. Unconsumed/backlogged tokens expire below.
    auto n=read(STDIN_FILENO, bytes, sizeof(bytes));
    if (n==0) break;
    if (n>0) buffer.append(bytes, n);
    if (buffer.size()>16384) { fault=true; buffer.clear(); }
    size_t end;
    while ((end=buffer.find('\n'))!=std::string::npos) {
      auto line=buffer.substr(0,end); buffer.erase(0,end+1);
      if (line=="STOP") { running=false; break; }
      std::istringstream in(line);
      long long echo; double a,b,c; std::string extra;
      if (!(in>>echo>>a>>b>>c) || (in>>extra) || !std::isfinite(a) ||
          !std::isfinite(b) || !std::isfinite(c) || std::hypot(a,b)>0.121 || std::abs(c)>0.251) {
        fault=true; continue;
      }
      // Echo of a recent remote-issued token prevents replay of SSH backlog.
      if (echo<=last_command_token || echo>token || token-echo>4) continue;
      last_command_token=echo; vx=a; vy=b; wz=c; command_time=start;
    }
    if (!running) break;
    double px,py,pa,age; bool received_state; uint32_t error; int64_t source_stamp;
    {
      std::lock_guard<std::mutex> lock(mutex);
      px=x; py=y; pa=yaw; received_state=have_state; error=error_code; source_stamp=last_stamp;
      age=std::chrono::duration<double>(start-state_time).count();
    }
    bool valid=received_state && age<0.3 && std::isfinite(px) && std::isfinite(py) && std::isfinite(pa);
    if (error!=0) fault=true;
    bool active=valid && !fault && std::chrono::duration<double>(start-command_time).count()<0.25;
    auto call_start=Clock::now();
    if (enabled) {
      rc=active ? client.Move(vx,vy,wz) : client.StopMove();
      if (rc!=0) { fault=true; client.StopMove(); }
    }
    double call_ms=std::chrono::duration<double,std::milli>(Clock::now()-call_start).count();
    ++token;
    std::ostringstream out;
    out << "{\"kind\":\"state\",\"token\":" << token
        << ",\"valid\":" << (valid?"true":"false")
        << ",\"odom\":[" << (valid?px:0) << ',' << (valid?py:0) << ',' << (valid?pa:0)
        << "],\"enabled\":" << (enabled?"true":"false")
        << ",\"accepted_token\":" << last_command_token
        << ",\"active\":" << (active?"true":"false")
        << ",\"applied_body\":[" << (enabled && active && rc==0?vx:0) << ','
        << (enabled && active && rc==0?vy:0) << ',' << (enabled && active && rc==0?wz:0) << ']'
        << ",\"sdk_call_ms\":" << call_ms
        << ",\"source_stamp_ns\":" << source_stamp << ",\"error_code\":" << error << ",\"fault\":" << (fault?"true":"false") << ",\"rc\":" << rc << "}\n";
    auto data=out.str();
    auto written=write(STDOUT_FILENO,data.data(),data.size());
    if (written!=static_cast<ssize_t>(data.size())) { fault=true; }
    std::this_thread::sleep_until(start+std::chrono::milliseconds(50));
  }
  // Repeated stop requests tolerate a transient transport timeout on exit.
  int stop_rc=0;
  if (enabled) {
    for (int attempt=0; attempt<3; ++attempt) {
      stop_rc=client.StopMove();
      if (stop_rc==0) break;
    }
  }
  std::string ack="{\"kind\":\"stopped\",\"rc\":"+std::to_string(stop_rc)+"}\n";
  if (write(STDOUT_FILENO,ack.data(),ack.size())!=static_cast<ssize_t>(ack.size()))
    std::cerr << "Could not deliver stop acknowledgement\n";
}
