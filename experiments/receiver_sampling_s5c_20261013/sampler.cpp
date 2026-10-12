// Private observation-only preload. Target executable and its math are unchanged.
#define _GNU_SOURCE
#define UNW_LOCAL_ONLY
#include <libunwind.h>
#include <signal.h>
#include <time.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#ifndef S5C_MAX_FRAMES
#define S5C_MAX_FRAMES 4096
#endif

namespace {
struct Record {
  uint64_t monotonic_ns, realtime_ns, thread_cpu_ns, unwind_cpu_ns;
  int32_t depth, unwind_error, overrun, tid;
  uint64_t pc[S5C_MAX_FRAMES];
};
static_assert(sizeof(Record)==48+8*S5C_MAX_FRAMES);
int fd=-1, signal_number=SIGPROF;
timer_t timer_id{};
char directory[4096]{};
uint64_t begin_cpu=0, begin_wall=0, interval=10000000;
volatile sig_atomic_t samples=0, failures=0, overruns=0, truncated=0;
struct sigaction previous{};
uint64_t now(clockid_t c) { timespec t{};clock_gettime(c,&t);return uint64_t(t.tv_sec)*1000000000+t.tv_nsec; }
void handler(int, siginfo_t* info, void* ctx) {
  int saved_errno=errno;
  Record r{};
  r.monotonic_ns=now(CLOCK_MONOTONIC);r.realtime_ns=now(CLOCK_REALTIME);
  r.thread_cpu_ns=now(CLOCK_THREAD_CPUTIME_ID);
  r.tid=int32_t(syscall(SYS_gettid));r.overrun=info->si_overrun;
  unw_cursor_t cursor;
  int status=unw_init_local2(&cursor,reinterpret_cast<unw_context_t*>(ctx),UNW_INIT_SIGNAL_FRAME);
  if(status>=0) do {
    unw_word_t ip=0;
    status=unw_get_reg(&cursor,UNW_REG_IP,&ip);
    if(status<0)break;
    r.pc[r.depth++]=uint64_t(ip);
    status=unw_step(&cursor);
  } while(status>0 && r.depth<S5C_MAX_FRAMES);
  r.unwind_error=status<0?status:0;
  r.unwind_cpu_ns=now(CLOCK_THREAD_CPUTIME_ID)-r.thread_cpu_ns;
  if(r.depth==S5C_MAX_FRAMES && status>0)truncated=truncated+1;
  const size_t length=48+8*r.depth;
  if(write(fd,&r,length)!=ssize_t(length))failures=failures+1;
  samples=samples+1;overruns=overruns+r.overrun;
  errno=saved_errno;
}
int create(const char* suffix) {
  char path[8192];snprintf(path,sizeof(path),"%s/%s",directory,suffix);
  int result=open(path,O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);
  if(result<0){perror(path);_exit(90);}return result;
}
void maps(const char* suffix) {
  int out=create(suffix),in=open("/proc/self/maps",O_RDONLY|O_CLOEXEC);
  if(in<0)_exit(91);
  char buf[8192];ssize_t n;
  while((n=read(in,buf,sizeof(buf)))>0)if(write(out,buf,n)!=n)_exit(92);
  close(in);close(out);
}
__attribute__((constructor)) void start() {
  const char* path=getenv("S5C_SAMPLER_OUTPUT");if(!path)return;
  if(strlen(path)>=sizeof(directory))_exit(93);
  strcpy(directory,path);
  if(const char* value=getenv("S5C_INTERVAL_NS"))interval=strtoull(value,nullptr,10);
  if(interval<1000000 || interval>100000000)_exit(94);
  sigaction(signal_number,nullptr,&previous);
  if(previous.sa_handler!=SIG_DFL)_exit(95);
  fd=create("samples.bin");maps("maps_start.txt");
  unw_set_caching_policy(unw_local_addr_space,UNW_CACHE_NONE);
  struct sigaction action{};action.sa_sigaction=handler;action.sa_flags=SA_SIGINFO|SA_RESTART;
  sigemptyset(&action.sa_mask);
  if(sigaction(signal_number,&action,nullptr))_exit(96);
  sigevent event{};event.sigev_notify=SIGEV_THREAD_ID;event.sigev_signo=signal_number;
  event._sigev_un._tid=syscall(SYS_gettid);
  if(timer_create(CLOCK_THREAD_CPUTIME_ID,&event,&timer_id))_exit(97);
  begin_cpu=now(CLOCK_THREAD_CPUTIME_ID);begin_wall=now(CLOCK_MONOTONIC);
  FILE* f=fdopen(create("start.json"),"w");
  fprintf(f,"{\"pid\":%d,\"tid\":%ld,\"interval_ns\":%lu,\"record_size\":0,\"maximum_frames\":%d,\"monotonic_ns\":%lu,\"realtime_ns\":%lu,\"thread_cpu_ns\":%lu}\n",getpid(),syscall(SYS_gettid),interval,S5C_MAX_FRAMES,begin_wall,now(CLOCK_REALTIME),begin_cpu);fclose(f);
  itimerspec ts{};ts.it_value.tv_nsec=interval;ts.it_interval.tv_nsec=interval;
  if(timer_settime(timer_id,0,&ts,nullptr))_exit(98);
}
__attribute__((destructor)) void finish() {
  if(fd<0)return;
  sigset_t mask;sigemptyset(&mask);sigaddset(&mask,signal_number);
  sigprocmask(SIG_BLOCK,&mask,nullptr);timer_delete(timer_id);
  struct sigaction final_action{};sigaction(signal_number,nullptr,&final_action);
  maps("maps_end.txt");close(fd);
  FILE* f=fdopen(create("end.json"),"w");
  fprintf(f,"{\"samples\":%d,\"write_failures\":%d,\"overruns\":%d,\"truncated\":%d,\"handler_unchanged\":%s,\"thread_cpu_ns\":%lu,\"elapsed_wall_ns\":%lu}\n",samples,failures,overruns,truncated,final_action.sa_sigaction==handler?"true":"false",now(CLOCK_THREAD_CPUTIME_ID)-begin_cpu,now(CLOCK_MONOTONIC)-begin_wall);fclose(f);
  sigaction(signal_number,&previous,nullptr);
  // Keep SIGPROF blocked during process teardown; do not deliver stale samples.
}
}
