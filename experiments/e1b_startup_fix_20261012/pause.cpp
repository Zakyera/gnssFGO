#include <csignal>
// Test-only preload: lets the host debugger attach before any application work.
__attribute__((constructor)) static void await_debugger() { raise(SIGSTOP); }
