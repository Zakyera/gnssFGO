#include <cmath>
#include <cstdio>
#include <ctime>
__attribute__((noinline)) double hot_loop(double x) {
  for(int i=0;i<200000;++i) x=std::sin(x+.123)+std::cos(x+.234);
  return x;
}
int main() {
  double x=.25;
  const double start=double(std::clock())/CLOCKS_PER_SEC;
  while(double(std::clock())/CLOCKS_PER_SEC-start<2.0) x=hot_loop(x);
  std::printf("%.17g\n",x);
}
