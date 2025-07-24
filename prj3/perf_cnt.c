#include "perf_cnt.h"

#define PERF_CNT_BASE      0x60010000
#define PERF_CNT_OFFSET_1  0x0008

unsigned long _uptime() {
  // TODO [COD]
  //   You can use this function to access performance counter related with time or cycle.
  volatile unsigned long *cycle_counter = (volatile unsigned long *)PERF_CNT_BASE;
  return *cycle_counter;
}

unsigned long _ins() {
    volatile unsigned long *ins_counter = (volatile unsigned long *)(PERF_CNT_BASE + PERF_CNT_OFFSET_1);
    return *ins_counter;
}

void bench_prepare(Result *res) {
  // TODO [COD]
  //   Add preprocess code, record performance counters' initial states.
  //   You can communicate between bench_prepare() and bench_done() through
  //   static variables or add additional fields in `struct Result`
  res->msec = _uptime();
  res->ins = _ins();
}

void bench_done(Result *res) {
  // TODO [COD]
  //  Add postprocess code, record performance counters' current states.
  res->msec = _uptime() - res->msec;
  res->ins  = _ins() - res->ins; 
}
