// x86 FPU activity-trace drain — see x86_fpu_trace.cpp.
#ifndef X86_FPU_TRACE_H
#define X86_FPU_TRACE_H

// Poll the ao486 fpu_trace counters (UIO class 0xF7) and append a row to
// /tmp/ao486_fpu.csv whenever FPU activity changed since the last poll.
// Safe no-op on a stock/old RBF that lacks the channel (magic word absent).
void x86_fpu_trace_drain(void);

#endif
