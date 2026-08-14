// Video-mode trace drain (debug): logs only *unique* video modes the ao486 core
// produces (and what the in-core scandoubler does to them) to /tmp/ao486_vid.csv.
// See x86_vid_trace.cpp for the channel layout.

#ifndef X86_VID_TRACE_H
#define X86_VID_TRACE_H

void x86_vid_trace_drain(void);

#endif
