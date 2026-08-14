// Video-mode trace drain (debug): proves on real silicon WHICH video modes the
// ao486 core emits, what the in-core scandoubler does, and whether the final
// analog pin-side path still has valid timing/RGB.
//
// RTL exposes two compatible read-only trace channels:
//   0xF8 = core/native video -> post-scandoubler/video_mixer
//   0xF9 = post-scandoubler/video_mixer -> sys_top final analog mux/pins
//
// Both channels use rtl/vid_trace.v. The first ten words are timing-compatible
// with iter-199/200; words 10-15 add per-frame RGB OR values so a black screen
// can be separated into "drawing black" vs "valid RGB lost later".

#include "x86_vid_trace.h"

#include <cstdio>
#include <cstdint>
#include <ctime>

#include "../../spi.h"
#include "../../user_io.h"

static constexpr uint32_t VID_TRACE_ADDR   = 0xF800;
static constexpr uint32_t PIN_TRACE_ADDR   = 0xF900;
static constexpr uint16_t VID_TRACE_MAGIC  = 0xFA87;
static constexpr double   CLK_VGA_HZ       = 90000000.0;
static constexpr char     VID_OUT_PATH[]   = "/tmp/ao486_vid.csv";
static constexpr char     PIN_OUT_PATH[]   = "/tmp/ao486_pin.csv";

struct VidTraceRaw {
	uint16_t in_ht;
	uint16_t in_vt;
	uint16_t in_va;
	uint16_t in_ha;
	uint16_t out_ht;
	uint16_t out_vt;
	uint16_t out_va;
	uint16_t out_ha;
	uint16_t flags;
	uint8_t  in_ro;
	uint8_t  in_go;
	uint8_t  in_bo;
	uint8_t  out_ro;
	uint8_t  out_go;
	uint8_t  out_bo;
};

struct VidTraceSig {
	uint16_t in_ht_q;
	uint16_t in_vt_q;
	uint16_t in_va;
	uint16_t in_ha;
	uint16_t out_ht_q;
	uint16_t out_vt_q;
	uint16_t out_va;
	uint16_t out_ha;
	uint16_t flags;
	uint8_t  in_rgb_nz;
	uint8_t  out_rgb_nz;
};

struct TraceState {
	FILE       *csv;
	uint32_t    seq;
	bool        have_last;
	bool        have_prev;
	uint8_t     stable_reads;
	VidTraceSig last_sig;
	VidTraceSig prev_sig;
	const char *path;
};

static TraceState g_vid_state = {nullptr, 0, false, false, 0, {}, {}, VID_OUT_PATH};
static TraceState g_pin_state = {nullptr, 0, false, false, 0, {}, {}, PIN_OUT_PATH};

static uint16_t quantize(uint16_t v, uint16_t step)
{
	return step ? (uint16_t)((v + (step / 2)) / step) : v;
}

static uint32_t rgb_or_in(const VidTraceRaw &r)
{
	return ((uint32_t)r.in_ro << 16) | ((uint32_t)r.in_go << 8) | r.in_bo;
}

static uint32_t rgb_or_out(const VidTraceRaw &r)
{
	return ((uint32_t)r.out_ro << 16) | ((uint32_t)r.out_go << 8) | r.out_bo;
}

static VidTraceSig make_sig(const VidTraceRaw &r)
{
	return {
		quantize(r.in_ht, 16), quantize(r.in_vt, 4),
		r.in_va, r.in_ha,
		quantize(r.out_ht, 16), quantize(r.out_vt, 4),
		r.out_va, r.out_ha,
		r.flags,
		(uint8_t)(rgb_or_in(r) != 0),
		(uint8_t)(rgb_or_out(r) != 0)
	};
}

static bool sig_equal(const VidTraceSig &a, const VidTraceSig &b)
{
	return a.in_ht_q    == b.in_ht_q    && a.in_vt_q    == b.in_vt_q    &&
	       a.in_va      == b.in_va      && a.in_ha      == b.in_ha      &&
	       a.out_ht_q   == b.out_ht_q   && a.out_vt_q   == b.out_vt_q   &&
	       a.out_va     == b.out_va     && a.out_ha     == b.out_ha     &&
	       a.flags      == b.flags      &&
	       a.in_rgb_nz  == b.in_rgb_nz  && a.out_rgb_nz == b.out_rgb_nz;
}

static void open_csv_if_needed(TraceState &st)
{
	if (st.csv) return;
	st.csv = fopen(st.path, "w");
	if (!st.csv) return;
	fprintf(st.csv,
		"seq,epoch_s,in_w,in_h,in_lkHz,in_fHz,out_w,out_h,out_lkHz,out_fHz,sd_en,in_rgb,out_rgb\n");
	fflush(st.csv);
}

static bool read_trace(uint32_t addr, VidTraceRaw &raw)
{
	uint16_t w[16];

	EnableIO();
	spi8(UIO_DMA_READ);
	spi32_w(addr);
	for (unsigned i = 0; i < 16; i++) w[i] = spi_w(0);
	DisableIO();

	if (w[9] != VID_TRACE_MAGIC) return false;   // channel absent (stock/old RBF)

	raw = {
		w[0], w[1], w[2], w[3],
		w[4], w[5], w[6], w[7],
		(uint16_t)(w[8] & 1),
		(uint8_t)w[10], (uint8_t)w[11], (uint8_t)w[12],
		(uint8_t)w[13], (uint8_t)w[14], (uint8_t)w[15]
	};
	return true;
}

static void drain_trace(uint32_t addr, TraceState &st)
{
	VidTraceRaw raw = {};
	if (!read_trace(addr, raw)) return;

	// Drop incomplete/transitional snapshots; they are common during mode switches.
	if (!raw.in_ht || !raw.in_vt || !raw.in_va || !raw.in_ha ||
	    !raw.out_ht || !raw.out_vt || !raw.out_va || !raw.out_ha) {
		return;
	}

	VidTraceSig sig = make_sig(raw);

	// Anti-tearing and mode-switch debounce: snapshot regs cross clk_vga->mgmt
	// async, and video_mixer h_total can wobble by a tick or two. Require
	// several quantized-equal reads before logging a new settled mode.
	if (!st.have_prev || !sig_equal(sig, st.prev_sig)) {
		st.prev_sig = sig;
		st.have_prev = true;
		st.stable_reads = 1;
		return;
	}
	if (st.stable_reads < 8) { st.stable_reads++; return; }
	if (st.have_last && sig_equal(sig, st.last_sig)) return;   // already logged

	st.have_last = true;
	st.last_sig  = sig;

	double in_lk  = raw.in_ht  ? CLK_VGA_HZ / 1000.0 / raw.in_ht  : 0.0;
	double in_fz  = (raw.in_ht  && raw.in_vt)  ? CLK_VGA_HZ / ((double)raw.in_ht  * raw.in_vt)  : 0.0;
	double out_lk = raw.out_ht ? CLK_VGA_HZ / 1000.0 / raw.out_ht : 0.0;
	double out_fz = (raw.out_ht && raw.out_vt) ? CLK_VGA_HZ / ((double)raw.out_ht * raw.out_vt) : 0.0;

	open_csv_if_needed(st);
	if (!st.csv) return;

	fprintf(st.csv, "%u,%ld,%u,%u,%.2f,%.2f,%u,%u,%.2f,%.2f,%u,%06X,%06X\n",
	        st.seq++, (long)time(nullptr),
	        raw.in_ha,  raw.in_va,  in_lk,  in_fz,
	        raw.out_ha, raw.out_va, out_lk, out_fz,
	        (unsigned)(raw.flags & 1),
	        (unsigned)rgb_or_in(raw),
	        (unsigned)rgb_or_out(raw));
	fflush(st.csv);
}

void x86_vid_trace_drain(void)
{
	drain_trace(VID_TRACE_ADDR, g_vid_state);
	drain_trace(PIN_TRACE_ADDR, g_pin_state);
}
