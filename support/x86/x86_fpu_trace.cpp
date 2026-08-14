// FPU activity-trace drain (debug): proves on real silicon whether software
// actually dispatches the x87 FPU — and the transcendental group specifically
// — without any cooperation from the running guest (DOS/the game).
//
// The ao486 core carries a tiny read-only counter peripheral (rtl/fpu_trace.v)
// on the HPS management bus at UIO class 0xF7. It holds two free-running 32-bit
// counters: total x87 ops retired, and transcendental ops retired
// (F2XM1/FYL2X/FYL2XP1/FPTAN/FPATAN/FSIN/FCOS/FSINCOS). This drain — called from
// x86_poll() every frame — reads them over the FPGA<->HPS bridge (the same
// mechanism the CD32 chipset_trace uses) and appends a row to
// /tmp/ao486_fpu.csv whenever a counter changed. Because the ARM reads the
// counters directly, it works while a full-screen game runs: you don't read
// from DOS, the HPS does it behind the guest's back.
//
// Readout (UIO_DMA_READ 0x62 @ 0xF700; hps_ext auto-increments the low byte):
//   word 0: total_ops[15:0]    word 1: total_ops[31:16]
//   word 2: transc_ops[15:0]   word 3: transc_ops[31:16]
//   word 4: MAGIC 0xFA86       (channel sanity — if absent, drain is a no-op)
//   word 15: MAGIC 0xFA87      (extended iter-204 snapshot fields present)
//
// Iter-205 keeps the same readout width. The high byte of last_exc_info is now
// {exc_src[4:0], flags[2:0]} where flags are {push_error, soft_int,
// soft_int_ib}. exc_src names the pipeline fault source that produced the
// vector/error pair.
//
// On a stock/old RBF that lacks the channel, word 4 reads back as open-bus and
// !=0xFA86, so this costs one SPI round-trip per poll and zero file I/O.

#include "x86_fpu_trace.h"

#include <cstdio>
#include <cstdint>
#include <ctime>

#include "../../spi.h"
#include "../../user_io.h"

// UIO class 0xF7 -> mgmt_address[15:8]==0xF7 (rtl/system.v).
static constexpr uint32_t FPU_TRACE_ADDR = 0xF700;
static constexpr uint16_t FPU_TRACE_MAGIC = 0xFA86;
static constexpr uint16_t FPU_TRACE_EXT_MAGIC = 0xFA87;
static constexpr uint32_t EXC_STORM_LOG_STEP = 10000;
static constexpr char     OUT_PATH[]      = "/tmp/ao486_fpu.csv";

static FILE *g_csv = nullptr;
static uint32_t g_seq = 0;
static bool     g_init = false;
static uint32_t g_last_total = 0;
static uint32_t g_last_transc = 0;
static uint32_t g_last_exc_count = 0;
static bool     g_have_seen_exc = false;
static uint32_t g_last_seen_exc_eip = 0;
static uint32_t g_last_seen_exc_info = 0;
static uint32_t g_last_logged_exc_count = 0;
static time_t   g_last_logged_exc_epoch = 0;
static time_t   g_last_logged_epoch = 0;

static const char *exc_src_name(uint32_t src)
{
	switch (src) {
	case 1:  return "wr_new_push_ss";
	case 2:  return "wr_string_es";
	case 3:  return "wr_push_ss";
	case 4:  return "write_ac";
	case 5:  return "write_pf";
	case 6:  return "wr_int";
	case 7:  return "exe_div";
	case 8:  return "exe_gp";
	case 9:  return "exe_ts";
	case 10: return "exe_ss";
	case 11: return "exe_np";
	case 12: return "exe_nm";
	case 13: return "exe_db";
	case 14: return "exe_pf";
	case 15: return "exe_bound";
	case 16: return "exe_load_seg_gp";
	case 17: return "exe_load_seg_ss";
	case 18: return "exe_load_seg_np";
	case 19: return "rd_seg_gp";
	case 20: return "rd_descriptor_gp";
	case 21: return "rd_seg_ss";
	case 22: return "rd_io_allow";
	case 23: return "rd_ss_esp_from_tss";
	case 24: return "read_ac";
	case 25: return "read_pf";
	case 26: return "dec_gp";
	case 27: return "dec_ud";
	case 28: return "dec_pf";
	default: return "unknown";
	}
}

static void open_csv_if_needed(void)
{
	if (g_csv) return;
	g_csv = fopen(OUT_PATH, "w");
	if (!g_csv) return;
	fprintf(g_csv, "seq,epoch_s,total_ops,transc_ops,d_total,d_transc,reset,last_fpu_eip,last_fpu_info,exc_count,d_exc,last_exc_eip,last_exc_info,exc_src,exc_src_name,exc_flags,exc_vector,exc_error\n");
	fflush(g_csv);
}

void x86_fpu_trace_drain(void)
{
	EnableIO();
	spi8(UIO_DMA_READ);
	spi32_w(FPU_TRACE_ADDR);
	uint16_t w[16];
	for (int i = 0; i < 16; i++) w[i] = spi_w(0);
	DisableIO();

	if (w[4] != FPU_TRACE_MAGIC) return; // channel absent (stock RBF) or misaligned

	uint32_t total  = (uint32_t)w[0] | ((uint32_t)w[1] << 16);
	uint32_t transc = (uint32_t)w[2] | ((uint32_t)w[3] << 16);
	bool ext = (w[15] == FPU_TRACE_EXT_MAGIC);

	uint32_t last_fpu_eip  = ext ? ((uint32_t)w[5]  | ((uint32_t)w[6]  << 16)) : 0;
	uint32_t last_fpu_info = ext ? ((uint32_t)w[7]  | ((uint32_t)w[8]  << 16)) : 0;
	uint32_t exc_count     = ext ? ((uint32_t)w[9]  | ((uint32_t)w[10] << 16)) : 0;
	uint32_t last_exc_eip  = ext ? ((uint32_t)w[11] | ((uint32_t)w[12] << 16)) : 0;
	uint32_t last_exc_info = ext ? ((uint32_t)w[13] | ((uint32_t)w[14] << 16)) : 0;

	if (!g_init) {
		g_init = true;
		g_last_total  = total;
		g_last_transc = transc;
		g_last_exc_count = exc_count;
		g_have_seen_exc = ext;
		g_last_seen_exc_eip = last_exc_eip;
		g_last_seen_exc_info = last_exc_info;
		g_last_logged_exc_count = exc_count;
		return;   // establish baseline; don't log the cold value
	}

	bool reset_seen = (total < g_last_total) || (transc < g_last_transc) || (ext && exc_count < g_last_exc_count);
	uint32_t d_total  = reset_seen ? total     : (total     - g_last_total);
	uint32_t d_transc = reset_seen ? transc    : (transc    - g_last_transc);
	uint32_t d_exc    = reset_seen ? exc_count : (exc_count - g_last_exc_count);

	time_t now = time(nullptr);
	uint32_t exc_src    = (last_exc_info >> 27) & 0x1F;
	uint32_t exc_flags  = (last_exc_info >> 24) & 0x7;
	uint32_t exc_vector = (last_exc_info >> 16) & 0xFF;
	uint32_t exc_error  = last_exc_info & 0xFFFF;

	bool exc_pair_changed = !g_have_seen_exc ||
	                        last_exc_eip != g_last_seen_exc_eip ||
	                        last_exc_info != g_last_seen_exc_info;
	bool log_exc = ext && (d_exc || reset_seen) &&
	               (reset_seen || exc_pair_changed ||
	                (exc_count - g_last_logged_exc_count) >= EXC_STORM_LOG_STEP ||
	                now != g_last_logged_exc_epoch);
	// d_total is deliberately excluded from the trigger: on the s3_trace.v
	// channel, word0/1 ("total_ops" in this FPU-trace layout) carries a
	// free-running per-cycle heartbeat, so it differs on essentially every
	// poll. Triggering on it turns every poll into a disk write -- at ~60Hz
	// that filled the 247MB /tmp tmpfs in well under an hour and wedged the
	// whole process inside a stalled fflush()/write() (iter-365 postmortem).
	// d_transc (S3 stuck_cycles+status, or the real FPU transcendental
	// counter) plus a 2s wall-clock heartbeat give liveness without the flood.
	bool log_row = reset_seen || d_transc || log_exc || (now - g_last_logged_epoch) >= 2;

	if (!log_row) {
		g_last_total  = total;
		g_last_transc = transc;
		g_last_exc_count = exc_count;
		if (ext && d_exc) {
			g_have_seen_exc = true;
			g_last_seen_exc_eip = last_exc_eip;
			g_last_seen_exc_info = last_exc_info;
		}
		return;
	}

	open_csv_if_needed();
	if (!g_csv) return;
	g_last_logged_epoch = now;

	fprintf(g_csv, "%u,%ld,%u,%u,%u,%u,%u,%08X,%08X,%u,%u,%08X,%08X,%u,%s,%u,%u,%u\n",
	        g_seq++, (long)now, total, transc, d_total, d_transc,
	        reset_seen ? 1U : 0U, last_fpu_eip, last_fpu_info,
	        exc_count, d_exc, last_exc_eip, last_exc_info,
	        exc_src, exc_src_name(exc_src), exc_flags, exc_vector, exc_error);
	fflush(g_csv);

	g_last_total  = total;
	g_last_transc = transc;
	g_last_exc_count = exc_count;
	if (ext && (d_exc || reset_seen)) {
		g_have_seen_exc = true;
		g_last_seen_exc_eip = last_exc_eip;
		g_last_seen_exc_info = last_exc_info;
		if (log_exc || reset_seen) {
			g_last_logged_exc_count = exc_count;
			g_last_logged_exc_epoch = now;
		}
	}
}
