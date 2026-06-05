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
static constexpr char     OUT_PATH[]      = "/tmp/ao486_fpu.csv";

static FILE *g_csv = nullptr;
static uint32_t g_seq = 0;
static bool     g_init = false;
static uint32_t g_last_total = 0;
static uint32_t g_last_transc = 0;

static void open_csv_if_needed(void)
{
	if (g_csv) return;
	g_csv = fopen(OUT_PATH, "w");
	if (!g_csv) return;
	fprintf(g_csv, "seq,epoch_s,total_ops,transc_ops,d_total,d_transc\n");
	fflush(g_csv);
}

void x86_fpu_trace_drain(void)
{
	EnableIO();
	spi8(UIO_DMA_READ);
	spi32_w(FPU_TRACE_ADDR);
	uint16_t w0 = spi_w(0);   // total_ops[15:0]
	uint16_t w1 = spi_w(0);   // total_ops[31:16]
	uint16_t w2 = spi_w(0);   // transc_ops[15:0]
	uint16_t w3 = spi_w(0);   // transc_ops[31:16]
	uint16_t w4 = spi_w(0);   // magic
	DisableIO();

	if (w4 != FPU_TRACE_MAGIC) return;   // channel absent (stock RBF) or misaligned

	uint32_t total  = (uint32_t)w0 | ((uint32_t)w1 << 16);
	uint32_t transc = (uint32_t)w2 | ((uint32_t)w3 << 16);

	if (!g_init) {
		g_init = true;
		g_last_total  = total;
		g_last_transc = transc;
		return;   // establish baseline; don't log the cold value
	}

	uint32_t d_total  = total  - g_last_total;
	uint32_t d_transc = transc - g_last_transc;
	if (!d_total && !d_transc) return;   // no FPU activity this interval -> no row

	open_csv_if_needed();
	if (!g_csv) return;

	fprintf(g_csv, "%u,%ld,%u,%u,%u,%u\n",
	        g_seq++, (long)time(nullptr), total, transc, d_total, d_transc);
	fflush(g_csv);

	g_last_total  = total;
	g_last_transc = transc;
}
