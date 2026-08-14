#include "scheduler.h"
#include <stdio.h>
#include "libco.h"
#include "menu.h"
#include "user_io.h"
#include "input.h"
#include "frame_timer.h"
#include "fpga_io.h"
#include "osd.h"
#include "profiling.h"
#include "video.h"

static cothread_t co_scheduler = nullptr;
static cothread_t co_poll = nullptr;
static cothread_t co_ui = nullptr;
static cothread_t co_last = nullptr;

static void scheduler_wait_fpga_ready(void)
{
	while (!is_fpga_ready(1))
	{
		fpga_wait_to_reset();
	}
}

// TEMP iter-365 diag: confirm both cothreads are actually cycling (vs. one
// getting stuck without ever calling scheduler_yield()). Remove once the S3
// trace investigation is resolved.
static void scheduler_diag_tick(int is_poll)
{
	static FILE *g_f = nullptr;
	static uint32_t g_poll_n = 0, g_ui_n = 0;
	if (!g_f) g_f = fopen("/tmp/ao486_s3sched.csv", "w");
	uint32_t *n = is_poll ? &g_poll_n : &g_ui_n;
	if (g_f && (*n < 5 || (*n % 200) == 0)) {
		fprintf(g_f, "%s,%u,poll=%u,ui=%u\n", is_poll ? "poll-enter" : "ui-enter", *n, g_poll_n, g_ui_n);
		fflush(g_f);
	}
	(*n)++;
}

static void scheduler_co_poll(void)
{
	for (;;)
	{
		scheduler_diag_tick(1);
		scheduler_wait_fpga_ready();

		{
			SPIKE_SCOPE("co_poll", 1000);
			user_io_poll();
			frame_timer();
			input_poll(0);
			video_poll();
		}

		scheduler_yield();
	}
}

static void scheduler_co_ui(void)
{
	for (;;)
	{
		scheduler_diag_tick(0);

		{
			SPIKE_SCOPE("co_ui", 1000);
			HandleUI();
			OsdUpdate();
		}

		scheduler_yield();
	}
}

static void scheduler_schedule(void)
{
	if (co_last == co_poll)
	{
		co_last = co_ui;
		co_switch(co_ui);
	}
	else
	{
		co_last = co_poll;
		co_switch(co_poll);
	}
}

void scheduler_init(void)
{
	const unsigned int co_stack_size = 262144 * sizeof(void*);

	co_poll = co_create(co_stack_size, scheduler_co_poll);
	co_ui = co_create(co_stack_size, scheduler_co_ui);
}

void scheduler_run(void)
{
	co_scheduler = co_active();

	for (;;)
	{
		scheduler_schedule();
	}

	co_delete(co_ui);
	co_delete(co_poll);
	co_delete(co_scheduler);
}

void scheduler_yield(void)
{
	co_switch(co_scheduler);
}
