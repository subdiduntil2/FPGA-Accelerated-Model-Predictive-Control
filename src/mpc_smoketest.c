/* mpc_smoketest.c -- Cora Z7-07S trajectory replay for the MPC IP. */

#include <stdio.h>
#include <stdint.h>

#include "xparameters.h"
#include "xil_io.h"
#include "xil_printf.h"
#include "xtime_l.h"
#include "sleep.h"

#include "trajectory_data.h"

#ifdef XPAR_MPC_CONTROLLER_AXI_0_S00_AXI_BASEADDR
  #define MPC_BASE_ADDR  XPAR_MPC_CONTROLLER_AXI_0_S00_AXI_BASEADDR
#else
  #warning "XPAR_MPC_CONTROLLER_AXI_0_S00_AXI_BASEADDR not found; using 0x43C00000"
  #define MPC_BASE_ADDR  0x43C00000u
#endif

#define MPC_OFF_CTRL    0x00u
#define MPC_OFF_STATUS  0x04u
#define MPC_OFF_X       0x10u
#define MPC_OFF_Y       0x14u
#define MPC_OFF_PSI     0x18u
#define MPC_OFF_V       0x1Cu
#define MPC_OFF_REF_X   0x20u
#define MPC_OFF_REF_Y   0x24u
#define MPC_OFF_ACCEL   0x30u
#define MPC_OFF_STEER   0x34u
#define MPC_OFF_TICK    0x40u

#define MPC_CTRL_KICK   (1u << 0)
#define MPC_CTRL_EN     (1u << 1)
#define MPC_STATUS_DONE (1u << 0)

static inline void     mpc_write(uint32_t off, uint32_t v) { Xil_Out32(MPC_BASE_ADDR + off, v); }
static inline uint32_t mpc_read (uint32_t off)             { return Xil_In32(MPC_BASE_ADDR + off); }

static inline uint64_t ticks_to_ns(uint64_t ticks) {
    return (ticks * 1000000000ull) / (uint64_t)COUNTS_PER_SECOND;
}

/* Sign-extend int16 result from the lower 16 bits of an AXI read */
static inline int32_t reg_to_s16(uint32_t r) {
    return (int32_t)(int16_t)(r & 0xFFFFu);
}

/* Output mode: SUMMARY (default) | VERBOSE | CSV (for compare_fpga_matlab.m) */
#define MODE_SUMMARY  0
#define MODE_VERBOSE  1
#define MODE_CSV      2

#ifndef OUTPUT_MODE
#define OUTPUT_MODE   MODE_SUMMARY
#endif

int main(void) {
    if (OUTPUT_MODE != MODE_CSV) {
        xil_printf("\r\n");
        xil_printf("=================================================\r\n");
        xil_printf(" Cora Z7-07S MPC trajectory validation\r\n");
        xil_printf(" MPC_BASE_ADDR    = 0x%08x\r\n", (unsigned)MPC_BASE_ADDR);
        xil_printf(" Global timer Hz  = %u\r\n",     (unsigned)COUNTS_PER_SECOND);
        xil_printf(" Trajectory len   = %d samples\r\n", TRAJECTORY_LEN);
        xil_printf("=================================================\r\n");

        /* AXI sanity check */
        xil_printf("\r\n[1] AXI write/read smoketest on REG_X (0x10)\r\n");
        mpc_write(MPC_OFF_X, 0x00001234u);
        uint32_t echo = mpc_read(MPC_OFF_X);
        xil_printf("    wrote 0x00001234, read 0x%08x  -> %s\r\n",
                   (unsigned)echo,
                   (echo == 0x00001234u) ? "OK" : "MISMATCH (BUS PROBLEM)");
        if (echo != 0x00001234u) {
            xil_printf("Aborting trajectory run.\r\n");
            while (1) sleep(1);
        }
    } else {
        xil_printf("idx,gx,gy,gpsi,gv,grx,gry,gacc,eacc,gstr,estr,pl_ns,e2_ns,ok\r\n");
    }

    mpc_write(MPC_OFF_CTRL, MPC_CTRL_EN);

    if (OUTPUT_MODE == MODE_SUMMARY) {
        xil_printf("\r\n[2] Wrapper enabled. Starting trajectory replay...\r\n");
    }

    /* Stats accumulators */
    uint64_t pl_min = UINT64_MAX, pl_max = 0, pl_sum = 0;
    uint64_t e2_min = UINT64_MAX, e2_max = 0, e2_sum = 0;
    int matches = 0;
    int mismatches_accel = 0;
    int mismatches_steer = 0;
    int valid = 0;
    int first_mismatch_idx = -1;

    XTime t_traj_start, t_traj_end;
    XTime_GetTime(&t_traj_start);

    if (OUTPUT_MODE == MODE_VERBOSE) {
        xil_printf("\r\n  idx | accel got/exp | steer got/exp |  PL ns | match\r\n");
        xil_printf(  "  ----+---------------+---------------+--------+------\r\n");
    }

    for (int t = 0; t < TRAJECTORY_LEN; t++) {
        XTime t0, t_kick, t_done, t1;

        XTime_GetTime(&t0);

        mpc_write(MPC_OFF_X,     (uint32_t)(int32_t)x_data[t]);
        mpc_write(MPC_OFF_Y,     (uint32_t)(int32_t)y_data[t]);
        mpc_write(MPC_OFF_PSI,   (uint32_t)(int32_t)psi_data[t]);
        mpc_write(MPC_OFF_V,     (uint32_t)(int32_t)v_data[t]);
        mpc_write(MPC_OFF_REF_X, (uint32_t)(int32_t)ref_x_data[t]);
        mpc_write(MPC_OFF_REF_Y, (uint32_t)(int32_t)ref_y_data[t]);

        XTime_GetTime(&t_kick);
        mpc_write(MPC_OFF_CTRL, MPC_CTRL_KICK | MPC_CTRL_EN);

        uint32_t spin = 0;
        while (!(mpc_read(MPC_OFF_STATUS) & MPC_STATUS_DONE)) {
            if (++spin > 1000000u) {
                xil_printf("# TIMEOUT at sample %d\r\n", t);
                goto done;
            }
        }
        XTime_GetTime(&t_done);

        uint32_t got_accel_raw = mpc_read(MPC_OFF_ACCEL);
        uint32_t got_steer_raw = mpc_read(MPC_OFF_STEER);

        XTime_GetTime(&t1);

        /* Input read-back (AXI bus integrity check, placed after t1) */
        int32_t bus_x   = reg_to_s16(mpc_read(MPC_OFF_X));
        int32_t bus_y   = reg_to_s16(mpc_read(MPC_OFF_Y));
        int32_t bus_psi = reg_to_s16(mpc_read(MPC_OFF_PSI));
        int32_t bus_v   = (int32_t)(mpc_read(MPC_OFF_V)     & 0x1FFu);
        int32_t bus_rx  = reg_to_s16(mpc_read(MPC_OFF_REF_X));
        int32_t bus_ry  = (int32_t)(mpc_read(MPC_OFF_REF_Y) & 0xFFFu);

        int32_t got_accel = reg_to_s16(got_accel_raw);
        int32_t got_steer = reg_to_s16(got_steer_raw);
        int32_t exp_accel = (int32_t)accel_exp[t];
        int32_t exp_steer = (int32_t)steer_exp[t];

        int accel_ok = (got_accel == exp_accel);
        int steer_ok = (got_steer == exp_steer);
        int both_ok  = accel_ok && steer_ok;

        if (both_ok)        matches++;
        if (!accel_ok)      mismatches_accel++;
        if (!steer_ok)      mismatches_steer++;
        if (!both_ok && first_mismatch_idx < 0) first_mismatch_idx = t;

        uint64_t pl_ns = ticks_to_ns(t_done - t_kick);
        uint64_t e2_ns = ticks_to_ns(t1 - t0);
        if (pl_ns < pl_min) pl_min = pl_ns;
        if (pl_ns > pl_max) pl_max = pl_ns;
        pl_sum += pl_ns;
        if (e2_ns < e2_min) e2_min = e2_ns;
        if (e2_ns > e2_max) e2_max = e2_ns;
        e2_sum += e2_ns;
        valid++;

        if (OUTPUT_MODE == MODE_CSV) {
            xil_printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%d\r\n",
                       t,
                       (int)bus_x, (int)bus_y, (int)bus_psi, (int)bus_v,
                       (int)bus_rx, (int)bus_ry,
                       (int)got_accel, (int)exp_accel,
                       (int)got_steer, (int)exp_steer,
                       (unsigned)pl_ns, (unsigned)e2_ns,
                       both_ok ? 1 : 0);
        } else {
            int near_edges = (t < 3) || (t >= TRAJECTORY_LEN - 3);
            if (OUTPUT_MODE == MODE_VERBOSE || !both_ok || near_edges) {
                xil_printf("  %3d | %4d / %4d  | %4d / %4d  | %6u | %s\r\n",
                           t,
                           (int)got_accel, (int)exp_accel,
                           (int)got_steer, (int)exp_steer,
                           (unsigned)pl_ns,
                           both_ok ? "OK"
                                   : (accel_ok ? "STEER"
                                               : (steer_ok ? "ACCEL" : "BOTH")));
            }
        }
    }

done:
    XTime_GetTime(&t_traj_end);
    uint64_t total_ns = ticks_to_ns(t_traj_end - t_traj_start);

    if (OUTPUT_MODE == MODE_CSV) {
        xil_printf("# done valid=%d matches=%d acc_diff=%d str_diff=%d first_mm=%d\r\n",
                   valid, matches, mismatches_accel, mismatches_steer, first_mismatch_idx);
        xil_printf("# pl_ns min=%u avg=%u max=%u\r\n",
                   (unsigned)pl_min, valid ? (unsigned)(pl_sum / valid) : 0u, (unsigned)pl_max);
        xil_printf("# e2_ns min=%u avg=%u max=%u\r\n",
                   (unsigned)e2_min, valid ? (unsigned)(e2_sum / valid) : 0u, (unsigned)e2_max);
        xil_printf("# total_us=%u\r\n", (unsigned)(total_ns / 1000ull));
        xil_printf("# tick_cnt=%u\r\n", (unsigned)mpc_read(MPC_OFF_TICK));
    } else {
        xil_printf("\r\n=================================================\r\n");
        xil_printf(" Trajectory complete: %d / %d samples ran\r\n", valid, TRAJECTORY_LEN);
        xil_printf("=================================================\r\n");

        xil_printf(" Match summary:\r\n");
        xil_printf("   both ok    : %d / %d  (%d%%)\r\n",
                   matches, valid, valid ? (matches * 100 / valid) : 0);
        xil_printf("   accel diff : %d\r\n", mismatches_accel);
        xil_printf("   steer diff : %d\r\n", mismatches_steer);
        if (first_mismatch_idx >= 0) {
            xil_printf("   first mismatch at sample index %d\r\n", first_mismatch_idx);
        }

        if (valid > 0) {
            xil_printf("\r\n Latency over %d samples:\r\n", valid);
            xil_printf("   PL-only    : min=%u  avg=%u  max=%u  ns\r\n",
                       (unsigned)pl_min, (unsigned)(pl_sum / valid), (unsigned)pl_max);
            xil_printf("   end-to-end : min=%u  avg=%u  max=%u  ns\r\n",
                       (unsigned)e2_min, (unsigned)(e2_sum / valid), (unsigned)e2_max);
            xil_printf("   total time : %u us  (avg %u us/sample)\r\n",
                       (unsigned)(total_ns / 1000ull),
                       (unsigned)(total_ns / 1000ull / (uint64_t)valid));
        }

        xil_printf("\r\nFinal TICK_CNT = %u\r\n", (unsigned)mpc_read(MPC_OFF_TICK));
    }

    xil_printf("\r\n=== done; entering idle loop ===\r\n");
    while (1) sleep(1);

    return 0;
}
