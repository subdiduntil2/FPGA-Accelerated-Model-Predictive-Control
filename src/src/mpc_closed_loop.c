/* Closed-loop validation harness for the FCS-MPC controller.
 * Backend (controller source): bare-metal ARM target (the Cora) -> FPGA, query the
 * IP over AXI; host build -> SW, the bit-exact C port. Override with -DMPC_BACKEND_FPGA
 * or -DMPC_BACKEND_SW. Output streams to the UART on bare metal (no fopen) and to a
 * file on the host. */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include "reference_track.h"

#define PI 3.14159265358979323846

/* Resolve backend once: explicit macro wins; else the target decides (ARM -> FPGA, host -> SW). */
#if defined(MPC_BACKEND_FPGA)
#  define USE_FPGA 1
#elif defined(MPC_BACKEND_SW)
#  define USE_FPGA 0
#elif defined(__arm__)
#  define USE_FPGA 1
#else
#  define USE_FPGA 0
#endif
/* Bare metal has no filesystem: stream CSV to the UART instead of a file. */
#if defined(__arm__)
#  define BARE_METAL 1
#else
#  define BARE_METAL 0
#endif

/* Software reference controller (LUTs + FCS search). Compiled ONLY when the SW
   backend is selected; the FPGA binary omits it entirely, so a closed-loop FPGA
   run can obtain its commands only from the IP over AXI — never in software. */
#if !USE_FPGA
static int16_t SIN_LUT[256], COS_LUT[256];
static void build_luts(void) {
    static const int16_t q1[64] = {
        0,3,6,9,12,16,19,22,25,28,31,34,37,40,43,46,
        49,51,54,57,60,63,65,68,71,73,76,78,81,83,85,88,
        90,92,94,96,98,100,102,104,106,107,109,111,112,114,115,116,
        118,119,120,121,122,123,124,124,125,126,126,127,127,127,127,127 };
    for (int i = 0; i < 64; i++) {
        int16_t Q1 = q1[i], Q2 = q1[63-i], Q3 = -q1[i], Q4 = -q1[63-i];
        SIN_LUT[i]=Q1; SIN_LUT[64+i]=Q2; SIN_LUT[128+i]=Q3; SIN_LUT[192+i]=Q4;
        COS_LUT[i]=Q2; COS_LUT[64+i]=Q3; COS_LUT[128+i]=Q4; COS_LUT[192+i]=Q1;
    }
}

static void mpc_step_sw(int16_t x, int16_t y, int16_t psi, int16_t v,
                        int16_t ref_x, int16_t ref_y, int16_t *accel, int16_t *steer) {
    static const int16_t STEER_OPTS[17] = {0,-2,2,-6,6,-10,10,-14,14,-18,18,-22,22,-24,24,-26,26};
    static const int16_t ACCEL_OPTS[8]  = {0,10,5,1,-5,-1,-10,-20};
    int16_t min_cost = 32767, best_a = 0, best_s = 0;
    for (int si = 0; si < 17; si++) {
        int d = STEER_OPTS[si], ad = d < 0 ? -d : d;
        int psin = (int16_t)(psi + (((int)v * d) >> 8));
        int idx = psin & 0xFF, c = COS_LUT[idx], s = SIN_LUT[idx];
        int vred = v >> 4;
        int xn = (int16_t)(x + (int16_t)((vred * c) >> 5));
        int yn = (int16_t)(y + (int16_t)((vred * s) >> 5));
        for (int ai = 0; ai < 8; ai++) {
            int a = ACCEL_OPTS[ai], vnext = v + a;
            if (vnext > 320) vnext = 320;
            if (vnext < 0) vnext = 0;
            int ex = xn - ref_x, ey = yn - ref_y;
            int axv = ex < 0 ? -ex : ex, ayv = ey < 0 ? -ey : ey;
            if (axv > 1000) axv = 1000;
            if (ayv > 1000) ayv = 1000;
            int cost = (axv + ayv) * 15 + ad * 2 - vnext * 2;
            if (cost < min_cost) { min_cost = (int16_t)cost; best_a = (int16_t)a; best_s = (int16_t)d; }
        }
    }
    *accel = best_a; *steer = best_s;
}
#endif  /* !USE_FPGA */

/* FPGA backend: same per-step AXI query as mpc_smoketest.c, closed in software */
#if USE_FPGA
#include "xparameters.h"
#include "xil_io.h"
#ifdef XPAR_MPC_CONTROLLER_AXI_0_S00_AXI_BASEADDR
  #define MPC_BASE XPAR_MPC_CONTROLLER_AXI_0_S00_AXI_BASEADDR
#else
  #define MPC_BASE 0x43C00000u
#endif
#define R_CTRL 0x00u
#define R_STATUS 0x04u
#define R_X 0x10u
#define R_Y 0x14u
#define R_PSI 0x18u
#define R_V 0x1Cu
#define R_REF_X 0x20u
#define R_REF_Y 0x24u
#define R_ACCEL 0x30u
#define R_STEER 0x34u
#define CTRL_KICK (1u<<0)
#define CTRL_EN   (1u<<1)
#define STATUS_DONE (1u<<0)
static void mpc_step_fpga(int16_t x, int16_t y, int16_t psi, int16_t v,
                          int16_t ref_x, int16_t ref_y, int16_t *accel, int16_t *steer) {
    Xil_Out32(MPC_BASE+R_X,(uint32_t)(int32_t)x);   Xil_Out32(MPC_BASE+R_Y,(uint32_t)(int32_t)y);
    Xil_Out32(MPC_BASE+R_PSI,(uint32_t)(int32_t)psi); Xil_Out32(MPC_BASE+R_V,(uint32_t)(int32_t)v);
    Xil_Out32(MPC_BASE+R_REF_X,(uint32_t)(int32_t)ref_x); Xil_Out32(MPC_BASE+R_REF_Y,(uint32_t)(int32_t)ref_y);
    Xil_Out32(MPC_BASE+R_CTRL, CTRL_KICK|CTRL_EN);
    while (!(Xil_In32(MPC_BASE+R_STATUS) & STATUS_DONE)) { }
    int a = Xil_In32(MPC_BASE+R_ACCEL) & 0x3F; if (a & 0x20) a -= 64;
    int s = Xil_In32(MPC_BASE+R_STEER) & 0x3F; if (s & 0x20) s -= 64;
    *accel = (int16_t)a; *steer = (int16_t)s;
}
  #define BACKEND_NAME "FPGA"
  #define mpc_step mpc_step_fpga
#else
  #define BACKEND_NAME "SW"
  #define mpc_step mpc_step_sw
#endif

/* Deterministic, libm-free primitives. Results are bit-identical to MATLAB's
   floor/round/sqrt (and to the PS libm), so no math library needs to be linked. */
static double det_floor(double x) { long long i = (long long)x; double f = (double)i; return (f > x) ? f - 1.0 : f; }
static long   det_lround(double x) { return (x >= 0.0) ? (long)(x + 0.5) : -(long)(0.5 - x); }
static double det_round(double x) { return (x >= 0.0) ? (double)(long long)(x + 0.5) : -(double)(long long)(0.5 - x); }
#if defined(__arm__)
static double det_sqrt(double x) { double r; __asm__ ("vsqrt.f64 %P0, %P1" : "=w"(r) : "w"(x)); return r; }  /* hardware VSQRT, IEEE-correct */
#else
static double det_sqrt(double x) { return sqrt(x); }
#endif

/* harness: nearest point, look-ahead, quantize, bicycle plant (mirrors fcs_mpc_v2_tb3.m) */
static double matmod(double a, double m) { return a - det_floor(a / m) * m; }

static int16_t q16(double v) {
    long r = det_lround(v);                   /* round half away from zero, like MATLAB int16() */
    if (r > 32767) r = 32767;
    if (r < -32768) r = -32768;
    return (int16_t)r;
}

static int nearest_idx(double x, double y, double *dmin) {
    double best = 1e30; int bi = 0;
    for (int i = 0; i < TRACK_POINTS; i++) {
        double dx = REF_X[i]-x, dy = REF_Y[i]-y, d = dx*dx + dy*dy;
        if (d < best) { best = d; bi = i; }
    }
    *dmin = det_sqrt(best); return bi;
}

static int lookahead_idx(int min_idx, double v) {
    double look = LOOKAHEAD_GAIN * v; if (look < LOOKAHEAD_MIN) look = LOOKAHEAD_MIN;
    double acc = 0; int t = min_idx;
    while (acc < look) {
        int nxt = (t + 1) % TRACK_POINTS;
        double dx = REF_X[nxt]-REF_X[t], dy = REF_Y[nxt]-REF_Y[t];
        acc += det_sqrt(dx*dx + dy*dy); t = nxt;
        if (acc > look + 5) break;
    }
    return t;
}

/* Deterministic sin/cos, IEEE-754 ops only, identical in MATLAB and C (shared plant). */
static void sincos_det(double z, double *s, double *c) {
    double zr = z - 6.283185307179586 * det_round(z / 6.283185307179586);   /* reduce to [-pi,pi] */
    double sg = 1.0;
    if (zr > 1.5707963267948966)        { zr = 3.141592653589793 - zr;  sg = -1.0; }
    else if (zr < -1.5707963267948966)  { zr = -3.141592653589793 - zr; sg = -1.0; }
    double z2 = zr * zr;
    *s = zr * (1.0 + z2*(-1.6666666666666666e-1 + z2*(8.333333333333333e-3 + z2*(-1.984126984126984e-4 +
         z2*(2.7557319223985893e-6 + z2*(-2.505210838544172e-8 + z2*1.6059043836821613e-10))))));
    *c = sg * (1.0 + z2*(-5.0e-1 + z2*(4.1666666666666664e-2 + z2*(-1.388888888888889e-3 +
         z2*(2.48015873015873e-5 + z2*(-2.7557319223985894e-7 + z2*2.08767569878681e-9))))));
}

/* CSV double field: full precision locally, integer-only on bare-metal (no float printf) */
#if BARE_METAL
static void wd(FILE *f, double v) {
    if (v < 0) { fputc('-', f); v = -v; }
    long ip = (long)v, fr = (long)((v - (double)ip) * 1000000.0 + 0.5);
    if (fr >= 1000000) { ip++; fr -= 1000000; }
    fprintf(f, "%ld.%06ld", ip, fr);
}
#else
static void wd(FILE *f, double v) { fprintf(f, "%.10g", v); }
#endif

int main(int argc, char **argv) {
    const char *out = (argc > 1) ? argv[1] : "c_closed_loop.csv";
#if !USE_FPGA
    build_luts();
#endif
    double x = X0, y = Y0, psi = PSI0, v = V0, max_err = 0;
    int laps = 0, prev1 = 1, last1 = 1;
#if BARE_METAL
    FILE *f = stdout; (void)out;        /* CSV streams over UART; capture and rename to c_closed_loop.csv */
#else
    FILE *f = fopen(out, "w");
    if (!f) { perror("fopen"); return 1; }
#endif
    fprintf(f, "step,x,y,psi,v,xq,yq,psiq,vq,rxq,ryq,accel,steer,err\n");
    for (int k = 0; k < SIM_STEPS; k++) {
        double dmin; int ni = nearest_idx(x, y, &dmin);
        int midx1 = ni + 1;
        if (midx1 < 50 && prev1 > TRACK_POINTS - 50) laps++;
        prev1 = midx1; last1 = midx1;
        if (dmin > max_err) max_err = dmin;
        int ti = lookahead_idx(ni, v);
        double rx = REF_X[ti], ry = REF_Y[ti];
        double pw = matmod(psi + PI, 2*PI) - PI;
        int16_t xq=q16(x*SF_POS), yq=q16(y*SF_POS), psiq=q16(pw*SF_PSI);
        int16_t vq=q16(v*SF_V), rxq=q16(rx*SF_POS), ryq=q16(ry*SF_POS), a_int, s_int;
        mpc_step(xq, yq, psiq, vq, rxq, ryq, &a_int, &s_int);
        fprintf(f, "%d,", k); wd(f, x); fputc(',', f); wd(f, y); fputc(',', f);
        wd(f, psi); fputc(',', f); wd(f, v); fputc(',', f);
        fprintf(f, "%d,%d,%d,%d,%d,%d,%d,%d,", (int)xq,(int)yq,(int)psiq,(int)vq,
                (int)rxq,(int)ryq,(int)a_int,(int)s_int);
        wd(f, dmin); fputc('\n', f);
        double acc = (double)a_int / SF_V, str = (double)s_int * STEER_DECODE;
        double sp, cp, ss, cs; sincos_det(psi, &sp, &cp); sincos_det(str, &ss, &cs);
        x += v * cp * TS; y += v * sp * TS;
        psi += (v / L_WHEELBASE) * (ss / cs) * TS; v = (v + acc) * V_DRAG;
    }
#if BARE_METAL
    fprintf(f, "# backend=%s steps=%d laps=", BACKEND_NAME, SIM_STEPS);
    wd(f, laps + (double)last1 / TRACK_POINTS); fprintf(f, " max_err="); wd(f, max_err);
    fprintf(f, " m\n");
    for (;;) { }                        /* idle so the UART capture completes */
#else
    fclose(f);
    printf("backend=%s steps=%d laps=%.2f max_err=%.3f m -> %s\n",
           BACKEND_NAME, SIM_STEPS, laps + (double)last1 / TRACK_POINTS, max_err, out);
#endif
    return 0;
}
