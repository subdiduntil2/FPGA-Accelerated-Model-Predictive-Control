# FPGA-Accelerated Model Predictive Control

This repository contains the software and hardware sources for an FPGA implementation of a low-latency model predictive controller (MPC) for vehicle path tracking. The target is a conventional, low-cost Cora Z7-07S board (Xilinx Zynq xc7z007s).

The controller is a finite-control-set MPC with a one-step horizon. It enumerates 17 steering and 8 acceleration candidates, evaluates a tracking-and-effort cost, and returns the pair that minimises it. The current core exploits the separability of that cost, so the hardware builds 17 kinematic cones and 8 speed terms instead of 136 full candidate evaluations. Everything runs in fixed point (6 to 16 bit). There is no QP solver and no floating-point arithmetic on chip. The core is packaged as a custom AXI4-Lite peripheral, and the Zynq PS drives it through a memory-mapped register interface.

Full numbers, tables, methodology and the comparison against published work are in the analytical report in `docs/`. This README gives the summary.

## Application scope

The present configuration targets a **known circuit**. Two quantities are computed offline in MATLAB and preloaded before the run:

1. the reference path, exported as a 358-vertex polyline at roughly 0.5 m spacing, and
2. the reference speed profile, derived from the path curvature and exported as the `VREF` table in `reference_track.h`.

At run time the controller receives the nearest path vertex, a look-ahead vertex, and the profile speed at the current position. It does not build a map, and it does not compute a speed profile online. This matches a race or test-track application in which the circuit is surveyed in advance. Section 1 of the report in `docs/` states the assumption in full, and the planned work below covers the unknown-track case.

## Results

**Timing.** The design closes with 10.449 ns of worst-case setup slack against a 20 ns period, no failing endpoints across 2,077 setup and 2,077 hold paths, and no unconstrained paths. The worst path is not in the controller: it runs from the processor system reset into the control register, has one logic level, and is 93% route delay.

**Area.** 7,854 of 14,400 LUTs (54.5%), 870 flip-flops, 40 DSP slices, no block RAM. The earlier 136-candidate core needed 16,939 LUTs, which exceeds the device, and never reached the placer. Exploiting the separability of the cost function cut LUT usage by 54% and moved the controller off the critical path. All 40 DSPs belong to the controller, since nothing else in the design infers one.

**Power.** Total on-chip power is 1.381 W. The MPC peripheral accounts for 12 mW of that and the whole programmable-logic side about 14 mW, against 1.251 W for the processing system. Running the control law in software would not avoid that 1.251 W; it would occupy it. The estimate is vectorless, so treat the absolute values as indicative and the distribution as the result.

**Latency.** A solve takes 341 ns on average, of which only 120 ns is the controller itself, six clocks at 50 MHz fixed by the capture state machine. The rest is the AXI status read and the timestamp cost. A full control step, including seven input writes, the kick and two result reads, averages 3.76 µs. Bus traffic is 96.8% of that and the controller is 3.2%, so the worthwhile optimisation is fewer bus transactions rather than a faster core. Against the 100 ms sampling period the whole exchange uses 0.0038% of the budget. Both maxima in the run belong to step 0, which pays for cold caches; every later step falls within 9 ns of the average.

**Closed-loop behaviour.** 2.54 laps in 80 s, 10.20 m/s top speed, 0.866 m peak tracking error against a 1.0 m limit.

## Validation

The implementation is checked at five levels, each against a bit-exact MATLAB golden model: algorithmic equivalence of the restructured controller against the previous revision, golden-vector replay in MATLAB, RTL simulation in Vivado xsim, a software closed loop on a host, and the closed loop on hardware.

The last of these is the one that matters. The plant runs on the PS, the MPC IP closes the loop over AXI, and errors compound instead of resetting. All 801 rows match the golden model on every integer column, with zero differences in the quantised state, the references, the profile speed and both commands. The residual position difference is 7.07e-7 m, which is the print resolution of the bare-metal CSV formatter rather than numerical drift: across 4,806 quantisation events not one produced a different integer. The capture was also re-checked independently of the comparison script against a separate reimplementation of the controller and plant, which reproduces the same result.

## How this compares

Published embedded MPC implementations span roughly 67 µs on a motor-drive DSP to 17 ms on an embedded CPU, drawing between 0.2 W and 6.5 W. Measured energy per control step in that literature ranges from tens of microjoules on a microcontroller to tens of millijoules on a GPU. This design uses 5.20 µJ per step for the whole chip and 45 nJ for the accelerator increment, which is two to three orders of magnitude below any of them.

That advantage is real but it partly reflects a smaller problem. Sampling-based implementations evaluate thousands of rollouts over a 40-step horizon, and nonlinear MPC for autonomous driving solves a program with tyre models and obstacle constraints; this controller evaluates 25 terms at a horizon of one. The fair claim is narrower: for horizon-1 finite-control-set path tracking, the design reaches microsecond end-to-end latency at 12 mW of incremental power, which places it in the latency and energy class of motor-drive FCS-MPC on a DSP while addressing a task that normally needs millisecond-class platforms. The price is the horizon-1 restriction.

Section 7 of the report gives the full comparison with per-platform figures, provenance for every power number, and citations.

## Repository layout

Every artifact below belongs to the current v4 closed-loop revision. The MATLAB testbench is the single generator: the vector files, the C header and the golden trajectory all come from one run, which is what keeps the four implementations from drifting apart.

**`matlab/`**

- `fcs_mpc_v4.m` — the controller, and the specification against which everything else is checked.
- `fcs_mpc_v4_tb.m` — closed-loop golden testbench. Builds the circuit and the curvature-based speed profile, runs the loop, and writes the nine `.dat` vectors, `reference_track.h` and `matlab_closed_loop.csv`.
- `fcs_mpc_v4_hdltb.m` — replays the exported vectors through the controller. This is the gate that must read 801/801 before any hardware is generated.
- `compare_fpga_matlab.m` — 15-column closed-loop comparison of the board capture against the golden run.

**`hdl/`**

- `fcs_mpc_v4.vhd`, `fcs_mpc_v4_pkg.vhd` — the generated combinational core (seven `int16` inputs, two `int16` outputs, registered input and output stages) and its type package.
- `fcs_mpc_v4_tb.vhd`, `fcs_mpc_v4_tb_pkg.vhd` — the generated self-checking testbench, which drives the vector files and raises `testFailure` on any command mismatch.
- `MPC_controller_AXI_v1_0.vhd`, `MPC_controller_AXI_v1_0_S00_AXI.vhd` — the IP top level, and the register file with the kick/run/capture state machine and the core instantiation.
- `design_1.vhd`, `design_1_MPC_controller_AXI_0_1.vhd`, `design_1_wrapper.vhd` — block design output.

**`hw/`**

- `design_1.bd` — block design source.
- `design_1_wrapper.xsa` — hardware handoff for Vitis, exported with the bitstream included.
- `mpc_timing.xdc` — the multicycle exception on the controller path, six cycles setup and five hold, coupled to the state machine terminal count.

**`src/`**

- `mpc_closed_loop.c` — closed-loop harness with the software and FPGA backends and the deterministic plant.
- `reference_track.h` — generated: the 358-vertex path, the `VREF` speed table, the plant constants and the scale factors.

**`docs/`** — the analytical report and the reference paper.

Two files are absent by design. The controller is no longer hand-written, so no maintained VHDL source exists separately from the generated core; the MATLAB function is edited and the hardware regenerated. The open-loop replay application has been retired in favour of the closed-loop harness, which subsumes it.

## Reproducing the result

Section 8 of the report gives the full procedure. In short:

1. Run `fcs_mpc_v4_tb` in MATLAB. It writes the nine `.dat` vectors, `reference_track.h`, and `matlab_closed_loop.csv`.
2. Regenerate the VHDL with HDL Coder from the command line, passing seven `int16` arguments so that fixed-point conversion is skipped.
3. Package the core as `MPC_controller_AXI`, add `mpc_timing.xdc`, synthesise and implement with `-jobs 1`, then export the XSA with the bitstream included.
4. Build the Vitis application from `mpc_closed_loop.c` and `reference_track.h` with `-ffp-contract=off`, run it on the board, and capture the UART output.
5. Run `compare_fpga_matlab.m` against the capture and the golden CSV.

## Planned work

**Configuration A, known track (current).** Keep the preloaded path and the offline speed profile. Remaining items: raise the corner speed limit through cost retuning, refine the steering grid near zero to reduce peak tracking error, and pack the seven input register writes into a single burst, which is the only change that would materially cut end-to-end latency.

**Configuration B, unknown track (planned).** Remove the offline profile and run the same core against a reference the vehicle discovers as it drives. Two variants are in scope:

- a conservative constant reference speed, which needs no new hardware and only changes what the PS writes to `REF_V`; and
- an online speed profile computed over a receding window of the path ahead, using the same curvature-to-speed rule that the offline generator applies, so the controller keeps its speed-tracking behaviour without prior knowledge of the full circuit.

Both variants leave the MPC core unchanged. The work sits in the PS software and in the path estimator that feeds it.

**Engineering backlog.**

1. Re-measure after any interface change. The instrumentation is in the harness and costs 43 ns per timestamp, so the effect of a burst-write interface can be quantified by re-running the loop and reading the footer.
2. Extend the track generator with further geometries to stress the controller beyond the present circuit.
3. Feed live inertial data to the PS and insert a state estimator ahead of the quantisation step, for a more realistic hardware-in-the-loop run.
4. Re-verify the multicycle exception. The restructured core may now close under plain single-cycle analysis, which would let `mpc_timing.xdc` be retired.
