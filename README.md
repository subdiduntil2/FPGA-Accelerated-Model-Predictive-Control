# FPGA-Accelerated-Model-Predictive-Control

This repository contains all software and hardware files for the FPGA implementation of a low-latency model predictive controller (MPC) for path tracking. The target is a conventional, low-cost Cora Z7-07S board (Xilinx Zynq xc7z007s).

The controller is a finite-control-set MPC. Each step it enumerates 17 x 8 = 136 discrete (steer, accel) candidates, evaluates a single-step tracking and effort cost for all of them in parallel combinational logic, and outputs the pair with the minimum cost. Everything runs in fixed point (6 to 16 bit), with no QP solver and no floating point on chip. The MPC core is packaged as a custom AXI4-Lite peripheral, and the Zynq PS drives it over a memory-mapped register interface.

## What is validated

The current version validates the implementation at two levels, both against a bit-exact MATLAB golden model:

- **Open-loop replay.** The PS streams recorded trajectory states through the FPGA and checks every returned command against the MATLAB-expected value. Verified result: 801 / 801 samples match bit-exactly, with a 739.8 ns average solve time in the PL.
- **Closed-loop hardware in the loop.** The PS integrates the vehicle plant, feeds the live state to the MPC IP each step, and applies whatever the controller returns, so any error compounds instead of resetting. Verified on the Cora: all 801 command pairs and quantized states match MATLAB, and the two trajectories agree to the UART print resolution.

The controller also exists as a bit-exact C port, so the closed loop can run purely in software on a host for fast iteration before a hardware run. The docs folder holds the extended documentation, including the full reproduction procedure and a comparison against the HDL Coder MPC of Purraji et al.

## Repository layout

- `matlab/` holds the MPC algorithm, the closed-loop golden testbench that also generates all data artifacts, and the comparison scripts.
- `hdl/` holds the hand-written VHDL controller, its self-checking testbench, the AXI wrapper, and the block design sources.
- `src/` holds the closed-loop C harness (software and FPGA backends) and the auto-generated track and parameter header.
- `hw/` holds the Vivado block design and the exported hardware handoff.
- `docs/` holds the workflow report and the reference paper.

## Remaining engineering work

1. **Close timing.** The 136-candidate combinational core violates timing (WNS/TNS) at 50 MHz. Either declare the multicycle relationship that the capture FSM already enforces (set_multicycle_path), or pipeline the candidate evaluation and update the latency accounting. Re-run the full validation chain after the fix.
2. **Report utilization and achieved timing.** Extract the real LUT, FF, and DSP counts and the achieved Fmax and WNS from the implemented design to characterize the footprint.
3. **Raise the speed ceiling.** Vehicle speed is currently capped at 5 m/s. The velocity signal (ufix9) has headroom, so the first step costs nothing. Going further requires wider signals across the MATLAB, VHDL, and C implementations, plus retuned cost weights.
4. **Reduce tracking error.** Schedule the lookahead gain and refine the steering option grid near zero, then measure the result against the continuous-track error metric.
5. **Cut AXI transfer overhead.** Per-step AXI traffic (about 3 us) dominates the end-to-end latency (about 3.75 us). Packing the six input writes into a single burst is the main lever.
6. **Add more complex tracks.** Extend the track generator with additional path geometries to stress the controller beyond the current oval.
7. **Feed real sensor data into the HIL loop.** Send live IMU data (accelerometer, gyroscope) to the PS and insert a state estimator ahead of the quantization step for more realistic hardware-in-the-loop runs.
