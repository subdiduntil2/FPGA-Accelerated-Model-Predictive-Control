# FPGA-Accelerated-Model-Predictive-Control
This repository includes all software and hardware files for the FPGA implementation of a low-cost, low-latency model predictive controller, targeting a conventional CORAZ7 board. Current version supports end-to-end validation of the proposed implementation, software-hardware comparison metrics as well as an AXI-stream protocol for MCU/PL communication.
Future steps include:
1. Addition of more complex tracks for path tracking
2. Fix of timing violations on FPGA
3. Performance Improvements regarding increasing simulation speed and path tracking error
4. Addition of real-time sensory data (e.g. from accelerometers, gyroscopes etc) for more realistic hardware-in-the-loop runs
