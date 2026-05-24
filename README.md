# FPGA-Accelerated-Model-Predictive-Control
This repository includes all software and hardware files for the FPGA implementation of a low-latency model predictive controller, targeting a conventional, low-cost CORAZ7 board. Current version supports end-to-end validation of the proposed implementation, software-hardware comparison metrics as well as an AXI-stream protocol for MCU/PL communication. Extended documentation can be found in the docs folder.
Future steps include:
1. Addition of more complex tracks for path tracking
2. Fix of timing violations (WNS/TNS) on FPGA
3. Performance improvements regarding increasing vehicle speed (currently limited at 5m/s) and path tracking error
4. Addition of real-time sensory data fed to the MCU (e.g. from accelerometers, gyroscopes etc) for more realistic hardware-in-the-loop runs
