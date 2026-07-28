# mpc_timing.xdc -- multicycle exception for the combinational MPC core.
#
# NOTE: XDC files accept only a restricted Tcl subset. Do NOT use if / puts /
# expr / foreach / proc here (Designutils 20-1307). Everything below is plain
# constraint syntax.
#
# Why this is correct, not a cheat:
#   fcs_mpc_v3.vhd is purely combinational (7 int16 in -> 2 int16 out).
#   The only flop-to-flop path through it is
#       slv_reg4..slv_reg10  ->  136-candidate cost cloud  ->  slv_reg12/13
#   The kick/run/capture FSM in MPC_controller_AXI_v1_0_S00_AXI.vhd puts the
#   capture edge 6 clocks after the KICK write, and the input registers cannot
#   change on the KICK cycle (that write decodes to CTRL). So the data really
#   does have 6 periods (120 ns at 50 MHz) to settle; the default single-cycle
#   20 ns analysis is simply the wrong requirement.
#
# To buy more budget: raise BOTH numbers below (N setup, N-1 hold) AND the FSM
# terminal count, keeping   wait_cnt = N - 2.   Currently N = 6, wait_cnt = 4.
# Each extra cycle adds 20 ns of budget and 20 ns of solve latency.

set_multicycle_path 6 -setup \
  -from [get_cells -hier -regexp {.*MPC_S00_AXI_inst/slv_reg(4|5|6|7|8|9|10)_reg.*}] \
  -to   [get_cells -hier -regexp {.*MPC_S00_AXI_inst/slv_reg(12|13)_reg.*}]

set_multicycle_path 5 -hold \
  -from [get_cells -hier -regexp {.*MPC_S00_AXI_inst/slv_reg(4|5|6|7|8|9|10)_reg.*}] \
  -to   [get_cells -hier -regexp {.*MPC_S00_AXI_inst/slv_reg(12|13)_reg.*}]
