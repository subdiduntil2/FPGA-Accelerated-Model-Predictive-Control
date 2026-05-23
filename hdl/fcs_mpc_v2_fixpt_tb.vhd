-- Self-checking testbench for fcs_mpc_v2_fixpt (DUT latency = 2 cycles).

LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;
USE STD.textio.ALL;
USE IEEE.std_logic_textio.ALL;

USE work.trajectory_data_pkg.ALL;

ENTITY fcs_mpc_v2_fixpt_tb IS
END fcs_mpc_v2_fixpt_tb;

ARCHITECTURE sim OF fcs_mpc_v2_fixpt_tb IS

  CONSTANT CLK_PERIOD : time := 10 ns;
  SIGNAL clk          : std_logic := '0';
  SIGNAL reset        : std_logic := '1';
  SIGNAL sim_done     : boolean   := false;

  SIGNAL x_in     : std_logic_vector(13 DOWNTO 0);
  SIGNAL y_in     : std_logic_vector(12 DOWNTO 0);
  SIGNAL psi_in   : std_logic_vector( 8 DOWNTO 0);
  SIGNAL v_in     : std_logic_vector( 8 DOWNTO 0);
  SIGNAL ref_x_in : std_logic_vector(13 DOWNTO 0);
  SIGNAL ref_y_in : std_logic_vector(11 DOWNTO 0);
  SIGNAL acc_out  : std_logic_vector( 5 DOWNTO 0);
  SIGNAL str_out  : std_logic_vector( 5 DOWNTO 0);

  CONSTANT DUT_LATENCY : integer := 2;

  FUNCTION to_slv_signed(v : integer; w : integer) RETURN std_logic_vector IS
  BEGIN
    RETURN std_logic_vector(to_signed(v, w));
  END;

  FUNCTION to_slv_unsigned(v : integer; w : integer) RETURN std_logic_vector IS
  BEGIN
    RETURN std_logic_vector(to_unsigned(v, w));
  END;

  FUNCTION slv_to_int_signed(s : std_logic_vector) RETURN integer IS
  BEGIN
    RETURN to_integer(signed(s));
  END;

  SIGNAL n_checked : integer := 0;
  SIGNAL n_acc_ok  : integer := 0;
  SIGNAL n_str_ok  : integer := 0;
  SIGNAL n_both_ok : integer := 0;
  SIGNAL first_mm  : integer := -1;

BEGIN

  -- Clock generator
  clk_proc : PROCESS
  BEGIN
    WHILE NOT sim_done LOOP
      clk <= '0'; WAIT FOR CLK_PERIOD/2;
      clk <= '1'; WAIT FOR CLK_PERIOD/2;
    END LOOP;
    WAIT;
  END PROCESS;

  -- DUT instance
  dut : ENTITY work.fcs_mpc_v2_fixpt
    PORT MAP (
      clk       => clk,
      reset     => reset,
      x         => x_in,
      y         => y_in,
      psi       => psi_in,
      v         => v_in,
      ref_x     => ref_x_in,
      ref_y     => ref_y_in,
      accel_cmd => acc_out,
      steer_cmd => str_out
    );

  -- Stimulus + self-check
  stim_proc : PROCESS
    VARIABLE l           : line;
    VARIABLE feed_idx    : integer;
    VARIABLE check_idx   : integer;
    VARIABLE exp_acc     : integer;
    VARIABLE exp_str     : integer;
    VARIABLE got_acc     : integer;
    VARIABLE got_str     : integer;
    VARIABLE acc_ok      : boolean;
    VARIABLE str_ok      : boolean;
  BEGIN
    reset <= '1';
    x_in     <= (OTHERS => '0');
    y_in     <= (OTHERS => '0');
    psi_in   <= (OTHERS => '0');
    v_in     <= (OTHERS => '0');
    ref_x_in <= (OTHERS => '0');
    ref_y_in <= (OTHERS => '0');
    WAIT UNTIL rising_edge(clk);
    WAIT UNTIL rising_edge(clk);
    reset <= '0';
    WAIT UNTIL rising_edge(clk);

    write(l, string'("[TB] Reset deasserted.  Driving "));
    write(l, TRAJECTORY_LEN);
    write(l, string'(" samples, DUT latency = "));
    write(l, DUT_LATENCY);
    write(l, string'(" cycles."));
    writeline(output, l);

    FOR i IN 0 TO TRAJECTORY_LEN + DUT_LATENCY - 1 LOOP

      IF i < TRAJECTORY_LEN THEN
        feed_idx := i;
      ELSE
        feed_idx := TRAJECTORY_LEN - 1;
      END IF;

      x_in     <= to_slv_signed  (X_DATA(feed_idx),     14);
      y_in     <= to_slv_signed  (Y_DATA(feed_idx),     13);
      psi_in   <= to_slv_signed  (PSI_DATA(feed_idx),    9);
      v_in     <= to_slv_unsigned(V_DATA(feed_idx),      9);
      ref_x_in <= to_slv_signed  (REF_X_DATA(feed_idx), 14);
      ref_y_in <= to_slv_unsigned(REF_Y_DATA(feed_idx), 12);

      WAIT UNTIL rising_edge(clk);

      IF i >= DUT_LATENCY THEN
        check_idx := i - DUT_LATENCY;
        exp_acc   := ACCEL_EXP(check_idx);
        exp_str   := STEER_EXP(check_idx);
        got_acc   := slv_to_int_signed(acc_out);
        got_str   := slv_to_int_signed(str_out);
        acc_ok    := (got_acc = exp_acc);
        str_ok    := (got_str = exp_str);

        n_checked <= n_checked + 1;
        IF acc_ok  THEN n_acc_ok  <= n_acc_ok  + 1; END IF;
        IF str_ok  THEN n_str_ok  <= n_str_ok  + 1; END IF;
        IF acc_ok AND str_ok THEN
          n_both_ok <= n_both_ok + 1;
        ELSE
          IF first_mm = -1 THEN
            first_mm <= check_idx;
          END IF;
          IF check_idx < 8 OR (check_idx mod 100) = 0 THEN
            write(l, string'("[TB] MISMATCH @ "));
            write(l, check_idx);
            write(l, string'(" got=("));
            write(l, got_acc); write(l, string'(","));
            write(l, got_str);
            write(l, string'(") exp=("));
            write(l, exp_acc); write(l, string'(","));
            write(l, exp_str);
            write(l, string'(")"));
            writeline(output, l);
          END IF;
        END IF;
      END IF;
    END LOOP;

    write(l, string'("[TB] ============================================"));
    writeline(output, l);
    write(l, string'("[TB] Samples checked  : "));
    write(l, n_checked);
    writeline(output, l);
    write(l, string'("[TB] accel matches    : "));
    write(l, n_acc_ok);
    write(l, string'(" / "));
    write(l, n_checked);
    writeline(output, l);
    write(l, string'("[TB] steer matches    : "));
    write(l, n_str_ok);
    write(l, string'(" / "));
    write(l, n_checked);
    writeline(output, l);
    write(l, string'("[TB] both ok          : "));
    write(l, n_both_ok);
    write(l, string'(" / "));
    write(l, n_checked);
    writeline(output, l);

    IF n_both_ok = n_checked THEN
      write(l, string'("[TB] PASS -- all "));
      write(l, n_checked);
      write(l, string'(" samples match MATLAB bit-exactly."));
      writeline(output, l);
    ELSE
      write(l, string'("[TB] FAIL -- first mismatch at sample "));
      write(l, first_mm);
      writeline(output, l);
    END IF;
    write(l, string'("[TB] ============================================"));
    writeline(output, l);

    sim_done <= true;
    WAIT FOR CLK_PERIOD;

    ASSERT n_both_ok = n_checked
      REPORT "fcs_mpc_v2_fixpt_tb FAILED -- see console for mismatch list"
      SEVERITY FAILURE;

    ASSERT false
      REPORT "fcs_mpc_v2_fixpt_tb done -- simulation finished"
      SEVERITY NOTE;
    WAIT;
  END PROCESS;

END sim;
