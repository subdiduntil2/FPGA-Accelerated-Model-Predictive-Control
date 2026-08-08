-- fcs_mpc_v4_tb: file-driven testbench for fcs_mpc_v4, base rate 1 (model and subsystem).
-- Generated 2026-07-28 by MATLAB 9.11 / MATLAB Coder 5.3 / HDL Coder 3.19.
LIBRARY IEEE;
USE IEEE.std_logic_textio.ALL;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;
LIBRARY STD;
USE STD.textio.ALL;
LIBRARY work;
USE work.fcs_mpc_v4_pkg.ALL;
USE work.fcs_mpc_v4_tb_pkg.ALL;

ENTITY fcs_mpc_v4_tb IS
END fcs_mpc_v4_tb;


ARCHITECTURE rtl OF fcs_mpc_v4_tb IS

  -- Component Declarations
  COMPONENT fcs_mpc_v4
    PORT( x                               :   IN    std_logic_vector(15 DOWNTO 0);
          y                               :   IN    std_logic_vector(15 DOWNTO 0);
          psi                             :   IN    std_logic_vector(15 DOWNTO 0);
          v                               :   IN    std_logic_vector(15 DOWNTO 0);
          ref_x                           :   IN    std_logic_vector(15 DOWNTO 0);
          ref_y                           :   IN    std_logic_vector(15 DOWNTO 0);
          ref_v                           :   IN    std_logic_vector(15 DOWNTO 0);
          accel_cmd                       :   OUT   std_logic_vector(15 DOWNTO 0);
          steer_cmd                       :   OUT   std_logic_vector(15 DOWNTO 0)
          );
  END COMPONENT;

  -- Component Configuration Statements
  FOR ALL : fcs_mpc_v4
    USE ENTITY work.fcs_mpc_v4(rtl);

  -- Signals
  SIGNAL clk                              : std_logic;
  SIGNAL reset                            : std_logic;
  SIGNAL enb                              : std_logic;
  SIGNAL steer_cmd_done                   : std_logic;
  SIGNAL rdEnb                            : std_logic;
  SIGNAL steer_cmd_done_enb               : std_logic;
  SIGNAL accel_cmd_addr                   : unsigned(9 DOWNTO 0);
  SIGNAL steer_cmd_lastAddr               : std_logic;
  SIGNAL resetn                           : std_logic;
  SIGNAL check2_done                      : std_logic;
  SIGNAL accel_cmd_done                   : std_logic;
  SIGNAL accel_cmd_done_enb               : std_logic;
  SIGNAL accel_cmd_active                 : std_logic;
  SIGNAL snkDone                          : std_logic;
  SIGNAL snkDonen                         : std_logic;
  SIGNAL tb_enb                           : std_logic;
  SIGNAL ce_out                           : std_logic;
  SIGNAL accel_cmd_enb                    : std_logic;
  SIGNAL accel_cmd_lastAddr               : std_logic;
  SIGNAL check1_done                      : std_logic;
  SIGNAL x_addr                           : unsigned(9 DOWNTO 0);
  SIGNAL x_active                         : std_logic;
  SIGNAL x_enb                            : std_logic;
  SIGNAL x_addr_delay_1                   : unsigned(9 DOWNTO 0);
  SIGNAL rawData_x                        : signed(15 DOWNTO 0);
  SIGNAL holdData_x                       : signed(15 DOWNTO 0);
  SIGNAL y_addr_delay_1                   : unsigned(9 DOWNTO 0);
  SIGNAL rawData_y                        : signed(15 DOWNTO 0);
  SIGNAL holdData_y                       : signed(15 DOWNTO 0);
  SIGNAL psi_addr_delay_1                 : unsigned(9 DOWNTO 0);
  SIGNAL rawData_psi                      : signed(15 DOWNTO 0);
  SIGNAL holdData_psi                     : signed(15 DOWNTO 0);
  SIGNAL v_addr_delay_1                   : unsigned(9 DOWNTO 0);
  SIGNAL rawData_v                        : signed(15 DOWNTO 0);
  SIGNAL holdData_v                       : signed(15 DOWNTO 0);
  SIGNAL ref_x_addr_delay_1               : unsigned(9 DOWNTO 0);
  SIGNAL rawData_ref_x                    : signed(15 DOWNTO 0);
  SIGNAL holdData_ref_x                   : signed(15 DOWNTO 0);
  SIGNAL ref_y_addr_delay_1               : unsigned(9 DOWNTO 0);
  SIGNAL rawData_ref_y                    : signed(15 DOWNTO 0);
  SIGNAL holdData_ref_y                   : signed(15 DOWNTO 0);
  SIGNAL ref_v_addr_delay_1               : unsigned(9 DOWNTO 0);
  SIGNAL rawData_ref_v                    : signed(15 DOWNTO 0);
  SIGNAL holdData_ref_v                   : signed(15 DOWNTO 0);
  SIGNAL x_offset                         : signed(15 DOWNTO 0);
  SIGNAL x_1                              : signed(15 DOWNTO 0);
  SIGNAL x_2                              : std_logic_vector(15 DOWNTO 0);
  SIGNAL y_offset                         : signed(15 DOWNTO 0);
  SIGNAL y                                : signed(15 DOWNTO 0);
  SIGNAL y_1                              : std_logic_vector(15 DOWNTO 0);
  SIGNAL psi_offset                       : signed(15 DOWNTO 0);
  SIGNAL psi                              : signed(15 DOWNTO 0);
  SIGNAL psi_1                            : std_logic_vector(15 DOWNTO 0);
  SIGNAL v_offset                         : signed(15 DOWNTO 0);
  SIGNAL v                                : signed(15 DOWNTO 0);
  SIGNAL v_1                              : std_logic_vector(15 DOWNTO 0);
  SIGNAL ref_x_offset                     : signed(15 DOWNTO 0);
  SIGNAL ref_x                            : signed(15 DOWNTO 0);
  SIGNAL ref_x_1                          : std_logic_vector(15 DOWNTO 0);
  SIGNAL ref_y_offset                     : signed(15 DOWNTO 0);
  SIGNAL ref_y                            : signed(15 DOWNTO 0);
  SIGNAL ref_y_1                          : std_logic_vector(15 DOWNTO 0);
  SIGNAL ref_v_offset                     : signed(15 DOWNTO 0);
  SIGNAL ref_v                            : signed(15 DOWNTO 0);
  SIGNAL ref_v_1                          : std_logic_vector(15 DOWNTO 0);
  SIGNAL accel_cmd_1                      : std_logic_vector(15 DOWNTO 0);
  SIGNAL steer_cmd                        : std_logic_vector(15 DOWNTO 0);
  SIGNAL accel_cmd_signed                 : signed(15 DOWNTO 0);
  SIGNAL accel_cmd_addr_delay_1           : unsigned(9 DOWNTO 0);
  SIGNAL accel_cmd_expected               : signed(15 DOWNTO 0);
  SIGNAL accel_cmd_ref                    : signed(15 DOWNTO 0);
  SIGNAL accel_cmd_testFailure            : std_logic;
  SIGNAL steer_cmd_signed                 : signed(15 DOWNTO 0);
  SIGNAL steer_cmd_addr_delay_1           : unsigned(9 DOWNTO 0);
  SIGNAL steer_cmd_expected               : signed(15 DOWNTO 0);
  SIGNAL steer_cmd_ref                    : signed(15 DOWNTO 0);
  SIGNAL steer_cmd_testFailure            : std_logic;
  SIGNAL testFailure                      : std_logic;

BEGIN
  u_fcs_mpc_v4 : fcs_mpc_v4
    PORT MAP( x => x_2,
              y => y_1,
              psi => psi_1,
              v => v_1,
              ref_x => ref_x_1,
              ref_y => ref_y_1,
              ref_v => ref_v_1,
              accel_cmd => accel_cmd_1,
              steer_cmd => steer_cmd
              );

  steer_cmd_done_enb <= steer_cmd_done AND rdEnb;

  
  steer_cmd_lastAddr <= '1' WHEN accel_cmd_addr >= to_unsigned(16#320#, 10) ELSE
      '0';

  steer_cmd_done <= steer_cmd_lastAddr AND resetn;

  -- Delay to allow last sim cycle to complete
  checkDone_2_process: PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      check2_done <= '0';
    ELSIF clk'event AND clk = '1' THEN
      IF steer_cmd_done_enb = '1' THEN
        check2_done <= steer_cmd_done;
      END IF;
    END IF;
  END PROCESS checkDone_2_process;

  accel_cmd_done_enb <= accel_cmd_done AND rdEnb;

  
  accel_cmd_active <= '1' WHEN accel_cmd_addr /= to_unsigned(16#320#, 10) ELSE
      '0';

  enb <= rdEnb AFTER 2 ns;

  snkDonen <=  NOT snkDone;

  clk_gen: PROCESS 
  BEGIN
    clk <= '1';
    WAIT FOR 5 ns;
    clk <= '0';
    WAIT FOR 5 ns;
    IF snkDone = '1' THEN
      clk <= '1';
      WAIT FOR 5 ns;
      clk <= '0';
      WAIT FOR 5 ns;
      WAIT;
    END IF;
  END PROCESS clk_gen;

  reset_gen: PROCESS 
  BEGIN
    reset <= '1';
    WAIT FOR 20 ns;
    WAIT UNTIL clk'event AND clk = '1';
    WAIT FOR 2 ns;
    reset <= '0';
    WAIT;
  END PROCESS reset_gen;

  resetn <=  NOT reset;

  tb_enb <= resetn AND snkDonen;

  
  rdEnb <= tb_enb WHEN snkDone = '0' ELSE
      '0';

  ce_out <= enb AND (rdEnb AND tb_enb);

  accel_cmd_enb <= ce_out AND accel_cmd_active;

  -- Count-limited unsigned counter: 0 to 800, step 1
  accel_cmd_process : PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      accel_cmd_addr <= to_unsigned(16#000#, 10);
    ELSIF clk'EVENT AND clk = '1' THEN
      IF accel_cmd_enb = '1' THEN
        IF accel_cmd_addr >= to_unsigned(16#320#, 10) THEN 
          accel_cmd_addr <= to_unsigned(16#000#, 10);
        ELSE 
          accel_cmd_addr <= accel_cmd_addr + to_unsigned(16#001#, 10);
        END IF;
      END IF;
    END IF;
  END PROCESS accel_cmd_process;


  
  accel_cmd_lastAddr <= '1' WHEN accel_cmd_addr >= to_unsigned(16#320#, 10) ELSE
      '0';

  accel_cmd_done <= accel_cmd_lastAddr AND resetn;

  -- Delay to allow last sim cycle to complete
  checkDone_1_process: PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      check1_done <= '0';
    ELSIF clk'event AND clk = '1' THEN
      IF accel_cmd_done_enb = '1' THEN
        check1_done <= accel_cmd_done;
      END IF;
    END IF;
  END PROCESS checkDone_1_process;

  snkDone <= check1_done AND check2_done;

  
  x_active <= '1' WHEN x_addr /= to_unsigned(16#320#, 10) ELSE
      '0';

  x_enb <= x_active AND (rdEnb AND tb_enb);

  -- Count-limited unsigned counter: 0 to 800, step 1
  x_process : PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      x_addr <= to_unsigned(16#000#, 10);
    ELSIF clk'EVENT AND clk = '1' THEN
      IF x_enb = '1' THEN
        IF x_addr >= to_unsigned(16#320#, 10) THEN 
          x_addr <= to_unsigned(16#000#, 10);
        ELSE 
          x_addr <= x_addr + to_unsigned(16#001#, 10);
        END IF;
      END IF;
    END IF;
  END PROCESS x_process;


  x_addr_delay_1 <= x_addr AFTER 1 ns;

  -- Data source for x
  x_fileread: PROCESS (x_addr_delay_1, tb_enb, rdEnb)
    FILE fp: TEXT open READ_MODE is "x.dat";
    VARIABLE l: LINE;
    VARIABLE read_data: std_logic_vector(15 DOWNTO 0);

  BEGIN
    IF tb_enb /= '1' THEN
    ELSIF rdEnb = '1' AND NOT ENDFILE(fp) THEN
      READLINE(fp, l);
      HREAD(l, read_data);
    END IF;
    rawData_x <= signed(read_data(15 DOWNTO 0));
  END PROCESS x_fileread;

  -- holdData reg for x
  stimuli_x_process: PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      holdData_x <= (OTHERS => 'X');
    ELSIF clk'event AND clk = '1' THEN
      holdData_x <= rawData_x;
    END IF;
  END PROCESS stimuli_x_process;

  y_addr_delay_1 <= x_addr AFTER 1 ns;

  -- Data source for y
  y_fileread: PROCESS (y_addr_delay_1, tb_enb, rdEnb)
    FILE fp: TEXT open READ_MODE is "y.dat";
    VARIABLE l: LINE;
    VARIABLE read_data: std_logic_vector(15 DOWNTO 0);

  BEGIN
    IF tb_enb /= '1' THEN
    ELSIF rdEnb = '1' AND NOT ENDFILE(fp) THEN
      READLINE(fp, l);
      HREAD(l, read_data);
    END IF;
    rawData_y <= signed(read_data(15 DOWNTO 0));
  END PROCESS y_fileread;

  -- holdData reg for y
  stimuli_y_process: PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      holdData_y <= (OTHERS => 'X');
    ELSIF clk'event AND clk = '1' THEN
      holdData_y <= rawData_y;
    END IF;
  END PROCESS stimuli_y_process;

  psi_addr_delay_1 <= x_addr AFTER 1 ns;

  -- Data source for psi
  psi_fileread: PROCESS (psi_addr_delay_1, tb_enb, rdEnb)
    FILE fp: TEXT open READ_MODE is "psi.dat";
    VARIABLE l: LINE;
    VARIABLE read_data: std_logic_vector(15 DOWNTO 0);

  BEGIN
    IF tb_enb /= '1' THEN
    ELSIF rdEnb = '1' AND NOT ENDFILE(fp) THEN
      READLINE(fp, l);
      HREAD(l, read_data);
    END IF;
    rawData_psi <= signed(read_data(15 DOWNTO 0));
  END PROCESS psi_fileread;

  -- holdData reg for psi
  stimuli_psi_process: PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      holdData_psi <= (OTHERS => 'X');
    ELSIF clk'event AND clk = '1' THEN
      holdData_psi <= rawData_psi;
    END IF;
  END PROCESS stimuli_psi_process;

  v_addr_delay_1 <= x_addr AFTER 1 ns;

  -- Data source for v
  v_fileread: PROCESS (v_addr_delay_1, tb_enb, rdEnb)
    FILE fp: TEXT open READ_MODE is "v.dat";
    VARIABLE l: LINE;
    VARIABLE read_data: std_logic_vector(15 DOWNTO 0);

  BEGIN
    IF tb_enb /= '1' THEN
    ELSIF rdEnb = '1' AND NOT ENDFILE(fp) THEN
      READLINE(fp, l);
      HREAD(l, read_data);
    END IF;
    rawData_v <= signed(read_data(15 DOWNTO 0));
  END PROCESS v_fileread;

  -- holdData reg for v
  stimuli_v_process: PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      holdData_v <= (OTHERS => 'X');
    ELSIF clk'event AND clk = '1' THEN
      holdData_v <= rawData_v;
    END IF;
  END PROCESS stimuli_v_process;

  ref_x_addr_delay_1 <= x_addr AFTER 1 ns;

  -- Data source for ref_x
  ref_x_fileread: PROCESS (ref_x_addr_delay_1, tb_enb, rdEnb)
    FILE fp: TEXT open READ_MODE is "ref_x.dat";
    VARIABLE l: LINE;
    VARIABLE read_data: std_logic_vector(15 DOWNTO 0);

  BEGIN
    IF tb_enb /= '1' THEN
    ELSIF rdEnb = '1' AND NOT ENDFILE(fp) THEN
      READLINE(fp, l);
      HREAD(l, read_data);
    END IF;
    rawData_ref_x <= signed(read_data(15 DOWNTO 0));
  END PROCESS ref_x_fileread;

  -- holdData reg for ref_x
  stimuli_ref_x_process: PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      holdData_ref_x <= (OTHERS => 'X');
    ELSIF clk'event AND clk = '1' THEN
      holdData_ref_x <= rawData_ref_x;
    END IF;
  END PROCESS stimuli_ref_x_process;

  ref_y_addr_delay_1 <= x_addr AFTER 1 ns;

  -- Data source for ref_y
  ref_y_fileread: PROCESS (ref_y_addr_delay_1, tb_enb, rdEnb)
    FILE fp: TEXT open READ_MODE is "ref_y.dat";
    VARIABLE l: LINE;
    VARIABLE read_data: std_logic_vector(15 DOWNTO 0);

  BEGIN
    IF tb_enb /= '1' THEN
    ELSIF rdEnb = '1' AND NOT ENDFILE(fp) THEN
      READLINE(fp, l);
      HREAD(l, read_data);
    END IF;
    rawData_ref_y <= signed(read_data(15 DOWNTO 0));
  END PROCESS ref_y_fileread;

  -- holdData reg for ref_y
  stimuli_ref_y_process: PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      holdData_ref_y <= (OTHERS => 'X');
    ELSIF clk'event AND clk = '1' THEN
      holdData_ref_y <= rawData_ref_y;
    END IF;
  END PROCESS stimuli_ref_y_process;

  ref_v_addr_delay_1 <= x_addr AFTER 1 ns;

  -- Data source for ref_v
  ref_v_fileread: PROCESS (ref_v_addr_delay_1, tb_enb, rdEnb)
    FILE fp: TEXT open READ_MODE is "ref_v.dat";
    VARIABLE l: LINE;
    VARIABLE read_data: std_logic_vector(15 DOWNTO 0);

  BEGIN
    IF tb_enb /= '1' THEN
    ELSIF rdEnb = '1' AND NOT ENDFILE(fp) THEN
      READLINE(fp, l);
      HREAD(l, read_data);
    END IF;
    rawData_ref_v <= signed(read_data(15 DOWNTO 0));
  END PROCESS ref_v_fileread;

  -- holdData reg for ref_v
  stimuli_ref_v_process: PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      holdData_ref_v <= (OTHERS => 'X');
    ELSIF clk'event AND clk = '1' THEN
      holdData_ref_v <= rawData_ref_v;
    END IF;
  END PROCESS stimuli_ref_v_process;

  stimuli_x_1: PROCESS (rawData_x, rdEnb)
  BEGIN
    IF rdEnb = '0' THEN
      x_offset <= holdData_x;
    ELSE
      x_offset <= rawData_x;
    END IF;
  END PROCESS stimuli_x_1;

  x_1 <= x_offset AFTER 2 ns;

  x_2 <= std_logic_vector(x_1);

  stimuli_y_1: PROCESS (rawData_y, rdEnb)
  BEGIN
    IF rdEnb = '0' THEN
      y_offset <= holdData_y;
    ELSE
      y_offset <= rawData_y;
    END IF;
  END PROCESS stimuli_y_1;

  y <= y_offset AFTER 2 ns;

  y_1 <= std_logic_vector(y);

  stimuli_psi_1: PROCESS (rawData_psi, rdEnb)
  BEGIN
    IF rdEnb = '0' THEN
      psi_offset <= holdData_psi;
    ELSE
      psi_offset <= rawData_psi;
    END IF;
  END PROCESS stimuli_psi_1;

  psi <= psi_offset AFTER 2 ns;

  psi_1 <= std_logic_vector(psi);

  stimuli_v_1: PROCESS (rawData_v, rdEnb)
  BEGIN
    IF rdEnb = '0' THEN
      v_offset <= holdData_v;
    ELSE
      v_offset <= rawData_v;
    END IF;
  END PROCESS stimuli_v_1;

  v <= v_offset AFTER 2 ns;

  v_1 <= std_logic_vector(v);

  stimuli_ref_x_1: PROCESS (rawData_ref_x, rdEnb)
  BEGIN
    IF rdEnb = '0' THEN
      ref_x_offset <= holdData_ref_x;
    ELSE
      ref_x_offset <= rawData_ref_x;
    END IF;
  END PROCESS stimuli_ref_x_1;

  ref_x <= ref_x_offset AFTER 2 ns;

  ref_x_1 <= std_logic_vector(ref_x);

  stimuli_ref_y_1: PROCESS (rawData_ref_y, rdEnb)
  BEGIN
    IF rdEnb = '0' THEN
      ref_y_offset <= holdData_ref_y;
    ELSE
      ref_y_offset <= rawData_ref_y;
    END IF;
  END PROCESS stimuli_ref_y_1;

  ref_y <= ref_y_offset AFTER 2 ns;

  ref_y_1 <= std_logic_vector(ref_y);

  stimuli_ref_v_1: PROCESS (rawData_ref_v, rdEnb)
  BEGIN
    IF rdEnb = '0' THEN
      ref_v_offset <= holdData_ref_v;
    ELSE
      ref_v_offset <= rawData_ref_v;
    END IF;
  END PROCESS stimuli_ref_v_1;

  ref_v <= ref_v_offset AFTER 2 ns;

  ref_v_1 <= std_logic_vector(ref_v);

  accel_cmd_signed <= signed(accel_cmd_1);

  accel_cmd_addr_delay_1 <= accel_cmd_addr AFTER 1 ns;

  -- Data source for accel_cmd_expected
  accel_cmd_expected_fileread: PROCESS (accel_cmd_addr_delay_1, tb_enb, rdEnb)
    FILE fp: TEXT open READ_MODE is "accel_cmd_expected.dat";
    VARIABLE l: LINE;
    VARIABLE read_data: std_logic_vector(15 DOWNTO 0);

  BEGIN
    IF tb_enb /= '1' THEN
    ELSIF rdEnb = '1' AND NOT ENDFILE(fp) THEN
      READLINE(fp, l);
      HREAD(l, read_data);
    END IF;
    accel_cmd_expected <= signed(read_data(15 DOWNTO 0));
  END PROCESS accel_cmd_expected_fileread;

  accel_cmd_ref <= accel_cmd_expected;

  accel_cmd_signed_checker: PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      accel_cmd_testFailure <= '0';
    ELSIF clk'event AND clk = '1' THEN
      IF ce_out = '1' AND accel_cmd_signed /= accel_cmd_ref THEN
        accel_cmd_testFailure <= '1';
        ASSERT FALSE
          REPORT "Error in accel_cmd_signed: Expected " & to_hex(accel_cmd_ref) & (" Actual " & to_hex(accel_cmd_signed))
          SEVERITY ERROR;
      END IF;
    END IF;
  END PROCESS accel_cmd_signed_checker;

  steer_cmd_signed <= signed(steer_cmd);

  steer_cmd_addr_delay_1 <= accel_cmd_addr AFTER 1 ns;

  -- Data source for steer_cmd_expected
  steer_cmd_expected_fileread: PROCESS (steer_cmd_addr_delay_1, tb_enb, ce_out)
    FILE fp: TEXT open READ_MODE is "steer_cmd_expected.dat";
    VARIABLE l: LINE;
    VARIABLE read_data: std_logic_vector(15 DOWNTO 0);

  BEGIN
    IF tb_enb /= '1' THEN
    ELSIF ce_out = '1' AND NOT ENDFILE(fp) THEN
      READLINE(fp, l);
      HREAD(l, read_data);
    END IF;
    steer_cmd_expected <= signed(read_data(15 DOWNTO 0));
  END PROCESS steer_cmd_expected_fileread;

  steer_cmd_ref <= steer_cmd_expected;

  steer_cmd_signed_checker: PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      steer_cmd_testFailure <= '0';
    ELSIF clk'event AND clk = '1' THEN
      IF ce_out = '1' AND steer_cmd_signed /= steer_cmd_ref THEN
        steer_cmd_testFailure <= '1';
        ASSERT FALSE
          REPORT "Error in steer_cmd_signed: Expected " & to_hex(steer_cmd_ref) & (" Actual " & to_hex(steer_cmd_signed))
          SEVERITY ERROR;
      END IF;
    END IF;
  END PROCESS steer_cmd_signed_checker;

  testFailure <= accel_cmd_testFailure OR steer_cmd_testFailure;

  completed_msg: PROCESS (clk)
  BEGIN
    IF clk'event AND clk = '1' THEN
      IF snkDone = '1' THEN
        IF testFailure = '0' THEN
          ASSERT FALSE
            REPORT "**************TEST COMPLETED (PASSED)**************"
            SEVERITY NOTE;
        ELSE
          ASSERT FALSE
            REPORT "**************TEST COMPLETED (FAILED)**************"
            SEVERITY NOTE;
        END IF;
      END IF;
    END IF;
  END PROCESS completed_msg;

END rtl;

