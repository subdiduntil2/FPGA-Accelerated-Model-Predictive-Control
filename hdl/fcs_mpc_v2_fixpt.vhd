-- =========================================================================
-- File Name : fcs_mpc_v2_fixpt.vhd
--
-- Hand-written behavioural implementation of fcs_mpc_v2.m for the Cora
-- Z7-07S MPC IP.  Replaces the HDL-Coder-generated version, which mis-sized
-- the accel_cmd output (ufix4 instead of sfix5/sfix6), narrowed the err
-- clamp logic into wrap-around truncation, and undersized the cost
-- variable to 13 bits.
--
-- Algorithm (identical to fcs_mpc_v2.m):
--   17 steer candidates  : {0, +-2, +-6, +-10, +-14, +-18, +-22, +-24, +-26}
--    8 accel candidates  : {0, 10, 5, 1, -5, -1, -10, -20}
--    Cost = (|err_x|+|err_y|)*15 + |delta|*2 - v_next*2
--           with each axis-error clamped to 1000 before weighting.
--    Pick (delta, accel) that minimise cost over all 17*8 = 136 candidates.
--
-- Port widths must match mpc_axi_wrapper.vhd's slice expressions.
-- IMPORTANT: accel_cmd is WIDENED from sfix5 to sfix6 to be able to carry
--            ACCEL_OPTS = -20 (which doesn't fit in 5-bit two's complement).
--            mpc_axi_wrapper.vhd must be updated so that the accel readback
--            register slices reg_acc(5 DOWNTO 0) (instead of (4 DOWNTO 0)),
--            and the C code's reg_to_s16 already sign-extends correctly.
--
-- Pipeline structure (matches the existing IP wrapper expectation):
--   clk rising_edge   : latch inputs into _reg
--   combinational     : evaluate all 136 candidates, pick min_cost
--   clk rising_edge   : latch best (accel,steer) into output regs
-- Net latency: 1 cycle from input register to output register.
-- =========================================================================

LIBRARY IEEE;
USE IEEE.std_logic_1164.ALL;
USE IEEE.numeric_std.ALL;

ENTITY fcs_mpc_v2_fixpt IS
  PORT( clk       : IN  std_logic;
        reset     : IN  std_logic;
        x         : IN  std_logic_vector(13 DOWNTO 0);  -- sfix14
        y         : IN  std_logic_vector(12 DOWNTO 0);  -- sfix13
        psi       : IN  std_logic_vector( 8 DOWNTO 0);  -- sfix9
        v         : IN  std_logic_vector( 8 DOWNTO 0);  -- ufix9
        ref_x     : IN  std_logic_vector(13 DOWNTO 0);  -- sfix14
        ref_y     : IN  std_logic_vector(11 DOWNTO 0);  -- ufix12
        accel_cmd : OUT std_logic_vector( 5 DOWNTO 0);  -- sfix6  (was sfix5, widened for -20)
        steer_cmd : OUT std_logic_vector( 5 DOWNTO 0)   -- sfix6
        );
END fcs_mpc_v2_fixpt;


ARCHITECTURE rtl OF fcs_mpc_v2_fixpt IS

  ATTRIBUTE DONT_TOUCH : string;

  -- -----------------------------------------------------------------------
  -- Inline types (replace HDL Coder's separate _pkg.vhd)
  -- -----------------------------------------------------------------------
  TYPE sfix6_array IS ARRAY (NATURAL RANGE <>) OF signed(5 DOWNTO 0);
  TYPE sfix8_array IS ARRAY (NATURAL RANGE <>) OF signed(7 DOWNTO 0);

  -- -----------------------------------------------------------------------
  -- Algorithm constants (verbatim from fcs_mpc_v2.m)
  -- -----------------------------------------------------------------------
  CONSTANT V_MAX     : signed(15 DOWNTO 0) := to_signed(  320, 16);  -- 5 m/s at SF_V=64
  CONSTANT W_ERR     : signed(15 DOWNTO 0) := to_signed(   15, 16);
  CONSTANT W_STEER   : signed(15 DOWNTO 0) := to_signed(    2, 16);
  CONSTANT W_SPEED   : signed(15 DOWNTO 0) := to_signed(    2, 16);
  CONSTANT CLAMP_VAL : signed(15 DOWNTO 0) := to_signed( 1000, 16);  -- ~15 m
  CONSTANT COST_INIT : signed(15 DOWNTO 0) := to_signed(32767, 16);  -- int16 max

  -- 17 steering candidates  (.m: STEER_OPTS)
  CONSTANT STEER_OPTS : sfix6_array(0 TO 16) := (
    to_signed(  0, 6), to_signed( -2, 6), to_signed(  2, 6),
    to_signed( -6, 6), to_signed(  6, 6), to_signed(-10, 6),
    to_signed( 10, 6), to_signed(-14, 6), to_signed( 14, 6),
    to_signed(-18, 6), to_signed( 18, 6), to_signed(-22, 6),
    to_signed( 22, 6), to_signed(-24, 6), to_signed( 24, 6),
    to_signed(-26, 6), to_signed( 26, 6)
  );

  -- 8 acceleration candidates  (.m: ACCEL_OPTS)
  -- Stored in sfix6 (not sfix5!) because -20 needs at least 6 bits signed.
  CONSTANT ACCEL_OPTS : sfix6_array(0 TO 7) := (
    to_signed(  0, 6), to_signed( 10, 6), to_signed(  5, 6), to_signed(  1, 6),
    to_signed( -5, 6), to_signed( -1, 6), to_signed(-10, 6), to_signed(-20, 6)
  );

  -- 256-entry sin / cos LUTs.  Built in .m from the q1_a/b/c/d quartets:
  --   Q1 = [q1_a, q1_b, q1_c, q1_d];      -- 64 ascending values 0..127
  --   Q2 = Q1(end:-1:1);  Q3 = -Q1;  Q4 = -Q2;
  --   SIN_LUT = [Q1, Q2, Q3, Q4];
  --   COS_LUT = [Q2, Q3, Q4, Q1];
  -- Generated programmatically -- do not edit by hand.
  CONSTANT SIN_LUT : sfix8_array(0 TO 255) := (
    to_signed(   0, 8), to_signed(   3, 8), to_signed(   6, 8), to_signed(   9, 8), to_signed(  12, 8), to_signed(  16, 8), to_signed(  19, 8), to_signed(  22, 8),
    to_signed(  25, 8), to_signed(  28, 8), to_signed(  31, 8), to_signed(  34, 8), to_signed(  37, 8), to_signed(  40, 8), to_signed(  43, 8), to_signed(  46, 8),
    to_signed(  49, 8), to_signed(  51, 8), to_signed(  54, 8), to_signed(  57, 8), to_signed(  60, 8), to_signed(  63, 8), to_signed(  65, 8), to_signed(  68, 8),
    to_signed(  71, 8), to_signed(  73, 8), to_signed(  76, 8), to_signed(  78, 8), to_signed(  81, 8), to_signed(  83, 8), to_signed(  85, 8), to_signed(  88, 8),
    to_signed(  90, 8), to_signed(  92, 8), to_signed(  94, 8), to_signed(  96, 8), to_signed(  98, 8), to_signed( 100, 8), to_signed( 102, 8), to_signed( 104, 8),
    to_signed( 106, 8), to_signed( 107, 8), to_signed( 109, 8), to_signed( 111, 8), to_signed( 112, 8), to_signed( 114, 8), to_signed( 115, 8), to_signed( 116, 8),
    to_signed( 118, 8), to_signed( 119, 8), to_signed( 120, 8), to_signed( 121, 8), to_signed( 122, 8), to_signed( 123, 8), to_signed( 124, 8), to_signed( 124, 8),
    to_signed( 125, 8), to_signed( 126, 8), to_signed( 126, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8),
    to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 126, 8), to_signed( 126, 8), to_signed( 125, 8),
    to_signed( 124, 8), to_signed( 124, 8), to_signed( 123, 8), to_signed( 122, 8), to_signed( 121, 8), to_signed( 120, 8), to_signed( 119, 8), to_signed( 118, 8),
    to_signed( 116, 8), to_signed( 115, 8), to_signed( 114, 8), to_signed( 112, 8), to_signed( 111, 8), to_signed( 109, 8), to_signed( 107, 8), to_signed( 106, 8),
    to_signed( 104, 8), to_signed( 102, 8), to_signed( 100, 8), to_signed(  98, 8), to_signed(  96, 8), to_signed(  94, 8), to_signed(  92, 8), to_signed(  90, 8),
    to_signed(  88, 8), to_signed(  85, 8), to_signed(  83, 8), to_signed(  81, 8), to_signed(  78, 8), to_signed(  76, 8), to_signed(  73, 8), to_signed(  71, 8),
    to_signed(  68, 8), to_signed(  65, 8), to_signed(  63, 8), to_signed(  60, 8), to_signed(  57, 8), to_signed(  54, 8), to_signed(  51, 8), to_signed(  49, 8),
    to_signed(  46, 8), to_signed(  43, 8), to_signed(  40, 8), to_signed(  37, 8), to_signed(  34, 8), to_signed(  31, 8), to_signed(  28, 8), to_signed(  25, 8),
    to_signed(  22, 8), to_signed(  19, 8), to_signed(  16, 8), to_signed(  12, 8), to_signed(   9, 8), to_signed(   6, 8), to_signed(   3, 8), to_signed(   0, 8),
    to_signed(   0, 8), to_signed(  -3, 8), to_signed(  -6, 8), to_signed(  -9, 8), to_signed( -12, 8), to_signed( -16, 8), to_signed( -19, 8), to_signed( -22, 8),
    to_signed( -25, 8), to_signed( -28, 8), to_signed( -31, 8), to_signed( -34, 8), to_signed( -37, 8), to_signed( -40, 8), to_signed( -43, 8), to_signed( -46, 8),
    to_signed( -49, 8), to_signed( -51, 8), to_signed( -54, 8), to_signed( -57, 8), to_signed( -60, 8), to_signed( -63, 8), to_signed( -65, 8), to_signed( -68, 8),
    to_signed( -71, 8), to_signed( -73, 8), to_signed( -76, 8), to_signed( -78, 8), to_signed( -81, 8), to_signed( -83, 8), to_signed( -85, 8), to_signed( -88, 8),
    to_signed( -90, 8), to_signed( -92, 8), to_signed( -94, 8), to_signed( -96, 8), to_signed( -98, 8), to_signed(-100, 8), to_signed(-102, 8), to_signed(-104, 8),
    to_signed(-106, 8), to_signed(-107, 8), to_signed(-109, 8), to_signed(-111, 8), to_signed(-112, 8), to_signed(-114, 8), to_signed(-115, 8), to_signed(-116, 8),
    to_signed(-118, 8), to_signed(-119, 8), to_signed(-120, 8), to_signed(-121, 8), to_signed(-122, 8), to_signed(-123, 8), to_signed(-124, 8), to_signed(-124, 8),
    to_signed(-125, 8), to_signed(-126, 8), to_signed(-126, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8),
    to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-126, 8), to_signed(-126, 8), to_signed(-125, 8),
    to_signed(-124, 8), to_signed(-124, 8), to_signed(-123, 8), to_signed(-122, 8), to_signed(-121, 8), to_signed(-120, 8), to_signed(-119, 8), to_signed(-118, 8),
    to_signed(-116, 8), to_signed(-115, 8), to_signed(-114, 8), to_signed(-112, 8), to_signed(-111, 8), to_signed(-109, 8), to_signed(-107, 8), to_signed(-106, 8),
    to_signed(-104, 8), to_signed(-102, 8), to_signed(-100, 8), to_signed( -98, 8), to_signed( -96, 8), to_signed( -94, 8), to_signed( -92, 8), to_signed( -90, 8),
    to_signed( -88, 8), to_signed( -85, 8), to_signed( -83, 8), to_signed( -81, 8), to_signed( -78, 8), to_signed( -76, 8), to_signed( -73, 8), to_signed( -71, 8),
    to_signed( -68, 8), to_signed( -65, 8), to_signed( -63, 8), to_signed( -60, 8), to_signed( -57, 8), to_signed( -54, 8), to_signed( -51, 8), to_signed( -49, 8),
    to_signed( -46, 8), to_signed( -43, 8), to_signed( -40, 8), to_signed( -37, 8), to_signed( -34, 8), to_signed( -31, 8), to_signed( -28, 8), to_signed( -25, 8),
    to_signed( -22, 8), to_signed( -19, 8), to_signed( -16, 8), to_signed( -12, 8), to_signed(  -9, 8), to_signed(  -6, 8), to_signed(  -3, 8), to_signed(   0, 8)
  );

  CONSTANT COS_LUT : sfix8_array(0 TO 255) := (
    to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 126, 8), to_signed( 126, 8), to_signed( 125, 8),
    to_signed( 124, 8), to_signed( 124, 8), to_signed( 123, 8), to_signed( 122, 8), to_signed( 121, 8), to_signed( 120, 8), to_signed( 119, 8), to_signed( 118, 8),
    to_signed( 116, 8), to_signed( 115, 8), to_signed( 114, 8), to_signed( 112, 8), to_signed( 111, 8), to_signed( 109, 8), to_signed( 107, 8), to_signed( 106, 8),
    to_signed( 104, 8), to_signed( 102, 8), to_signed( 100, 8), to_signed(  98, 8), to_signed(  96, 8), to_signed(  94, 8), to_signed(  92, 8), to_signed(  90, 8),
    to_signed(  88, 8), to_signed(  85, 8), to_signed(  83, 8), to_signed(  81, 8), to_signed(  78, 8), to_signed(  76, 8), to_signed(  73, 8), to_signed(  71, 8),
    to_signed(  68, 8), to_signed(  65, 8), to_signed(  63, 8), to_signed(  60, 8), to_signed(  57, 8), to_signed(  54, 8), to_signed(  51, 8), to_signed(  49, 8),
    to_signed(  46, 8), to_signed(  43, 8), to_signed(  40, 8), to_signed(  37, 8), to_signed(  34, 8), to_signed(  31, 8), to_signed(  28, 8), to_signed(  25, 8),
    to_signed(  22, 8), to_signed(  19, 8), to_signed(  16, 8), to_signed(  12, 8), to_signed(   9, 8), to_signed(   6, 8), to_signed(   3, 8), to_signed(   0, 8),
    to_signed(   0, 8), to_signed(  -3, 8), to_signed(  -6, 8), to_signed(  -9, 8), to_signed( -12, 8), to_signed( -16, 8), to_signed( -19, 8), to_signed( -22, 8),
    to_signed( -25, 8), to_signed( -28, 8), to_signed( -31, 8), to_signed( -34, 8), to_signed( -37, 8), to_signed( -40, 8), to_signed( -43, 8), to_signed( -46, 8),
    to_signed( -49, 8), to_signed( -51, 8), to_signed( -54, 8), to_signed( -57, 8), to_signed( -60, 8), to_signed( -63, 8), to_signed( -65, 8), to_signed( -68, 8),
    to_signed( -71, 8), to_signed( -73, 8), to_signed( -76, 8), to_signed( -78, 8), to_signed( -81, 8), to_signed( -83, 8), to_signed( -85, 8), to_signed( -88, 8),
    to_signed( -90, 8), to_signed( -92, 8), to_signed( -94, 8), to_signed( -96, 8), to_signed( -98, 8), to_signed(-100, 8), to_signed(-102, 8), to_signed(-104, 8),
    to_signed(-106, 8), to_signed(-107, 8), to_signed(-109, 8), to_signed(-111, 8), to_signed(-112, 8), to_signed(-114, 8), to_signed(-115, 8), to_signed(-116, 8),
    to_signed(-118, 8), to_signed(-119, 8), to_signed(-120, 8), to_signed(-121, 8), to_signed(-122, 8), to_signed(-123, 8), to_signed(-124, 8), to_signed(-124, 8),
    to_signed(-125, 8), to_signed(-126, 8), to_signed(-126, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8),
    to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-127, 8), to_signed(-126, 8), to_signed(-126, 8), to_signed(-125, 8),
    to_signed(-124, 8), to_signed(-124, 8), to_signed(-123, 8), to_signed(-122, 8), to_signed(-121, 8), to_signed(-120, 8), to_signed(-119, 8), to_signed(-118, 8),
    to_signed(-116, 8), to_signed(-115, 8), to_signed(-114, 8), to_signed(-112, 8), to_signed(-111, 8), to_signed(-109, 8), to_signed(-107, 8), to_signed(-106, 8),
    to_signed(-104, 8), to_signed(-102, 8), to_signed(-100, 8), to_signed( -98, 8), to_signed( -96, 8), to_signed( -94, 8), to_signed( -92, 8), to_signed( -90, 8),
    to_signed( -88, 8), to_signed( -85, 8), to_signed( -83, 8), to_signed( -81, 8), to_signed( -78, 8), to_signed( -76, 8), to_signed( -73, 8), to_signed( -71, 8),
    to_signed( -68, 8), to_signed( -65, 8), to_signed( -63, 8), to_signed( -60, 8), to_signed( -57, 8), to_signed( -54, 8), to_signed( -51, 8), to_signed( -49, 8),
    to_signed( -46, 8), to_signed( -43, 8), to_signed( -40, 8), to_signed( -37, 8), to_signed( -34, 8), to_signed( -31, 8), to_signed( -28, 8), to_signed( -25, 8),
    to_signed( -22, 8), to_signed( -19, 8), to_signed( -16, 8), to_signed( -12, 8), to_signed(  -9, 8), to_signed(  -6, 8), to_signed(  -3, 8), to_signed(   0, 8),
    to_signed(   0, 8), to_signed(   3, 8), to_signed(   6, 8), to_signed(   9, 8), to_signed(  12, 8), to_signed(  16, 8), to_signed(  19, 8), to_signed(  22, 8),
    to_signed(  25, 8), to_signed(  28, 8), to_signed(  31, 8), to_signed(  34, 8), to_signed(  37, 8), to_signed(  40, 8), to_signed(  43, 8), to_signed(  46, 8),
    to_signed(  49, 8), to_signed(  51, 8), to_signed(  54, 8), to_signed(  57, 8), to_signed(  60, 8), to_signed(  63, 8), to_signed(  65, 8), to_signed(  68, 8),
    to_signed(  71, 8), to_signed(  73, 8), to_signed(  76, 8), to_signed(  78, 8), to_signed(  81, 8), to_signed(  83, 8), to_signed(  85, 8), to_signed(  88, 8),
    to_signed(  90, 8), to_signed(  92, 8), to_signed(  94, 8), to_signed(  96, 8), to_signed(  98, 8), to_signed( 100, 8), to_signed( 102, 8), to_signed( 104, 8),
    to_signed( 106, 8), to_signed( 107, 8), to_signed( 109, 8), to_signed( 111, 8), to_signed( 112, 8), to_signed( 114, 8), to_signed( 115, 8), to_signed( 116, 8),
    to_signed( 118, 8), to_signed( 119, 8), to_signed( 120, 8), to_signed( 121, 8), to_signed( 122, 8), to_signed( 123, 8), to_signed( 124, 8), to_signed( 124, 8),
    to_signed( 125, 8), to_signed( 126, 8), to_signed( 126, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8), to_signed( 127, 8)
  );

  -- -----------------------------------------------------------------------
  -- Pipeline signals
  -- -----------------------------------------------------------------------
  SIGNAL x_reg     : signed(13 DOWNTO 0);
  SIGNAL y_reg     : signed(12 DOWNTO 0);
  SIGNAL psi_reg   : signed( 8 DOWNTO 0);
  SIGNAL v_reg     : unsigned(8 DOWNTO 0);
  SIGNAL ref_x_reg : signed(13 DOWNTO 0);
  SIGNAL ref_y_reg : unsigned(11 DOWNTO 0);

  SIGNAL accel_cmd_tmp : signed(5 DOWNTO 0);
  SIGNAL steer_cmd_tmp : signed(5 DOWNTO 0);

  SIGNAL accel_cmd_reg : signed(5 DOWNTO 0);
  SIGNAL steer_cmd_reg : signed(5 DOWNTO 0);

  ATTRIBUTE DONT_TOUCH OF x_reg         : SIGNAL IS "true";
  ATTRIBUTE DONT_TOUCH OF y_reg         : SIGNAL IS "true";
  ATTRIBUTE DONT_TOUCH OF psi_reg       : SIGNAL IS "true";
  ATTRIBUTE DONT_TOUCH OF v_reg         : SIGNAL IS "true";
  ATTRIBUTE DONT_TOUCH OF ref_x_reg     : SIGNAL IS "true";
  ATTRIBUTE DONT_TOUCH OF ref_y_reg     : SIGNAL IS "true";
  ATTRIBUTE DONT_TOUCH OF accel_cmd_tmp : SIGNAL IS "true";
  ATTRIBUTE DONT_TOUCH OF steer_cmd_tmp : SIGNAL IS "true";
  ATTRIBUTE DONT_TOUCH OF accel_cmd_reg : SIGNAL IS "true";
  ATTRIBUTE DONT_TOUCH OF steer_cmd_reg : SIGNAL IS "true";


BEGIN

  -- =======================================================================
  -- Input register stage
  -- =======================================================================
  input_reg_process : PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      x_reg     <= (OTHERS => '0');
      y_reg     <= (OTHERS => '0');
      psi_reg   <= (OTHERS => '0');
      v_reg     <= (OTHERS => '0');
      ref_x_reg <= (OTHERS => '0');
      ref_y_reg <= (OTHERS => '0');
    ELSIF rising_edge(clk) THEN
      x_reg     <= signed  (x);
      y_reg     <= signed  (y);
      psi_reg   <= signed  (psi);
      v_reg     <= unsigned(v);
      ref_x_reg <= signed  (ref_x);
      ref_y_reg <= unsigned(ref_y);
    END IF;
  END PROCESS;


  -- =======================================================================
  -- Combinational compute: evaluate 17*8 = 136 candidates, pick min cost
  -- All arithmetic in signed(15 DOWNTO 0) -- matches int16 semantics of .m.
  -- =======================================================================
  compute_process : PROCESS (x_reg, y_reg, psi_reg, v_reg, ref_x_reg, ref_y_reg)
    -- Promoted inputs (int16-equivalent)
    VARIABLE x_v, y_v, psi_v, v_v, ref_x_v, ref_y_v : signed(15 DOWNTO 0);

    -- Inner-loop scratch
    VARIABLE delta, accel, abs_delta : signed(15 DOWNTO 0);
    VARIABLE v_next                  : signed(15 DOWNTO 0);
    VARIABLE prod32                  : signed(31 DOWNTO 0);
    VARIABLE dpsi, psi_next          : signed(15 DOWNTO 0);
    VARIABLE lut_idx                 : INTEGER RANGE 0 TO 255;
    VARIABLE c_val, s_val            : signed(15 DOWNTO 0);
    VARIABLE v_red, dx, dy           : signed(15 DOWNTO 0);
    VARIABLE x_next, y_next          : signed(15 DOWNTO 0);
    VARIABLE err_x, err_y            : signed(15 DOWNTO 0);
    VARIABLE abs_x, abs_y            : signed(15 DOWNTO 0);
    VARIABLE dist_cost, steer_cost, speed_rwd, cost : signed(15 DOWNTO 0);

    -- Best-so-far across all 136 candidates
    VARIABLE min_cost : signed(15 DOWNTO 0);
    VARIABLE best_acc : signed(5 DOWNTO 0);
    VARIABLE best_str : signed(5 DOWNTO 0);
  BEGIN
    -- Promote inputs to 16-bit signed.  v and ref_y are unsigned, so
    -- they go through resize-as-unsigned then re-tag as signed.
    x_v     := resize(x_reg, 16);
    y_v     := resize(y_reg, 16);
    psi_v   := resize(psi_reg, 16);
    v_v     := signed(resize(v_reg, 16));
    ref_x_v := resize(ref_x_reg, 16);
    ref_y_v := signed(resize(ref_y_reg, 16));

    min_cost := COST_INIT;
    best_acc := to_signed(0, 6);
    best_str := to_signed(0, 6);

    FOR s_idx IN 0 TO 16 LOOP
      delta     := resize(STEER_OPTS(s_idx), 16);
      IF delta < 0 THEN abs_delta := -delta; ELSE abs_delta := delta; END IF;

      -- dpsi = bitsra(v * delta, 8)  -- arithmetic right shift on full product
      prod32 := v_v * delta;
      dpsi   := resize(shift_right(prod32, 8), 16);

      -- psi_next = psi + dpsi
      psi_next := psi_v + dpsi;

      -- LUT index = low 8 bits of psi_next, interpreted unsigned (0..255).
      -- (.m: bitand(psi_next, 255).  The +1 in .m is just 1-based indexing
      --  and is absorbed into 0-based VHDL indexing.)
      lut_idx := to_integer(unsigned(psi_next(7 DOWNTO 0)));
      c_val   := resize(COS_LUT(lut_idx), 16);
      s_val   := resize(SIN_LUT(lut_idx), 16);

      -- v_red = bitsra(v, 4)
      v_red := shift_right(v_v, 4);

      -- dx = bitsra(v_red * c_val, 5);  dy = bitsra(v_red * s_val, 5)
      prod32 := v_red * c_val;
      dx     := resize(shift_right(prod32, 5), 16);
      prod32 := v_red * s_val;
      dy     := resize(shift_right(prod32, 5), 16);

      x_next := x_v + dx;
      y_next := y_v + dy;

      FOR a_idx IN 0 TO 7 LOOP
        accel := resize(ACCEL_OPTS(a_idx), 16);

        -- v_next = saturate(v + accel, [0, V_MAX])
        v_next := v_v + accel;
        IF v_next > V_MAX             THEN v_next := V_MAX;             END IF;
        IF v_next < to_signed(0, 16)  THEN v_next := to_signed(0, 16);  END IF;

        err_x := x_next - ref_x_v;
        err_y := y_next - ref_y_v;

        -- abs() and clamp to CLAMP_VAL
        IF err_x < 0          THEN abs_x := -err_x;    ELSE abs_x := err_x;    END IF;
        IF err_y < 0          THEN abs_y := -err_y;    ELSE abs_y := err_y;    END IF;
        IF abs_x > CLAMP_VAL  THEN abs_x := CLAMP_VAL; END IF;
        IF abs_y > CLAMP_VAL  THEN abs_y := CLAMP_VAL; END IF;

        -- Cost terms (all fit in signed(15:0) -- see comment block above).
        --   (abs_x+abs_y) max 2000, * 15  -> 30000.   |delta| max 26, * 2 -> 52.
        --    v_next max 320, * 2          -> 640.     cost in [-640, 30052].
        dist_cost  := resize((abs_x + abs_y) * W_ERR, 16);
        steer_cost := resize(abs_delta       * W_STEER, 16);
        speed_rwd  := resize(v_next          * W_SPEED, 16);
        cost       := dist_cost + steer_cost - speed_rwd;

        IF cost < min_cost THEN
          min_cost := cost;
          best_acc := resize(ACCEL_OPTS(a_idx), 6);
          best_str := resize(STEER_OPTS(s_idx), 6);
        END IF;
      END LOOP;
    END LOOP;

    accel_cmd_tmp <= best_acc;
    steer_cmd_tmp <= best_str;
  END PROCESS;


  -- =======================================================================
  -- Output register stage
  -- =======================================================================
  output_reg_process : PROCESS (clk, reset)
  BEGIN
    IF reset = '1' THEN
      accel_cmd_reg <= (OTHERS => '0');
      steer_cmd_reg <= (OTHERS => '0');
    ELSIF rising_edge(clk) THEN
      accel_cmd_reg <= accel_cmd_tmp;
      steer_cmd_reg <= steer_cmd_tmp;
    END IF;
  END PROCESS;

  accel_cmd <= std_logic_vector(accel_cmd_reg);
  steer_cmd <= std_logic_vector(steer_cmd_reg);

END rtl;