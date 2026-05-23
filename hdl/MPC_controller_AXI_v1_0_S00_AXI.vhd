-- AXI4-Lite slave: register bank + kick FSM driving the fcs_mpc_v2_fixpt core.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity MPC_controller_AXI_v1_0_S00_AXI is
    generic (
        C_S_AXI_DATA_WIDTH    : integer    := 32;
        C_S_AXI_ADDR_WIDTH    : integer    := 7
    );
    port (
        mpc_irq         : out std_logic;
        S_AXI_ACLK      : in std_logic;
        S_AXI_ARESETN   : in std_logic;
        S_AXI_AWADDR    : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
        S_AXI_AWPROT    : in std_logic_vector(2 downto 0);
        S_AXI_AWVALID   : in std_logic;
        S_AXI_AWREADY   : out std_logic;
        S_AXI_WDATA     : in std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        S_AXI_WSTRB     : in std_logic_vector((C_S_AXI_DATA_WIDTH/8)-1 downto 0);
        S_AXI_WVALID    : in std_logic;
        S_AXI_WREADY    : out std_logic;
        S_AXI_BRESP     : out std_logic_vector(1 downto 0);
        S_AXI_BVALID    : out std_logic;
        S_AXI_BREADY    : in std_logic;
        S_AXI_ARADDR    : in std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
        S_AXI_ARPROT    : in std_logic_vector(2 downto 0);
        S_AXI_ARVALID   : in std_logic;
        S_AXI_ARREADY   : out std_logic;
        S_AXI_RDATA     : out std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
        S_AXI_RRESP     : out std_logic_vector(1 downto 0);
        S_AXI_RVALID    : out std_logic;
        S_AXI_RREADY    : in std_logic
    );
end MPC_controller_AXI_v1_0_S00_AXI;

architecture arch_imp of MPC_controller_AXI_v1_0_S00_AXI is

    signal axi_awaddr   : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    signal axi_awready  : std_logic;
    signal axi_wready   : std_logic;
    signal axi_bresp    : std_logic_vector(1 downto 0);
    signal axi_bvalid   : std_logic;
    signal axi_araddr   : std_logic_vector(C_S_AXI_ADDR_WIDTH-1 downto 0);
    signal axi_arready  : std_logic;
    signal axi_rdata    : std_logic_vector(C_S_AXI_DATA_WIDTH-1 downto 0);
    signal axi_rresp    : std_logic_vector(1 downto 0);
    signal axi_rvalid   : std_logic;

    signal slv_reg0     : std_logic_vector(31 downto 0);
    signal slv_reg1     : std_logic_vector(31 downto 0);
    signal slv_reg4     : std_logic_vector(31 downto 0);
    signal slv_reg5     : std_logic_vector(31 downto 0);
    signal slv_reg6     : std_logic_vector(31 downto 0);
    signal slv_reg7     : std_logic_vector(31 downto 0);
    signal slv_reg8     : std_logic_vector(31 downto 0);
    signal slv_reg9     : std_logic_vector(31 downto 0);
    signal slv_reg12    : std_logic_vector(31 downto 0);
    signal slv_reg13    : std_logic_vector(31 downto 0);
    signal slv_reg16    : std_logic_vector(31 downto 0);

    signal slv_reg_wren : std_logic;
    signal slv_reg_rden : std_logic;
    signal aw_en        : std_logic;
    constant ADDR_LSB   : integer := 2;

    signal mpc_reset    : std_logic;
    signal mpc_accel    : std_logic_vector(5 downto 0);
    signal mpc_steer    : std_logic_vector(5 downto 0);
    type state_t is (S_IDLE, S_RUN, S_CAPTURE);
    signal state        : state_t := S_IDLE;
    signal wait_cnt     : unsigned(3 downto 0) := (others => '0');

begin
    S_AXI_AWREADY <= axi_awready;
    S_AXI_WREADY  <= axi_wready;
    S_AXI_BRESP   <= axi_bresp;
    S_AXI_BVALID  <= axi_bvalid;
    S_AXI_ARREADY <= axi_arready;
    S_AXI_RDATA   <= axi_rdata;
    S_AXI_RRESP   <= axi_rresp;
    S_AXI_RVALID  <= axi_rvalid;
    mpc_reset     <= not S_AXI_ARESETN;

    -- MPC core instance
    u_mpc : entity work.fcs_mpc_v2_fixpt
        port map (
            clk       => S_AXI_ACLK,
            reset     => mpc_reset,
            x         => slv_reg4(13 downto 0),
            y         => slv_reg5(12 downto 0),
            psi       => slv_reg6(8 downto 0),
            v         => slv_reg7(8 downto 0),
            ref_x     => slv_reg8(13 downto 0),
            ref_y     => slv_reg9(11 downto 0),
            accel_cmd => mpc_accel,
            steer_cmd => mpc_steer
        );

    -- AXI write handshake
    process (S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then 
            if S_AXI_ARESETN = '0' then
                axi_awready <= '0';
                axi_wready  <= '0';
                axi_bvalid  <= '0';
                aw_en <= '1';
            else
                if (axi_awready = '0' and S_AXI_AWVALID = '1' and S_AXI_WVALID = '1' and aw_en = '1') then
                    axi_awready <= '1';
                    aw_en <= '0';
                    axi_awaddr <= S_AXI_AWADDR;
                elsif (S_AXI_BREADY = '1' and axi_bvalid = '1') then
                    aw_en <= '1';
                    axi_awready <= '0';
                else
                    axi_awready <= '0';
                end if;
                if (axi_wready = '0' and S_AXI_WVALID = '1' and S_AXI_AWVALID = '1' and aw_en = '1') then
                    axi_wready <= '1';
                else
                    axi_wready <= '0';
                end if;
                if (axi_awready = '1' and S_AXI_AWVALID = '1' and axi_wready = '1' and S_AXI_WVALID = '1' and axi_bvalid = '0') then
                    axi_bvalid <= '1';
                    axi_bresp  <= "00"; 
                elsif (S_AXI_BREADY = '1' and axi_bvalid = '1') then
                    axi_bvalid <= '0';
                end if;
            end if;
        end if;
    end process;

    -- Register write + kick FSM
    slv_reg_wren <= axi_wready and S_AXI_WVALID and axi_awready and S_AXI_AWVALID;
    process (S_AXI_ACLK)
        variable loc_addr : std_logic_vector(4 downto 0);
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                state <= S_IDLE;
                slv_reg0 <= (others => '0');
                slv_reg4 <= (others => '0');
                slv_reg5 <= (others => '0');
                slv_reg6 <= (others => '0');
                slv_reg7 <= (others => '0');
                slv_reg8 <= (others => '0');
                slv_reg9 <= (others => '0');
                slv_reg16 <= (others => '0');
            else
                loc_addr := axi_awaddr(6 downto 2);
                slv_reg0(0) <= '0';

                if (slv_reg_wren = '1') then
                    case loc_addr is
                        when "00000" => slv_reg0 <= S_AXI_WDATA;
                        when "00100" => slv_reg4 <= S_AXI_WDATA;
                        when "00101" => slv_reg5 <= S_AXI_WDATA;
                        when "00110" => slv_reg6 <= S_AXI_WDATA;
                        when "00111" => slv_reg7 <= S_AXI_WDATA;
                        when "01000" => slv_reg8 <= S_AXI_WDATA;
                        when "01001" => slv_reg9 <= S_AXI_WDATA;
                        when others  => null;
                    end case;
                end if;

                case state is
                    when S_IDLE =>
                        slv_reg1(1) <= '0';
                        if (slv_reg_wren = '1' and loc_addr = "00000" and S_AXI_WDATA(0) = '1') then
                            state <= S_RUN;
                            wait_cnt <= (others => '0');
                            slv_reg1(0) <= '0';
                            slv_reg1(1) <= '1';
                        end if;
                    when S_RUN =>
                        wait_cnt <= wait_cnt + 1;
                        if wait_cnt = 4 then state <= S_CAPTURE; end if;
                    when S_CAPTURE =>
                        slv_reg12(31 downto 6) <= (others => mpc_accel(5));
                        slv_reg12(5 downto 0)  <= mpc_accel;
                        slv_reg13(31 downto 6) <= (others => mpc_steer(5));
                        slv_reg13(5 downto 0)  <= mpc_steer;
                        slv_reg1(0) <= '1';
                        slv_reg16 <= std_logic_vector(unsigned(slv_reg16) + 1);
                        state <= S_IDLE;
                end case;
            end if;
        end if;
    end process;

    -- AXI read handshake + readback mux
    process (S_AXI_ACLK)
    begin
        if rising_edge(S_AXI_ACLK) then
            if S_AXI_ARESETN = '0' then
                axi_arready <= '0';
                axi_rvalid  <= '0';
                axi_rresp   <= "00";
            else
                if (axi_arready = '0' and S_AXI_ARVALID = '1') then
                    axi_arready <= '1';
                    axi_araddr  <= S_AXI_ARADDR;
                else
                    axi_arready <= '0';
                end if;

                if (axi_arready = '1' and S_AXI_ARVALID = '1' and axi_rvalid = '0') then
                    axi_rvalid <= '1';
                    axi_rresp  <= "00";
                    case axi_araddr(6 downto 2) is
                        when "00000" => axi_rdata <= slv_reg0;
                        when "00001" => axi_rdata <= slv_reg1;
                        when "00100" => axi_rdata <= slv_reg4;
                        when "00101" => axi_rdata <= slv_reg5;
                        when "00110" => axi_rdata <= slv_reg6;
                        when "00111" => axi_rdata <= slv_reg7;
                        when "01000" => axi_rdata <= slv_reg8;
                        when "01001" => axi_rdata <= slv_reg9;
                        when "01100" => axi_rdata <= slv_reg12;
                        when "01101" => axi_rdata <= slv_reg13;
                        when "10000" => axi_rdata <= slv_reg16;
                        when others  => axi_rdata <= (others => '0');
                    end case;
                elsif (axi_rvalid = '1' and S_AXI_RREADY = '1') then
                    axi_rvalid <= '0';
                end if;
            end if;
        end if;
    end process;

    mpc_irq <= slv_reg1(0) and slv_reg0(2);

end arch_imp;
