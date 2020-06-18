component PLL_DEV_12M is
    port(
        ref_clk_i: in std_logic;
        rst_n_i: in std_logic;
        bypass_i: in std_logic;
        lock_o: out std_logic;
        outcore_o: out std_logic;
        outglobal_o: out std_logic
    );
end component;

__: PLL_DEV_12M port map(
    ref_clk_i=>,
    rst_n_i=>,
    bypass_i=>,
    lock_o=>,
    outcore_o=>,
    outglobal_o=>
);
