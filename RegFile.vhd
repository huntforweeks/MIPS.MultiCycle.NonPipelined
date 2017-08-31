library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.numeric_std.all;
use work.Glob_dcls.all;

entity RegFile is 
  port(
        clk, wr_en                    : in STD_LOGIC;
        rd_addr_1, rd_addr_2, wr_addr : in REG_addr;
        d_in                          : in word; 
        d_out_1, d_out_2              : out word
  );
end RegFile;

architecture RF_arch of RegFile is
-- component declaration
type regArray is array (31 downto 0) of word;
-- signal declaration
SIGNAL reg: regArray;


begin
reg(0) <= x"00000000";
d_out_1 <= reg(TO_INTEGER(UNSIGNED(rd_addr_1))) when not (rd_addr_1 = "00000") else x"00000000";
d_out_2 <= reg(TO_INTEGER(UNSIGNED(rd_addr_2))) when not (rd_addr_2 = "00000") else x"00000000";

Process (clk)

begin
if (clk'event and clk='1') then
	if (not (wr_addr ="00000") and wr_en='1') then
		reg(TO_INTEGER(UNSIGNED(wr_addr))) <= d_in;
	end if;
end if;
end Process;

end RF_arch;
