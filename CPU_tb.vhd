LIBRARY IEEE; 
USE IEEE.std_logic_1164.all;
USE IEEE.std_logic_unsigned.all;
USE work.Glob_dcls.all;

entity CPU_tb is
end CPU_tb;

architecture CPU_test of CPU_tb is
-- component declaration
	-- CPU (you just built)
COMPONENT CPU
port (
	clk	:in std_logic;
	reset_N	:in std_logic);
end COMPONENT;
-- component specification
-- signal declaration
	-- You'll need clock and reset.

signal clk: std_logic := '1';
signal reset_n : std_logic := '0';

begin
myCPU: CPU PORT MAP( clk, reset_N);

clk <= not clk after 20 ns; -- clock period = 40 ns;

tb: process
begin
	wait for 40 ns;
	reset_N<='1';
	WAIT;
end process;

end CPU_test;
