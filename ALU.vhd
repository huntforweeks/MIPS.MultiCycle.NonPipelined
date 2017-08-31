LIBRARY IEEE; 
USE IEEE.std_logic_1164.all;
USE IEEE.std_logic_unsigned.all;
use work.Glob_dcls.all;

entity ALU is 
  PORT( op_code  : in ALU_opcode;
        in0, in1 : in word;	
        C	 : in std_logic_vector(4 downto 0);  -- shift amount	
        ALUout   : out word;
        Zero     : out std_logic
  );
end ALU;

architecture ALU_arch of ALU is
-- signal declaration
SIGNAL adderArg2: word;
SIGNAL  adderResult : word;
SIGNAL outSig : word;

begin
adderArg2 <= in1 when op_code(0)='0' else ((not in1)+x"00000001");
adderResult <= in0 + adderArg2;
Zero <= '1' when outSig=0 else '0';
ALUout <= outSig;
Process (op_code,in0,in1,C,adderResult)
-- declarations
Begin
--statements
case op_code is
	when "000" => outSig <= adderResult;
	when "001" => outSig <= adderResult;
	when "010" => outSig <= SHL(in1, C);
	when "011" => outSig <= SHR(in1, C);
	when "100" => outSig <= in0 and in1;
	when "101" => outSig <= in0 or in1;
	when "110" => outSig <= in0 xor in1;
	when others => outSig <= in0 nor in1;
end case;

End Process;



end ALU_arch;
