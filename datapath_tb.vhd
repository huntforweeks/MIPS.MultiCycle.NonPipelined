LIBRARY IEEE; 
USE IEEE.std_logic_1164.all;
USE IEEE.std_logic_unsigned.all;
use work.Glob_dcls.all;

entity datapath_tb is
end datapath_tb;

architecture datapath_tb_arch of datapath_tb is
-- component declaration
COMPONENT datapath
 port (
    clk        : in  std_logic;
    reset_N    : in  std_logic;
    
    PCUpdate   : in  std_logic;         -- write_enable of PC

    IorD       : in  std_logic;         -- Address selection for memory (PC vs. store address)
    MemRead    : in  std_logic;		-- read_enable for memory
    MemWrite   : in  std_logic;		-- write_enable for memory

    IRWrite    : in  std_logic;         -- write_enable for Instruction Register
    MemtoReg   : in  std_logic_vector(1 downto 0);  -- selects ALU or MEMORY or PC to write to register file.
    RegDst     : in  std_logic_vector(1 downto 0);  -- selects rt, rd, or "31" as destination of operation
    RegWrite   : in  std_logic;         -- Register File write-enable
    ALUSrcA    : in  std_logic;         -- selects source of A port of ALU
    ALUSrcB    : in  std_logic_vector(1 downto 0);  -- selects source of B port of ALU
    
    ALUControl : in  ALU_opcode;	-- receives ALU opcode from the controller
    PCSource   : in  std_logic_vector(1 downto 0);  -- selects source of PC

    opcode_out : out opcode;		-- send opcode to controller
    func_out   : out std_logic_vector(5 downto 0);  -- send func field to controller
    shamt_out  : out std_logic_vector(4 downto 0);  -- send shamt field to controller
    zero       : out std_logic);	-- send zero to controller (cond. branch);
end COMPONENT;
-- component specification
	
-- signal declaration
SIGNAL clk		: std_logic := '1';
SIGNAL reset_N		: std_logic := '1';

SIGNAL PCUpdate   :  std_logic := '0';

SIGNAL IorD       :  std_logic := '0';    
SIGNAL MemRead    :  std_logic := '0';		
SIGNAL MemWrite   :  std_logic := '0';		

SIGNAL IRWrite    :  std_logic := '0';      
SIGNAL MemtoReg   :  std_logic_vector(1 downto 0) :="00";  
SIGNAL RegDst     :  std_logic_vector(1 downto 0) :="00"; 
SIGNAL RegWrite   :  std_logic := '0';        
SIGNAL ALUSrcA    :  std_logic := '0';      
SIGNAL ALUSrcB    :  std_logic_vector(1 downto 0) :="00";
    
SIGNAL ALUControl :  ALU_opcode :="000";	
SIGNAL PCSource   :  std_logic_vector(1 downto 0) :="00";  

SIGNAL opcode_out :  opcode;		
SIGNAL func_out   :  std_logic_vector(5 downto 0);
SIGNAL shamt_out  :  std_logic_vector(4 downto 0); 
SIGNAL zero       :  std_logic := '0';

begin

clk <= not clk after 20 ns; --clock cycle will be 40ns

UUT: datapath PORT MAP ( 
	clk=>clk,
	reset_N=>reset_N,
	PCUpdate=>PCUpdate,
	IorD=>IorD,
	MemRead=>MemRead,
	MemWrite=>MemWrite,
	IRWrite=>IRWrite,
	MemtoReg=>memtoReg,
	RegDst=>RegDst,
	RegWrite=>RegWrite,
	ALUSrcA=>ALUSrcA,
	ALUSrcB=>ALUSrcB,
	ALUControl=>ALUControl,
	PCSource=>PCSource,
	opcode_out=>opcode_out,
	func_out=>func_out,
	shamt_out=>shamt_out,
	zero=>zero
);

tb: PROCESS
BEGIN

	wait for 1 ns;
	reset_N <= '0';
----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
-----------------------------------------------------------------
	--& lw r1,80(r0) load mem at byte 80 into r1
	-- 5 step instruction
	-- 40ns to 240ns
-----------------------------------------------------------------
	-- Instruction Fetch
	-- IR = Memory[PC]
	-- PC +=4
	reset_N <= '0';
	PCUpdate <= '1';-- write to PC register this cycle, the value stored in ALUReg

	IorD<= '0'; -- mem address is PC
	MemRead <= '1'; -- read a value from mem
	MemWrite <= '0';

	IRWRite <= '1'; --enable write to IR
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '0'; -- ALU operand A = PC
	ALUSrcB <= "11"; -- ALU operand B = 4;

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00"; -- PC from ALUOut
	wait for 40 ns;

-----------------------------------------------------------------
	-- instruction decode/ register fetch
	-- A = REG[IR25-21]
	-- B = REG[IR20-16]
	-- ALUOut = PC+signext(IR15-0 <<2)
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '0';
	MemRead <= '0'; 
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "01";
	RegWrite<='0';
	ALUSrcA <= '0'; -- ALU A = PC
	ALUSrcB <= "10"; -- ALU B = immediate << 2

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00";
	wait for 40 ns;
-----------------------------------------------------------------
	-- address comp for mem-reference instruction
	-- ALUOut = A+ signext(immediate)
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '0';
	MemRead <= '0'; 
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '1';  -- ALU A = A
	ALUSrcB <= "01"; -- ALU B = immediate

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00";
	wait for 40 ns;
-----------------------------------------------------------------
	-- mem access completion for load
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '1'; --load mem address from ALUOut
	MemRead <= '1';  --begin a read into MDR
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '1';
	ALUSrcB <= "10";

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00";
	wait for 40 ns;
-----------------------------------------------------------------
	-- mem read completion load mdr into register file
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '0';
	MemRead <= '0';
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "01"; --d_in source is MDR
	RegDst <= "00"; --load into address [20-16] into register file
	RegWrite<='1'; --enable write
	ALUSrcA <= '1'; 
	ALUSrcB <= "10"; 

	ALUControl <= "000";
	PCSource <= "01";
	wait for 40 ns;
-----------------------------------------------------------------
	-- lw finished

----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------

-----------------------------------------------------------------
	--& addi r2,r1,5  r2 = r1+5
	-- 4 step instruction
	-- 240ns to 400ns
-----------------------------------------------------------------
	-- Instruction Fetch
	-- IR = Memory[PC]
	-- PC +=4
	reset_N <= '0';
	PCUpdate <= '1';-- write to PC register this cycle, the value stored in ALUReg

	IorD<= '0'; -- mem address is PC
	MemRead <= '1'; -- read a value from mem
	MemWrite <= '0';

	IRWRite <= '1'; --enable write to IR
	MemtoReg <= "00";
	RegDst <= "01"; -- reg writeAddr = rt for i-type instructions
	RegWrite<='0';
	ALUSrcA <= '0'; -- ALU operand A = PC
	ALUSrcB <= "11"; -- ALU operand B = 4;

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00"; -- PC from ALUOut
	wait for 40 ns;
-----------------------------------------------------------------
	-- instruction decode/ register fetch
	-- A = REG[IR25-21]
	-- B = REG[IR20-16]
	-- ALUOut = PC+signext(IR15-0 <<2)
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '0';
	MemRead <= '0'; 
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "01";
	RegWrite<='0';
	ALUSrcA <= '0'; -- ALU A = PC
	ALUSrcB <= "10"; -- ALU B = immediate << 2

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00";
	wait for 40 ns;
-----------------------------------------------------------------
	-- ALU op 
	-- ALUOut = A + imm
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '0';
	MemRead <= '0'; 
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '1';  -- ALU A = A
	ALUSrcB <= "01"; -- ALU B = immediate

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00";
	wait for 40 ns;
-----------------------------------------------------------------
	-- write result into RF
	-- Reg[IR[15-11]] = ALUOut
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '0';
	MemRead <= '0';
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00"; -- d_in = ALUOut
	RegDst <= "00"; -- address = IR[20-16] - i format destination reg
	RegWrite<='1'; --enable write
	ALUSrcA <= '1'; 
	ALUSrcB <= "10"; 

	ALUControl <= "000";
	PCSource <= "01";
	wait for 40 ns;
-----------------------------------------------------------------
	-- addi finished
----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
-----------------------------------------------------------------
	--& sw r2,84(r0) store r2 into mem at byte 84
	-- 4 step instruction
	-- 400ns to 560ns
-----------------------------------------------------------------
	-- Instruction Fetch
	-- IR = Memory[PC]
	-- PC +=4
	reset_N <= '0';
	PCUpdate <= '1';-- write to PC register this cycle, the value stored in ALUReg

	IorD<= '0'; -- mem address is PC
	MemRead <= '1'; -- read a value from mem
	MemWrite <= '0';

	IRWRite <= '1'; --enable write to IR
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '0'; -- ALU operand A = PC
	ALUSrcB <= "11"; -- ALU operand B = 4;

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00"; -- PC from ALUOut
	wait for 40 ns;

-----------------------------------------------------------------
	-- instruction decode/ register fetch
	-- A = REG[IR25-21]
	-- B = REG[IR20-16]
	-- ALUOut = PC+signext(IR15-0 <<2)
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '0';
	MemRead <= '0'; 
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "01";
	RegWrite<='0';
	ALUSrcA <= '0'; -- ALU A = PC
	ALUSrcB <= "10"; -- ALU B = immediate << 2

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00";
	wait for 40 ns;
-----------------------------------------------------------------
	-- address comp for mem-reference instruction
	-- ALUOut = A+ signext(immediate)
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '0';
	MemRead <= '0'; 
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '1';  -- ALU A = A
	ALUSrcB <= "01"; -- ALU B = immediate

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00";
	wait for 40 ns;
-----------------------------------------------------------------
	-- mem access completion for save
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '1'; --mem address from ALUOut
	MemRead <= '0';  
	MemWrite <= '1'; --begin a write into MDR

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '1';
	ALUSrcB <= "10";

	ALUControl <= "000"; 
	PCSource <= "00";
	wait for 40 ns;
-----------------------------------------------------------------
	-- sw finished

----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
-----------------------------------------------------------------
	--& bne r1,r2,8 branch to PC + 8 if r2 != r1
	-- 3 step instruction
	-- 560ns to 680ns
-----------------------------------------------------------------
	-- Instruction Fetch
	-- IR = Memory[PC]
	-- PC +=4
	reset_N <= '0';
	PCUpdate <= '1';-- write to PC register this cycle, the value stored in ALUReg

	IorD<= '0'; -- mem address is PC
	MemRead <= '1'; -- read a value from mem
	MemWrite <= '0';

	IRWRite <= '1'; --enable write to IR
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '0'; -- ALU operand A = PC
	ALUSrcB <= "11"; -- ALU operand B = 4;

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00"; -- PC from ALUOut
	wait for 40 ns;

-----------------------------------------------------------------
	-- instruction decode/ register fetch
	-- A = REG[IR25-21]
	-- B = REG[IR20-16]
	-- ALUOut = PC+signext(IR15-0 <<2)
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '0';
	MemRead <= '0'; 
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "01";
	RegWrite<='0';
	ALUSrcA <= '0'; -- ALU A = PC
	ALUSrcB <= "10"; -- ALU B = immediate << 2

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00";
	wait for 40 ns;
-----------------------------------------------------------------
	-- branch completion
	-- if (A==B) then PC=ALUOut
	reset_N <= '0';

	PCUpdate<= '1'; 

	IorD<= '0';
	MemRead <= '0'; 
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '1';  -- ALU A = A
	ALUSrcB <= "00"; -- ALU B = immediate

	ALUControl <= "001"; -- ALU operation SUB
	PCSource <= "01"; -- PC should be PC+signext(IR15-0 <<2)
	wait for 40 ns;
	assert Zero ='0' report "branch failed ('zero' output is wrong on BNE test)" severity Error;
-----------------------------------------------------------------
	-- bne finished

----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
-----------------------------------------------------------------
	--& lw r1,84(r0) load mem at byte 84 into r1
	-- 5 step instruction
	-- 680ns to 880ns
-----------------------------------------------------------------
	-- Instruction Fetch
	-- IR = Memory[PC]
	-- PC +=4
	reset_N <= '0';
	PCUpdate <= '1';-- write to PC register this cycle, the value stored in ALUReg

	IorD<= '0'; -- mem address is PC
	MemRead <= '1'; -- read a value from mem
	MemWrite <= '0';

	IRWRite <= '1'; --enable write to IR
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '0'; -- ALU operand A = PC
	ALUSrcB <= "11"; -- ALU operand B = 4;

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00"; -- PC from ALUOut
	wait for 40 ns;

-----------------------------------------------------------------
	-- instruction decode/ register fetch
	-- A = REG[IR25-21]
	-- B = REG[IR20-16]
	-- ALUOut = PC+signext(IR15-0 <<2)
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '0';
	MemRead <= '0'; 
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "01";
	RegWrite<='0';
	ALUSrcA <= '0'; -- ALU A = PC
	ALUSrcB <= "10"; -- ALU B = immediate << 2

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00";
	wait for 40 ns;
-----------------------------------------------------------------
	-- address comp for mem-reference instruction
	-- ALUOut = A+ signext(immediate)
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '0';
	MemRead <= '0'; 
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '1';  -- ALU A = A
	ALUSrcB <= "01"; -- ALU B = immediate

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00";
	wait for 40 ns;
-----------------------------------------------------------------
	-- mem access completion for load
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '1'; --load mem address from ALUOut
	MemRead <= '1';  --begin a read into MDR
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '1';
	ALUSrcB <= "10";

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00";
	wait for 40 ns;
-----------------------------------------------------------------
	-- mem read completion load mdr into register file
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '0';
	MemRead <= '0';
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "01"; --d_in source is MDR
	RegDst <= "00"; --load into address [20-16] into register file
	RegWrite<='1'; --enable write
	ALUSrcA <= '1'; 
	ALUSrcB <= "10"; 

	ALUControl <= "000";
	PCSource <= "01";
	wait for 40 ns;
-----------------------------------------------------------------
	-- lw finished

----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
	--& beq r1,r2,-24 branch to PC-24 if r1 = r2
	-- 3 step instruction
	-- 880ns to 1000ns
-----------------------------------------------------------------
	-- Instruction Fetch
	-- IR = Memory[PC]
	-- PC +=4
	reset_N <= '0';
	PCUpdate <= '1';-- write to PC register this cycle, the value stored in ALUReg

	IorD<= '0'; -- mem address is PC
	MemRead <= '1'; -- read a value from mem
	MemWrite <= '0';

	IRWRite <= '1'; --enable write to IR
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '0'; -- ALU operand A = PC
	ALUSrcB <= "11"; -- ALU operand B = 4;

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00"; -- PC from ALUOut
	wait for 40 ns;

-----------------------------------------------------------------
	-- instruction decode/ register fetch
	-- A = REG[IR25-21]
	-- B = REG[IR20-16]
	-- ALUOut = PC+signext(IR15-0 <<2)
	reset_N <= '0';

	PCUpdate<= '0'; 

	IorD<= '0';
	MemRead <= '0'; 
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "01";
	RegWrite<='0';
	ALUSrcA <= '0'; -- ALU A = PC
	ALUSrcB <= "10"; -- ALU B = immediate

	ALUControl <= "000"; -- ALU operation ADD
	PCSource <= "00";
	wait for 40 ns;
-----------------------------------------------------------------
	-- branch completion
	-- if (A==B) then PC=ALUOut
	reset_N <= '0';

	PCUpdate<= '1'; 

	IorD<= '0';
	MemRead <= '0'; 
	MemWrite <= '0';

	IRWRite <= '0';
	MemtoReg <= "00";
	RegDst <= "00";
	RegWrite<='0';
	ALUSrcA <= '1';  -- ALU A = A
	ALUSrcB <= "00"; -- ALU B = B

	ALUControl <= "001"; -- ALU operation SUB
	PCSource <= "01"; -- PC should be PC+signext(IR15-0 <<2)
	wait for 40 ns;
	assert Zero ='1' report "branch failed ('zero' output is wrong on BEQ test)" severity Error;
-----------------------------------------------------------------
	-- beq finished

----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------
----------------------------------------------------------------------------------------------------------------------------------

	WAIT;
END PROCESS;
	
end datapath_tb_arch;


