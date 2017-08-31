LIBRARY IEEE; 
USE IEEE.std_logic_1164.all;
USE IEEE.std_logic_unsigned.all;
USE work.Glob_dcls.all;

entity control is 
   port(
        clk   	    : IN STD_LOGIC; 
        reset_N	    : IN STD_LOGIC; 
        
        opcode      : IN work.Glob_dcls.opcode;     -- declare type for the 6 most significant bits of IR
        funct       : IN work.Glob_dcls.opcode;     -- declare type for the 6 least significant bits of IR 
     	zero        : IN STD_LOGIC;
        
     	PCUpdate    : OUT STD_LOGIC; -- this signal controls whether PC is updated or not
     	IorD        : OUT STD_LOGIC;
     	MemRead     : OUT STD_LOGIC;
     	MemWrite    : OUT STD_LOGIC;

     	IRWrite     : OUT STD_LOGIC;
     	MemtoReg    : OUT STD_LOGIC_VECTOR (1 downto 0); -- the extra bit is for JAL
     	RegDst      : OUT STD_LOGIC_VECTOR (1 downto 0); -- the extra bit is for JAL
     	RegWrite    : OUT STD_LOGIC;
     	ALUSrcA     : OUT STD_LOGIC;
     	ALUSrcB     : OUT STD_LOGIC_VECTOR (1 downto 0);
     	ALUcontrol  : OUT ALU_opcode;
     	PCSource    : OUT STD_LOGIC_VECTOR (1 downto 0)
	);
end control;

architecture control_arch of control is
-- enumerated type declarations
type cpu_state is
	(fetch, decode, branch, jump, addresscomplw, addresscompsw, rexec, iexec,
	 regstorei,regstorer, mdrload, memstore, memloadcomplete, idle, init);
-- component declaration
	
-- component specification

-- signal declaration
signal Next_IorD, Next_MemWrite, Next_MemRead,Next_IRWrite,Next_RegWrite,Next_ALUSrcA,Next_PCUpdate : std_logic;
signal Next_MemtoReg,Next_RegDst,Next_ALUSrcB,Next_PCSource : std_logic_vector(1 downto 0);
signal Next_ALUcontrol : ALU_opcode;
signal PCUpdateReg :std_logic;
signal cpu_current_state, cpu_next_state : cpu_state;
begin


PCUpdate <=  -- PCUpdate signal depends on zero flag, which will update mid-cycle, thus pcupdate needs to be driven by additional logic
	'1' when (cpu_current_state = branch and zero='1' and opcode = "000100") else --branch equal
	'1' when (cpu_current_state = branch and zero='0' and opcode = "000101") else --branch not equal
	PCUpdateReg;

-- evaluate next state based on current state and inputs
-- will be registered 1 ns after clock rising edge


-- change signals based on state
cpu_next_state <= 
	fetch when cpu_current_state = init else
	decode when cpu_current_state = fetch else
	branch when cpu_current_state = decode and ( opcode = "000100" or opcode = "000101") else
	jump when cpu_current_state = decode and ( opcode = "000010" ) else
	addresscomplw when cpu_current_state = decode and ( opcode = "100011" ) else
	addresscompsw when cpu_current_state = decode and ( opcode = "101011" ) else
	rexec when cpu_current_state = decode and ( opcode = "000000" ) else
	iexec when cpu_current_state = decode and ( opcode = "001000" or opcode = "001100" or opcode = "001101" ) else
	fetch when cpu_current_state = branch else
	fetch when cpu_current_state = jump else
	mdrload when cpu_current_state = addresscomplw else
	memstore when cpu_current_state = addresscompsw else
	regstorer when cpu_current_state = rexec else
	regstorei when cpu_current_state = iexec else
	memloadcomplete when cpu_current_state = mdrload else
	fetch when cpu_current_state = regstorer else
	fetch when cpu_current_state = regstorei else
	fetch when cpu_current_state = memloadcomplete else
	fetch when cpu_current_state = memstore else
	idle; -- go idle if the instruction was invalid

PROCESS(cpu_next_state,opcode,funct)
begin 
case cpu_next_state is
when fetch =>
	-- Instruction Fetch
	-- IR = Memory[PC]
	-- PC +=4
	Next_PCUpdate <= '1';-- write to PC register this cycle, the value stored in ALUReg

	Next_IorD<= '0'; -- mem address is PC
	Next_MemRead <= '1'; -- read a value from mem
	Next_MemWrite <= '0';

	Next_IRWrite <= '1'; --enable write to IR
	Next_MemtoReg <= "00";
	Next_RegDst <= "00";
	Next_RegWrite<='0';
	Next_ALUSrcA <= '0'; -- ALU operand A = PC
	Next_ALUSrcB <= "11"; -- ALU operand B = 4;

	Next_ALUcontrol <= "000"; -- ALU operation ADD
	Next_PCSource <= "00"; -- PC from ALUOut
when decode =>
	-- instruction decode/ register fetch
	-- A = REG[IR25-21]
	-- B = REG[IR20-16]
	-- ALUOut = PC+signext(IR15-0 <<2)
	Next_PCUpdate<= '0'; 

	Next_IorD<= '0';
	Next_MemRead <= '0'; 
	Next_MemWrite <= '0';

	Next_IRWrite <= '0';
	Next_MemtoReg <= "00";
	Next_RegDst <= "01";
	Next_RegWrite<='0';
	Next_ALUSrcA <= '0'; -- ALU A = PC
	Next_ALUSrcB <= "10"; -- ALU B = immediate << 2

	Next_ALUcontrol <= "000"; -- ALU operation ADD
	Next_PCSource <= "00";


when branch =>
	-- branch evaluation
	-- if (A==B) then PC=ALUOut
	Next_PCUpdate<= '0'; 

	Next_IorD<= '0';
	Next_MemRead <= '0'; 
	Next_MemWrite <= '0';

	Next_IRWrite <= '0';
	Next_MemtoReg <= "00";
	Next_RegDst <= "00";
	Next_RegWrite<='0';
	Next_ALUSrcA <= '1';  -- ALU A = A
	Next_ALUSrcB <= "00"; -- ALU B = B

	Next_ALUcontrol <= "001"; -- ALU operation SUB
	Next_PCSource <= "01"; -- PC should be PC[31-28] || (IR[25-0] << 2)

when jump =>
	-- instruction decode/ register fetch
	-- ALUOut = PC+signext(IR15-0 <<2)
	Next_PCUpdate<= '1'; 

	Next_IorD<= '0';
	Next_MemRead <= '0'; 
	Next_MemWrite <= '0';

	Next_IRWrite <= '0';
	Next_MemtoReg <= "00";
	Next_RegDst <= "00";
	Next_RegWrite<='0';
	Next_ALUSrcA <= '0'; -- ALU A = PC
	Next_ALUSrcB <= "10"; -- ALU B = immediate << 2

	Next_ALUcontrol <= "000"; -- ALU operation ADD
	Next_PCSource <= "10";

when addresscomplw =>
	-- address comp for mem-reference instruction
	-- ALUOut = A+ signext(immediate)
	Next_PCUpdate<= '0'; 

	Next_IorD<= '1';
	Next_MemRead <= '0'; 
	Next_MemWrite <= '0';

	Next_IRWrite <= '0';
	Next_MemtoReg <= "00";
	Next_RegDst <= "00";
	Next_RegWrite<='0';
	Next_ALUSrcA <= '1';  -- ALU A = A
	Next_ALUSrcB <= "01"; -- ALU B = immediate

	Next_ALUcontrol <= "000"; -- ALU operation ADD
	Next_PCSource <= "00";

when addresscompsw =>
	-- address comp for mem-reference instruction
	-- ALUOut = A+ signext(immediate)
	Next_PCUpdate<= '0'; 

	Next_IorD<= '1';
	Next_MemRead <= '0'; 
	Next_MemWrite <= '0';

	Next_IRWrite <= '0';
	Next_MemtoReg <= "00";
	Next_RegDst <= "00";
	Next_RegWrite<='0';
	Next_ALUSrcA <= '1';  -- ALU A = A
	Next_ALUSrcB <= "01"; -- ALU B = immediate

	Next_ALUcontrol <= "000"; -- ALU operation ADD
	Next_PCSource <= "00";

when rexec =>
	-- ALU immediate op 
	-- ALUOut = A op B
	Next_PCUpdate<= '0'; 

	Next_IorD<= '0';
	Next_MemRead <= '0'; 
	Next_MemWrite <= '0';

	Next_IRWrite <= '0';
	Next_MemtoReg <= "00";
	Next_RegDst <= "00";
	Next_RegWrite<='0';
	Next_ALUSrcA <= '1';  -- ALU A = A
	Next_ALUSrcB <= "00"; -- ALU B = B

	case funct is
	when "100000" =>
		Next_ALUcontrol <= "000";-- ALU operation ADD
	when  "100010"=>
		Next_ALUcontrol <= "001";-- ALU operation SUB
	when  "000000"=>
		Next_ALUcontrol <= "010";-- ALU operation SLL
	when  "000010"=>
		Next_ALUcontrol <= "011";-- ALU operation SRL
	when  "100100"=>
		Next_ALUcontrol <= "100";-- ALU operation AND
	when  "100101"=>
		Next_ALUcontrol <= "101";-- ALU operation OR
	when  others =>
		Next_ALUcontrol <= "101";
	end case;
	Next_PCSource <= "00";

when iexec =>
	-- ALU immediate op 
	-- ALUOut = A op imm
	Next_PCUpdate<= '0'; 

	Next_IorD<= '0';
	Next_MemRead <= '0'; 
	Next_MemWrite <= '0';

	Next_IRWrite <= '0';
	Next_MemtoReg <= "00";
	Next_RegDst <= "00";
	Next_RegWrite<='0';
	Next_ALUSrcA <= '1';  -- ALU A = A
	Next_ALUSrcB <= "01"; -- ALU B = immediate

	case opcode is
	when "001000" =>
		Next_ALUcontrol <= "000"; -- ALU operation ADD
	when "001100" =>
		Next_ALUcontrol <= "100"; -- ALU operation AND
	when "001101" =>
		Next_ALUcontrol <= "101"; -- ALU operation OR
	when others =>
		Next_ALUcontrol <= "101";
	end case;

	Next_PCSource <= "00";

when regstorer =>
	-- write result into RF
	-- Reg[IR[15-11]] = ALUOut
	Next_PCUpdate<= '0'; 

	Next_IorD<= '0';
	Next_MemRead <= '0';
	Next_MemWrite <= '0';

	Next_IRWrite <= '0';
	Next_MemtoReg <= "00"; -- d_in = ALUOut
	Next_RegDst <= "01"; -- address = IR[15-11] - r format destination reg
	Next_RegWrite<='1'; --enable write
	Next_ALUSrcA <= '1'; 
	Next_ALUSrcB <= "10"; 

	Next_ALUcontrol <= "000";
	Next_PCSource <= "01";
when regstorei =>
	-- write result into RF
	-- Reg[IR[15-11]] = ALUOut
	Next_PCUpdate<= '0'; 

	Next_IorD<= '0';
	Next_MemRead <= '0';
	Next_MemWrite <= '0';

	Next_IRWrite <= '0';
	Next_MemtoReg <= "00"; -- d_in = ALUOut
	Next_RegDst <= "00"; -- address = IR[20-16] - i format destination reg
	Next_RegWrite<='1'; --enable write
	Next_ALUSrcA <= '1'; 
	Next_ALUSrcB <= "10"; 

	Next_ALUcontrol <= "000";
	Next_PCSource <= "01";

when mdrload =>
	-- mem access completion for load
	Next_PCUpdate<= '0'; 

	Next_IorD<= '1'; --load mem address from ALUOut
	Next_MemRead <= '1';  --begin a read into MDR
	Next_MemWrite <= '0';

	Next_IRWrite <= '0';
	Next_MemtoReg <= "00";
	Next_RegDst <= "00";
	Next_RegWrite<='0';
	Next_ALUSrcA <= '1';
	Next_ALUSrcB <= "10";

	Next_ALUcontrol <= "000"; -- ALU operation ADD
	Next_PCSource <= "00";

when memstore =>
	-- mem access completion for sw
	Next_PCUpdate<= '0'; 

	Next_IorD<= '1'; --mem address from ALUOut
	Next_MemRead <= '0';  
	Next_MemWrite <= '1'; --begin a write into MDR

	Next_IRWrite <= '0';
	Next_MemtoReg <= "00";
	Next_RegDst <= "00";
	Next_RegWrite<='0';
	Next_ALUSrcA <= '1';
	Next_ALUSrcB <= "10";

	Next_ALUcontrol <= "000"; 
	Next_PCSource <= "00";


when memloadcomplete =>
	-- mem read completion load mdr into register file
	Next_PCUpdate<= '0'; 

	Next_IorD<= '0';
	Next_MemRead <= '0';
	Next_MemWrite <= '0';

	Next_IRWrite <= '0';
	Next_MemtoReg <= "01"; --d_in source is MDR
	Next_RegDst <= "00"; --load into address [20-16] into register file
	Next_RegWrite<='1'; --enable write
	Next_ALUSrcA <= '1'; 
	Next_ALUSrcB <= "10"; 

	Next_ALUcontrol <= "000";
	Next_PCSource <= "01";


when others =>

end case;
end process;

PROCESS(clk, reset_N) -- registers implemented as D flipflops
begin
if (reset_N='0') then
	PCUpdateReg<='0';
	IorD<='0';
	MemWrite<='0';
	MemRead<='0';
	IRWrite<='0';
	RegWrite<='0';
	ALUSrcA<='0';
	MemtoReg<="00";
	RegDst<="00";
	ALUSrcB<="00";
	PCSource<="00";
	ALUControl<="000";
	cpu_current_state <= init; -- reset should start state machine at fetch state
	

elsif (clk'event and clk='1') then
	IorD <= Next_IorD;
	MemWrite<=Next_MemWrite;
	MemRead<=Next_MemRead;
	IRWrite<=Next_IRWrite;
	RegWrite<=Next_RegWrite;
	ALUSrcA<=Next_ALUSrcA;
	PCUpdateReg<=Next_PCUpdate;
	MemtoReg<=Next_MemtoReg;
	RegDst<=Next_RegDst;
	ALUSrcB<=Next_ALUSrcB;
	PCSource<=Next_PCSource;
	ALUControl<=Next_ALUControl;
	cpu_current_state <= cpu_next_state; -- set the next state control lines after rising edge passes

	
end if;
end PROCESS;

end control_arch;



