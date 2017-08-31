LIBRARY IEEE; 
USE IEEE.std_logic_1164.all;
USE IEEE.numeric_std.all;
USE IEEE.std_logic_unsigned.all;
USE work.Glob_dcls.all;

entity datapath is
  
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
    zero       : out std_logic);	-- send zero to controller (cond. branch)

end datapath;


architecture datapath_arch of datapath is
-- component declaration
COMPONENT ALU
PORT( op_code: in ALU_opcode;
	in0, in1: in word;
	C : in std_logic_vector(4 downto 0);
	ALUout : out word;
	Zero : out std_logic
);
END COMPONENT;

COMPONENT RegFile
PORT( clk, wr_en : in STD_LOGIC;
	rd_addr_1,rd_addr_2,wr_addr : in REG_addr;
	d_in	:in word;
	d_out_1,d_out_2	: out word
);
END COMPONENT;

COMPONENT Mem
   PORT (MemRead	: IN std_logic;
	 MemWrite	: IN std_logic;
	 d_in		: IN   word;		 
	 address	: IN   word;
	 d_out		: OUT  word 
	 );
END COMPONENT;
-- component specification

-- signal declaration

-- declare the register signals----------------------------------
signal RegPC,RegIR,RegMDR,RegALUOut,RegA,RegB : word;
-----------------------------------------------------------------

-- declare the mux signals---------------------------------------
signal MuxIRwr_en, MuxA, MuxB, MuxPC, MuxPCwr_en, MuxD_in,MuxMem : word;
signal MuxWr_addr: std_logic_vector(4 downto 0);
-----------------------------------------------------------------

-- declare misc. signals ----------------------------------------
signal d_out, d_out_1,d_out_2, ALUOut : word;
signal PCJump : word; -- the signal containing jump destination for PC
signal immval : word; -- the sign extended immediate value from instuction reg
signal immvalshifted : word; --sign extended and lshifted by 2 imm value
-----------------------------------------------------------------

begin

-- Map ports on subcomponents -----------------------------------
MyALU: ALU PORT MAP(
	op_code => ALUControl,
	in0 => MuxA,
	in1 => MuxB,
	C => RegIR(10 downto 6), 
--the shift amount is in the least 5 significant bits imm argument on the mux for arg B
	ALUout => ALUOut,
	Zero => zero
);
MyRegFile: RegFile PORT MAP(
	clk => clk,
	wr_en => RegWrite,
	rd_addr_1 => RegIR(25 downto 21),
	rd_addr_2 => RegIR(20 downto 16),
	wr_addr => MuxWr_addr,
	d_in => MuxD_in,
	d_out_1 => d_out_1,
	d_out_2 => d_out_2
);
MyMem: Mem PORT MAP(
	MemRead => MemRead,
	MemWrite => MemWrite,
	d_in => RegB,
	address => MuxMem,
	d_out => d_out
);
shamt_out <= RegIR(10 downto 6);
func_out <= RegIR(5 downto 0);
opcode_out <= RegIR(31 downto 26);
-----------------------------------------------------------------

-- mux logic ----------------------------------------------------
MuxWr_addr <= -- mux for selecting wr_addr source for register file
	RegIR(20 downto 16) when RegDst = "00" else
	RegIR(15 downto 11) when RegDst = "01" else
	"11111";

MuxD_in <= -- mux for selecting d_in source for register file
	RegALUOut when MemtoReg = "00" else
	RegMDR when MemtoReg = "01" else
	RegPC;

MuxMem <= -- mux for selecting address source for memory
	RegPC when IorD = '0' else
	RegALUOut;

MuxPC <= -- mux for selecting source for next PC value
	ALUOut when PCSource = "00" else
	REGAluOut when PCSource = "01" else
	PCJump;

MuxPCwr_en <= --mux for changing PC or keeping value (wr_en)
	RegPC when PCUpdate = '0' else
	MuxPC;

MuxIRwr_en <= --mux for changing IR or keeping value (wr_en)
	RegIR when IRWrite = '0' else
	d_out;

MuxA <= -- mux for ALU port A source
	RegPC when ALUSrcA = '0' else
	RegA;

MuxB <= -- mux for ALU port B source
	RegB when ALUSrcB = "00" else
	immval when ALUSrcB = "01" else
	immvalshifted when ALUSrcB = "10" else
	x"00000004"; -- constant 4 for pc incrementing
-----------------------------------------------------------------

-- left shifts, concatenations, and sign extentions for immediate value and PC jump
PCJump(31 downto 28) <=  RegPC(31 downto 28); -- fill upper 4 bits from PC upper 4
PCJump(27 downto 2) <= regIR(25 downto 0); -- shift left by 2
PCJump(1 downto 0) <= "00"; --fill bits with 0

immval(15 downto 0) <= RegIR(15 downto 0);
immval(31 downto 16) <=  --sign extention by filling new bits with the sign bit
	X"FFFF" when RegIR(15) = '1' else
	X"0000";
immvalshifted(31 downto 2) <= immval(29 downto 0); --shift left by 2
immvalshifted (1 downto 0) <= "00"; -- fill bits with 0
-----------------------------------------------------------------

PROCESS(clk, reset_N) -- registers implemented as D flipflops
begin
if (reset_N='0') then -- active low
	RegPC <= x"00000000";
	RegIR <= x"00000000";
	RegMDR <= x"00000000";
	RegALUOut <= x"00000000";
	RegA <= x"00000000";
	RegB <= x"00000000";
elsif (clk'event and clk='1') then
	RegPC <= MuxPCwr_en;
	RegIR <= MuxIRwr_en; --d_out from mem
	RegMDR <= d_out; --d_out from mem
	RegALUOut <= ALUOut;
	RegA <=	d_out_1; --d_out_1 from register file
	RegB <= d_out_2; --d_out_2 from register file

end if;
end PROCESS;
end datapath_arch;
