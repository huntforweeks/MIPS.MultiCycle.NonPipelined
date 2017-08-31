# MIPS.MultiCycle.NonPipelined
A synthesize-able VHDL implementation of a multi-cycle non-pipelined MIPS RISC processor. Created for a computer design project course.

Implements 14 instructions with 3 instruction formats.


I-format instructions:


  Immediate ALU ops (4 cycles):
    addi
    andi
    ori
    
    
  Branch instructions (3 cycles):
    beq
    bne
    
    
  Memory instructions (sw 4 cycles, lw 5 cycles):
    sw
    lw
    
    
R-format instructions:


  ALU ops (4 cycles):
    add
    sub
    and
    or
    sll
    srl
