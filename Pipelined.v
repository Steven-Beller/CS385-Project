
// Pipelined

module Reg_File(RR1,RR2,WR,WD,RegWrite,RD1,RD2,clock);
    input [1:0] RR1,RR2,WR;
    input [15:0] WD;
    input RegWrite,clock;
    output [15:0] RD1,RD2;
    reg [15:0] Regs[0:3];
    assign RD1 = Regs[RR1];
    assign RD2 = Regs[RR2];
    initial Regs[0] = 0;
    always @(negedge clock)
        if (RegWrite == 1 & WR != 0)
            Regs[WR] <= WD;
endmodule

module MUX_2x1(I0,I1,S0,Result);
    input I0,I1,S0;
    output Result;
    and(w1,I0,S0);
    and(w2,I1,~S0);
    or(Result,w1,w2);
endmodule
 
module MUX_4x1(I0,I1,I2,I3,S0,S1,Result);
    input I0,I1,I2,I3,S0,S1;
    output Result;
    and(w1,I0,~S1,~S0);
    and(w2,I1,~S1,S0);
    and(w3,I2,S1,~S0);
    and(w4,I3,S1,S0);
    or(Result,w1,w2,w3,w4);
endmodule
 
module MUX_8x1(I0,I1,I2,I3,I4,I5,I6,I7,S0,S1,S2,Result);
    input I0,I1,I2,I3,I4,I5,I6,I7,S0,S1,S2;
    output Result;
    MUX_4x1 m1(I0,I1,I2,I3,S0,S1,w1);
    MUX_4x1 m2(I4,I5,I6,I7,S0,S1,w2);
    MUX_2x1 m3(w2,w1,S2,Result);
endmodule
 
module MUX_2Bit_2x1(IO,I1,SO,Result);
    input [1:0] IO,I1;
    input SO;
    output [1:0] Result;
    MUX_2x1 b0(IO[0],I1[0],SO,Result[0]);
    MUX_2x1 b1(IO[1],I1[1],SO,Result[1]);
endmodule
 
module MUX_16Bit_2x1(IO,I1,SO,Result);
    input [15:0] IO,I1;
    input SO;
    output [15:0] Result;
    MUX_2x1 b0(IO[0],I1[0],SO,Result[0]);
    MUX_2x1 b1(IO[1],I1[1],SO,Result[1]);
    MUX_2x1 b2(IO[2],I1[2],SO,Result[2]);
    MUX_2x1 b3(IO[3],I1[3],SO,Result[3]);
    MUX_2x1 b4(IO[4],I1[4],SO,Result[4]);
    MUX_2x1 b5(IO[5],I1[5],SO,Result[5]);
    MUX_2x1 b6(IO[6],I1[6],SO,Result[6]);
    MUX_2x1 b7(IO[7],I1[7],SO,Result[7]);
    MUX_2x1 b8(IO[8],I1[8],SO,Result[8]);
    MUX_2x1 b9(IO[9],I1[9],SO,Result[9]);
    MUX_2x1 b10(IO[10],I1[10],SO,Result[10]);
    MUX_2x1 b11(IO[11],I1[11],SO,Result[11]);
    MUX_2x1 b12(IO[12],I1[12],SO,Result[12]);
    MUX_2x1 b13(IO[13],I1[13],SO,Result[13]);
    MUX_2x1 b14(IO[14],I1[14],SO,Result[14]);
    MUX_2x1 b15(IO[15],I1[15],SO,Result[15]);
endmodule

module Full_Adder(I0,I1,I2,S,C);
    input I0,I1,I2;
    output S,C;
    nand(w1,I0,I1);
    nand(w2,I0,I2);
    nand(w3,I1,I2);
    nand(C,w1,w2,w3);
    nand(w4,I0,I1,I2);
    nand(w5,I0,~I1,~I2);
    nand(w6,~I0,I1,~I2);
    nand(w7,~I0,~I1,I2);
    nand(S,w4,w5,w6,w7);
endmodule
 
module ALU_1Bit(a,b,BInvert,CarryIn,Operation,Result,CarryOut,Less);
    input a,b,BInvert,CarryIn,Less;
    input [2:0] Operation;
    output Result,CarryOut;
    MUX_8x1 mux(o1,o2,o3,s,Less,o6,o7,o8,Operation[0],Operation[1],Operation[2],Result);
    Full_Adder FA(a,b1,CarryIn,s,CarryOut);
    xor(b1,b,BInvert);
    and(o1,a,b1);
    or(o2,a,b1);
    xor(o3,a,b1);
    not(o6,o2);
endmodule
 
module ALU_1Bit_Last(a,b,BInvert,CarryIn,Operation,Result,Less,Set,Overflow);
    input a,b,BInvert,CarryIn,Less;
    input [2:0] Operation;
    output Result,Set,Overflow;
    MUX_8x1 mux(o1,o2,o3,Set,Less,o6,o7,o8,Operation[0],Operation[1],Operation[2],Result);
    Full_Adder FA(a,b1,CarryIn,Set,CarryOut);
    xor(Overflow1,CarryIn,CarryOut);
    and(Overflow,Overflow1,Operation[2]);
    xor(b1,b,BInvert);
    and(o1,a,b1);
    or(o2,a,b1);
    xor(o3,a,b1);
    not(o6,o2);
endmodule
 
module ALU_16Bit(ALUctl,a,b,Result,Zero,Overflow);
    input [3:0] ALUctl;
    input signed [15:0] a,b;
    output [15:0] Result;
    output Zero,Overflow;
    ALU_1Bit ALU0(a[0],b[0],ALUctl[3],ALUctl[3],ALUctl[2:0],Result[0],c1,Set);
    ALU_1Bit ALU1(a[1],b[1],ALUctl[3],c1,ALUctl[2:0],Result[1],c2,1'b0);
    ALU_1Bit ALU2(a[2],b[2],ALUctl[3],c2,ALUctl[2:0],Result[2],c3,1'b0);
    ALU_1Bit ALU3(a[3],b[3],ALUctl[3],c3,ALUctl[2:0],Result[3],c4,1'b0);
    ALU_1Bit ALU4(a[4],b[4],ALUctl[3],c4,ALUctl[2:0],Result[4],c5,1'b0);
    ALU_1Bit ALU5(a[5],b[5],ALUctl[3],c5,ALUctl[2:0],Result[5],c6,1'b0);
    ALU_1Bit ALU6(a[6],b[6],ALUctl[3],c6,ALUctl[2:0],Result[6],c7,1'b0);
    ALU_1Bit ALU7(a[7],b[7],ALUctl[3],c7,ALUctl[2:0],Result[7],c8,1'b0);
    ALU_1Bit ALU8(a[8],b[8],ALUctl[3],c8,ALUctl[2:0],Result[8],c9,1'b0);
    ALU_1Bit ALU9(a[9],b[9],ALUctl[3],c9,ALUctl[2:0],Result[9],c10,1'b0);
    ALU_1Bit ALU10(a[10],b[10],ALUctl[3],c10,ALUctl[2:0],Result[10],c11,1'b0);
    ALU_1Bit ALU11(a[11],b[11],ALUctl[3],c11,ALUctl[2:0],Result[11],c12,1'b0);
    ALU_1Bit ALU12(a[12],b[12],ALUctl[3],c12,ALUctl[2:0],Result[12],c13,1'b0);
    ALU_1Bit ALU13(a[13],b[13],ALUctl[3],c13,ALUctl[2:0],Result[13],c14,1'b0);
    ALU_1Bit ALU14(a[14],b[14],ALUctl[3],c14,ALUctl[2:0],Result[14],c15,1'b0);
    ALU_1Bit_Last ALU15(a[15],b[15],ALUctl[3],c15,ALUctl[2:0],Result[15],1'b0,Set,Overflow);
    nor(Zero,Result[0],Result[1],Result[2],Result[3],Result[4],Result[5],Result[6],Result[7],
        Result[8],Result[9],Result[10],Result[11],Result[12],Result[13],Result[14],Result[15]);
endmodule
 
module Main_Control(Op,Control);
    input [4:0] Op;
    output reg [11:0] Control;
    // IDEX_RegDst,IDEX_ALUSrc,IDEX_MemtoReg,IDEX_RegWrite,IDEX_MemWrite,IDEX_Beq,IDEX_Bne,IDEX_J,IDEX_ALUctl
    always @(Op) case (Op)
        5'b00000: Control <= 12'b100100000011; // ADD
        5'b00001: Control <= 12'b100100001011; // SUB
        5'b00010: Control <= 12'b100100000000; // AND
        5'b00011: Control <= 12'b100100000001; // OR
        5'b00100: Control <= 12'b100100000010; // XOR
        5'b00101: Control <= 12'b100100000101; // NOR
        5'b00110: Control <= 12'b100100001100; // SLT
        5'b00111: Control <= 12'b010100000011; // ADDI
        5'b01000: Control <= 12'b010100001100; // SLTI
        5'b01001: Control <= 12'b010100000000; // ANDI
        5'b01010: Control <= 12'b010100000001; // ORI
        5'b01011: Control <= 12'b010100000010; // XORI
        5'b01100: Control <= 12'b011100000011; // LW
        5'b01101: Control <= 12'b011010000011; // SW
        5'b01110: Control <= 12'b000001001011; // BEQ
        5'b01111: Control <= 12'b000000101011; // BNE
        5'b10000: Control <= 12'b000000011011; // J
    endcase
endmodule

module CPU(clock,PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD);
    input clock;
    output [15:0] PC,WD;
    output [16:0] IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR;

    initial begin
        // Program: swap memory cells (if needed) and compute absolute value |5-7| = 2
        IMemory[0]   = 17'b00111_00_01_00000101;     // addi $1, $0, 5
        IMemory[1]   = 17'b00111_00_10_00000111;     // addi $2, $0, 7
        IMemory[2]   = 17'b01101_00_01_00000000;     // sw $1, 0($0)
        IMemory[3]   = 17'b01101_00_10_00000010;     // sw $2, 2($0)
        IMemory[4]   = 17'b00000_00_00_00_000000;    // nop
        IMemory[5]   = 17'b00110_01_10_11_000000;    // slt $3, $1, $2
        IMemory[6]   = 17'b01110_11_00_00001010;     // beq $3, $0, IMemory[10]
        IMemory[7]   = 17'b01101_00_01_00000010;     // sw $1, 2($0)
        IMemory[8]   = 17'b01101_00_10_00000000;     // sw $2, 0($0)
        IMemory[9]   = 17'b00000_00_00_00_000000;    // nop
        IMemory[10]  = 17'b00000_00_00_00_000000;    // nop
        IMemory[11]  = 17'b00000_00_00_00_000000;    // nop
        IMemory[12]  = 17'b01100_00_01_00000000;     // lw $1, 0($0)
        IMemory[13]  = 17'b01100_00_10_00000010;     // lw $2, 2($0)
        IMemory[14]  = 17'b00000_00_00_00_000000;    // nop
        IMemory[15]  = 17'b00000_00_00_00_000000;    // nop
        IMemory[16]  = 17'b00000_00_00_00_000000;    // nop
        IMemory[17]  = 17'b00001_01_10_11_000000;    // sub $3, $1, $2
    end

// Pipeline

// IF
    wire [15:0] PCplus2,NextPC;
    reg [15:0] PC,IFID_PCplus2;
    reg [16:0] IMemory[0:1023],IFID_IR;
    ALU_16Bit fetch(4'b0011,PC,16'd2,PCplus2,Unused1,overflow0);
   
// ID
    wire [11:0] Control;
    reg IDEX_RegWrite,IDEX_MemtoReg,IDEX_Beq,IDEX_Bne,IDEX_MemWrite,IDEX_ALUSrc,IDEX_RegDst,IDEX_J;
    reg [3:0] IDEX_ALUctl;
    wire [15:0] RD1,RD2,SignExtend,WD;
    reg [15:0] IDEX_PCplus2,IDEX_RD1,IDEX_RD2,IDEX_SignExt;
    reg [16:0] IDEX_IR;
    reg [1:0] IDEX_rt,IDEX_rd;
    reg MEMWB_RegWrite;
    reg [1:0] MEMWB_rd;
    assign SignExtend = {{8{IFID_IR[7]}},IFID_IR[7:0]};
    Reg_File rf(IFID_IR[11:10],IFID_IR[9:8],MEMWB_rd,WD,MEMWB_RegWrite,RD1,RD2,clock);
    Main_Control MainCtr(IFID_IR[16:12],Control);
 
// EXE
    reg EXMEM_RegWrite,EXMEM_MemtoReg,EXMEM_Beq,EXMEM_Bne,EXMEM_J,EXMEM_MemWrite;
    wire [15:0] Target;
    reg EXMEM_Zero;
    reg [15:0] EXMEM_Target,EXMEM_ALUOut,EXMEM_RD2;
    reg [16:0] EXMEM_IR;
    reg [1:0] EXMEM_rd;
    wire [15:0] B,ALUOut;
    wire [3:0] ALUctl;
    wire [1:0] WR;
    ALU_16Bit branch(4'b0011,IDEX_SignExt<<1,IDEX_PCplus2,Target,Unused2,overflow2);
    ALU_16Bit ex(IDEX_ALUctl,IDEX_RD1,B,ALUOut,Zero,overflow1);
    MUX_16Bit_2x1 ALUSrcMux(IDEX_SignExt,IDEX_RD2,IDEX_ALUSrc,B);
    MUX_2Bit_2x1 RegDstMux(IDEX_rd,IDEX_rt,IDEX_RegDst,WR);
    
    // Forwarding
    wire [15:0] FWD_RD1,FWD_RD2;
    and(f1,IDEX_RegWrite,WR==IFID_IR[11:10]);
    MUX_16Bit_2x1 fwd1(ALUOut,RD1,f1,FWD_RD1);
    and(f2,IDEX_RegWrite,WR==IFID_IR[9:8]);
    MUX_16Bit_2x1 fwd2(ALUOut,RD2,f2,FWD_RD2);

    // Branch and Jump Logic
    and(w1,EXMEM_Beq,EXMEM_Zero);
    and(w2,EXMEM_Bne,~EXMEM_Zero);
    or(w3,w1,w2);
    wire [15:0] w4;
    MUX_16Bit_2x1 mux1(EXMEM_Target,PCplus2,w3,w4);
    MUX_16Bit_2x1 mux2({{5{EXMEM_IR[10]}},EXMEM_IR[10:0]},w4,EXMEM_J,NextPC);
   
// MEM
    reg MEMWB_MemtoReg;
    reg [15:0] DMemory[0:1023],MEMWB_MemOut,MEMWB_ALUOut;
    reg [16:0] MEMWB_IR;
    wire [15:0] MemOut;
    assign MemOut = DMemory[EXMEM_ALUOut>>1];
 
// WB
    MUX_16Bit_2x1 MemtoRegMux(MEMWB_MemOut,MEMWB_ALUOut,MEMWB_MemtoReg,WD);

    initial begin
        PC=0;
        IFID_IR=0;
        IDEX_RegWrite=0;IDEX_MemtoReg=0;IDEX_Beq=0;IDEX_Bne=0;IDEX_MemWrite=0;IDEX_ALUSrc=0;IDEX_RegDst=0;IDEX_J=0;IDEX_ALUctl=0;
        EXMEM_RegWrite=0;EXMEM_MemtoReg=0;EXMEM_Beq=0;EXMEM_Bne=0;EXMEM_J=0;EXMEM_MemWrite=0;EXMEM_Target=0;
        MEMWB_RegWrite=0;MEMWB_MemtoReg=0;
    end

// Running the Pipeline

    always @(negedge clock) begin
    
// IF
        PC <= NextPC;
        IFID_PCplus2 <= PCplus2;
        IFID_IR <= IMemory[PC>>1];

// ID
        IDEX_IR <= IFID_IR;
        {IDEX_RegDst,IDEX_ALUSrc,IDEX_MemtoReg,IDEX_RegWrite,IDEX_MemWrite,IDEX_Beq,IDEX_Bne,IDEX_J,IDEX_ALUctl} <= Control;  
        IDEX_PCplus2 <= IFID_PCplus2;
        IDEX_RD1 <= FWD_RD1; 
        IDEX_RD2 <= FWD_RD2;
        IDEX_SignExt <= SignExtend;
        IDEX_rt <= IFID_IR[9:8];
        IDEX_rd <= IFID_IR[7:6];

// EXE
        EXMEM_IR <= IDEX_IR;
        EXMEM_RegWrite <= IDEX_RegWrite;
        EXMEM_MemtoReg <= IDEX_MemtoReg;
        EXMEM_Beq <= IDEX_Beq;
        EXMEM_Bne <= IDEX_Bne;
        EXMEM_J <= IDEX_J;
        EXMEM_MemWrite <= IDEX_MemWrite;
        EXMEM_Target <= Target;
        EXMEM_Zero <= Zero;
        EXMEM_ALUOut <= ALUOut;
        EXMEM_RD2 <= IDEX_RD2;
        EXMEM_rd <= WR;

// MEM
        MEMWB_IR <= EXMEM_IR;
        MEMWB_RegWrite <= EXMEM_RegWrite;
        MEMWB_MemtoReg <= EXMEM_MemtoReg;
        MEMWB_MemOut <= MemOut;
        MEMWB_ALUOut <= EXMEM_ALUOut;
        MEMWB_rd <= EXMEM_rd;
        if (EXMEM_MemWrite) DMemory[EXMEM_ALUOut>>1] <= EXMEM_RD2;

// WB
// Register write happens on neg edge of the clock (if MEMWB_RegWrite is asserted)

    end

endmodule

// Test Module

module Test();
    reg clock;
    wire [15:0] PC,WD;
    wire [16:0] IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR;
    CPU c1(clock,PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD);
    always #1 clock = ~clock;
    initial begin
        $display ("Time   PC     IFID_IR            IDEX_IR            EXMEM_IR           MEMWB_IR             WD");
        $monitor ("%3d    %3d    %b  %b  %b  %b    %h", $time,PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD);
        clock = 1;
        #50 $finish;
    end
endmodule

/* Test Output:

Time   PC    IFID_IR            IDEX_IR            EXMEM_IR           MEMWB_IR             WD
 0      0    00000000000000000  xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx    xxxx
 1      2    00111000100000101  00000000000000000  xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx    xxxx
 3      4    00111001000000111  00111000100000101  00000000000000000  xxxxxxxxxxxxxxxxx    xxxx
 5      6    01101000100000000  00111001000000111  00111000100000101  00000000000000000    0000
 7      8    01101001000000010  01101000100000000  00111001000000111  00111000100000101    0005
 9     10    00000000000000000  01101001000000010  01101000100000000  00111001000000111    0007
11     12    00110011011000000  00000000000000000  01101001000000010  01101000100000000    xxxx
13     14    01110110000001010  00110011011000000  00000000000000000  01101001000000010    xxxx
15     16    01101000100000010  01110110000001010  00110011011000000  00000000000000000    0000
17     18    01101001000000000  01101000100000010  01110110000001010  00110011011000000    0001
19     20    00000000000000000  01101001000000000  01101000100000010  01110110000001010    0001
21     22    00000000000000000  00000000000000000  01101001000000000  01101000100000010    xxxx
23     24    00000000000000000  00000000000000000  00000000000000000  01101001000000000    xxxx
25     26    01100000100000000  00000000000000000  00000000000000000  00000000000000000    0000
27     28    01100001000000010  01100000100000000  00000000000000000  00000000000000000    0000
29     30    00000000000000000  01100001000000010  01100000100000000  00000000000000000    0000
31     32    00000000000000000  00000000000000000  01100001000000010  01100000100000000    0007
33     34    00000000000000000  00000000000000000  00000000000000000  01100001000000010    0005
35     36    00001011011000000  00000000000000000  00000000000000000  00000000000000000    0000
37     38    xxxxxxxxxxxxxxxxx  00001011011000000  00000000000000000  00000000000000000    0000
39     40    xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx  00001011011000000  00000000000000000    0000
41     42    xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx  00001011011000000    0002
43     44    xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx    xxxx
45     46    xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx    xxxx
47     48    xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx    xxxx
49     50    xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx  xxxxxxxxxxxxxxxxx    xxxx

*/



