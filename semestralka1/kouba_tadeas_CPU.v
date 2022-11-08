`default_nettype none
//=========================================================================================
//=========================================================================================
//                 CPU
//=========================================================================================
//=========================================================================================
module processor( input         clk, reset, //
                  output [31:0] PC, //
                  input  [31:0] instruction, //
                  output        WE, //
                  output [31:0] address_to_mem, //
                  output [31:0] data_to_mem, //
                  input  [31:0] data_from_mem //
                );
    
    wire ALUSrc; // 0/1 Which source to use for SrcB
    wire [3:0] AluControl; // 0-7 which operation should ALU make
    wire MemWrite; // 0/1 Write to memory?
    wire MemToReg; // 0/1 Which source to use for registerSet write
    wire RegWrite; // 0/1 Write to register?
    wire BranchBeq; // 0/1 Branch?
    wire BranchJal; // 0/1 Jump? 
    wire BranchJalr; // 0/1 Return?
    wire [2:0] immControl; // 0-7 Controls how immDecode decodes imm

    wire [31:0] ImmOp; // return value of immDecode
    wire [31:0] res; // Value to write to register
    wire [31:0] Reg1; // Value from register1
    wire [31:0] Reg2; // Value from register2
   
    wire [31:0] SrcB; // Operand2 of ALU (Decided by ALUSrc - can be ImmOp or Reg2)

    wire [31:0] ALUout; // Output of ALU
    wire Zero; // 1 if ALUout == 0 else 1
    wire [31:0] branchTarget;
    reg BranchJalx;
    reg BranchOutcome;
    wire [31:0] brJalxMuxOut;
    wire [31:0] PCn;
    
    wire [31:0] pc;
    wire [31:0] PCPlus4;
    wire [31:0] ImmOpPlusPC;
    wire [31:0] SrcA;
    wire [31:0] readData;    

    assign PCPlus4 = pc + 4;
    assign ImmOpPlusPC = ImmOp + pc;
    assign SrcA = Reg1; // Operand1 of ALU 
    assign readData = data_from_mem; // Data read from data memory 

    always @(BranchBeq or BranchJal or BranchJalr)
    begin
        BranchJalx = BranchJal || BranchJalr;
        BranchOutcome = (BranchBeq && Zero) || BranchJalx;
    end


    CTRL_unit ctrl(instruction[6:0], instruction[31:25], 
                   instruction[14:12],
                   ALUSrc, AluControl, MemWrite, MemToReg, RegWrite, BranchBeq,
                   BranchJal, BranchJalr, immControl
                   );
    
    immDecode immDec(instruction[31:0],
                       immControl,
                       ImmOp);

    registerSet registers(instruction[19:15], 
                          instruction[24:20],
                          instruction[11:7],
                          clk, reset, RegWrite,
                          res, Reg1, Reg2);
    
    ALU alu(AluControl, pc, SrcA, SrcB, ALUout, Zero);
    
    register PCRegister(PCn, clk, reset, pc);
    
    mux2_1 aluMux(Reg2, ImmOp, ALUSrc, SrcB);
    
    mux2_1 brTargetMux( ImmOpPlusPC,ALUout,BranchJalr,branchTarget);

    mux2_1 brJalxMux(ALUout, PCPlus4, BranchJalx, brJalxMuxOut );
    
    mux2_1 dataMemMux( brJalxMuxOut, readData, MemToReg, res);

    mux2_1 PCMux(PCPlus4, branchTarget, BranchOutcome, PCn);


    assign PC = pc;
    assign WE = MemWrite;
    assign address_to_mem = ALUout;
    assign data_to_mem = Reg2;

endmodule
//=========================================================================================
//=========================================================================================
//                 ARITHMETIC AND LOGIC UNIT
//=========================================================================================
//=========================================================================================
module ALU( input [3:0] ALUControl,
            input [31:0] pc,
            input signed [31:0] SrcA, SrcB,
            output reg [31:0] ALUout, 
            output reg Zero
          );

// ALUControl
// 0000 = +
// 0001 = &
// 0010 = -
// 0011 = <
// 0100 = /
// 0101 = %
// 0110 = !<
// 0111 = LUI
// 1000 = <<
// 1001 = >>    
// 1010 = auipc


    always @(*)
    begin
        case (ALUControl)
            0: ALUout = SrcA + SrcB;
            1: ALUout = SrcA & SrcB;
            2: ALUout = SrcA - SrcB;
            3: ALUout = SrcA < SrcB;
            4: ALUout = SrcA / SrcB;
            5: ALUout = SrcA % SrcB;
            6: ALUout = !(SrcA < SrcB);
            7: ALUout = {SrcB, 12'b0};
            8: ALUout = SrcA << SrcB;
            9: ALUout = SrcA >> SrcB;
            10: ALUout = pc + {SrcB, 12'b0};
            default: ALUout = 32'bX;
        endcase
    end

    always @(ALUout)
    begin
        if (ALUout==0)
            Zero = 1;
        else
            Zero = 0;
    end
endmodule
//=========================================================================================
//=========================================================================================
//                 IMMIDIATE CONTROL
//=========================================================================================
//=========================================================================================
module immDecode(input [31:0] inst, // value of instruction[31:0]
                  input [2:0] control,
                  output reg [31:0] immOp
                 );
// immControl
// 000 I-Type
// 001 S-Type
// 010 B-Type
// 011 J-Type
// 100 U-Type


    always @(*)
    begin
        case(control)
            0: immOp = { {20{inst[31]}}, inst[31:20]}; // I
            1: immOp = { {20{inst[31]}}, inst[31:25], inst[11:7]}; // S
            2: immOp = { {19{inst[31]}}, inst[7], inst[31:25], inst[11:8], 1'b0}; // B
            3: immOp = { {12{inst[31]}}, inst[19:12], inst[20], inst[30:21],1'b0}; // J
            4: immOp = { inst[31:12], 12'b0}; // U
            default:;
        endcase
    end

endmodule
//=========================================================================================
//=========================================================================================
//                 CONTROL UNIT       
//=========================================================================================
//=========================================================================================
module CTRL_unit( input [6:0] opCode, funct7,
                  input [2:0] funct3,
                  output ALUSrc,
                  output [3:0] ALUControl,
                  output MemWrite,
                  output MemToReg,
                  output RegWrite,
                  output BranchBeq,
                  output BranchJal,
                  output BranchJalr,
                  output [2:0] immControl
                );
    reg [13:0] out; // output bits in order of module arguments

// ALUControl
// 0000 = +    // immControl
// 0001 = &    // 000 I-Type
// 0010 = -    // 001 S-Type
// 0011 = <    // 010 B-Type
// 0100 = /    // 011 J-Type
// 0101 = %    // 100 U-Type
// 0110 = !<
// 0111 = LUI
// 1000 = <<
// 1001 = >>    
// 1010 = auipc

    always @(*)
    begin
        casez ({opCode,funct7,funct3})
            'b0110011_0000000_000: out = 'b0_0000_0_0_1_0_0_0_XXX;   // add

            'b0010011_???????_000: out = 'b1_0000_0_0_1_0_0_0_000;   // addi

            'b0110011_0000000_111: out = 'b0_0001_0_0_1_0_0_0_XXX;   // and
            'b0110011_0100000_000: out = 'b0_0010_0_0_1_0_0_0_XXX;   // sub
            'b0110011_0000000_010: out = 'b0_0011_0_0_1_0_0_0_XXX;   // slt
            'b0110011_0000001_100: out = 'b0_0100_0_0_1_0_0_0_XXX;   // div
            'b0110011_0000001_110: out = 'b0_0101_0_0_1_0_0_0_XXX;   // rem
            'b0110011_0000000_001: out = 'b0_1000_0_0_1_0_0_0_XXX;   // sll           
            'b0110011_0000000_101: out = 'b0_1001_0_0_1_0_0_0_XXX;   // srl           
            'b0110011_0100000_101: out = 'b0_1001_0_0_1_0_0_0_XXX;   // sra           

            'b1100011_???????_000: out = 'b0_0010_0_X_0_1_0_0_010;   // beq
            'b1100011_???????_100: out = 'b0_0110_0_X_0_1_0_0_010;   // blt

            'b0000011_???????_010: out = 'b1_0000_0_1_1_0_0_0_000;   // lw
            'b0100011_???????_010: out = 'b1_0000_1_X_0_0_0_0_001;   // sw

            'b0110111_???????_???: out = 'b1_0111_0_0_1_0_0_0_100;   // lui
            'b0010111_???????_???: out = 'b1_1010_0_0_1_0_0_0_100;   // auipc

            'b1101111_???????_???: out = 'bX_XXXX_0_0_1_0_1_0_011;   // jal
            'b1100111_???????_000: out = 'b1_0000_0_0_1_0_0_1_000;   // jalr
            
            default: out = 'bX_XXXX_X_X_X_X_X_X_XX;
        endcase
    end

    assign ALUSrc = out[13];
    assign ALUControl = out[12:9];
    assign MemWrite = out[8];
    assign MemToReg = out[7];
    assign RegWrite = out[6];
    assign BranchBeq = out[5];
    assign BranchJal = out[4];
    assign BranchJalr = out[3];
    assign immControl = out[2:0];

endmodule
//=========================================================================================
//=========================================================================================
//                 Multiplexer 2 to 1
//=========================================================================================
//=========================================================================================
module mux2_1(	input [31:0] a,b, 
                input select,
		output reg [31:0] y);

	always@(*)
    begin
		if (select==0) y = a;
		else y = b;
    end

endmodule
//=========================================================================================
//=========================================================================================
//                 REGISTER SET
//=========================================================================================
//=========================================================================================
module registerSet ( input [4:0] A1, A2, A3,
                     input clk, reset, WE3,
                     input [31:0] WD3,
                     output reg [31:0] RD1, RD2
                    );

    reg [31:0] rf[31:0];
    // wire zero = 0;

    integer i;
    always @(posedge clk or reset)
    begin
        if ( reset == 1 )
            for (i = 0; i <= 31 ; i=i+1)
            begin
                rf[i] = 0;    
            end
    end

    always @(A1)
    begin
        RD1 = rf[A1];
    end
    
    always @(A2)
    begin
        RD2 = rf[A2];
    end

    always @(posedge clk)
    begin
        if (WE3)
            if (A3 != 0)
                rf[A3] = WD3;
            else;
        else;
    end
endmodule
//=========================================================================================
//=========================================================================================
//                 PC REGISTER
//=========================================================================================
//=========================================================================================
module register(input [31:0] in,
            input clk, reset,
            output reg [31:0] out);
    
    always @(posedge clk or reset)
    begin
        if (reset==1)
            out <= 0;
        else
            out <= in;
    end
endmodule
//=========================================================================================
//=========================================================================================
//                 END OF FILE
//=========================================================================================
//=========================================================================================

`default_nettype wire
