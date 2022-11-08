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
                  input  [31:0] data_from_mem
                );
    
    reg ALUSrc; //
    wire [2:0] AluControl; //
    reg MemWrite; //
    reg MemToReg; //
    reg RegWrite; //
    reg BranchBeq; //
    reg BranchJal; //
    reg BranchJalr; //
    wire [2:0] immControl; //
    
    ctrl CTRL_unit(instruction[6:0], instruction[31:25], 
                   instruction[12:14],
                   ALUSrc, MemWrite, MemToReg, RegWrite, BranchBeq,
                   BranchJal, BranchJalr, immControl
                   );
    
    wire [31:0] ImmOp;
    wire [31:0] res;
    wire [31:0] Reg1;
    wire [31:0] Reg2;
    
    immCtrl immControl(instruction[31:7],
                       immControl,
                       ImmOp);

    registers registerSet(instruction[19:15], 
                          instruction[24:20],
                          instruction[11:7],
                          clk, RegWrite,
                          res, Reg1, Reg2);
    
    wire [31:0] SrcA = Reg1;
    wire [31:0] SrcB;
    reg Zero;
    wire [31:0] ALUout;
    wire [31:0] readData;
    wire [31:0] branchTarget;
    wire [31:0] PC;

    aluMux mux2_1(Reg2, ImmOp, ALUSrc, SrcB);

    alu ALU(AluControl, SrcA, SrcB, ALUout, Zero);
    
    dataMem data_mem(clk, MemWrite, ALUout, SrcB, readData);


    branchTargetMux mux2_1( /*kombinace?TODO*/,ALUout,BranchJalr,branchTarget);

    reg BranchJalx;
    reg BranchOutcome;

    always @(BranchBeq or BranchJal or BranchJalr)
    begin
        BranchJalx <= BranchJal || BranchJalr;
        BranchOutcome <= (BranchBeq && Zero) || BranchJalx;
    end

    wire [31:0] branchJalxMuxOut;
    wire [31:0] PCn;
    wire [31:0] pc;

    branchJalxMux mux2_1(ALUout, /*PCplus4TODO*/, BranchJalx, branchJalxMuxOut );
    
    dataMemMux mux2_1( branchJalxMuxOut, readData, MemToReg, res);

    PCMux mux2_1(/*PCplus4*/, res, BranchOutcome, PCn)

    PCRegister register(PCn, clk, pc);

    assign PC = pc;
    assign WE = MemWrite;
    assign address_to_mem = ALUout;
    assign data_to_mem = SrcB;
endmodule
//=========================================================================================
//=========================================================================================
//                 ARITHMETIC AND LOGIC UNIT
//=========================================================================================
//=========================================================================================
module ALU( input [2:0] ALUControl, 
            input [31:0] SrcA, SrcB,
            output [31:0] ALUout, 
            output Zero
          );

// ALUControl
// 000 = +
// 001 = &
// 010 = -
// 011 = <
// 100 = /
// 101 = %
// 110 = !<
// 111 = 

    always @(*)
    begin
        case (ALUControl)
            0: ALUout = SrcA + SrcB;
            1: ALUout = SrcA & SrcB;
            2: ALUout = SrcA - SrcB;
            3: ALUout = SrcA < SrcB;
            4: ALUout = SrcA / SrcB;
            5: ALUout = SrcA % SrcB;
            6: ALUout = SrcA + SrcB;
            7: ALUout = !(SrcA < SrcB);
            default: ALUout = X;
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
module immControl(input inst[24:0], // value of instruction[31:7]
                  input [2:0] control,
                  output [31:0] immOp
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
            0: immOp = { 20'b0, inst[24:13]}; // I
            1: immOp = { 20'b0, inst[24:18], inst[4:0]}; // S
            2: immOp = { 19'b0, inst[24], inst[0], inst[23:18], inst[4:1], 1'b0}; // B
            3: immOp = { 11'b0, inst[24], inst[12:5], inst[13], inst[23:14],1'b0}; // J
            4: immOp = { inst[24:5], 12'b0}; // U
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
                  output [2:0] ALUControl,
                  output MemWrite,
                  output MemToReg,
                  output RegWrite,
                  output BranchBeq,
                  output BranchJal,
                  output BranchJalr,
                  output [2:0] immControl
                );
    reg [12:0] out; // output bits in order of module arguments

// ALUControl
// 000 = +    // immControl
// 001 = &    // 000 I-Type
// 010 = -    // 001 S-Type
// 011 = <    // 010 B-Type
// 100 = /    // 011 J-Type
// 101 = %    // 100 U-Type
// 110 = !<
// 111 = ERROR

    always @(*)
    begin
        casez ({opCode,funct7,funct3)
            'b0110011_0000000_000: out = 'b0_000_0_0_1_0_0_0_XXX;   // add

            'b0010011_???????_000: out = 'b1_000_0_0_1_0_0_0_000;   // addi

            'b0110011_0000000_111: out = 'b0_001_0_0_1_0_0_0_XXX;   // and
            'b0110011_0100000_000: out = 'b0_010_0_0_1_0_0_0_XXX;   // sub
            'b0110011_0000000_010: out = 'b0_011_0_0_1_0_0_0_XXX;   // slt
            'b0110011_0000001_100: out = 'b0_100_0_0_1_0_0_0_XXX;   // div
            'b0110011_0000001_110: out = 'b0_101_0_0_1_0_0_0_XXX;   // rem

            'b1100011_???????_000: out = 'b0_010_0_X_0_1_0_0_010;   // beq
            'b1100011_???????_100: out = 'b0_110_0_X_0_1_0_0_010;   // blt

            'b0000011_???????_010: out = 'bX_000_0_X_1_0_0_0_000;   // lw
            'b0100011_???????_010: out = 'bX_000_1_X_0_0_0_0_001;   // sw
            'b0110111_???????_???: out = 'b0_000_0_1_1_0_0_0_100;   // lui TODO

            'b1101111_???????_???: out = 'bX_XXX_0_0_1_0_1_0_011;   // jal
            'b1100111_???????_000: out = 'b1_000_0_0_1_0_0_1_000;   // jalr
            
            default: out = 'bX_XXX_X_X_X_X_X_X_XX;
        endcase
    end

    assign ALUSrc = out[11];
    assign ALUControl[2:0] = out[10:8];
    assign MemWrite = out[7];
    assign MemToReg = out[6];
    assign RegWrite = out[5];
    assign BranchBeq = out[4];
    assign BranchJal = out[3];
    assign BranchJalr = out[2];
    assign [1:0] immControl = out[1:0];

endmodule
//=========================================================================================
//=========================================================================================
//                 Multiplexer 2 to 1
//=========================================================================================
//=========================================================================================
module mux2_1(	input [31:0] a,b, select,
		output reg y);

	always@(*)
		if (select==0) y = a;
		else y = b;
endmodule
//=========================================================================================
//=========================================================================================
//                 REGISTER SET
//=========================================================================================
//=========================================================================================
module registerSet ( input [4:0] A1, A2, A3,
                     input clk, WE3
                     input [31:0] WD3,
                     output reg [31:0] RD1, RD2
                    );

    reg [31:0] rf[31:0];
    rf[0] = 0;

    always @(A1)
    begin
        RD1 = rf[A1]
    end

    always @(A2)
    begin
        RD2 = rf[A2]
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

module register(input [31:0] in,
            input clk,
            output reg [31:0] out);
    always @(posedge clk)
    begin
        out = in;
    end
endmodule
//=========================================================================================
//=========================================================================================
//                 END OF FILE
//=========================================================================================
//=========================================================================================

`default_nettype wire
