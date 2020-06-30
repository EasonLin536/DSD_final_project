module CONTROL(
    Op,
    ctrl_flush,
    RegDst,
    Jump,
    Branch,
    MemRead,
    MemtoReg,
    ALUOp,
    MemWrite,
    ALUSrc,
    RegWrite,
    Jal,
    ExtOp
);
//==== io Declaration =========================
    input      [5:0] Op;
    input            ctrl_flush;
    output reg       RegDst, Jump, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, Jal, ExtOp;
    output reg [1:0] Branch; // 00 don't branch, 01 beq, 10 bne
    output reg [2:0] ALUOp;
    
    always @(*) begin
        case(Op)
            8'h00: begin // R type
                RegDst   = 1;
                Jump     = 0;
                Branch   = 2'b0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b010;
                MemWrite = 0;
                ALUSrc   = 0;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 0; // x
            end
            8'h02: begin // j
                RegDst   = 0; // x
                Jump     = 1;
                Branch   = 2'b0;
                MemRead  = 0;
                MemtoReg = 0; // x
                ALUOp    = 3'b000;
                MemWrite = 0;
                ALUSrc   = 0; // x
                RegWrite = 0;
                Jal      = 0;
                ExtOp    = 0; // x
            end
            8'h03: begin // jal
                RegDst   = 0; // x
                Jump     = 1;
                Branch   = 2'b0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b000;
                MemWrite = 0;
                ALUSrc   = 0; // x
                RegWrite = 1;
                Jal      = 1;
                ExtOp    = 0; // x
            end
            8'h04: begin // beq
                RegDst   = 0; // x
                Jump     = 0;
                Branch   = 2'b01;
                MemRead  = 0;
                MemtoReg = 0; // x
                ALUOp    = 3'b001;
                MemWrite = 0;
                ALUSrc   = 0;
                RegWrite = 0;
                Jal      = 0;
                ExtOp    = 1;
            end
            8'h05: begin // bne
                RegDst   = 0; // x
                Jump     = 0;
                Branch   = 2'b10;
                MemRead  = 0;
                MemtoReg = 0; // x
                ALUOp    = 3'b011;
                MemWrite = 0;
                ALUSrc   = 0;
                RegWrite = 0;
                Jal      = 0;
                ExtOp    = 1;
            end
            8'h08: begin // addi
                RegDst   = 0;
                Jump     = 0;
                Branch   = 2'b0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b000;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 1;
            end
            8'h23: begin // lw
                RegDst   = 0;
                Jump     = 0;
                Branch   = 2'b0;
                MemRead  = 1;
                MemtoReg = 1;
                ALUOp    = 3'b000;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 1;
            end
            8'h2b: begin // sw
                RegDst   = 0; // x
                Jump     = 0;
                Branch   = 2'b0;
                MemRead  = 0;
                MemtoReg = 0; // x
                ALUOp    = 3'b000;
                MemWrite = 1;
                ALUSrc   = 1;
                RegWrite = 0;
                Jal      = 0;
                ExtOp    = 1;
            end
            8'h0a: begin // slti
                RegDst   = 0;
                Jump     = 0;
                Branch   = 2'b0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b100;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 1;
            end
            8'h0c: begin // andi
                RegDst   = 0;
                Jump     = 0;
                Branch   = 2'b0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b101;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 0;
            end
            8'h0d: begin // ori
                RegDst   = 0;
                Jump     = 0;
                Branch   = 2'b0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b110;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 0;
            end
            8'h0e: begin // xori
                RegDst   = 0;
                Jump     = 0;
                Branch   = 2'b0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b111;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 0;
            end
            default: begin
                RegDst   = 0;
                Jump     = 0;
                Branch   = 2'b0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b000;
                MemWrite = 0;
                ALUSrc   = 0;
                RegWrite = 0;
                Jal      = 0;
                ExtOp    = 0;
            end
        endcase
        if (~ctrl_flush) begin
            RegDst   = 0;
            Jump     = 0;
            Branch   = 2'b0;
            MemRead  = 0;
            MemtoReg = 0;
            ALUOp    = 3'b000;
            MemWrite = 0;
            ALUSrc   = 0;
            RegWrite = 0;
            Jal      = 0;
            ExtOp    = 0;
        end
    end
endmodule