module ALU_CONTROL(
    funct,
    ALUOp,
    ALUCtrl,
    JumpReg, 
    Shift,
    Jalr
);
//==== io Declaration =========================
    input      [5:0] funct;
    input      [2:0] ALUOp;
    output reg [3:0] ALUCtrl;
    output reg       JumpReg, Shift, Jalr;
    
//==== Reg/Wire Declaration ===================
    wire       [8:0] temp;
    assign temp = { ALUOp, funct };
    
    always @(*) begin
        casex(temp)
            9'b000xxxxxx: begin // j, addi, lw, sw ==> add
                ALUCtrl = 4'b0010;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b001xxxxxx: begin // beq ==> substract
                ALUCtrl = 4'b0110;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010000000: begin // 00: sll
                ALUCtrl = 4'b1010;
                JumpReg = 0;
                Shift   = 1;
                Jalr    = 0;
            end
            9'b010000010: begin // 02: srl
                ALUCtrl = 4'b1100;
                JumpReg = 0;
                Shift   = 1;
                Jalr    = 0;
            end
            9'b010000011: begin // 03: sra
                ALUCtrl = 4'b1011;
                JumpReg = 0;
                Shift   = 1;
                Jalr    = 0;
            end
            9'b010001000: begin // 08: jr
                ALUCtrl = 4'b0000; // xxxx
                JumpReg = 1;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010001001: begin // 08: jalr
                ALUCtrl = 4'b0000; // xxxx
                JumpReg = 1;
                Shift   = 0;
                Jalr    = 1;
            end
            9'b010100000: begin // 20: add
                ALUCtrl = 4'b0010;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010100010: begin // 22: sub
                ALUCtrl = 4'b0110;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010100100: begin // 24: and
                ALUCtrl = 4'b0000;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010100101: begin // 25: or
                ALUCtrl = 4'b0001;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010100110: begin // 26: xor
                ALUCtrl = 4'b1001;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010100111: begin // 27: nor
                ALUCtrl = 4'b1000;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010101010: begin // 2a: slt
                ALUCtrl = 4'b0111;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b011xxxxxx: begin // bne
                ALUCtrl = 4'b0011;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b100xxxxxx: begin // slti
                ALUCtrl = 4'b0111;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b101xxxxxx: begin // andi
                ALUCtrl = 4'b0000;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b110xxxxxx: begin // ori
                ALUCtrl = 4'b0001;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b111xxxxxx: begin // xori
                ALUCtrl = 4'b1001;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            default: begin
                ALUCtrl = 4'b0000;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
        endcase
    end
endmodule