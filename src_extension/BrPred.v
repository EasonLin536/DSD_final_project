module PredictionUnit(  
    clk,
    rst_n,
    stall,
    PreWrong,
    opcode,
    BrPre
);

    //==== input/output definition ============================
    input            clk, rst_n, stall, PreWrong;
    input      [5:0] opcode;
    output reg       BrPre;
    //==== state definition ===================================
    parameter        NotTaken1 = 2'b00;
    parameter        NotTaken2 = 2'b01;
    parameter        Taken1    = 2'b10;
    parameter        Taken2    = 2'b11;
    //==== wire/reg definition ================================
    wire             BranchSignal;
    wire       [1:0] state_old;
    reg        [1:0] state, nxt_state;
    //==== combinational circuit ==============================
    assign BranchSignal = ((opcode == 6'h4) || (opcode == 6'h5)) ? 1'b1 : 1'b0;
    assign state_old = state;
    
    always @(*) begin
        BrPre = 1'b0;
        nxt_state =  state;
        case(state)
            NotTaken1: begin
                BrPre = 1'b0;
                if (PreWrong) nxt_state = NotTaken2;
                else nxt_state = NotTaken1;
            end
            NotTaken2: begin
                BrPre = 1'b0;
                if (PreWrong) nxt_state = Taken1;
                else nxt_state = NotTaken1;
            end
            Taken1: begin
                BrPre = 1'b1 & BranchSignal;
                if (PreWrong) nxt_state = NotTaken2;
                else nxt_state = Taken2;
            end
            Taken2: begin
                BrPre = 1'b1 & BranchSignal;
                if (PreWrong) nxt_state = Taken1;
                else nxt_state = Taken2;
            end
            default: begin
                BrPre = 1'b0;
                nxt_state = state;
            end
        endcase
    end
    //==== sequential circuit =================================
    always @(posedge clk) begin
        if (!rst_n) begin
            state <= NotTaken1;
        end
        else if (stall) begin
            state <= state_old;
        end
        else begin
            state <= nxt_state;
        end
    end
endmodule

module Comparater(  
    BrPre,
    equal,
    Ctrl_Br,
    PreWrong
);
    //==== input/output definition ============================
    input            BrPre, equal;
    input      [1:0] Ctrl_Br;    // 00 don't branch, 01 beq, 10 bne
    output reg       PreWrong;
    //==== combinational circuit ==============================
    always @(*) begin
        PreWrong = 1'b0;
        if (BrPre && ((Ctrl_Br == 2'b01 && equal == 1'b0) || (Ctrl_Br == 2'b10 && equal == 1'b1))) begin
            // if predicted branch but actually doesn't have to branch
            PreWrong = 1'b1;
        end
        else if ((~BrPre) && ((Ctrl_Br == 2'b01 && equal == 1'b1) || (Ctrl_Br == 2'b10 && equal == 1'b0))) begin
            // if predicted not branch but actually has to branch
            PreWrong = 1'b1;
        end
        else PreWrong = 1'b0;
    end
endmodule