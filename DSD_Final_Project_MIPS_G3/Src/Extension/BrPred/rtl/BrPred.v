module PredictionUnit(  clk,
                        rst_n,
                        stall,
                        PreWrong,
                        opcode,
                        BranchPC_IF,
                        BranchPC_ID,
                        BrPre
);

    //==== input/output definition ============================
    input           clk, rst_n, stall;
    input     [1:0] PreWrong;
    input     [5:0] opcode;
    input     [1:0] BranchPC_IF, BranchPC_ID;
    output reg      BrPre;
    //==== state definition ===================================
    parameter       NotTaken1 = 2'b00;
    parameter       NotTaken2 = 2'b01;
    parameter       Taken1 = 2'b10;
    parameter       Taken2 = 2'b11;
    //==== wire/reg definition ================================
    wire            BranchSignal;
    wire      [1:0] state_old[0:3];
    reg       [1:0] state[0:3], nxt_state;
    integer         i;
    //==== combinational circuit ==============================
    assign BranchSignal = ((opcode == 6'h4) || (opcode == 6'h5)) ? 1'b1 : 1'b0; //  beq, bne
    assign state_old[0] = state[0];
    assign state_old[1] = state[1];
    assign state_old[2] = state[2];
    assign state_old[3] = state[3];
    // FSM
    always @(*) begin
        case(BranchPC_ID)
            2'd0: begin
                nxt_state =  state[0];
                case(state[0])
                    NotTaken1: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = NotTaken2;
                        else nxt_state = NotTaken1;
                    end
                    NotTaken2: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = Taken1;
                        else if(PreWrong == 2'b00) nxt_state = NotTaken1;
                        else nxt_state =  state[0];
                    end
                    Taken1: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = NotTaken2;
                        else if(PreWrong == 2'b00) nxt_state = Taken2;
                        else nxt_state =  state[0];
                    end
                    Taken2: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = Taken1;
                        else nxt_state = Taken2;
                    end
                    default: begin
                        nxt_state = state[0];
                    end
                endcase
            end
            2'd1: begin
                nxt_state =  state[1];
                case(state[1])
                    NotTaken1: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = NotTaken2;
                        else nxt_state = NotTaken1;
                    end
                    NotTaken2: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = Taken1;
                        else if(PreWrong == 2'b00) nxt_state = NotTaken1;
                        else nxt_state =  state[1];
                    end
                    Taken1: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = NotTaken2;
                        else if(PreWrong == 2'b00) nxt_state = Taken2;
                        else nxt_state =  state[1];
                    end
                    Taken2: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = Taken1;
                        else nxt_state = Taken2;
                    end
                    default: begin
                        nxt_state = state[1];
                    end
                endcase
            end
            2'd2: begin
                nxt_state =  state[2];
                case(state[2])
                    NotTaken1: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = NotTaken2;
                        else nxt_state = NotTaken1;
                    end
                    NotTaken2: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = Taken1;
                        else if(PreWrong == 2'b00) nxt_state = NotTaken1;
                        else nxt_state =  state[2];
                    end
                    Taken1: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = NotTaken2;
                        else if(PreWrong == 2'b00) nxt_state = Taken2;
                        else nxt_state =  state[2];
                    end
                    Taken2: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = Taken1;
                        else nxt_state = Taken2;
                    end
                    default: begin
                        nxt_state = state[2];
                    end
                endcase
            end
            2'd3: begin
                nxt_state =  state[3];
                case(state[3])
                    NotTaken1: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = NotTaken2;
                        else nxt_state = NotTaken1;
                    end
                    NotTaken2: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = Taken1;
                        else if(PreWrong == 2'b00) nxt_state = NotTaken1;
                        else nxt_state =  state[3];
                    end
                    Taken1: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = NotTaken2;
                        else if(PreWrong == 2'b00) nxt_state = Taken2;
                        else nxt_state =  state[3];
                    end
                    Taken2: begin
                        if((PreWrong == 2'b01) || (PreWrong == 2'b11)) nxt_state = Taken1;
                        else nxt_state = Taken2;
                    end
                    default: begin
                        nxt_state = state[3];
                    end
                endcase
            end
            default: begin
                nxt_state =  NotTaken1;
            end
        endcase
    end
    //Control BrPre
    always@(*) begin
        BrPre = 1'b0;
        case(BranchPC_IF)
            2'd0: begin
                case(state[0])
                    NotTaken1: begin
                        BrPre = 1'b0;
                    end
                    NotTaken2: begin
                        BrPre = 1'b0;
                    end
                    Taken1: begin
                        BrPre = 1'b1 & BranchSignal;
                    end
                    Taken2: begin
                        BrPre = 1'b1 & BranchSignal;
                    end
                    default: begin
                        BrPre = 1'b0;
                    end
                endcase
            end
            2'd1: begin
                case(state[1])
                    NotTaken1: begin
                        BrPre = 1'b0;
                    end
                    NotTaken2: begin
                        BrPre = 1'b0;
                    end
                    Taken1: begin
                        BrPre = 1'b1 & BranchSignal;
                    end
                    Taken2: begin
                        BrPre = 1'b1 & BranchSignal;
                    end
                    default: begin
                        BrPre = 1'b0;
                    end
                endcase
            end
            2'd2: begin
                case(state[2])
                    NotTaken1: begin
                        BrPre = 1'b0;
                    end
                    NotTaken2: begin
                        BrPre = 1'b0;
                    end
                    Taken1: begin
                        BrPre = 1'b1 & BranchSignal;
                    end
                    Taken2: begin
                        BrPre = 1'b1 & BranchSignal;
                    end
                    default: begin
                        BrPre = 1'b0;
                    end
                endcase
            end
            2'd3: begin
                case(state[3])
                    NotTaken1: begin
                        BrPre = 1'b0;
                    end
                    NotTaken2: begin
                        BrPre = 1'b0;
                    end
                    Taken1: begin
                        BrPre = 1'b1 & BranchSignal;
                    end
                    Taken2: begin
                        BrPre = 1'b1 & BranchSignal;
                    end
                    default: begin
                        BrPre = 1'b0;
                    end
                endcase
            end
            default: BrPre = 1'b0;
        endcase
    end
    //==== sequential circuit =================================
    always @(posedge clk) begin
        if(!rst_n) begin
            for(i = 0; i <= 4; i = i + 1) state[i] <= NotTaken1;
        end
        else if(stall) begin
            for(i = 0; i <= 4; i = i + 1) state[i] <= state_old[i];
        end
        else begin
            case(BranchPC_ID)
                2'd0: state[0] <= nxt_state;
                2'd1: state[1] <= nxt_state;
                2'd2: state[2] <= nxt_state;
                2'd3: state[3] <= nxt_state;
                default: state[0] <= nxt_state;
            endcase
        end
    end
endmodule

module Comparater(  BrPre,
                    equal,
                    Ctrl_Br,
                    PreWrong
);
    //==== input/output definition ============================
    input               BrPre, equal;
    input         [1:0] Ctrl_Br;    // 00 don't branch, 01 beq, 10 bne
    output reg    [1:0] PreWrong;   // 00 pre right, 01 if predicted branch but actually doesn't have to branch, 10 not branch, 11 if predicted not branch but actually has to branch
    //==== combinational circuit ==============================
    always @(*) begin
        PreWrong = 2'b10;
        if(BrPre && ((Ctrl_Br==2'b01 && equal==1'b0) || (Ctrl_Br==2'b10 && equal==1'b1))) begin
            // if predicted branch but actually doesn't have to branch
            PreWrong = 2'b01;
        end
        else if((~BrPre) && ((Ctrl_Br==2'b01 && equal==1'b1) || (Ctrl_Br==2'b10 && equal==1'b0))) begin
            // if predicted not branch but actually has to branch
            PreWrong = 2'b11;
        end
        else if(Ctrl_Br==2'b01 || Ctrl_Br==2'b10) begin
            // predict right
            PreWrong = 2'b00;
        end
        else begin
            PreWrong = 2'b10;
        end
    end
endmodule