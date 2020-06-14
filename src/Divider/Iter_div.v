module IterDivider(
    clk,
    rst_n,
    in_valid,
    dividend,
    divisor,
    remainder, // store to $HI
    quotient, // store to $LO
    out_valid,
    stall
);
//==== io Declaration =========================
    input             clk, rst_n, in_valid;
    input      [31:0] dividend, divisor;
    output     [31:0] remainder, quotient;
    output reg        out_valid, stall;

//==== Reg/Wire Declaration ===================
    reg [31:0] dividend_r, dividend_w;
    reg [31:0] divisor_r, divisor_w;
    reg  [1:0] state, state_next;
    reg  [4:0] op_cnt_r, op_cnt_w; // uses 32 cycles to complete calculation
    reg [31:0] remainder_r, remainder_w;
    reg [31:0] quotient_r, quotient_w;
//==== FSM ====================================
    parameter S_IDLE = 2'd0;
    parameter S_OP   = 2'd1;
    parameter S_END  = 2'd2;

    always @(*) begin
        case (state)
            S_IDLE:  state_next = (in_valid)? S_OP : S_IDLE;
            S_OP:    state_next = (op_cnt_r == 31)? S_END : S_OP;
            S_END:   state_next = S_IDLE;
            default: state_next = S_IDLE;
        endcase
    end

//==== Combinational ==========================
    // dividend, dividor
    always @(*) begin
        dividend_w = (in_valid)? dividend : dividend_r;
        dividor_w  = (in_valid)? dividor :  dividor_r;
    end

    // opt_cnt
    always @(*) begin
        op_cnt_w = (state == S_OP)? op_cnt_r + 1 : 5'd0;
    end

    // quotient
    always @(*) begin
        
    end

    // remainder

    // product
    always @(*) begin
        
    end
    
    // out_valid && stall
    always @(*) begin
        out_valid = (state == S_END)? 1 : 0;
        stall = ((state == S_IDLE && !in_valid) | state == S_END)? 0 : 1;
    end

//==== Sequential =============================
    always @(posedge clk) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            op_cnt_r    <= 5'd0;
            quotient_r  <= 32'd0;
            remainder_r <= 32'd0;
            dividend_r  <= 32'd0;
            divisor_r   <= 32'd0;
        end
        else begin
            state       <= state_next;
            op_cnt_r    <= op_cnt_w;
            quotient_r  <= quotient_w;
            remainder_r <= remainder_w;
            dividend_r  <= dividend_w;
            divisor_r   <= divisor_w;
        end
    end
endmodule