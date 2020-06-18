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
    // reg [31:0] remainder_r, remainder_w;
    // reg [31:0] quotient_r, quotient_w;

    reg [31:0] q, next_q; // Working Quotient
    reg [31:0] t, next_t; // Working Term
    reg [62:0] r, next_r; // Working Remainder
    reg [62:0] p, next_p; // Working Product

    assign remainder = r[31:0];
    assign quotient = q;

//==== FSM ====================================
    parameter S_IDLE = 2'd0;
    parameter S_OP   = 2'd1;
    parameter S_END  = 2'd2;

    always @(*) begin
        case (state)
            S_IDLE:  state_next = (in_valid)? S_OP : S_IDLE;
            S_OP:    state_next = (t == 1)? S_END : S_OP;
            S_END:   state_next = S_IDLE;
            default: state_next = S_IDLE;
        endcase
    end

//==== Combinational ==========================
    // dividend, divisor
    always @(*) begin
        dividend_w = (in_valid)? dividend : dividend_r;
        divisor_w  = (in_valid)? divisor :  divisor_r;
    end

    always @(*) begin
        case (state)
            S_IDLE: begin
                next_t = 32'h80000000;        // t = 2^31
                next_p = { divisor, 31'd0 };  // p = divisor*2^31
                next_q = 32'd0;               // q = 0
                next_r = { 31'd0, dividend }; // r = dividend
            end
            S_OP: begin
                next_t = t >> 1; // t = t/2
                next_p = p >> 1; // p = p/2
                if (p > r) begin
                    next_q = q;
                    next_r = r;
                end
                else begin
                    next_q = q + t;
                    next_r = r - p;
                end
            end
            S_END: begin
                next_t = t;
                next_p = p;
                next_q = q;              
                next_r = r;
            end
            default: begin
                next_t = 32'd0;
                next_p = 64'd0;
                next_q = 32'd0;              
                next_r = 64'd0;
            end
        endcase
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
            // quotient_r  <= 32'd0;
            // remainder_r <= 32'd0;
            dividend_r  <= 32'd0;
            divisor_r   <= 32'd0;
            q <= 32'd0;
            r <= 63'd0;
            p <= 63'd0;
            t <= 32'd0;
        end
        else begin
            state       <= state_next;
            // quotient_r  <= quotient_w;
            // remainder_r <= remainder_w;
            dividend_r  <= dividend_w;
            divisor_r   <= divisor_w;
            q <= next_q;
            r <= next_r;
            p <= next_p;
            t <= next_t;
        end
    end
endmodule