/* Use 32 cycles to complete calculation */
module Iter1Multiplier(
    clk,
    rst_n,
    in_valid,
    mplier,
    mcand,
    product,
    out_valid, // tell the processor to fetch output
    stall // tell the processor to stall cycles
);
//==== io Declaration =========================
    input             clk, rst_n, in_valid;
    input      [31:0] mplier, mcand;
    output     [63:0] product;
    output reg        out_valid, stall;

//==== Reg/Wire Declaration ===================
    reg [31:0] mplier_r, mplier_w;
    reg [31:0] mcand_r, mcand_w;
    reg  [1:0] state, state_next;
    reg [63:0] product_r, product_w;

    assign product = product_r;

//==== FSM ====================================
    parameter S_IDLE = 2'd0;
    parameter S_OP   = 2'd1;
    parameter S_END  = 2'd2;

    always @(*) begin
        case (state)
            S_IDLE:  state_next = (in_valid)? S_OP : S_IDLE;
            S_OP:    state_next = S_END;
            S_END:   state_next = S_IDLE;
            default: state_next = S_IDLE;
        endcase
    end

//==== Combinational ==========================
    // mplier, mcand
    always @(*) begin
        mplier_w = (in_valid)? mplier : mplier_r;
        mcand_w  = (in_valid)? mcand :  mcand_r;
    end

    // product
    always @(*) begin
        case (state)
            S_IDLE:  product_w = 64'd0;
            S_OP:    product_w = mplier_r * mcand_r;
            S_END:   product_w = product_r;
            default: product_w = 64'd0;
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
            state     <= S_IDLE;
            product_r <= 64'd0;
            mplier_r  <= 32'd0;
            mcand_r   <= 32'd0;
        end
        else begin
            state     <= state_next;
            product_r <= product_w;
            mplier_r  <= mplier_w;
            mcand_r   <= mcand_w;
        end
    end
endmodule