/* Use 32 cycles to complete calculation */
module Iter32Multiplier(
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
    reg  [4:0] op_cnt_r, op_cnt_w; // uses 32 cycles to complete calculation
    reg [63:0] product_r, product_w;
    reg [63:0] partial_product, partial_temp;

    assign product = product_r;

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
    // mplier, mcand
    always @(*) begin
        mplier_w = (in_valid)? mplier : mplier_r;
        mcand_w  = (in_valid)? mcand :  mcand_r;
    end

    // opt_cnt
    always @(*) begin
        op_cnt_w = (state == S_OP)? op_cnt_r + 1 : 5'd0;
    end

    // partial product
    always @(*) begin
        if (state == S_OP) begin
            partial_temp[31:0] = (mplier_r[op_cnt_r])? mcand_r : 32'd0;
        end
        else begin
            partial_temp = 32'd0;
        end
    end

    always @(*) begin
        partial_product = (state == S_OP)? partial_temp << op_cnt_r : 32'd0;
    end

    // product
    always @(*) begin
        case (state)
            S_IDLE:  product_w = 64'd0;
            S_OP:    product_w = product_r + partial_product;
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
            op_cnt_r  <= 5'd0;
            product_r <= 64'd0;
            mplier_r  <= 32'd0;
            mcand_r   <= 32'd0;
        end
        else begin
            state     <= state_next;
            op_cnt_r  <= op_cnt_w;
            product_r <= product_w;
            mplier_r  <= mplier_w;
            mcand_r   <= mcand_w;
        end
    end
endmodule