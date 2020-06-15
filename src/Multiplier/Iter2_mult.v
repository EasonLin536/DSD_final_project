/* Use 2 cycles to complete calculation */
module Iter2Multiplier(
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
    reg  [31:0] mplier_r, mcand_r;
    wire [31:0] mplier_w, mcand_w;
    reg   [1:0] state, state_next;
    reg   [4:0] op_cnt_r; // uses 32 cycles to complete calculation
    wire [31:0] op_cnt_w;
    reg  [63:0] product_r, product_w;
    wire [63:0] partial_product;
    reg  [63:0] partial_temp0, partial_temp1, partial_temp2, partial_temp3,
                partial_temp4, partial_temp5, partial_temp6, partial_temp7,
                partial_temp8, partial_temp9, partial_temp10, partial_temp11,
                partial_temp12, partial_temp13, partial_temp14, partial_temp15;
    wire  [4:0] idx0, idx1, idx2, idx3, idx4, idx5, idx6, idx7,
                idx8, idx9, idx10, idx11, idx12, idx13, idx14, idx15;

//==== FSM ====================================
    parameter S_IDLE = 2'd0;
    parameter S_OP   = 2'd1;
    parameter S_END  = 2'd2;

    always @(*) begin
        case (state)
            S_IDLE:  state_next = (in_valid)? S_OP : S_IDLE;
            S_OP:    state_next = (op_cnt_r == 16)? S_END : S_OP;
            S_END:   state_next = S_IDLE;
            default: state_next = S_IDLE;
        endcase
    end

//==== Combinational ==========================
    // output
    assign product = product_r;
    
    // index for processing 
    assign idx0  = op_cnt_r;
    assign idx1  = op_cnt_r + 1;
    assign idx2  = op_cnt_r + 2;
    assign idx3  = op_cnt_r + 3;
    assign idx4  = op_cnt_r + 4;
    assign idx5  = op_cnt_r + 5;
    assign idx6  = op_cnt_r + 6;
    assign idx7  = op_cnt_r + 7;
    assign idx8  = op_cnt_r + 8;
    assign idx9  = op_cnt_r + 9;
    assign idx10 = op_cnt_r + 10;
    assign idx11 = op_cnt_r + 11;
    assign idx12 = op_cnt_r + 12;
    assign idx13 = op_cnt_r + 13;
    assign idx14 = op_cnt_r + 14;
    assign idx15 = op_cnt_r + 15;

    // sum up partial_temps
    assign partial_product = (((partial_temp0 + partial_temp1) +
                             (partial_temp2 + partial_temp3)) +
                             ((partial_temp4 + partial_temp5) +
                             (partial_temp6 + partial_temp7))) +
                             (((partial_temp8 + partial_temp9) +
                             (partial_temp10 + partial_temp11)) +
                             ((partial_temp12 + partial_temp13) +
                             (partial_temp14 + partial_temp15)));

    // mplier, mcand
    assign mplier_w = (in_valid)? mplier : mplier_r;
    assign mcand_w  = (in_valid)? mcand :  mcand_r;

    // opt_cnt
    assign op_cnt_w = (state == S_OP)? op_cnt_r + 8 : 0;

    // partial product
    always @(*) begin
        if (state == S_OP) begin
            partial_temp0  = (mplier_r[idx0])?  { 32'd0, mcand_r } << idx0  : 0;
            partial_temp1  = (mplier_r[idx1])?  { 32'd0, mcand_r } << idx1  : 0;
            partial_temp2  = (mplier_r[idx2])?  { 32'd0, mcand_r } << idx2  : 0;
            partial_temp3  = (mplier_r[idx3])?  { 32'd0, mcand_r } << idx3  : 0;
            partial_temp4  = (mplier_r[idx4])?  { 32'd0, mcand_r } << idx4  : 0;
            partial_temp5  = (mplier_r[idx5])?  { 32'd0, mcand_r } << idx5  : 0;
            partial_temp6  = (mplier_r[idx6])?  { 32'd0, mcand_r } << idx6  : 0;
            partial_temp7  = (mplier_r[idx7])?  { 32'd0, mcand_r } << idx7  : 0;
            partial_temp8  = (mplier_r[idx8])?  { 32'd0, mcand_r } << idx8  : 0;
            partial_temp9  = (mplier_r[idx9])?  { 32'd0, mcand_r } << idx9  : 0;
            partial_temp10 = (mplier_r[idx10])? { 32'd0, mcand_r } << idx10 : 0;
            partial_temp11 = (mplier_r[idx11])? { 32'd0, mcand_r } << idx11 : 0;
            partial_temp12 = (mplier_r[idx12])? { 32'd0, mcand_r } << idx12 : 0;
            partial_temp13 = (mplier_r[idx13])? { 32'd0, mcand_r } << idx13 : 0;
            partial_temp14 = (mplier_r[idx14])? { 32'd0, mcand_r } << idx14 : 0;
            partial_temp15 = (mplier_r[idx15])? { 32'd0, mcand_r } << idx15 : 0;
        end
        else begin
            partial_temp0  = 0;
            partial_temp1  = 0;
            partial_temp2  = 0;
            partial_temp3  = 0;
            partial_temp4  = 0;
            partial_temp5  = 0;
            partial_temp6  = 0;
            partial_temp7  = 0;
            partial_temp8  = 0;
            partial_temp9  = 0;
            partial_temp10 = 0;
            partial_temp11 = 0;
            partial_temp12 = 0;
            partial_temp13 = 0;
            partial_temp14 = 0;
            partial_temp15 = 0;
        end
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
            op_cnt_r  <= 0;
            product_r <= 0;
            mplier_r  <= 0;
            mcand_r   <= 0;
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