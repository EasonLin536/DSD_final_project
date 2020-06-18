module BoothMultiplier(
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
    reg  [63:0] product_r, product_w;
    wire [63:0] partial_product;
    wire [33:0] extend_mplier;
    wire  [2:0] gp [0:16]; // groups of booth encoding
    reg  [63:0] pp [0:16];
    reg   [4:0] sh [0:16];
    
    integer i;
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
    // process input
    assign extend_mplier = { 1'b0, mplier_r, 1'b0 };
    // output
    assign product = product_r;
    // get groups
    assign gp[0]  = extend_mplier[2:0]; 
    assign gp[1]  = extend_mplier[4:2]; 
    assign gp[2]  = extend_mplier[6:4]; 
    assign gp[3]  = extend_mplier[8:6]; 
    assign gp[4]  = extend_mplier[10:8]; 
    assign gp[5]  = extend_mplier[12:10]; 
    assign gp[6]  = extend_mplier[14:12]; 
    assign gp[7]  = extend_mplier[16:14]; 
    assign gp[8]  = extend_mplier[18:16]; 
    assign gp[9]  = extend_mplier[20:18]; 
    assign gp[10] = extend_mplier[22:20]; 
    assign gp[11] = extend_mplier[24:22]; 
    assign gp[12] = extend_mplier[26:24]; 
    assign gp[13] = extend_mplier[28:26]; 
    assign gp[14] = extend_mplier[30:28]; 
    assign gp[15] = extend_mplier[32:30]; 
    assign gp[16] = extend_mplier[34:32];
    // caculate pp
    always @(*) begin
        for (i=0;i<17;i=i+1) begin
            case (gp[i])
                3'b000:  pp[i] = 0;
                3'b001:  pp[i] = mcand_r   << i*2;
                3'b010:  pp[i] = mcand_r   << i*2;
                3'b011:  pp[i] = mcand_r   << (i*2 + 1);
                3'b100:  pp[i] = -(mcand_r << (i*2 + 1));
                3'b101:  pp[i] = -(mcand_r << i*2);
                3'b110:  pp[i] = -(mcand_r << i*2);
                3'b111:  pp[i] = 0;
                default: pp[i] = 0;
            endcase
        end
    end
    // calculate sum
    // Wallace tree design
    CSA csa(
        pp[0], pp[1], pp[2], pp[3], pp[4], pp[5], pp[6],
        pp[7], pp[8], pp[9], pp[10], pp[11], pp[12],
        pp[13], pp[14], pp[15], pp[16],
        partial_product
    );
    // conventional design
    // assign partial_product = ((((pp[0]) + (pp[1])) + ((pp[2]) + (pp[3]))) + 
    //                          (((pp[4]) + (pp[5])) + ((pp[6]) + (pp[7])))) + 
    //                          ((((pp[8]) + (pp[9])) + ((pp[10]) + (pp[11]))) + 
    //                          (((pp[12]) + (pp[13])) + ((pp[14]) + (pp[15])))) + pp[16];

    // mplier, mcand
    assign mplier_w = (in_valid)? mplier : mplier_r;
    assign mcand_w  = (in_valid)? mcand :  mcand_r;
    
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
            // op_cnt_r  <= 0;
            product_r <= 0;
            mplier_r  <= 0;
            mcand_r   <= 0;
        end
        else begin
            state     <= state_next;
            // op_cnt_r  <= op_cnt_w;
            product_r <= product_w;
            mplier_r  <= mplier_w;
            mcand_r   <= mcand_w;
        end
    end
endmodule

module CSA(
    pp0, pp1, pp2, pp3, pp4, pp5, pp6,
    pp7, pp8, pp9, pp10, pp11, pp12,
    pp13, pp14, pp15, pp16,
    out
);
    input  [63:0] pp0, pp1, pp2, pp3, pp4, pp5, pp6,
                  pp7, pp8, pp9, pp10, pp11, pp12,
                  pp13, pp14, pp15, pp16;
    output [63:0] out;

    wire [63:0] co [0:14];
    reg  [63:0] sh [0:14];
    wire [63:0] s  [0:14];

    integer i;

    always @(*) begin
        for (i=0;i<15;i=i+1)
            sh[i] = co[i] << 1;
    end
                
    FA64 A0 (co[0],  s[0],  pp0,   pp1,   pp2);
    FA64 A1 (co[1],  s[1],  pp3,   pp4,   pp5);
    FA64 A2 (co[2],  s[2],  pp6,   pp7,   pp8);
    FA64 A3 (co[3],  s[3],  pp9,   pp10,  pp11);
    FA64 A4 (co[4],  s[4],  pp12,  pp13,  pp14);
    FA64 A5 (co[5],  s[5],  sh[0], s[0],  sh[1]);
    FA64 A6 (co[6],  s[6],  s[1],  sh[2], s[2]);
    FA64 A7 (co[7],  s[7],  sh[3], s[3],  sh[4]);
    FA64 A8 (co[8],  s[8],  s[4],  pp15,  pp16);
    FA64 A9 (co[9],  s[9],  sh[5], s[5],  sh[6]);
    FA64 A10(co[10], s[10], s[6], sh[7], s[7]);
    FA64 A11(co[11], s[11], sh[9], s[9], sh[10]);
    FA64 A12(co[12], s[12], s[10], sh[8], s[8]);
    FA64 A13(co[13], s[13], sh[11], s[11], sh[12]);
    FA64 A14(co[14], s[14], sh[13], s[13], s[12]);

    assign out = sh[14] + s[14];

endmodule

module FA64(co, s, a, b, ci);

    input  [63:0] a, b, ci;
	output [63:0] co, s;

    FA fa0 (co[0],  s[0],  a[0],  b[0],  ci[0]);
    FA fa1 (co[1],  s[1],  a[1],  b[1],  ci[1]);
    FA fa2 (co[2],  s[2],  a[2],  b[2],  ci[2]);
    FA fa3 (co[3],  s[3],  a[3],  b[3],  ci[3]);
    FA fa4 (co[4],  s[4],  a[4],  b[4],  ci[4]);
    FA fa5 (co[5],  s[5],  a[5],  b[5],  ci[5]);
    FA fa6 (co[6],  s[6],  a[6],  b[6],  ci[6]);
    FA fa7 (co[7],  s[7],  a[7],  b[7],  ci[7]);
    FA fa8 (co[8],  s[8],  a[8],  b[8],  ci[8]);
    FA fa9 (co[9],  s[9],  a[9],  b[9],  ci[9]);
    FA fa10(co[10], s[10], a[10], b[10], ci[10]);
    FA fa11(co[11], s[11], a[11], b[11], ci[11]);
    FA fa12(co[12], s[12], a[12], b[12], ci[12]);
    FA fa13(co[13], s[13], a[13], b[13], ci[13]);
    FA fa14(co[14], s[14], a[14], b[14], ci[14]);
    FA fa15(co[15], s[15], a[15], b[15], ci[15]);
    FA fa16(co[16], s[16], a[16], b[16], ci[16]);
    FA fa17(co[17], s[17], a[17], b[17], ci[17]);
    FA fa18(co[18], s[18], a[18], b[18], ci[18]);
    FA fa19(co[19], s[19], a[19], b[19], ci[19]);
    FA fa20(co[20], s[20], a[20], b[20], ci[20]);
    FA fa21(co[21], s[21], a[21], b[21], ci[21]);
    FA fa22(co[22], s[22], a[22], b[22], ci[22]);
    FA fa23(co[23], s[23], a[23], b[23], ci[23]);
    FA fa24(co[24], s[24], a[24], b[24], ci[24]);
    FA fa25(co[25], s[25], a[25], b[25], ci[25]);
    FA fa26(co[26], s[26], a[26], b[26], ci[26]);
    FA fa27(co[27], s[27], a[27], b[27], ci[27]);
    FA fa28(co[28], s[28], a[28], b[28], ci[28]);
    FA fa29(co[29], s[29], a[29], b[29], ci[29]);
    FA fa30(co[30], s[30], a[30], b[30], ci[30]);
    FA fa31(co[31], s[31], a[31], b[31], ci[31]);
    FA fa32(co[32], s[32], a[32], b[32], ci[32]);
    FA fa33(co[33], s[33], a[33], b[33], ci[33]);
    FA fa34(co[34], s[34], a[34], b[34], ci[34]);
    FA fa35(co[35], s[35], a[35], b[35], ci[35]);
    FA fa36(co[36], s[36], a[36], b[36], ci[36]);
    FA fa37(co[37], s[37], a[37], b[37], ci[37]);
    FA fa38(co[38], s[38], a[38], b[38], ci[38]);
    FA fa39(co[39], s[39], a[39], b[39], ci[39]);
    FA fa40(co[40], s[40], a[40], b[40], ci[40]);
    FA fa41(co[41], s[41], a[41], b[41], ci[41]);
    FA fa42(co[42], s[42], a[42], b[42], ci[42]);
    FA fa43(co[43], s[43], a[43], b[43], ci[43]);
    FA fa44(co[44], s[44], a[44], b[44], ci[44]);
    FA fa45(co[45], s[45], a[45], b[45], ci[45]);
    FA fa46(co[46], s[46], a[46], b[46], ci[46]);
    FA fa47(co[47], s[47], a[47], b[47], ci[47]);
    FA fa48(co[48], s[48], a[48], b[48], ci[48]);
    FA fa49(co[49], s[49], a[49], b[49], ci[49]);
    FA fa50(co[50], s[50], a[50], b[50], ci[50]);
    FA fa51(co[51], s[51], a[51], b[51], ci[51]);
    FA fa52(co[52], s[52], a[52], b[52], ci[52]);
    FA fa53(co[53], s[53], a[53], b[53], ci[53]);
    FA fa54(co[54], s[54], a[54], b[54], ci[54]);
    FA fa55(co[55], s[55], a[55], b[55], ci[55]);
    FA fa56(co[56], s[56], a[56], b[56], ci[56]);
    FA fa57(co[57], s[57], a[57], b[57], ci[57]);
    FA fa58(co[58], s[58], a[58], b[58], ci[58]);
    FA fa59(co[59], s[59], a[59], b[59], ci[59]);
    FA fa60(co[60], s[60], a[60], b[60], ci[60]);
    FA fa61(co[61], s[61], a[61], b[61], ci[61]);
    FA fa62(co[62], s[62], a[62], b[62], ci[62]);
    FA fa63(co[63], s[63], a[63], b[63], ci[63]);

endmodule

module FA(co, s, a, b, ci);
	
    input	a, b, ci;
	output	co, s;
	wire 	ab, bc, ca;

	// co = a*b + b*ci + ci*a
	and g0(ab, a, b);
	and g1(bc, b, ci);
	and g2(ca, ci, a);
	or 	g3(co, ab, bc, ca);
	xor g4(s, a, b, ci);

endmodule