module HazardDetection(
    id_ex_MemRead,
    id_ex_rt,
    if_id_rs,
    if_id_rt,
    PCWrite,
    if_id_Write,
    mux_Ctrl
);
//==== io Declaration =========================
    input            id_ex_MemRead;
    input      [4:0] id_ex_rt, if_id_rs, if_id_rt;
    output reg       PCWrite, if_id_Write;
    output reg       mux_Ctrl; //1'b0 ctrl all zero, 1'b1 original

    always @(*) begin
        PCWrite     = 1'b1;
        if_id_Write = 1'b1;
        mux_Ctrl    = 1'b1;

        if (id_ex_MemRead && ((id_ex_rt == if_id_rs) || (id_ex_rt == if_id_rt))) begin
            PCWrite     = 1'b0;
            if_id_Write = 1'b0;
            mux_Ctrl    = 1'b0;
        end 
        else begin
            PCWrite     = 1'b1;
            if_id_Write = 1'b1;
            mux_Ctrl    = 1'b1;
        end
    end
endmodule

module ForwardUnit(
    ex_mem_rd,
    mem_wb_rd,
    id_ex_rs,
    id_ex_rt,
    ex_mem_RegWrite,
    mem_wb_RegWrite,
    ForwardA,
    ForwardB
);

input      [4:0] ex_mem_rd, mem_wb_rd, id_ex_rs, id_ex_rt;
input            ex_mem_RegWrite, mem_wb_RegWrite;
output reg [1:0] ForwardA, ForwardB; //00 original, 01 forward from mem/wb, 10 forward from ex/mem

    always @(*) begin
        ForwardA = 2'b0;
        ForwardB = 2'b0;

        if (mem_wb_RegWrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rs) &&
        !(ex_mem_RegWrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs))) 
        begin
            ForwardA = 2'b01;   //forward from mem/wb
        end 
        else if (ex_mem_RegWrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs)) begin
            ForwardA = 2'b10;   //forward from ex/mem
        end
        else ForwardA = 2'b00;  //original

        if (mem_wb_RegWrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rt) &&
        !(ex_mem_RegWrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rt))) 
        begin
            ForwardB = 2'b01;   //forward from mem/wb
        end
        else if (ex_mem_RegWrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rt)) begin
            ForwardB = 2'b10;   //forward from ex/mem

        end
        else ForwardB = 2'b00;  //original
    end
endmodule