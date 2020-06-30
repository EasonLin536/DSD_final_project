module HazardDetection(
    id_ex_MemRead,
    id_ex_RegWrite,
    id_ex_rt,
    id_ex_rd,
    if_id_rs,
    if_id_rt,
    //Branch_ID,
    opcode_ID,
    PCWrite,
    if_id_Write,
    mux_Ctrl
);

input           id_ex_MemRead;
input           id_ex_RegWrite;
input     [4:0] id_ex_rt, id_ex_rd, if_id_rs, if_id_rt;
//input     [1:0] Branch_ID;
input     [5:0] opcode_ID;
output reg      PCWrite, if_id_Write;
output reg      mux_Ctrl; //1'b0 ctrl all zero, 1'b1 original

always@ (*) begin
    PCWrite = 1'b1;
    if_id_Write = 1'b1;
    mux_Ctrl = 1'b1;

    if((id_ex_MemRead && ((id_ex_rt == if_id_rs) || (id_ex_rt == if_id_rt))))
    begin
        PCWrite = 1'b0;
        if_id_Write = 1'b0;
        mux_Ctrl = 1'b0;
    end 
    else begin
        PCWrite = 1'b1;
        if_id_Write = 1'b1;
        mux_Ctrl = 1'b1;
    end
end
endmodule

module ForwardUnit(
    ex_mem_rd,
    ex_mem_rt,
    mem_wb_rd,
    if_id_rs,
    if_id_rt,
    id_ex_rs,
    id_ex_rt,
    id_ex_rd,
    id_ex_RegWrite,
    ex_mem_RegWrite,
    mem_wb_RegWrite,
    ex_mem_MemWrite,
    //Branch_ID,
    opcode_ID,
    ForwardA,
    ForwardB,
    ForwardA_ID,
    ForwardB_ID,
    ForwardWriteData_MEM
);

input      [4:0] ex_mem_rd, ex_mem_rt, mem_wb_rd, if_id_rs, if_id_rt, id_ex_rs, id_ex_rt, id_ex_rd;
input            id_ex_RegWrite, ex_mem_RegWrite, mem_wb_RegWrite;
input            ex_mem_MemWrite;
//input      [1:0] Branch_ID;
input      [5:0] opcode_ID;
output reg [1:0] ForwardA, ForwardB; //00 original, 01 forward from mem/wb, 10 forward from ex/mem
output reg       ForwardA_ID, ForwardB_ID;
output reg       ForwardWriteData_MEM;

always@(*) begin
    ForwardA = 2'b0;
    ForwardB = 2'b0;

    if(mem_wb_RegWrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rs) &&
     !(ex_mem_RegWrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs))) 
    begin
        ForwardA = 2'b01;   //forward from mem/wb
    end 
    else if(ex_mem_RegWrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs)) begin
        ForwardA = 2'b10;   //forward from ex/mem
    end
    else ForwardA = 2'b00;  //original

    if(mem_wb_RegWrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rt) &&
     !(ex_mem_RegWrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rt))) 
    begin
        ForwardB = 2'b01;   //forward from mem/wb
    end
    else if(ex_mem_RegWrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rt)) begin
        ForwardB = 2'b10;   //forward from ex/mem

    end
    else ForwardB = 2'b00;  //original
end

// deal with branch 
always@(*) begin
    ForwardA_ID = 1'b0;
    ForwardB_ID = 1'b0;

    if(id_ex_RegWrite && ((opcode_ID==4'h4) || (opcode_ID==4'h5)) && (id_ex_rd != 5'd0) && (id_ex_rd == if_id_rs)) begin
        ForwardA_ID = 1'b1;
    end
    else ForwardA_ID = 1'b0;

    if(id_ex_RegWrite && ((opcode_ID==4'h4) || (opcode_ID==4'h5)) && (id_ex_rd != 5'd0) && (id_ex_rd == if_id_rt)) begin
        ForwardB_ID = 1'b1;
    end
    else ForwardB_ID = 1'b0;
end

always@(*) begin
    ForwardWriteData_MEM = 1'b0;

    if(mem_wb_RegWrite && ex_mem_MemWrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == ex_mem_rt)) begin
        //deal with save hazard
        ForwardWriteData_MEM = 1'b1;
    end
    else ForwardWriteData_MEM = 1'b0;
end
endmodule