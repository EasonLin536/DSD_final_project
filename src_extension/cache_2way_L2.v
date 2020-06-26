`define L2_INDEX_LEN 5
`define L2_TAG_LEN   28 - `L2_INDEX_LEN
`define L2_BLOCK_NUM 2 ** `L2_INDEX_LEN
`define L2_DATA_LEN  130 + `L2_TAG_LEN

module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output         proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output         mem_read, mem_write;
    output  [27:0] mem_addr;
    output [127:0] mem_wdata;
//==== state definition ===================================
    parameter IDLE     = 3'd0;
    parameter COMPARE  = 3'd1;
    parameter ALLOCATE = 3'd2;
    parameter WB       = 3'd3;
//==== wire/reg definition ================================
    // io proc
    reg     [31:0] proc_rdata;
    reg            proc_stall;
    // state, cache
    reg    [155:0] cache_dm[0:3][0:1], next_cache_dm[0:3][0:1];
    reg      [7:0] LRU, next_LRU;
    reg      [2:0] state, next_state;
    // proc addr
    wire     [1:0] index;
    wire    [25:0] tag;
    // data choose
    wire           dirty[0:1];
    wire           valid[0:1];
    wire    [25:0] cache_tag[0:1];
    // COMPARE
    wire           hit[0:1];
    wire     [1:0] word_idx;
    // L2
    wire          L1_reset;
    wire          L1_stall;
    wire  [127:0] L1_rdata;
    reg           L1_read, next_L1_read;
    reg           L1_write, next_L1_write;
    reg    [27:0] L1_addr, next_L1_addr;
    reg   [127:0] L1_wdata, next_L1_wdata;
// ==== L2 ==============================================
    L2 L2_1(
        .clk(clk),
        .proc_reset(L1_reset),
        .proc_read(L1_read),
        .proc_write(L1_write),
        .proc_addr(L1_addr),
        .proc_rdata(L1_rdata),
        .proc_wdata(L1_wdata),
        .proc_stall(L1_stall),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),
        .mem_wdata(mem_wdata),
        .mem_ready(mem_ready)
    );
    assign L1_reset = proc_reset;
//==== combinational circuit ==============================
    // io proc
    // io mem
    // proc addr
    assign index = proc_addr[3:2];
    assign tag = proc_addr[29:4];
    // data choose
    assign dirty[0] = cache_dm[index][0][155];
    assign valid[0] = cache_dm[index][0][154];
    assign cache_tag[0] = cache_dm[index][0][153:128];
    assign dirty[1] = cache_dm[index][1][155];
    assign valid[1] = cache_dm[index][1][154];
    assign cache_tag[1] = cache_dm[index][1][153:128];
    // COMPARE
    assign hit[0] = (tag == cache_tag[0]);
    assign hit[1] = (tag == cache_tag[1]);
    assign word_idx = proc_addr[1:0];
    
    integer i;
    always @(*) begin
        for(i = 0; i < 4; i = i + 1) begin
            next_cache_dm[i][0] = cache_dm[i][0];
            next_cache_dm[i][1] = cache_dm[i][1];
        end
        next_LRU = LRU;
        next_state = state;
        proc_stall = 0;
        proc_rdata = 0;
        next_L1_read = L1_read;
        next_L1_write = L1_write;
        next_L1_addr = L1_addr;
        next_L1_wdata = L1_wdata;
        case(state)
            IDLE: begin
                if(!proc_reset) begin
                    next_state = COMPARE;
                end
                else begin
                    next_state = IDLE;
                end
            end
            COMPARE: begin
                if(~(proc_read | proc_write)) next_state = COMPARE;
                else if(valid[0] && hit[0]) begin // cache hit 0
                    next_LRU[index] = 1;
                    next_state = COMPARE;
                    if(proc_read) begin
                        case(word_idx)
                            2'b00: begin
                                proc_rdata = cache_dm[index][0][31:0];
                            end
                            2'b01: begin
                                proc_rdata = cache_dm[index][0][63:32];
                            end
                            2'b10: begin
                                proc_rdata = cache_dm[index][0][95:64];
                            end
                            2'b11: begin
                                proc_rdata = cache_dm[index][0][127:96];
                            end
                            default: proc_rdata = 0;
                        endcase
                    end
                    else if(proc_write) begin
                        next_cache_dm[index][0][155] = 1; // set dirty
                        case(word_idx)
                            2'b00: begin
                                next_cache_dm[index][0][31:0] = proc_wdata;
                            end
                            2'b01: begin
                                next_cache_dm[index][0][63:32] = proc_wdata;
                            end
                            2'b10: begin
                                next_cache_dm[index][0][95:64] = proc_wdata;
                            end
                            2'b11: begin
                                next_cache_dm[index][0][127:96] = proc_wdata;
                            end
                            default: begin
                                next_cache_dm[index][0] = 0;
                            end
                        endcase
                    end
                end
                else if(valid[1] && hit[1]) begin // cache hit 1
                    next_LRU[index] = 0;
                    next_state = COMPARE;
                    if(proc_read) begin
                        case(word_idx)
                            2'b00: begin
                                proc_rdata = cache_dm[index][1][31:0];
                            end
                            2'b01: begin
                                proc_rdata = cache_dm[index][1][63:32];
                            end
                            2'b10: begin
                                proc_rdata = cache_dm[index][1][95:64];
                            end
                            2'b11: begin
                                proc_rdata = cache_dm[index][1][127:96];
                            end
                            default: proc_rdata = 0;
                        endcase
                    end
                    else if(proc_write) begin
                        next_cache_dm[index][1][155] = 1; // set dirty
                        case(word_idx)
                            2'b00: begin
                                next_cache_dm[index][1][31:0] = proc_wdata;
                            end
                            2'b01: begin
                                next_cache_dm[index][1][63:32] = proc_wdata;
                            end
                            2'b10: begin
                                next_cache_dm[index][1][95:64] = proc_wdata;
                            end
                            2'b11: begin
                                next_cache_dm[index][1][127:96] = proc_wdata;
                            end
                            default: begin
                                next_cache_dm[index][1] = 0;
                            end
                        endcase
                    end
                end
                else if(valid[LRU[index]] && dirty[LRU[index]]) begin // cache miss && (block valid && dirty)
                    next_state = WB;
                    proc_stall = 1;
                    next_L1_write = 1;
                    next_L1_wdata = cache_dm[index][LRU[index]];
                    next_L1_addr = {cache_tag[LRU[index]], index};
                end
                else begin // cache miss && (block clean or invalid)
                    next_state = ALLOCATE;
                    proc_stall = 1;
                    next_L1_read = 1;
                    next_L1_addr = proc_addr[29:2];
                end
            end
            ALLOCATE: begin
                if(!L1_stall) begin
                    next_state = COMPARE;
                    if(proc_write)
                        proc_stall = 1;
                    else
                        proc_stall = 0;
                    next_L1_read = 0;
                    next_cache_dm[index][LRU[index]][155] = 1'b0; // set dirty
                    next_cache_dm[index][LRU[index]][154] = 1'b1; // set valid
                    next_cache_dm[index][LRU[index]][153:128] = tag; // set tag
                    next_cache_dm[index][LRU[index]][127:0] = L1_rdata;
                    case(word_idx)
                        2'b00: begin
                            proc_rdata = L1_rdata[31:0];
                        end
                        2'b01: begin
                            proc_rdata = L1_rdata[63:32];
                        end
                        2'b10: begin
                            proc_rdata = L1_rdata[95:64];
                        end
                        2'b11: begin
                            proc_rdata = L1_rdata[127:96];
                        end
                        default: proc_rdata = 0;
                    endcase
                    next_LRU[index] = ~LRU[index];
                end
                else begin
                    next_state = ALLOCATE;
                    proc_stall = 1;
                    next_L1_read = 1;
                end
            end
            WB: begin
                proc_stall = 1;
                if(!L1_stall) begin
                    next_L1_write = 0;
                    next_L1_addr = proc_addr[29:2];
                    next_state = ALLOCATE;
                end
                else begin
                    next_L1_write = 1;
                    next_state = WB;
                end
            end
            default: begin
                proc_stall = 0;
            end
        endcase
    end
//==== sequential circuit =================================
    integer j;
    always @( posedge clk ) begin
        if( proc_reset ) begin
            for(j = 0; j < 4; j = j + 1) begin
                cache_dm[j][0] <= 0;
                cache_dm[j][1] <= 0;
            end
            LRU <= 0;
            state <= IDLE;
            // data_ready <= 0;
            L1_read <= 0;
            L1_write <= 0;
            L1_addr <= 0;
            L1_wdata <= 0;
        end
        else begin
            for(j = 0; j < 4; j = j + 1) begin 
                cache_dm[j][0] <= next_cache_dm[j][0];
                cache_dm[j][1] <= next_cache_dm[j][1];
            end
            LRU <= next_LRU;
            state <= next_state;
            // data_ready <= L1_ready;
            L1_read <= next_L1_read;
            L1_write <= next_L1_write;
            L1_addr <= next_L1_addr;
            L1_wdata <= next_L1_wdata;
        end
    end

endmodule

module L2(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
//==== input/output definition ============================
    input                    clk;
    // processor interface
    input                    proc_reset;
    input                    proc_read, proc_write;
    input             [27:0] proc_addr;
    input            [127:0] proc_wdata;
    output                   proc_stall;
    output           [127:0] proc_rdata;
    // memory interface
    input            [127:0] mem_rdata;
    input                    mem_ready;
    output                   mem_read, mem_write;
    output            [27:0] mem_addr;
    output           [127:0] mem_wdata;
//==== state definition ===================================
    parameter IDLE     = 3'd0;
    parameter COMPARE  = 3'd1;
    parameter ALLOCATE = 3'd2;
    parameter WB       = 3'd3;
//==== wire/reg definition ================================
    // io proc
    reg              [127:0] proc_rdata;
    reg                      proc_stall;
    // io mem
    reg                      data_ready;
    reg                      mem_read, mem_write;
    reg               [27:0] mem_addr, next_mem_addr;
    reg              [127:0] mem_wdata, next_mem_wdata;
    // state, cache
    reg   [`L2_DATA_LEN-1:0] cache_dm      [0:`L2_BLOCK_NUM-1];
    reg   [`L2_DATA_LEN-1:0] next_cache_dm [0:`L2_BLOCK_NUM-1];
    reg                [2:0] state, next_state;
    // proc addr
    wire [`L2_INDEX_LEN-1:0] index;
    wire   [`L2_TAG_LEN-1:0] tag;
    // data choose
    wire                     dirty;
    wire                     valid;
    wire   [`L2_TAG_LEN-1:0] cache_tag;
    // COMPARE
    wire                     hit;
//==== combinational circuit ==============================
    // io proc
    // io mem
    // proc addr
    assign index     = proc_addr[`L2_INDEX_LEN-1:0];
    assign tag       = proc_addr[27:`L2_INDEX_LEN];
    // data choose
    assign dirty     = cache_dm[index][`L2_DATA_LEN-1];
    assign valid     = cache_dm[index][`L2_DATA_LEN-2];
    assign cache_tag = cache_dm[index][127+`L2_TAG_LEN:128];
    // COMPARE
    assign hit       = (tag == cache_tag);
    
    integer i;
    always @(*) begin
        for(i = 0; i < `L2_BLOCK_NUM; i = i + 1) next_cache_dm[i] = cache_dm[i];
        next_state     = state;
        proc_stall     = 0;
        proc_rdata     = cache_dm[index];
        mem_read       = 0;
        mem_write      = 0;
        next_mem_addr  = mem_addr;
        next_mem_wdata = mem_wdata;
        case(state)
            IDLE: begin
                if (!proc_reset) begin
                    next_state = COMPARE;
                end
                else begin
                    next_state = IDLE;
                end
            end
            COMPARE: begin
                if (valid && hit) begin // cache hit
                    next_state = COMPARE;
                    if (proc_read) begin
                        proc_rdata = cache_dm[index];
                    end
                    else if (proc_write) begin
                        next_cache_dm[index][`L2_DATA_LEN-1] = 1; // set dirty
                        next_cache_dm[index][127:0]          = proc_wdata;
                    end
                end
                else if (valid && dirty) begin // cache miss && (block valid && dirty)
                    next_state     = WB;
                    proc_stall     = 1;
                    mem_write      = 1;
                    next_mem_wdata = cache_dm[index];
                    next_mem_addr  = { cache_tag, index };
                end
                else if (proc_read | proc_write) begin // cache miss && (block clean or invalid)
                    next_state    = ALLOCATE;
                    proc_stall    = 1;
                    mem_read      = 1;
                    next_mem_addr = proc_addr;
                end
            end
            ALLOCATE: begin
                if (data_ready) begin
                    next_state = COMPARE;
                    if (proc_write)
                        proc_stall = 1;
                    else
                        proc_stall = 0;
                    
                    mem_read                                  = 0;
                    next_cache_dm[index][`L2_DATA_LEN-1]      = 1'b0; // set dirty
                    next_cache_dm[index][`L2_DATA_LEN-2]      = 1'b1; // set valid
                    next_cache_dm[index][127+`L2_TAG_LEN:128] = tag; // set tag
                    next_cache_dm[index][127:0]               = mem_rdata;
                    proc_rdata                                = mem_rdata;
                end
                else begin
                    next_state = ALLOCATE;
                    proc_stall = 1;
                    mem_read   = 1;
                end
            end
            WB: begin
                proc_stall = 1;
                if (data_ready) begin
                    mem_write     = 0;
                    next_mem_addr = proc_addr;
                    next_state    = ALLOCATE;
                end
                else begin
                    mem_write  = 1;
                    next_state = WB;
                end
            end
            default: begin
                proc_stall = 0;
            end
        endcase
    end
//==== sequential circuit =================================
    integer j;
    always @(posedge clk) begin
        if (proc_reset) begin
            for (j = 0; j < `L2_BLOCK_NUM; j = j + 1) cache_dm[j] <= 0;
            state      <= IDLE;
            data_ready <= 0;
            mem_addr   <= 0;
            mem_wdata  <= 0;
        end
        else begin
            for (j = 0; j < `L2_BLOCK_NUM; j = j + 1) cache_dm[j] <= next_cache_dm[j];
            state      <= next_state;
            data_ready <= mem_ready;
            mem_addr   <= next_mem_addr;
            mem_wdata  <= next_mem_wdata;
        end
    end

endmodule