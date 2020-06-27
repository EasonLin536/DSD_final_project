// Top module of your design, you cannot modify this module!!
module CHIP(	
	clk,
	rst_n,
//----------for slow_memD------------
	mem_read_D,
	mem_write_D,
	mem_addr_D,
	mem_wdata_D,
	mem_rdata_D,
	mem_ready_D,
//----------for slow_memI------------
	mem_read_I,
	mem_write_I,
	mem_addr_I,
	mem_wdata_I,
	mem_rdata_I,
	mem_ready_I,
//----------for TestBed--------------				
	DCACHE_addr, 
	DCACHE_wdata,
	DCACHE_wen   
);
    input			clk, rst_n;
    //--------------------------
    output			mem_read_D;
    output			mem_write_D;
    output	[31:4]	mem_addr_D;
    output	[127:0]	mem_wdata_D;
    input	[127:0]	mem_rdata_D;
    input			mem_ready_D;
    //--------------------------
    output			mem_read_I;
    output			mem_write_I;
    output	[31:4]	mem_addr_I;
    output	[127:0]	mem_wdata_I;
    input	[127:0]	mem_rdata_I;
    input			mem_ready_I;
//----------for TestBed--------------
    output	[29:0]	DCACHE_addr;
    output	[31:0]	DCACHE_wdata;
    output			DCACHE_wen;
    //--------------------------

    // wire declaration
    wire        ICACHE_ren;
    wire        ICACHE_wen;
    wire [29:0] ICACHE_addr;
    wire [31:0] ICACHE_wdata;
    wire        ICACHE_stall;
    wire [31:0] ICACHE_rdata;

    wire        DCACHE_ren;
    wire        DCACHE_wen;
    wire [29:0] DCACHE_addr;
    wire [31:0] DCACHE_wdata;
    wire        DCACHE_stall;
    wire [31:0] DCACHE_rdata;

//=========================================
	// Note that the overall design of your MIPS includes:
	// 1. pipelined MIPS processor
	// 2. data cache
	// 3. instruction cache

	MIPS_Pipeline i_MIPS(
		// control interface
		.clk            (clk)           , 
		.rst_n          (rst_n)         ,
	//----------I cache interface-------		
		.ICACHE_ren     (ICACHE_ren)    ,
		.ICACHE_wen     (ICACHE_wen)    ,
		.ICACHE_addr    (ICACHE_addr)   ,
		.ICACHE_wdata   (ICACHE_wdata)  ,
		.ICACHE_stall   (ICACHE_stall)  ,
		.ICACHE_rdata   (ICACHE_rdata)  ,
	//----------D cache interface-------
		.DCACHE_ren     (DCACHE_ren)    ,
		.DCACHE_wen     (DCACHE_wen)    ,
		.DCACHE_addr    (DCACHE_addr)   ,
		.DCACHE_wdata   (DCACHE_wdata)  ,
		.DCACHE_stall   (DCACHE_stall)  ,
		.DCACHE_rdata   (DCACHE_rdata)
	);
	
	cache D_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (DCACHE_ren)  ,
        .proc_write (DCACHE_wen)  ,
        .proc_addr  (DCACHE_addr) ,
        .proc_rdata (DCACHE_rdata),
        .proc_wdata (DCACHE_wdata),
        .proc_stall (DCACHE_stall),
        .mem_read   (mem_read_D)  ,
        .mem_write  (mem_write_D) ,
        .mem_addr   (mem_addr_D)  ,
        .mem_wdata  (mem_wdata_D) ,
        .mem_rdata  (mem_rdata_D) ,
        .mem_ready  (mem_ready_D)
	);

	cache I_cache(
        .clk        (clk)         ,
        .proc_reset (~rst_n)      ,
        .proc_read  (ICACHE_ren)  ,
        .proc_write (ICACHE_wen)  ,
        .proc_addr  (ICACHE_addr) ,
        .proc_rdata (ICACHE_rdata),
        .proc_wdata (ICACHE_wdata),
        .proc_stall (ICACHE_stall),
        .mem_read   (mem_read_I)  ,
        .mem_write  (mem_write_I) ,
        .mem_addr   (mem_addr_I)  ,
        .mem_wdata  (mem_wdata_I) ,
        .mem_rdata  (mem_rdata_I) ,
        .mem_ready  (mem_ready_I)
	);

endmodule

module MIPS_Pipeline(
	// control interface
	clk           , 
	rst_n         ,
//----------I cache interface-------		
	ICACHE_ren    ,
	ICACHE_wen    ,
	ICACHE_addr   ,
	ICACHE_wdata  ,
	ICACHE_stall  ,
	ICACHE_rdata  ,
//----------D cache interface-------
	DCACHE_ren    ,
	DCACHE_wen    ,
	DCACHE_addr   ,
	DCACHE_wdata  ,
	DCACHE_stall  ,
	DCACHE_rdata
);
//==== io Declaration =========================
    input         clk;
    input         rst_n;

    output        ICACHE_ren;
    output        ICACHE_wen;
    output [29:0] ICACHE_addr;
    output [31:0] ICACHE_wdata;
    input         ICACHE_stall;
    input  [31:0] ICACHE_rdata;

    output        DCACHE_ren;
    output        DCACHE_wen;
    output [29:0] DCACHE_addr;
    output [31:0] DCACHE_wdata;
    input         DCACHE_stall;
    input  [31:0] DCACHE_rdata;
//==== Reg/Wire Declaration ===================
    // NET
    // PC wire
    reg  [31:0] PC_r;
    wire [31:0] PC_w;

    wire [31:0] instruction_IF;
    reg  [31:0] instruction_ID;
    // for debug
    reg  [31:0] PC_ID, PC_EX, PC_MEM, PC_WB;
    reg  [31:0] instruction_EX, instruction_MEM, instruction_WB;
    // end debug

    wire [31:0] PC_plus4_IF;
    reg  [31:0] PC_plus4_ID, PC_plus4_EX, PC_plus4_MEM, PC_plus4_WB;

    wire [31:0] jump_addr;
    wire [31:0] PC_add_imm_EX;
    reg  [31:0] PC_add_imm_MEM;
    wire [31:0] if_branch;
    wire [31:0] if_jump;
    wire [31:0] if_jr;
    wire        PCsrc;
    // instruction memory output
    wire [25:0] instr_0_ID;
    wire  [5:0] instr_1_ID;
    wire  [4:0] instr_2_ID;
    reg   [4:0] instr_2_EX;
    wire  [4:0] instr_3_ID;
    reg   [4:0] instr_3_EX;
    reg   [4:0] instr_3_MEM;
    wire  [4:0] instr_4_ID;
    reg   [4:0] instr_4_EX;
    wire [15:0] instr_5_ID;
    // Register input
    wire  [4:0] WriteFromInstr_EX;
    reg   [4:0] WriteFromInstr_MEM, WriteFromInstr_WB;
    wire  [4:0] WriteFromJal;
    wire  [4:0] WriteReg;
    // Register output
    wire [31:0] ReadData1_ID;
    reg  [31:0] ReadData1_EX; 
    wire [31:0] ReadData2_ID;
    reg  [31:0] ReadData2_EX, ReadData2_MEM;
    // Sign extend
    wire [31:0] extended_instr_5_ID;
    reg  [31:0] extended_instr_5_EX;
    wire [31:0] extended_shift2;
    // ALU input
    wire [31:0] alu_in_0, alu_in_1;
    wire [31:0] alu_orig_in_0;
    // ALU output
    wire        Zero_EX;
    reg         Zero_MEM;
    wire [31:0] ALU_result_EX;
    reg  [31:0] ALU_result_MEM, ALU_result_WB;
    // Data Memory output
    wire [31:0] rdata_D_MEM;
    reg  [31:0] rdata_D_WB;
    wire [31:0] return_data;

    // Submodule
    // CONTROL
    wire        RegDst_ID;
    wire        Jump_ID;
    wire        Branch_ID;
    wire        MemRead_ID;
    wire        MemtoReg_ID;
    wire  [2:0] ALUOp_ID;
    wire        MemWrite_ID;
    wire        ALUSrc_ID;
    wire        RegWrite_ID;
    wire        Jal_ID;
    wire        ExtOp_ID;

    reg         RegDst_EX;
    reg         Branch_EX;
    reg         MemRead_EX;
    reg         MemtoReg_EX;
    reg   [2:0] ALUOp_EX;
    reg         MemWrite_EX;
    reg         ALUSrc_EX;
    reg         RegWrite_EX;
    reg         Jal_EX;
    reg         ExtOp_EX;

    reg         Branch_MEM;
    reg         MemRead_MEM;
    reg         MemtoReg_MEM;
    reg         MemWrite_MEM;
    reg         RegWrite_MEM;
    reg         Jal_MEM;

    reg         MemtoReg_WB;
    reg         RegWrite_WB;
    reg         Jal_WB;
    // REGISTER
    wire [31:0] WriteData;
    // ALU CONTROL
    wire  [3:0] ALUCtrl;
    wire        JumReg_EX;
    wire        Shift_EX;
    wire        Jalr_EX;
    reg         Jalr_MEM, Jalr_WB;

    // old data
    // ==== stage IF  ====
    // PC wire
    wire [31:0] instruction_ID_old;

    // for debug
    wire [31:0] PC_ID_old, PC_EX_old, PC_MEM_old, PC_WB_old;
    wire [31:0] instruction_EX_old, instruction_MEM_old, instruction_WB_old;
    // end debug

    wire [31:0] PC_plus4_ID_old, PC_plus4_EX_old, PC_plus4_MEM_old, PC_plus4_WB_old;

    wire [31:0] PC_add_imm_MEM_old;
    // instruction memory output
    wire  [4:0] instr_2_EX_old;
    wire  [4:0] instr_3_EX_old;
    wire  [4:0] instr_4_EX_old;
    wire  [4:0] instr_3_MEM_old;
    // Register input
    wire  [4:0] WriteFromInstr_MEM_old, WriteFromInstr_WB_old;

    // Register output
    wire [31:0] ReadData1_EX_old; 
    wire [31:0] ReadData2_EX_old, ReadData2_MEM_old;
    // Sign extend
    wire [31:0] extended_instr_5_EX_old;
    // ALU input
    // ALU output
    wire        Zero_MEM_old;
    wire [31:0] ALU_result_MEM_old, ALU_result_WB_old;
    // Data Memory output
    wire [31:0] rdata_D_WB_old;

    // CONTROL
    wire        RegDst_EX_old;
    wire        Branch_EX_old;
    wire        MemRead_EX_old;
    wire        MemtoReg_EX_old;
    wire  [2:0] ALUOp_EX_old;
    wire        MemWrite_EX_old;
    wire        ALUSrc_EX_old;
    wire        RegWrite_EX_old;
    wire        Jal_EX_old;
    wire        ExtOp_EX_old;

    wire        Branch_MEM_old;
    wire        MemRead_MEM_old;
    wire        MemtoReg_MEM_old;
    wire        MemWrite_MEM_old;
    wire        RegWrite_MEM_old;
    wire        Jal_MEM_old;

    wire        MemtoReg_WB_old;
    wire        RegWrite_WB_old;
    wire        Jal_WB_old;

    wire        Jalr_MEM_old;
    wire        Jalr_WB_old;

    // ==== Hazard Handling ====
    wire        PCWrite;
    wire        IF_ID_write;
    wire        ctrl_flush;

    wire  [1:0] ForwardA, ForwardB;
    wire        ForwardWriteData_MEM;

//==== Submodule Connection ===================
    CONTROL control_1(
        .Op(instr_1_ID),
        .ctrl_flush(ctrl_flush),
        .RegDst(RegDst_ID),
        .Jump(Jump_ID),
        .Branch(Branch_ID),
        .MemRead(MemRead_ID),
        .MemtoReg(MemtoReg_ID),
        .ALUOp(ALUOp_ID),
        .MemWrite(MemWrite_ID),
        .ALUSrc(ALUSrc_ID),
        .RegWrite(RegWrite_ID),
        .Jal(Jal_ID),
        .ExtOp(ExtOp_ID)
    );
    REGISTER register_1(
        .clk(clk),
        .rst_n(rst_n),
        .RegWrite(RegWrite_WB),
        .ReadReg1(instr_2_ID),
        .ReadReg2(instr_3_ID),
        .WriteReg(WriteReg),
        .WriteData(WriteData),
        .ReadData1(ReadData1_ID),
        .ReadData2(ReadData2_ID)
    );
    ALU_CONTROL ALU_CONTROL_1(
        .funct(extended_instr_5_EX[5:0]),
        .ALUOp(ALUOp_EX),
        .ALUCtrl(ALUCtrl),
        .JumpReg(JumpReg_EX),
        .Shift(Shift_EX),
        .Jalr(Jalr_EX)
    );
    ALU ALU_1(
        .ctrl(ALUCtrl),
        .in_0(alu_in_0),
        .in_1(alu_in_1),
        .result(ALU_result_EX),
        .Zero(Zero_EX)
    );

//==== Sequential Part ========================
    // ==== stage IF  ====
    always @(posedge clk) begin
        if (!rst_n) begin
            PC_r <= 0;
        end
        else begin
            PC_r <= PC_w;
        end
    end
    // ==== stage ID  ====
    always @(posedge clk) begin
        if (!rst_n | ((Jump_ID | JumpReg_EX | PCsrc) & ~(ICACHE_stall | DCACHE_stall))) begin
            PC_ID          <= 0;
            PC_plus4_ID    <= 0;
            instruction_ID <= 0;
        end
        else if (ICACHE_stall | DCACHE_stall | ~IF_ID_write) begin
            PC_ID          <= PC_ID_old;
            PC_plus4_ID    <= PC_plus4_ID_old;
            instruction_ID <= instruction_ID_old;
        end
        else begin
            PC_ID          <= PC_r;
            PC_plus4_ID    <= PC_plus4_IF;
            instruction_ID <= instruction_IF;
        end
    end
    // ==== stage EX  ====
    always @(posedge clk) begin
        if (!rst_n | ((JumpReg_EX | PCsrc) & ~(ICACHE_stall | DCACHE_stall))) begin
            PC_EX               <= 0;
            instruction_EX      <= 0;
            PC_plus4_EX         <= 0;
            RegDst_EX           <= 0;
            Branch_EX           <= 0;
            MemRead_EX          <= 0;
            MemtoReg_EX         <= 0;
            ALUOp_EX            <= 0;
            MemWrite_EX         <= 0;
            ALUSrc_EX           <= 0;
            RegWrite_EX         <= 0;
            Jal_EX              <= 0;
            ExtOp_EX            <= 0;

            PC_plus4_EX         <= 0;
            ReadData1_EX        <= 0;
            ReadData2_EX        <= 0;
            extended_instr_5_EX <= 0;
            instr_2_EX          <= 0;
            instr_3_EX          <= 0;
            instr_4_EX          <= 0;
        end
        else if(ICACHE_stall | DCACHE_stall) begin
            PC_EX               <= PC_EX_old;
            instruction_EX      <= instruction_EX_old;
            PC_plus4_EX         <= PC_plus4_EX_old;
            RegDst_EX           <= RegDst_EX_old;
            Branch_EX           <= Branch_EX_old;
            MemRead_EX          <= MemRead_EX_old;
            MemtoReg_EX         <= MemtoReg_EX_old;
            ALUOp_EX            <= ALUOp_EX_old;
            MemWrite_EX         <= MemWrite_EX_old;
            ALUSrc_EX           <= ALUSrc_EX_old;
            RegWrite_EX         <= RegWrite_EX_old;
            Jal_EX              <= Jal_EX_old;
            ExtOp_EX            <= ExtOp_EX_old;

            PC_plus4_EX         <= PC_plus4_EX_old;
            ReadData1_EX        <= ReadData1_EX_old;
            ReadData2_EX        <= ReadData2_EX_old;
            extended_instr_5_EX <= extended_instr_5_EX_old;
            instr_2_EX          <= instr_2_EX_old;
            instr_3_EX          <= instr_3_EX_old; // read reg 2 / write reg
            instr_4_EX          <= instr_4_EX_old; // write reg
        end
        else begin
            PC_EX               <= PC_ID;
            instruction_EX      <= instruction_ID;
            PC_plus4_EX         <= PC_plus4_ID;
            RegDst_EX           <= RegDst_ID;
            Branch_EX           <= Branch_ID;
            MemRead_EX          <= MemRead_ID;
            MemtoReg_EX         <= MemtoReg_ID;
            ALUOp_EX            <= ALUOp_ID;
            MemWrite_EX         <= MemWrite_ID;
            ALUSrc_EX           <= ALUSrc_ID;
            RegWrite_EX         <= RegWrite_ID;
            Jal_EX              <= Jal_ID;
            ExtOp_EX            <= ExtOp_ID;

            PC_plus4_EX         <= PC_plus4_ID;
            ReadData1_EX        <= ReadData1_ID;
            ReadData2_EX        <= ReadData2_ID;
            extended_instr_5_EX <= extended_instr_5_ID;
            instr_2_EX          <= instr_2_ID;
            instr_3_EX          <= instr_3_ID; // read reg 2 / write reg
            instr_4_EX          <= instr_4_ID; // write reg
        end
    end
    // ==== stage MEM ====
    always @(posedge clk) begin
        if (!rst_n | (PCsrc & ~(ICACHE_stall | DCACHE_stall))) begin
            PC_MEM             <= 0;
            instruction_MEM    <= 0;
            PC_plus4_MEM       <= 0;
            Branch_MEM         <= 0;
            MemRead_MEM        <= 0;
            MemtoReg_MEM       <= 0;
            MemWrite_MEM       <= 0;
            RegWrite_MEM       <= 0;
            Jal_MEM            <= 0;
            Jalr_MEM           <= 0;
            
            PC_add_imm_MEM     <= 0;
            Zero_MEM           <= 0;
            ALU_result_MEM     <= 0;
            ReadData2_MEM      <= 0;
            WriteFromInstr_MEM <= 0;
            instr_3_MEM        <= 0;
        end
        else if (ICACHE_stall | DCACHE_stall) begin
            PC_MEM             <= PC_MEM_old;
            instruction_MEM    <= instruction_MEM_old;
            PC_plus4_MEM       <= PC_plus4_MEM_old;
            Branch_MEM         <= Branch_MEM_old;
            MemRead_MEM        <= MemRead_MEM_old;
            MemtoReg_MEM       <= MemtoReg_MEM_old;
            MemWrite_MEM       <= MemWrite_MEM_old;
            RegWrite_MEM       <= RegWrite_MEM_old;
            Jal_MEM            <= Jal_MEM_old;
            Jalr_MEM           <= Jalr_MEM_old;

            PC_add_imm_MEM     <= PC_add_imm_MEM_old;
            Zero_MEM           <= Zero_MEM_old;
            ALU_result_MEM     <= ALU_result_MEM_old;
            ReadData2_MEM      <= ReadData2_MEM_old;
            WriteFromInstr_MEM <= WriteFromInstr_MEM_old;
            instr_3_MEM        <= instr_3_MEM_old;
        end
        else begin
            PC_MEM             <= PC_EX;
            instruction_MEM    <= instruction_EX;
            PC_plus4_MEM       <= PC_plus4_EX;
            Branch_MEM         <= Branch_EX;
            MemRead_MEM        <= MemRead_EX;
            MemtoReg_MEM       <= MemtoReg_EX;
            MemWrite_MEM       <= MemWrite_EX;
            RegWrite_MEM       <= RegWrite_EX;
            Jal_MEM            <= Jal_EX;
            Jalr_MEM           <= Jalr_EX;

            PC_add_imm_MEM     <= PC_add_imm_EX;
            Zero_MEM           <= Zero_EX;
            ALU_result_MEM     <= ALU_result_EX;
            ReadData2_MEM      <= ReadData2_EX;
            WriteFromInstr_MEM <= WriteFromInstr_EX;
            instr_3_MEM        <= instr_3_EX;
        end
    end
    // ==== stage WB  ====
    always @(posedge clk) begin
        if (!rst_n) begin
            PC_WB             <= 0;
            instruction_WB    <= 0;
            PC_plus4_WB       <= 0;
            MemtoReg_WB       <= 0;
            RegWrite_WB       <= 0;
            Jal_WB            <= 0;
            Jalr_WB           <= 0;

            rdata_D_WB        <= 0;
            ALU_result_WB     <= 0;
            WriteFromInstr_WB <= 0;
        end
        else if (ICACHE_stall | DCACHE_stall) begin
            PC_WB             <= PC_WB_old;
            instruction_WB    <= instruction_WB_old;
            PC_plus4_WB       <= PC_plus4_WB_old;
            MemtoReg_WB       <= MemtoReg_WB_old;
            RegWrite_WB       <= RegWrite_WB_old;
            Jal_WB            <= Jal_WB_old;
            Jalr_WB           <= Jalr_WB_old;

            rdata_D_WB        <= rdata_D_WB_old;
            ALU_result_WB     <= ALU_result_WB_old;
            WriteFromInstr_WB <= WriteFromInstr_WB_old;
        end
        else begin
            PC_WB             <= PC_MEM;
            instruction_WB    <= instruction_MEM;
            PC_plus4_WB       <= PC_plus4_MEM;
            MemtoReg_WB       <= MemtoReg_MEM;
            RegWrite_WB       <= RegWrite_MEM;
            Jal_WB            <= Jal_MEM;
            Jalr_WB           <= Jalr_MEM;

            rdata_D_WB        <= rdata_D_MEM;
            ALU_result_WB     <= ALU_result_MEM;
            WriteFromInstr_WB <= WriteFromInstr_MEM;
        end
    end
//==== Combinational Part =====================
    // ==== stage IF  ====
    // PC wire
    assign PC_plus4_IF = PC_r + 4;
    assign jump_addr   = { PC_plus4_ID[31:28], instr_0_ID, 2'b00 };
    assign if_jump     = (Jump_ID)? jump_addr : PC_plus4_IF; // MUX
    assign if_jr       = (JumpReg_EX)? alu_in_0 : if_jump; // MUX
    assign if_branch   = (PCsrc)? PC_add_imm_MEM : if_jr; // MUX
    assign PC_w        = (ICACHE_stall | DCACHE_stall | ~PCWrite)? PC_r : if_branch;
	// Instruction Fetch
	assign ICACHE_ren     = 1'b1;
    assign ICACHE_wen     = 1'b0;
    assign ICACHE_addr    = PC_r[31:2];
    assign instruction_IF = ICACHE_rdata;
    assign ICACHE_wdata   = 0;
    // ==== stage ID  ====
    // Instruction memory output
    assign instr_0_ID = instruction_ID[25:0];  // PC shift left 2
    assign instr_1_ID = instruction_ID[31:26]; // control
    assign instr_2_ID = instruction_ID[25:21]; // read reg 1, rs
    assign instr_3_ID = instruction_ID[20:16]; // read reg 2 / write reg, rt
    assign instr_4_ID = instruction_ID[15:11]; // write reg, rd
    assign instr_5_ID = instruction_ID[15:0];  // signed extend
    // Sign extend
    assign extended_instr_5_ID = { { 16{instr_5_ID[15] } }, instr_5_ID };
    // ==== stage EX  ====
    // Register write
    assign WriteFromInstr_EX = (RegDst_EX)? instr_4_EX : instr_3_EX; // MUX
    assign WriteFromJal = 5'd31;
    // PC add imm
    assign extended_shift2 = { extended_instr_5_EX[29:0], 2'b00 };
    assign PC_add_imm_EX = PC_plus4_EX + extended_shift2;
    // ALU input
    assign alu_orig_in_0 = (Shift_EX)? { 27'b0, extended_instr_5_EX[10:6] } : ReadData1_EX;
    // Hazard Handling
    assign alu_in_0 = (ForwardA[0])? WriteData :
                      (ForwardA[1])? ALU_result_MEM : alu_orig_in_0;
    assign alu_in_1 = (ALUSrc_EX)?   extended_instr_5_EX :
                      (ForwardB[0])? WriteData :
                      (ForwardB[1])? ALU_result_MEM : ReadData2_EX;
    // ==== stage MEM ====
    // Branch
    assign PCsrc = Branch_MEM & Zero_MEM;
    // Data Memory input
    assign DCACHE_addr  = ALU_result_MEM[31:2];
    // assign DCACHE_wdata = ReadData2_MEM;
    assign DCACHE_wdata = (ForwardWriteData_MEM)? WriteData : ReadData2_MEM;
    assign DCACHE_wen   = MemWrite_MEM;
    assign DCACHE_ren   = MemRead_MEM;
    assign rdata_D_MEM  = DCACHE_rdata;
    // ==== stage WB  ====
    // Data Memory output
    assign return_data = (MemtoReg_WB)? rdata_D_WB : ALU_result_WB; // MUX
    // Register Write
    assign WriteReg  = (Jal_WB)? WriteFromJal : WriteFromInstr_WB;
    assign WriteData = (Jal_WB | Jalr_WB)? PC_plus4_WB : return_data;

//==== Restore Part ===========================
    // ==== stage ID  ====
    assign PC_ID_old          = PC_ID;
    assign PC_plus4_ID_old    = PC_plus4_ID;
    assign instruction_ID_old = instruction_ID;
    // ==== stage EX  ====
    assign PC_EX_old          = PC_EX;
    assign instruction_EX_old = instruction_EX;
    assign PC_plus4_EX_old    = PC_plus4_EX;
    assign RegDst_EX_old      = RegDst_EX;
    assign Branch_EX_old      = Branch_EX;
    assign MemRead_EX_old     = MemRead_EX;
    assign MemtoReg_EX_old    = MemtoReg_EX;
    assign ALUOp_EX_old       = ALUOp_EX;
    assign MemWrite_EX_old    = MemWrite_EX;
    assign ALUSrc_EX_old      = ALUSrc_EX;
    assign RegWrite_EX_old    = RegWrite_EX;
    assign Jal_EX_old         = Jal_EX;
    assign ExtOp_EX_old       = ExtOp_EX;

    assign PC_plus4_EX_old         = PC_plus4_EX;
    assign ReadData1_EX_old        = ReadData1_EX;
    assign ReadData2_EX_old        = ReadData2_EX;
    assign extended_instr_5_EX_old = extended_instr_5_EX;
    assign instr_2_EX_old          = instr_2_EX;
    assign instr_3_EX_old          = instr_3_EX;
    assign instr_4_EX_old          = instr_4_EX;
    // ==== stage MEM ====
    assign PC_MEM_old          = PC_MEM;
    assign instruction_MEM_old = instruction_MEM;
    assign PC_plus4_MEM_old    = PC_plus4_MEM;
    assign Branch_MEM_old      = Branch_MEM;
    assign MemRead_MEM_old     = MemRead_MEM;
    assign MemtoReg_MEM_old    = MemtoReg_MEM;
    assign MemWrite_MEM_old    = MemWrite_MEM;
    assign RegWrite_MEM_old    = RegWrite_MEM;
    assign Jal_MEM_old         = Jal_MEM;
    assign Jalr_MEM_old        = Jalr_MEM;
            
    assign PC_add_imm_MEM_old     = PC_add_imm_MEM;
    assign Zero_MEM_old           = Zero_MEM;
    assign ALU_result_MEM_old     = ALU_result_MEM;
    assign ReadData2_MEM_old      = ReadData2_MEM;
    assign WriteFromInstr_MEM_old = WriteFromInstr_MEM;
    // ==== stage WB  ====
    assign PC_WB_old          = PC_WB;
    assign instruction_WB_old = instruction_WB;
    assign PC_plus4_WB_old    = PC_plus4_WB;
    assign MemtoReg_WB_old    = MemtoReg_WB;
    assign RegWrite_WB_old    = RegWrite_WB;
    assign Jal_WB_old         = Jal_WB;
    assign Jalr_WB_old        = Jalr_WB;

    assign rdata_D_WB_old        = rdata_D_WB;
    assign ALU_result_WB_old     = ALU_result_WB;
    assign WriteFromInstr_WB_old = WriteFromInstr_WB;

//==== Hazard Handling Part ===================
    HazardDetection HazardDetection_1(
        .id_ex_MemRead(MemRead_EX),
        .id_ex_rt(instr_3_EX),
        .if_id_rs(instr_2_ID),
        .if_id_rt(instr_3_ID),
        .PCWrite(PCWrite),
        .if_id_Write(IF_ID_write),
        .mux_Ctrl(ctrl_flush)
    );

   ForwardUnit ForwardUnit_1(
        .ex_mem_rd(WriteFromInstr_MEM),
        .ex_mem_rt(instr_3_MEM),
        .mem_wb_rd(WriteReg),
        .if_id_rs(instr_2_ID),
        .if_id_rt(instr_3_ID),
        .id_ex_rs(instr_2_EX),
        .id_ex_rt(instr_3_EX),
        .ex_mem_RegWrite(RegWrite_MEM),
        .mem_wb_RegWrite(RegWrite_WB),
        .ex_mem_MemWrite(MemWrite_MEM),
        //.Branch_ID(Branch_ID),
        .opcode_ID(instr_1_ID),
        .ForwardA(ForwardA),
        .ForwardB(ForwardB),
        .ForwardWriteData_MEM(ForwardWriteData_MEM)
    );

endmodule

module ALU(
    ctrl, 
    in_0, 
    in_1,
    result,
    Zero
);
//==== io Declaration =========================
    input       [3:0] ctrl;
    input      [31:0] in_0, in_1;
    output reg [31:0] result;
    output reg        Zero;
    
    always @(*) begin
        case(ctrl)
            4'b0000: begin // and
                result = in_0 & in_1;
            end
            4'b0001: begin // or
                result = in_0 | in_1;
            end
            4'b0010: begin // add
                result = in_0 + in_1;
            end
            4'b0011: begin // bne sub
                result = in_0 - in_1;
            end
            4'b0110: begin // sub
                result = in_0 - in_1;
            end
            4'b0111: begin // slt
                result = ($signed(in_0) < $signed(in_1))? 1 : 0;
            end
            4'b1000: begin // nor
                result = ~(in_0 | in_1);
            end
            4'b1001: begin // xor
                result = in_0 ^ in_1;
            end
            4'b1010: begin // sll
                result = in_1 << in_0;
            end
            4'b1011: begin // sra
                result = $signed(in_1) >>> in_0;
            end
            4'b1100: begin // srl
                result = in_1 >> in_0;
            end
            default: begin
                result = 32'b0;
            end
        endcase
        Zero = ((result == 32'b0 && ctrl[2]) || (result != 32'b0 && ~ctrl[2])) ? 1 : 0;
    end
endmodule

module ALU_CONTROL(
    funct,
    ALUOp,
    ALUCtrl,
    JumpReg, 
    Shift,
    Jalr
);
//==== io Declaration =========================
    input      [5:0] funct;
    input      [2:0] ALUOp;
    output reg [3:0] ALUCtrl;
    output reg       JumpReg, Shift, Jalr;
    
//==== Reg/Wire Declaration ===================
    wire       [8:0] temp;
    assign temp = { ALUOp, funct };
    
    always @(*) begin
        casex(temp)
            9'b000xxxxxx: begin // j, addi, lw, sw ==> add
                ALUCtrl = 4'b0010;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b001xxxxxx: begin // beq ==> substract
                ALUCtrl = 4'b0110;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010000000: begin // 00: sll
                ALUCtrl = 4'b1010;
                JumpReg = 0;
                Shift   = 1;
                Jalr    = 0;
            end
            9'b010000010: begin // 02: srl
                ALUCtrl = 4'b1100;
                JumpReg = 0;
                Shift   = 1;
                Jalr    = 0;
            end
            9'b010000011: begin // 03: sra
                ALUCtrl = 4'b1011;
                JumpReg = 0;
                Shift   = 1;
                Jalr    = 0;
            end
            9'b010001000: begin // 08: jr
                ALUCtrl = 4'b0000; // xxxx
                JumpReg = 1;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010001001: begin // 08: jalr
                ALUCtrl = 4'b0000; // xxxx
                JumpReg = 1;
                Shift   = 0;
                Jalr    = 1;
            end
            9'b010100000: begin // 20: add
                ALUCtrl = 4'b0010;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010100010: begin // 22: sub
                ALUCtrl = 4'b0110;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010100100: begin // 24: and
                ALUCtrl = 4'b0000;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010100101: begin // 25: or
                ALUCtrl = 4'b0001;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010100110: begin // 26: xor
                ALUCtrl = 4'b1001;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010100111: begin // 27: nor
                ALUCtrl = 4'b1000;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b010101010: begin // 2a: slt
                ALUCtrl = 4'b0111;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b011xxxxxx: begin // bne
                ALUCtrl = 4'b0011;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b100xxxxxx: begin // slti
                ALUCtrl = 4'b0111;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b101xxxxxx: begin // andi
                ALUCtrl = 4'b0000;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b110xxxxxx: begin // ori
                ALUCtrl = 4'b0001;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            9'b111xxxxxx: begin // xori
                ALUCtrl = 4'b1001;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
            default: begin
                ALUCtrl = 4'b0000;
                JumpReg = 0;
                Shift   = 0;
                Jalr    = 0;
            end
        endcase
    end
endmodule

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
    // io mem
    reg            data_ready;
    reg            mem_read, mem_write;
    reg     [27:0] mem_addr, next_mem_addr;
    reg    [127:0] mem_wdata, next_mem_wdata;
    // state, cache
    reg    [155:0] cache_dm[0:3][0:1], next_cache_dm[0:3][0:1];
    reg      [3:0] LRU, next_LRU;
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
    always@(*) begin
        for(i = 0; i < 4; i = i + 1) begin
            next_cache_dm[i][0] = cache_dm[i][0];
            next_cache_dm[i][1] = cache_dm[i][1];
        end
        next_LRU = LRU;
        next_state = state;
        proc_stall = 0;
        proc_rdata = 0;
        mem_read = 0;
        mem_write = 0;
        next_mem_addr = mem_addr;
        next_mem_wdata = mem_wdata;
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
                if(valid[0] && hit[0]) begin // cache hit 0
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
                else if(valid[LRU[index]] && dirty[LRU[index]] && (proc_read | proc_write)) begin // cache miss && (block valid && dirty)
                    next_state = WB;
                    proc_stall = 1;
                    mem_write = 1;
                    next_mem_wdata = cache_dm[index][LRU[index]];
                    next_mem_addr = {cache_tag[LRU[index]], index};
                end
                else if(proc_read | proc_write) begin // cache miss && (block clean or invalid)
                    next_state = ALLOCATE;
                    proc_stall = 1;
                    mem_read = 1;
                    next_mem_addr = proc_addr[29:2];
                end
            end
            ALLOCATE: begin
                if(data_ready) begin
                    next_state = COMPARE;
                    if(proc_write)
                        proc_stall = 1;
                    else
                        proc_stall = 0;
                    mem_read = 0;
                    next_cache_dm[index][LRU[index]][155] = 1'b0; // set dirty
                    next_cache_dm[index][LRU[index]][154] = 1'b1; // set valid
                    next_cache_dm[index][LRU[index]][153:128] = tag; // set tag
                    next_cache_dm[index][LRU[index]][127:0] = mem_rdata;
                    case(word_idx)
                        2'b00: begin
                            proc_rdata = mem_rdata[31:0];
                        end
                        2'b01: begin
                            proc_rdata = mem_rdata[63:32];
                        end
                        2'b10: begin
                            proc_rdata = mem_rdata[95:64];
                        end
                        2'b11: begin
                            proc_rdata = mem_rdata[127:96];
                        end
                        default: proc_rdata = 0;
                    endcase
                    next_LRU[index] = ~LRU[index];
                end
                else begin
                    next_state = ALLOCATE;
                    proc_stall = 1;
                    mem_read = 1;
                end
            end
            WB: begin
                proc_stall = 1;
                if(data_ready) begin
                    mem_write = 0;
                    next_mem_addr = proc_addr[29:2];
                    next_state = ALLOCATE;
                end
                else begin
                    mem_write = 1;
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
    always@( posedge clk or posedge proc_reset) begin
        if( proc_reset ) begin
            for(j = 0; j < 4; j = j + 1) begin
                cache_dm[j][0] <= 0;
                cache_dm[j][1] <= 0;
            end
            LRU <= 0;
            state <= IDLE;
            data_ready <= 0;
            mem_addr <= 0;
            mem_wdata <= 0;
        end
        else begin
            for(j = 0; j < 4; j = j + 1) begin 
                cache_dm[j][0] <= next_cache_dm[j][0];
                cache_dm[j][1] <= next_cache_dm[j][1];
            end
            LRU <= next_LRU;
            state <= next_state;
            data_ready <= mem_ready;
            mem_addr <= next_mem_addr;
            mem_wdata <= next_mem_wdata;
        end
    end

endmodule

module CONTROL(
    Op,
    ctrl_flush,
    RegDst,
    Jump,
    Branch,
    MemRead,
    MemtoReg,
    ALUOp,
    MemWrite,
    ALUSrc,
    RegWrite,
    Jal,
    ExtOp
);
//==== io Declaration =========================
    input      [5:0] Op;
    input            ctrl_flush;
    output reg       RegDst, Jump, Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite, Jal, ExtOp;
    output reg [2:0] ALUOp;
    
    always @(*) begin
        case(Op)
            8'h00: begin // R type
                RegDst   = 1;
                Jump     = 0;
                Branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b010;
                MemWrite = 0;
                ALUSrc   = 0;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 0; // x
            end
            8'h02: begin // j
                RegDst   = 0; // x
                Jump     = 1;
                Branch   = 0;
                MemRead  = 0;
                MemtoReg = 0; // x
                ALUOp    = 3'b000;
                MemWrite = 0;
                ALUSrc   = 0; // x
                RegWrite = 0;
                Jal      = 0;
                ExtOp    = 0; // x
            end
            8'h03: begin // jal
                RegDst   = 0; // x
                Jump     = 1;
                Branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b000;
                MemWrite = 0;
                ALUSrc   = 0; // x
                RegWrite = 1;
                Jal      = 1;
                ExtOp    = 0; // x
            end
            8'h04: begin // beq
                RegDst   = 0; // x
                Jump     = 0;
                Branch   = 1;
                MemRead  = 0;
                MemtoReg = 0; // x
                ALUOp    = 3'b001;
                MemWrite = 0;
                ALUSrc   = 0;
                RegWrite = 0;
                Jal      = 0;
                ExtOp    = 1;
            end
            8'h05: begin // bne
                RegDst   = 0; // x
                Jump     = 0;
                Branch   = 1;
                MemRead  = 0;
                MemtoReg = 0; // x
                ALUOp    = 3'b011;
                MemWrite = 0;
                ALUSrc   = 0;
                RegWrite = 0;
                Jal      = 0;
                ExtOp    = 1;
            end
            8'h08: begin // addi
                RegDst   = 0;
                Jump     = 0;
                Branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b000;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 1;
            end
            8'h23: begin // lw
                RegDst   = 0;
                Jump     = 0;
                Branch   = 0;
                MemRead  = 1;
                MemtoReg = 1;
                ALUOp    = 3'b000;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 1;
            end
            8'h2b: begin // sw
                RegDst   = 0; // x
                Jump     = 0;
                Branch   = 0;
                MemRead  = 0;
                MemtoReg = 0; // x
                ALUOp    = 3'b000;
                MemWrite = 1;
                ALUSrc   = 1;
                RegWrite = 0;
                Jal      = 0;
                ExtOp    = 1;
            end
            8'h0a: begin // slti
                RegDst   = 0;
                Jump     = 0;
                Branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b100;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 1;
            end
            8'h0c: begin // andi
                RegDst   = 0;
                Jump     = 0;
                Branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b101;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 0;
            end
            8'h0d: begin // ori
                RegDst   = 0;
                Jump     = 0;
                Branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b110;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 0;
            end
            8'h0e: begin // xori
                RegDst   = 0;
                Jump     = 0;
                Branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b111;
                MemWrite = 0;
                ALUSrc   = 1;
                RegWrite = 1;
                Jal      = 0;
                ExtOp    = 0;
            end
            default: begin
                RegDst   = 0;
                Jump     = 0;
                Branch   = 0;
                MemRead  = 0;
                MemtoReg = 0;
                ALUOp    = 3'b000;
                MemWrite = 0;
                ALUSrc   = 0;
                RegWrite = 0;
                Jal      = 0;
                ExtOp    = 0;
            end
        endcase
        if (~ctrl_flush) begin
            RegDst   = 0;
            Jump     = 0;
            Branch   = 0;
            MemRead  = 0;
            MemtoReg = 0;
            ALUOp    = 3'b000;
            MemWrite = 0;
            ALUSrc   = 0;
            RegWrite = 0;
            Jal      = 0;
            ExtOp    = 0;
        end
    end
endmodule

module HazardDetection(
    id_ex_MemRead,
    id_ex_rt,
    if_id_rs,
    if_id_rt,
    PCWrite,
    if_id_Write,
    mux_Ctrl
);
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
    ex_mem_rt,
    mem_wb_rd,
    if_id_rs,
    if_id_rt,
    id_ex_rs,
    id_ex_rt,
    ex_mem_RegWrite,
    mem_wb_RegWrite,
    ex_mem_MemWrite,
    opcode_ID,
    ForwardA,
    ForwardB,
    ForwardWriteData_MEM
);
    input      [4:0] ex_mem_rd, ex_mem_rt, mem_wb_rd, if_id_rs, if_id_rt, id_ex_rs, id_ex_rt;
    input            ex_mem_RegWrite, mem_wb_RegWrite;
    input            ex_mem_MemWrite;
    input      [5:0] opcode_ID;
    output reg [1:0] ForwardA, ForwardB; //00 original, 01 forward from mem/wb, 10 forward from ex/mem
    output reg       ForwardWriteData_MEM;

    always @(*) begin
        ForwardA = 2'b0;
        ForwardB = 2'b0;

        if (mem_wb_RegWrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rs) &&
            !(ex_mem_RegWrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs))) begin
            ForwardA = 2'b01;   //forward from mem/wb
        end 
        else if (ex_mem_RegWrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rs)) begin
            ForwardA = 2'b10;   //forward from ex/mem
        end
        else ForwardA = 2'b00;  //original

        if (mem_wb_RegWrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == id_ex_rt) &&
            !(ex_mem_RegWrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rt))) begin
            ForwardB = 2'b01;   //forward from mem/wb
        end
        else if (ex_mem_RegWrite && (ex_mem_rd != 5'd0) && (ex_mem_rd == id_ex_rt)) begin
            ForwardB = 2'b10;   //forward from ex/mem

        end
        else ForwardB = 2'b00;  //original
    end

    always @(*) begin
        ForwardWriteData_MEM = 1'b0;

        if (mem_wb_RegWrite && ex_mem_MemWrite && (mem_wb_rd != 5'd0) && (mem_wb_rd == ex_mem_rt)) begin
            //deal with save hazard
            ForwardWriteData_MEM = 1'b1;
        end
        else ForwardWriteData_MEM = 1'b0;
    end

endmodule

module REGISTER(
    clk,
    rst_n,
    RegWrite,
    ReadReg1,
    ReadReg2,
    WriteReg,
    WriteData,
    ReadData1,
    ReadData2
);
//==== io Declaration =========================
    input             RegWrite, clk, rst_n;
    input       [4:0] ReadReg1, ReadReg2, WriteReg;
    input      [31:0] WriteData;
    output reg [31:0] ReadData1, ReadData2;

//==== Reg/Wire Declaration ===================
    reg [31:0] Registers[0:31];

    integer i;
    
    always @(posedge clk) begin
        if (!rst_n) begin
            for(i = 0; i <= 31; i = i + 1) Registers[i] <= 32'd0;
        end
        // else if(RegWrite && WriteReg != 0) begin
        //     Registers[WriteReg] <= WriteData;
        // end
        else if (RegWrite) begin
            case(WriteReg)
                0: Registers[0]  <= 0;
		        1: Registers[1]  <= WriteData;
		        2: Registers[2]  <= WriteData;
		        3: Registers[3]  <= WriteData;
		        4: Registers[4]  <= WriteData;
		        5: Registers[5]  <= WriteData;
		        6: Registers[6]  <= WriteData;
		        7: Registers[7]  <= WriteData;
	            8: Registers[8]  <= WriteData;
		        9: Registers[9]  <= WriteData;
		        10:Registers[10] <= WriteData;
		        11:Registers[11] <= WriteData;
		        12:Registers[12] <= WriteData;
		        13:Registers[13] <= WriteData;
		        14:Registers[14] <= WriteData;
		        15:Registers[15] <= WriteData;
		        16:Registers[16] <= WriteData;
		        17:Registers[17] <= WriteData;
		        18:Registers[18] <= WriteData;
		        19:Registers[19] <= WriteData;
		        20:Registers[20] <= WriteData;
		        21:Registers[21] <= WriteData;
		        22:Registers[22] <= WriteData;
		        23:Registers[23] <= WriteData;
		        24:Registers[24] <= WriteData;
		        25:Registers[25] <= WriteData;
		        26:Registers[26] <= WriteData;
		        27:Registers[27] <= WriteData;
		        28:Registers[28] <= WriteData;
		        29:Registers[29] <= WriteData;
		        30:Registers[30] <= WriteData;
		        31:Registers[31] <= WriteData;
                default: Registers[0] <= 0;
            endcase
        end
    end

    // assign ReadData1 = Registers[ReadReg1];
    // assign ReadData2 = Registers[ReadReg2];
    always @(*) begin
        case(ReadReg1)
            0: ReadData1 = Registers[0];
            1: ReadData1 = Registers[1];
            2: ReadData1 = Registers[2];
            3: ReadData1 = Registers[3];
            4: ReadData1 = Registers[4];
            5: ReadData1 = Registers[5];
            6: ReadData1 = Registers[6];
            7: ReadData1 = Registers[7];
            8: ReadData1 = Registers[8];
            9: ReadData1 = Registers[9];
            10:ReadData1 = Registers[10];
            11:ReadData1 = Registers[11];
            12:ReadData1 = Registers[12];
            13:ReadData1 = Registers[13];
            14:ReadData1 = Registers[14];
            15:ReadData1 = Registers[15];
            16:ReadData1 = Registers[16];
            17:ReadData1 = Registers[17];
            18:ReadData1 = Registers[18];
            19:ReadData1 = Registers[19];
            20:ReadData1 = Registers[20];
            21:ReadData1 = Registers[21];
            22:ReadData1 = Registers[22];
            23:ReadData1 = Registers[23];
            23:ReadData1 = Registers[24];
            25:ReadData1 = Registers[25];
            26:ReadData1 = Registers[26];
            27:ReadData1 = Registers[27];
            28:ReadData1 = Registers[28];
            29:ReadData1 = Registers[29];
            30:ReadData1 = Registers[30];
            31:ReadData1 = Registers[31];
            default: ReadData1 = 0;
        endcase
        case(ReadReg2)
            0: ReadData2 = Registers[0];
            1: ReadData2 = Registers[1];
            2: ReadData2 = Registers[2];
            3: ReadData2 = Registers[3];
            4: ReadData2 = Registers[4];
            5: ReadData2 = Registers[5];
            6: ReadData2 = Registers[6];
            7: ReadData2 = Registers[7];
            8: ReadData2 = Registers[8];
            9: ReadData2 = Registers[9];
            10:ReadData2 = Registers[10];
            11:ReadData2 = Registers[11];
            12:ReadData2 = Registers[12];
            13:ReadData2 = Registers[13];
            14:ReadData2 = Registers[14];
            15:ReadData2 = Registers[15];
            16:ReadData2 = Registers[16];
            17:ReadData2 = Registers[17];
            18:ReadData2 = Registers[18];
            19:ReadData2 = Registers[19];
            20:ReadData2 = Registers[20];
            21:ReadData2 = Registers[21];
            22:ReadData2 = Registers[22];
            23:ReadData2 = Registers[23];
            23:ReadData2 = Registers[24];
            25:ReadData2 = Registers[25];
            26:ReadData2 = Registers[26];
            27:ReadData2 = Registers[27];
            28:ReadData2 = Registers[28];
            29:ReadData2 = Registers[29];
            30:ReadData2 = Registers[30];
            31:ReadData2 = Registers[31];
            default: ReadData2 = 0;
        endcase
        if (WriteReg == ReadReg1 && RegWrite && ReadReg1 != 0) ReadData1 = WriteData;
        if (WriteReg == ReadReg2 && RegWrite && ReadReg2 != 0) ReadData2 = WriteData;
    end
endmodule