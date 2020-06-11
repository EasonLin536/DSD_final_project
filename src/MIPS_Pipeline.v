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
    wire [31:0] alu_orig_in_0, alu_orig_in_1;
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

//==== Submodule Connection ===================
    CONTROL control_1(.Op(instr_1_ID),
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
    REGISTER register_1(.clk(clk),
                        .rst_n(rst_n),
                        .RegWrite(RegWrite_WB),
                        .ReadReg1(instr_2_ID),
                        .ReadReg2(instr_3_ID),
                        .WriteReg(WriteReg),
                        .WriteData(WriteData),
                        .ReadData1(ReadData1_ID),
                        .ReadData2(ReadData2_ID)
    );
    ALU_CONTROL ALU_CONTROL_1(.funct(extended_instr_5_EX[5:0]),
                              .ALUOp(ALUOp_EX),
                              .ALUCtrl(ALUCtrl),
                              .JumpReg(JumpReg_EX),
                              .Shift(Shift_EX),
                              .Jalr(Jalr_EX)
    );
    ALU ALU_1(.ctrl(ALUCtrl),
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
        end
    end
    // ==== stage WB  ====
    always @(posedge clk) begin
        if(!rst_n) begin
            PC_WB <= 0;
            instruction_WB <= 0;
            PC_plus4_WB <= 0;
            MemtoReg_WB <= 0;
            RegWrite_WB <= 0;
            Jal_WB <= 0;
            Jalr_WB <= 0;

            rdata_D_WB <= 0;
            ALU_result_WB <= 0;
            WriteFromInstr_WB <= 0;
        end
        else if(ICACHE_stall | DCACHE_stall) begin
            PC_WB <= PC_WB_old;
            instruction_WB <= instruction_WB_old;
            PC_plus4_WB <= PC_plus4_WB_old;
            MemtoReg_WB <= MemtoReg_WB_old;
            RegWrite_WB <= RegWrite_WB_old;
            Jal_WB <= Jal_WB_old;
            Jalr_WB <= Jalr_WB_old;

            rdata_D_WB <= rdata_D_WB_old;
            ALU_result_WB <= ALU_result_WB_old;
            WriteFromInstr_WB <= WriteFromInstr_WB_old;
        end
        else begin
            PC_WB <= PC_MEM;
            instruction_WB <= instruction_MEM;
            PC_plus4_WB <= PC_plus4_MEM;
            MemtoReg_WB <= MemtoReg_MEM;
            RegWrite_WB <= RegWrite_MEM;
            Jal_WB <= Jal_MEM;
            Jalr_WB <= Jalr_MEM;

            rdata_D_WB <= rdata_D_MEM;
            ALU_result_WB <= ALU_result_MEM;
            WriteFromInstr_WB <= WriteFromInstr_MEM;
        end
    end
//==== Combinational Part =====================
    // ==== stage IF  ====
    // PC wire
    assign PC_plus4_IF = PC_r + 4;
    assign jump_addr = { PC_plus4_ID[31:28], instr_0_ID, 2'b00 };
    assign if_jump = (Jump_ID)? jump_addr : PC_plus4_IF; // MUX
    assign if_jr = (JumpReg_EX)? alu_in_0 : if_jump; // MUX
    assign if_branch = (PCsrc)? PC_add_imm_MEM : if_jr; // MUX
    assign PC_w = (ICACHE_stall | DCACHE_stall | ~PCWrite)? PC_r : if_branch;
	// Instruction Fetch
	assign ICACHE_ren = 1'b1;
    assign ICACHE_wen = 1'b0;
    assign ICACHE_addr = PC_r[31:2];
    assign instruction_IF = ICACHE_rdata;
    assign ICACHE_wdata = 0;
    // ==== stage ID  ====
    // Instruction memory output
    assign instr_0_ID = instruction_ID[25:0];  // PC shift left 2
    assign instr_1_ID = instruction_ID[31:26]; // control
    assign instr_2_ID = instruction_ID[25:21]; // read reg 1, rs
    assign instr_3_ID = instruction_ID[20:16]; // read reg 2 / write reg, rt
    assign instr_4_ID = instruction_ID[15:11]; // write reg, rd
    assign instr_5_ID = instruction_ID[15:0];  // signed extend
    // Sign extend
    assign extended_instr_5_ID = {{16{instr_5_ID[15]}}, instr_5_ID};
    // ==== stage EX  ====
    // Register write
    assign WriteFromInstr_EX = (RegDst_EX)? instr_4_EX : instr_3_EX; // MUX
    assign WriteFromJal = 5'd31;
    // PC add imm
    assign extended_shift2 = {extended_instr_5_EX[29:0], 2'b00};
    assign PC_add_imm_EX = PC_plus4_EX + extended_shift2;
    // ALU input
    assign alu_orig_in_0 = (Shift_EX)? {27'b0, extended_instr_5_EX[10:6]} : ReadData1_EX;
    assign alu_orig_in_1 = (ALUSrc_EX)? extended_instr_5_EX : ReadData2_EX; // MUX
    // Hazard Handling
    assign alu_in_0 = (ForwardA[0])? WriteData :
                      (ForwardA[1])? ALU_result_MEM : alu_orig_in_0;
    assign alu_in_1 = (ForwardB[0] && ~ALUSrc_EX)? WriteData : // if imm, don't forwarding
                      (ForwardB[1] && ~ALUSrc_EX)? ALU_result_MEM : alu_orig_in_1;
    // ==== stage MEM ====
    // Branch
    assign PCsrc = Branch_MEM & Zero_MEM;
    // Data Memory input
    assign DCACHE_addr = ALU_result_MEM[31:2];
    assign DCACHE_wdata = ReadData2_MEM;
    assign DCACHE_wen = MemWrite_MEM;
    assign DCACHE_ren = MemRead_MEM;
    assign rdata_D_MEM = DCACHE_rdata;
    // ==== stage WB  ====
    // Data Memory output
    assign return_data = (MemtoReg_WB)? rdata_D_WB : ALU_result_WB; // MUX
    // Register Write
    assign WriteReg = (Jal_WB)? WriteFromJal : WriteFromInstr_WB;
    assign WriteData = (Jal_WB | Jalr_WB)? PC_plus4_WB : return_data;

//==== Restore Part ===========================
    // ==== stage ID  ====
    assign PC_ID_old = PC_ID;
    assign PC_plus4_ID_old = PC_plus4_ID;
    assign instruction_ID_old = instruction_ID;
    // ==== stage EX  ====
    assign PC_EX_old = PC_EX;
    assign instruction_EX_old = instruction_EX;
    assign PC_plus4_EX_old = PC_plus4_EX;
    assign RegDst_EX_old = RegDst_EX;
    assign Branch_EX_old = Branch_EX;
    assign MemRead_EX_old = MemRead_EX;
    assign MemtoReg_EX_old = MemtoReg_EX;
    assign ALUOp_EX_old = ALUOp_EX;
    assign MemWrite_EX_old = MemWrite_EX;
    assign ALUSrc_EX_old = ALUSrc_EX;
    assign RegWrite_EX_old = RegWrite_EX;
    assign Jal_EX_old = Jal_EX;
    assign ExtOp_EX_old = ExtOp_EX;

    assign PC_plus4_EX_old = PC_plus4_EX;
    assign ReadData1_EX_old = ReadData1_EX;
    assign ReadData2_EX_old = ReadData2_EX;
    assign extended_instr_5_EX_old = extended_instr_5_EX;
    assign instr_2_EX_old = instr_2_EX;
    assign instr_3_EX_old = instr_3_EX;
    assign instr_4_EX_old = instr_4_EX;
    // ==== stage MEM ====
    assign PC_MEM_old = PC_MEM;
    assign instruction_MEM_old = instruction_MEM;
    assign PC_plus4_MEM_old = PC_plus4_MEM;
    assign Branch_MEM_old = Branch_MEM;
    assign MemRead_MEM_old = MemRead_MEM;
    assign MemtoReg_MEM_old = MemtoReg_MEM;
    assign MemWrite_MEM_old = MemWrite_MEM;
    assign RegWrite_MEM_old = RegWrite_MEM;
    assign Jal_MEM_old = Jal_MEM;
    assign Jalr_MEM_old = Jalr_MEM;
            
    assign PC_add_imm_MEM_old = PC_add_imm_MEM;
    assign Zero_MEM_old = Zero_MEM;
    assign ALU_result_MEM_old = ALU_result_MEM;
    assign ReadData2_MEM_old = ReadData2_MEM;
    assign WriteFromInstr_MEM_old = WriteFromInstr_MEM;
    // ==== stage WB  ====
    assign PC_WB_old = PC_WB;
    assign instruction_WB_old = instruction_WB;
    assign PC_plus4_WB_old = PC_plus4_WB;
    assign MemtoReg_WB_old = MemtoReg_WB;
    assign RegWrite_WB_old = RegWrite_WB;
    assign Jal_WB_old = Jal_WB;
    assign Jalr_WB_old = Jalr_WB;

    assign rdata_D_WB_old = rdata_D_WB;
    assign ALU_result_WB_old = ALU_result_WB;
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
        .mem_wb_rd(WriteReg),
        .id_ex_rs(instr_2_EX),
        .id_ex_rt(instr_3_EX),
        .ex_mem_RegWrite(RegWrite_MEM),
        .mem_wb_RegWrite(RegWrite_WB),
        .ForwardA(ForwardA),
        .ForwardB(ForwardB)
    );

endmodule