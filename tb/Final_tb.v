// this is a test bench feeds initial instruction and data
// the processor output is not verified

`timescale 1 ns/10 ps

`define CYCLE 10 // You can modify your clock frequency

`define DMEM_INIT "./pattern/D_mem"
`define SDFFILE   "./CHIP_syn.sdf"	// Modify your SDF file name

// For different condition (I_mem, TestBed)
`ifdef noHazard
    `define IMEM_INIT "./pattern/I_mem_noHazard"
    `include "./tb/TestBed_noHazard.v"
`endif
`ifdef hasHazard
	`define IMEM_INIT "./pattern/I_mem_hasHazard"
	`include "./tb/TestBed_hasHazard.v"
`endif	
`ifdef BrPred
	`define IMEM_INIT "./pattern/I_mem_BrPred"
	`include "./tb/TestBed_BrPred.v"
`endif
`ifdef L2Cache
	`define IMEM_INIT "./pattern/I_mem_L2Cache"
	// `define IMEM_INIT "./pattern/L2CacheExp/nb5incre1/I_mem_L2Cache"
	`include "./tb/TestBed_L2Cache.v"
	// `include "./pattern/L2CacheExp/nb5incre1/TestBed_L2Cache.v"
`endif
`ifdef MultDiv
	`define IMEM_INIT "./pattern/I_mem_MultDiv"
	`include "./tb/TestBed_MultDiv.v"
`endif			

module Final_tb;

	reg clk;
	reg rst_n;
	
	wire mem_read_D;
	wire mem_write_D;
	wire [31:4] mem_addr_D;
	wire [127:0] mem_wdata_D;
	wire [127:0] mem_rdata_D;
	wire mem_ready_D;

	wire mem_read_I;
	wire mem_write_I;
	wire [31:4] mem_addr_I;
	wire [127:0] mem_wdata_I;
	wire [127:0] mem_rdata_I;
	wire mem_ready_I;
	
	wire [29:0]	DCACHE_addr;
	wire [31:0]	DCACHE_wdata;
	wire 		DCACHE_wen;
	
	wire [7:0] error_num;
	wire [15:0] duration;
	wire finish;

	// for experiments
	integer mem_cycle, mem_D_access, mem_I_access;

	// Note the design is connected at testbench, include:
	// 1. CHIP (MIPS + D_cache + I_chache)
	// 2. slow memory for data
	// 3. slow memory for instruction
	
	CHIP chip0 (clk,
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
	
	slow_memory slow_memD(
		.clk        (clk)           ,
		.mem_read   (mem_read_D)    ,
		.mem_write  (mem_write_D)   ,
		.mem_addr   (mem_addr_D)    ,
		.mem_wdata  (mem_wdata_D)   ,
		.mem_rdata  (mem_rdata_D)   ,
		.mem_ready  (mem_ready_D)
	);

	slow_memory slow_memI(
		.clk        (clk)           ,
		.mem_read   (mem_read_I)    ,
		.mem_write  (mem_write_I)   ,
		.mem_addr   (mem_addr_I)    ,
		.mem_wdata  (mem_wdata_I)   ,
		.mem_rdata  (mem_rdata_I)   ,
		.mem_ready  (mem_ready_I)
	);

	TestBed testbed(
		.clk        (clk)           ,
		.rst        (rst_n)         ,
		.addr       (DCACHE_addr)   ,
		.data       (DCACHE_wdata)  ,
		.wen        (DCACHE_wen)    ,
		.error_num  (error_num)     ,
		.duration   (duration)      ,
		.finish     (finish)
	);
	
`ifdef SDF
    initial $sdf_annotate(`SDFFILE, chip0);
`endif
	
// Initialize the data memory
	initial begin
		// for experiments
		mem_cycle = 0;
		mem_D_access = 0;
		mem_I_access = 0;

		$display("-----------------------------------------------------\n");
	 	$display("START!!! Simulation Start .....\n");
	 	$display("-----------------------------------------------------\n");
		$readmemb (`DMEM_INIT, slow_memD.mem ); // initialize data in DMEM
		$readmemb (`IMEM_INIT, slow_memI.mem ); // initialize data in IMEM

		// waveform dump
	    // $dumpfile("Final.vcd");
	    // $dumpvars;
	    $fsdbDumpfile("Final.fsdb");			
		$fsdbDumpvars(0,Final_tb,"+mda");
		$fsdbDumpvars;
	
		clk = 0;
		rst_n = 1'b1;
		#(`CYCLE*0.2) rst_n = 1'b0;
		#(`CYCLE*8.5) rst_n = 1'b1;
     
		#(`CYCLE*50000000) // calculate clock cycles for all operation (you can modify it)
		$display("============================================================================");
		$display("\n           Error!!! There is something wrong with your code ...!          ");
		$display("\n                       The test result is .....FAIL                     \n");
		$display("============================================================================");
		if (testbed.curstate == 2'b0)
			$display("Possible solution: The first answer may not be correct.\n");
		if (testbed.curstate == 2'b1)
			$display("Possible solution: The clock cycles may be too small. Please modify it.\n");
	 	$finish;
	end
		
	always #(`CYCLE*0.5) clk = ~clk;
	
	always@(finish)
	    if(finish) begin
			$display("============================================================================");
			$display("\ntotal memory access cycle: %d", mem_cycle);
			$display("total D_mem access: %d", mem_D_access);
			$display("total I_mem access: %d\n", mem_I_access);
			$display("============================================================================");
		    #(`CYCLE) $finish;		   
		end
	
	// for experiments
	always @(posedge clk) begin
		if (mem_read_I | mem_read_D | mem_write_D)
			mem_cycle = mem_cycle + 1;
	end
	always @(posedge clk) begin
		if (mem_ready_I)
			mem_I_access = mem_I_access + 1;
	end
	always @(posedge clk) begin
		if (mem_ready_D)
			mem_D_access = mem_D_access + 1;
	end

endmodule
