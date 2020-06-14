// this is a test bench feeds initial instruction and data
// the processor output is not verified

`timescale 1 ns/10 ps

`define CYCLE 10 // You can modify your clock frequency

module tb;

	reg clk;
	reg rst_n;
    reg in_valid;

    reg [31:0] mplier, mcand;
	
	wire [63:0] product;
    wire out_valid, stall;
	
    integer i;

	IterMultiplier multi(
        clk,
        rst_n,
        in_valid,
        mplier,
        mcand,
        product,
        out_valid,
        stall
    );
	
initial begin
		$dumpfile("Multi.fsdb");
     	$dumpvars;
    end

	initial begin
		i = 0;
	end

	initial begin
		clk        = 1'b0;
     	rst_n      = 1'b1;
		#2.5 rst_n = 1'b0;
    	#5 rst_n   = 1'b1;
	end

    initial begin
        #(`CYCLE)
        in_valid = 1;
        mplier = 5;
        mcand = 6;
        #(`CYCLE)
        #(35*`CYCLE)
        in_valid = 1;
        mplier = 6;
        mcand = 7;
        // mplier = {32{1'b1}};
        // mcand = {32{1'b1}};
        #(`CYCLE)
        #(35*`CYCLE) $finish;
    end

	always begin #(`CYCLE/2) clk = ~clk; end

	// always @(posedge clk) begin
	// 	if (out_valid) begin
    //         #(`CYCLE*2) $finish;
    //     end
	// end
endmodule