module Data_Memory(
	input logic clk, wr_en, rd_en,
	input logic [31:0] addr, wdata,
	output logic [31:0] rdata
);
	
	logic [31:0] memory [1023:0];
	
	always_ff @(negedge clk) begin
		if (wr_en)
			memory[addr] <= wdata;
	end

	assign rdata = (rd_en) ? memory[addr] : 32'b0;

endmodule