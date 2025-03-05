module Register_File(
	input logic clk, reg_write,
	input logic [4:0] rs1, rs2, rd, 
	input logic [31:0] wdata,
	output logic [31:0] rdata1, rdata2
);

	logic [31:0] registers [0:31];

	always_ff @(negedge clk) begin
		registers[0] <= 32'b0;
		if (reg_write)
			registers[rd] <= wdata;
	end

	assign rdata1 = registers[rs1];
	assign rdata2 = registers[rs2];

endmodule