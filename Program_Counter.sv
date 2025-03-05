module Program_Counter(
	input logic clk, reset,
	input logic [31:0] next_pc,
	output logic [31:0] pc
);
	
	always_ff @(posedge clk) begin

		if (reset)
			pc <= 32'b0;
		else
			pc <= next_pc;
	end

endmodule