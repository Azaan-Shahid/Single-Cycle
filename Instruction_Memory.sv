module Instruction_Memory(
	input logic [31:0] address,
	output logic [31:0] instruction
);
	
	logic [31:0] memory [0:1023];

	assign instruction = memory[address[11:2]];

endmodule