module Immediate (
    input logic [31:0] instruction,
    output logic [31:0] extended_imm
);
    logic [6:0] opcode;
    assign opcode = instruction[6:0];

    always_comb begin

        if (opcode == 7'b0100011) // S-Type
            extended_imm = {{20{instruction[31]}}, instruction[31:25], instruction[11:7]};
        else if (opcode == 7'b1100011) // B-Type
            extended_imm = {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
        else if (opcode == 7'b0110111 || opcode == 7'b0010111) // U-Type 
            extended_imm = {instruction[31:12], 12'b0};
        else if (opcode == 7'b1101111) // J-Type
            extended_imm = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};
        else
            extended_imm = {{20{instruction[31]}}, instruction[31:20]};

    end

endmodule