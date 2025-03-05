module Datapath(
    input logic clk, reset
);
    logic [31:0] pc, next_pc, extended_imm, instruction, rdata1, rdata2, alu_result, rdata, wdata;
    logic [4:0] rs1, rs2, rd;
    logic [6:0] opcode;
    logic [3:0] alu_ctrl;
    logic [2:0] br_type;
    logic [1:0] wb_sel;
    logic reg_wr, rd_en, wr_en, br_taken, sel_A, sel_B;

    // Instantiate modules
    Program_Counter pc_0 (clk, reset, br_taken ? alu_result : next_pc, pc);
    Instruction_Memory inst_mem_0 (pc, instruction);
    Register_File reg_file_0 (clk, reg_wr, rs1, rs2, rd, wdata, rdata1, rdata2);
    ALU alu_0 (alu_ctrl, sel_A ? rdata1 : pc, sel_B ? extended_imm : rdata2, alu_result);
    Data_Memory data_mem_0 (clk, wr_en, rd_en, alu_result, rdata2, rdata);
    Branch_Conditions branch_cond_0 (br_type, rdata1, rdata2, br_taken);
    Controller control_unit_0 (instruction, reg_wr, rd_en, wr_en, sel_A, sel_B, wb_sel, alu_ctrl, br_type);
    Immediate imd_generator_0 (instruction, extended_imm);

    // Assignments
    assign opcode = instruction[6:0];
    assign rs1 = instruction[19:15];
    assign rs2 = instruction[24:20];
    assign rd = instruction[11:7];
    assign next_pc = br_taken ? alu_result : pc + 4;

    always_comb begin
         case (wb_sel)
            2'b00: wdata = pc + 4;
            2'b01: wdata = alu_result;
            2'b10: wdata = rdata;
        endcase
    end

endmodule
