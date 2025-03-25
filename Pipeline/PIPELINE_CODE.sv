
module riscv_pipeline (
    input logic clk, reset
);
    // Pipeline registers
    logic [31:0] pc_f, pc_d, pc_m;           // Program counter for each stage
    logic [31:0] instruction_f, instruction_d, instruction_m; 
    logic [31:0] alu_result_d, alu_result_m; 
    logic [31:0] rdata1_d, rdata2_d;         
    logic [31:0] wdata_m;                    
    logic [31:0] immediate_d;                
    logic [31:0] rdata_m;                    

    
    logic reg_wr_f, reg_wr_d, reg_wr_m;      
    logic rd_en_f, rd_en_d, rd_en_m;         
    logic wr_en_f, wr_en_d, wr_en_m;         
    logic [1:0] wb_sel_f, wb_sel_d, wb_sel_m; 
    logic [3:0] alu_op_d;                   
    logic sel_A_d, sel_B_d;                  
    logic br_taken_d;                       

    // Instantiate modules
    pc pc_inst (.clk(clk), .reset(reset), .next_pc(br_taken_d ? alu_result_d : pc_f + 4), .pc(pc_f));
    inst_mem inst_mem_inst (.address(pc_f), .instruction(instruction_f));
    reg_file reg_file_inst (.clk(clk), .reg_write(reg_wr_m), .rs1(instruction_d[19:15]), .rs2(instruction_d[24:20]), 
                            .rd(instruction_m[11:7]), .wdata(wdata_m), .rdata1(rdata1_d), .rdata2(rdata2_d));
    alu alu_inst (.alu_op(alu_op_d), .A(sel_A_d ? rdata1_d : pc_d), .B(sel_B_d ? immediate_d : rdata2_d), .C(alu_result_d));
    data_mem data_mem_inst (.clk(clk), .wr_en(wr_en_m), .rd_en(rd_en_m), .addr(alu_result_m), .wdata(rdata2_d), .rdata(rdata_m));
    branch_cond branch_cond_inst (.br_type(instruction_d[14:12]), .rdata1(rdata1_d), .rdata2(rdata2_d), .br_taken(br_taken_d));
    control_unit control_unit_inst (.inst(instruction_d), .reg_wr(reg_wr_d), .rd_en(rd_en_d), .wr_en(wr_en_d), 
                                    .sel_A(sel_A_d), .sel_B(sel_B_d), .wb_sel(wb_sel_d), .alu_op(alu_op_d), .br_type());
    imd_generator imd_generator_inst (.instruction(instruction_d), .extended_imm(immediate_d));

    // Pipeline registers: Fetch -> Decode/Execute
    always_ff @(posedge clk) begin
        if (reset) begin
            pc_d <= 32'b0;
            instruction_d <= 32'b0;
            reg_wr_d <= 1'b0;
            rd_en_d <= 1'b0;
            wr_en_d <= 1'b0;
            wb_sel_d <= 2'b0;
        end else begin
            pc_d <= pc_f;
            instruction_d <= instruction_f;
            reg_wr_d <= reg_wr_f;
            rd_en_d <= rd_en_f;
            wr_en_d <= wr_en_f;
            wb_sel_d <= wb_sel_f;
        end
    end

    // Pipeline registers: Decode/execute -> Memory/Writeback
    always_ff @(posedge clk) begin
        if (reset) begin
            pc_m <= 32'b0;
            instruction_m <= 32'b0;
            alu_result_m <= 32'b0;
            reg_wr_m <= 1'b0;
            rd_en_m <= 1'b0;
            wr_en_m <= 1'b0;
            wb_sel_m <= 2'b0;
        end else begin
            pc_m <= pc_d;
            instruction_m <= instruction_d;
            alu_result_m <= alu_result_d;
            reg_wr_m <= reg_wr_d;
            rd_en_m <= rd_en_d;
            wr_en_m <= wr_en_d;
            wb_sel_m <= wb_sel_d;
        end
    end

   
    always_comb begin
        case (wb_sel_m)
            2'b00: wdata_m = pc_m + 4; // PC + 4 for JAL/JALR
            2'b01: wdata_m = alu_result_m; // ALU result
            2'b10: wdata_m = rdata_m; // Memory read data
            default: wdata_m = 32'b0;
        endcase
    end
endmodule


module alu(
    input logic [3:0] alu_op,
    input logic [31:0] A, B,
    output logic [31:0] C
);
    always_comb begin
        case (alu_op)
            4'b0000: C = A + B; // ADD
            4'b0001: C = A - B; // SUB
            4'b0010: C = A << B[4:0]; // SLL
            4'b0011: C = A >> B[4:0]; // SRL
            4'b0100: C = $signed(A) >>> B[4:0]; // SRA
            4'b0101: C = ($signed(A) < $signed(B)) ? 1 : 0; // SLT
            4'b0110: C = (A < B) ? 1 : 0; // SLTU
            4'b0111: C = A ^ B; // XOR
            4'b1000: C = A | B; // OR
            4'b1001: C = A & B; // AND
            4'b1010: C = B; // Just Pass B
            default: C = 32'b0;
        endcase
    end
endmodule

module branch_cond (
    input logic [2:0] br_type,
    input logic [31:0] rdata1, rdata2,
    output logic br_taken
);
    always_comb begin
        case (br_type)
            3'b000: br_taken = 0;
            3'b001: br_taken = (rdata1 == rdata2);  // BEQ
            3'b010: br_taken = (rdata1 != rdata2);  // BNE
            3'b011: br_taken = ($signed(rdata1) < $signed(rdata2));  // BLT
            3'b100: br_taken = ($signed(rdata1) >= $signed(rdata2)); // BGE
            3'b101: br_taken = (rdata1 < rdata2);   // BLTU
            3'b110: br_taken = (rdata1 >= rdata2);  // BGEU
            3'b111: br_taken = 1;                   // Unconditional Jump
            default: br_taken = 0;                  // Default case
        endcase
    end
endmodule

module control_unit (
    input logic [31:0] inst,
    output logic reg_wr, rd_en, wr_en, sel_A, sel_B,
    output logic [1:0] wb_sel,
    output logic [3:0] alu_op,
    output logic [2:0] br_type
);
    logic [4:0] rs1, rs2, rd;
    logic [6:0] opcode, func7;
    logic [2:0] func3;

    assign opcode = inst[6:0];
    assign rd = inst[11:7];
    assign func3 = inst[14:12];
    assign rs1 = inst[19:15];
    assign rs2 = inst[24:20];
    assign func7 = inst[31:25];

    always_comb begin
        // R-Type
        if (opcode == 7'b0110011) begin
            reg_wr = 1;
            rd_en = 0;
            wr_en = 0;
            sel_A = 1;
            sel_B = 0;
            wb_sel = 2'b01;
            br_type = 3'b000;
            if (func3 == 3'b000 && func7 == 7'b0000000) // ADD
                alu_op = 4'b0000;
            else if (func3 == 3'b000 && func7 == 7'b0100000) // SUB
                alu_op = 4'b0001;
            else if (func3 == 3'b001 && func7 == 7'b0000000) // SLL
                alu_op = 4'b0010;
            else if (func3 == 3'b101 && func7 == 7'b0000000) // SRL
                alu_op = 4'b0011;
            else if (func3 == 3'b101 && func7 == 7'b0100000) // SRA
                alu_op = 4'b0100;
            else if (func3 == 3'b010 && func7 == 7'b0000000) // SLT
                alu_op = 4'b0101;
            else if (func3 == 3'b011 && func7 == 7'b0000000) // SLTU
                alu_op = 4'b0110;
            else if (func3 == 3'b100 && func7 == 7'b0000000) // XOR
                alu_op = 4'b0111;            
            else if (func3 == 3'b110 && func7 == 7'b0000000) // OR
                alu_op = 4'b1000;
            else if (func3 == 3'b111 && func7 == 7'b0000000) // AND
                alu_op = 4'b1001;
        end

        // I-Type
        else if (opcode == 7'b0010011) begin
            reg_wr = 1;
            rd_en = 0;
            wr_en = 0;
            sel_A = 1;
            sel_B = 1;
            wb_sel = 2'b01;
            br_type = 3'b000;
            if (func3 == 3'b000) // ADDI
                alu_op = 4'b0000;
            else if (func3 == 3'b001) // SLLI
                alu_op = 4'b0010;
            else if (func3 == 3'b101 && func7 == 7'b0000000) //SRLI
                alu_op = 4'b0011;
            else if (func3 == 3'b101 && func7 == 7'b0100000) //SRAI
                alu_op = 4'b0100;
            else if (func3 == 3'b010) // SLTI
                alu_op = 4'b0101;
            else if (func3 == 3'b011) // SLTIU
                alu_op = 4'b0110;
            else if (func3 == 3'b100) // XOR
                alu_op = 4'b0111;
            else if (func3 == 3'b110) // OR
                alu_op = 4'b1000;
            else if (func3 == 3'b111) // AND
                alu_op = 4'b1001;
        end

        // I-Type (Load)
        else if (opcode == 7'b0000011) begin
            reg_wr = 1;
            rd_en = 1;
            wr_en = 0;
            sel_A = 1;
            sel_B = 1;
            wb_sel = 2'b10;
            br_type = 3'b000;
            alu_op = 4'b0000;
        end

        // S-Type   
        else if (opcode == 7'b0100011) begin
            reg_wr = 0;
            rd_en = 0;
            wr_en = 1;
            sel_A = 1;
            sel_B = 1;
            wb_sel = 2'b01;
            br_type = 3'b000;
            alu_op = 4'b0000;
        end

        // B-Type   
        else if (opcode == 7'b1100011) begin
            reg_wr = 0;
            rd_en = 0;
            wr_en = 0;
            sel_A = 0;
            sel_B = 1;
            wb_sel = 2'b01;
            alu_op = 4'b0000;

            if (func3 == 3'b000) // BEQ
                br_type = 3'b001;
            else if (func3 == 3'b001) // BNE
                br_type = 3'b010;
            else if (func3 == 3'b100) // BLT
                br_type = 3'b011;
            else if (func3 == 3'b101) // BGE
                br_type = 3'b100;
            else if (func3 == 3'b110) // BLTU
                br_type = 3'b101;
            else if (func3 == 3'b111) // BGEU
                br_type = 3'b110;
        end

        // U-Type   
        else if (opcode == 7'b0110111) begin // LUI
            reg_wr = 1;
            rd_en = 0;
            wr_en = 0;
            sel_A = 0;
            sel_B = 1;
            wb_sel = 2'b01;
            br_type = 3'b000;
            alu_op = 4'b1010;
        end
        else if (opcode == 7'b0010111) begin // AUIPC
            reg_wr = 1;
            rd_en = 0;
            wr_en = 0;
            sel_A = 0;
            sel_B = 1;
            wb_sel = 2'b01;
            br_type = 3'b000;
            alu_op = 4'b0000;
        end

        // J-Type
        else if (opcode == 7'b1101111) begin // JAL
            reg_wr = 1;
            rd_en = 0;
            wr_en = 0;
            sel_A = 0;
            sel_B = 1;
            wb_sel = 2'b00;
            br_type = 3'b111;
            alu_op = 4'b0000;
        end
        else if (opcode == 7'b1100111) begin // JALR
            reg_wr = 1;
            rd_en = 0;
            wr_en = 0;
            sel_A = 1;
            sel_B = 1;
            wb_sel = 2'b00;
            br_type = 3'b111;
            alu_op = 4'b0000;
        end
    end
endmodule

module data_mem(
    input logic clk, wr_en, rd_en,
    input logic [31:0] addr, wdata,
    output logic [31:0] rdata
);
    logic [31:0] memory [0:1023];

    always_ff @(negedge clk) begin
        if (wr_en)
            memory[addr] <= wdata;
    end

    assign rdata = (rd_en) ? memory[addr] : 32'b0;
endmodule

module imd_generator (
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
            extended_imm = {{11{instruction[31]}}, instruction[19:12], instruction[20], instruction[30:21], 1'b0};
        else
            extended_imm = {{20{instruction[31]}}, instruction[31:20]};
    end
endmodule

module inst_mem(
    input logic [31:0] address,
    output logic [31:0] instruction
);
    logic [31:0] memory [0:1023];

    assign instruction = memory[address[11:2]];
endmodule

module pc(
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

module reg_file(
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