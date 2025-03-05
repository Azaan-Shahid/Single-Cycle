module tb_processor;

logic clk;
logic reset;

// Instantiate the Processor
Datapath dut (
    .clk(clk),
    .reset(reset)
);

// Clock Generation
always begin
    #5 clk = ~clk; // 10ns clock period
end

always_ff @(negedge clk) begin
	$display("REG[1] = %h", dut.reg_file_0.registers[1]);
end

// Testbench
initial begin
    // Initialize signals
    clk = 0;
    reset = 1;

    // Instruction Memory
    dut.reg_file_0.registers[1] = 32'd2;
    dut.reg_file_0.registers[2] = 32'd4;
    dut.reg_file_0.registers[3] = 32'd6;
    dut.reg_file_0.registers[4] = 32'd8;
    dut.reg_file_0.registers[5] = 32'd10;
    dut.reg_file_0.registers[6] = 32'd12;
    dut.reg_file_0.registers[7] = 32'd14;
    dut.reg_file_0.registers[8] = 32'd16;
    dut.reg_file_0.registers[9] = 32'd18;
    dut.reg_file_0.registers[10] = 32'd20;

    // Data Memory
    dut.data_mem_0.memory[0] = 32'd5;
    dut.data_mem_0.memory[1] = 32'd10;
    dut.data_mem_0.memory[2] = 32'd20;
    dut.data_mem_0.memory[3] = 32'd30;
    dut.data_mem_0.memory[4] = 32'd40;


    dut.inst_mem_0.memory[0] = 32'hfd010113;
    dut.inst_mem_0.memory[1] = 32'h02812623;
    dut.inst_mem_0.memory[2] = 32'h03010413;
    dut.inst_mem_0.memory[3] = 32'h00500793;
    dut.inst_mem_0.memory[4] = 32'hfef42023;
    dut.inst_mem_0.memory[5] = 32'h00a00793;
    dut.inst_mem_0.memory[6] = 32'hfcf42e23;
    dut.inst_mem_0.memory[7] = 32'hfe042703;
    dut.inst_mem_0.memory[8] = 32'hfdc42783;
    dut.inst_mem_0.memory[9] = 32'h00f707b3;
    dut.inst_mem_0.memory[10] = 32'hfcf42c23;
    dut.inst_mem_0.memory[11] = 32'hfd842703;
    dut.inst_mem_0.memory[12] = 32'hfe042783;
    dut.inst_mem_0.memory[13] = 32'h40f707b3;
    dut.inst_mem_0.memory[14] = 32'hfcf42a23;
    dut.inst_mem_0.memory[15] = 32'hfe042623;
    dut.inst_mem_0.memory[16] = 32'hfe042423;
    dut.inst_mem_0.memory[17] = 32'hfe042223;
    dut.inst_mem_0.memory[18] = 32'h0200006f;
    dut.inst_mem_0.memory[19] = 32'hfec42703;
    dut.inst_mem_0.memory[20] = 32'hfd442783;
    dut.inst_mem_0.memory[21] = 32'h00f707b3;
    dut.inst_mem_0.memory[22] = 32'hfef42623;
    dut.inst_mem_0.memory[23] = 32'hfe442783;
    dut.inst_mem_0.memory[24] = 32'h00178793;
    dut.inst_mem_0.memory[25] = 32'hfef42223;
    dut.inst_mem_0.memory[26] = 32'hfe442703;
    dut.inst_mem_0.memory[27] = 32'hfdc42783;
    dut.inst_mem_0.memory[28] = 32'hfcf74ee3;
    dut.inst_mem_0.memory[29] = 32'h0200006f;
    dut.inst_mem_0.memory[30] = 32'hfec42703;
    dut.inst_mem_0.memory[31] = 32'hfe042783;
    dut.inst_mem_0.memory[32] = 32'h40f707b3;
    dut.inst_mem_0.memory[33] = 32'hfef42623;
    dut.inst_mem_0.memory[34] = 32'hfe842783;
    dut.inst_mem_0.memory[35] = 32'h00178793;
    dut.inst_mem_0.memory[36] = 32'hfef42423;
    dut.inst_mem_0.memory[37] = 32'hfec42703;
    dut.inst_mem_0.memory[38] = 32'hfe042783;
    dut.inst_mem_0.memory[39] = 32'hfcf75ee3;
    dut.inst_mem_0.memory[40] = 32'h0000006f;


    // Deassert reset
    #10 reset = 0;

    // Display
    #125 $display("MEM[2] = %h", dut.data_mem_0.memory[2]);

    // Run simulation for a set number of clock cycles
    #220;
    $finish;
end

endmodule