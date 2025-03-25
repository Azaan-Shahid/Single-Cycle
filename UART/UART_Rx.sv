
module Rx_FIFO (
    input logic clk, reset,
    input logic [7:0] data_in,
    input logic wr_en, data_read,  
    output logic [7:0] data_out,
    output logic data_available
);
    
    logic [7:0] fifo [7:0];
    logic [2:0] wr_ctr, rd_ctr;
    logic [3:0] stored_bytes;
    logic rxff, fifo_empty;

    always @(posedge clk) begin
        if (reset) begin
            for (int i = 0; i < 8; i++)
                fifo[i] <= 8'b0;
            wr_ctr = 3'b0;
            rd_ctr = 3'b0;
            stored_bytes <= 4'b0;
        end
        else begin
            if (wr_en && ~rxff) begin
                fifo[wr_ctr] <= data_in;
                stored_bytes <= stored_bytes + 1;
                if (wr_ctr == 3'd7) 
                    wr_ctr <= 3'b0;
                else
                    wr_ctr <= wr_ctr + 1;
            end 
            // When data has been read by processor
            else if (data_read) begin
                stored_bytes <= stored_bytes - 1;
                if (rd_ctr == 3'd7)
                    rd_ctr <= 3'b0;
                else
                    rd_ctr <= rd_ctr + 1;
            end
        end

    end

    assign rxff = (stored_bytes == 4'd8) ? 1'b1 : 1'b0;
    assign fifo_empty = (stored_bytes == 3'b0) ? 1'b1 : 1'b0;
    assign data_out = fifo_empty ? fifo[rd_ctr] : 8'b0;         
    assign data_available = ~fifo_empty;

endmodule




module Rx_Baud_Counter (
    input logic clk, reset,
    input logic [11:0] baud_div,
    output logic baud_comp
);
    
    logic [11:0] counter;

    always @(posedge clk) begin
        if (reset || baud_comp) begin
            baud_comp <= 1'b0;
            counter <= 12'b0;
        end
        else if (counter == baud_div)
            baud_comp <= 1'b1;
        else begin
            baud_comp <= 1'b0;
            counter <= counter + 1;
        end
    end

endmodule




module Rx_Controller (
	input logic clk, reset,
	input start_detected, rx_done, parity_error, stop_bit_error,
	output logic rx_start, rx_sel, store_en
);

	logic [1:0] c_state, n_state;
	parameter IDLE=2'b00, START=2'b01, RECEIVE=2'b10, STORE=2'b11;

	always_ff @ (posedge clk) begin
	if (reset)
		c_state <= IDLE;
	else
		c_state <= n_state;
	end

	always_comb	begin
		case (c_state)
			IDLE: begin 
				if (start_detected) n_state = START;
				else n_state = IDLE; 
			end
			START: begin 
				n_state = RECEIVE;
			end
			RECEIVE: begin 
				if (rx_done) begin
					if (parity_error || stop_bit_error) n_state = IDLE;
					else n_state = STORE;
				end
				else n_state = RECEIVE;
			end
			STORE: begin
				n_state = IDLE;
			end
			default: n_state = IDLE;
		endcase
	end

	always_comb begin
		case (c_state)
			IDLE: begin
				rx_start = 1'b0;
				rx_sel = 1'b1;
				store_en = 1'b0;
			end
			START: begin
				rx_start = 1'b1;
				rx_sel = 1'b1;
				store_en = 1'b0;
			end
			RECEIVE: begin
				rx_start = 1'b0;
				rx_sel = 1'b1;
				store_en = 1'b0;
			end
			STORE: begin
				rx_start = 1'b0;
				rx_sel = 1'b0;
				store_en = 1'b1;
			end
		endcase
	end
	
endmodule



module Rx_Datapath (
    input logic clk, reset,
    input logic rx_in,
    input logic [11:0] baud_divisor,
    input logic parity_sel, two_stop_bits, rx_start, rx_sel,
    output logic rx_done, start_detected, parity_error, stop_bit_error,
    output logic [7:0] data_out
);

    logic [11:0] shift_reg;
    logic [3:0] packet_size, shift_ctr;
    logic rx, rx_shift_en, parity;

    always_ff @(posedge clk) begin
        if (reset || rx_start) begin
            shift_ctr <= packet_size;
            rx_done <= 1'b0;
        end
        else if (rx_shift_en) begin
            if (shift_ctr == 3'b0)
                rx_done <= 1'b1;
            else begin
                shift_reg[shift_ctr] <= rx;
                shift_reg <= shift_reg >> 1;
                shift_ctr <= shift_ctr - 1;
                rx_done <= 1'b0;
            end
        end
    end


    // If parity_sel is 1, it is even else odd
    assign parity = parity_sel ? ~(^shift_reg[8:1]) : (^shift_reg[8:1]);
    assign parity_error = (parity == shift_reg[9]) ? 1'b0 : 1'b1;
    assign stop_bit_error = two_stop_bits ? ~(shift_reg[10] & shift_reg[11]) : ~shift_reg[10];


    assign rx = rx_sel ? rx_in : 1'b1;
    assign start_detected = rx ? 1'b0 : 1'b1;
    assign packet_size = two_stop_bits ? 4'd12 : 4'd11;

    Rx_Baud_Counter baud_counter_0 (clk, (reset | rx_start), baud_divisor, rx_shift_en);

endmodule



module UART_Rx (
    input logic clk, reset,
    input logic [11:0] baud_divisor,
    input logic parity_sel, two_stop_bits, rx_in, fifo_rd,
    output logic tx_out, data_available
);

    logic [7:0] fifo_data_in, fifo_data_out;
    logic rx_start, rx_sel, rx_done;
    logic parity_error, stop_bit_error, start_detected;

    Rx_FIFO rx_fifo_0 (clk, reset, fifo_data_in, store_en, fifo_rd, fifo_data_out, data_available);   
    Rx_DataPath datapath_0 (clk, reset, rx_in, baud_divisor, parity_sel, two_stop_bits, rx_start, rx_sel, rx_done, start_detected, parity_error, stop_bit_error, fifo_data_in);
    Rx_Controller controller_0 (clk, reset, start_detected, rx_done, parity_error, stop_bit_error, rx_start, rx_sel, store_en);

endmodule




