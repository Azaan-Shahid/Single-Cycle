module mux21_1bit(
      input logic s,a,b,
      output logic y
);


      assign y = s ? b:a;
endmodule



module baud_counter(
       input logic clear, clk,
       output logic [13:0]baud_out);


       always_ff @ (posedge clk) begin
              if (clear) begin
                   baud out <= 0;
                   end
              else if (baud_out==5) begin
                   baud_out <= 0;
                   end
              else begin
                   baud_out <= baud_out+1;
                   end

       end

endmodule



module bit_counter(
            input logic enable, clear, clk,
            output logic [3:0]bit_out);


            always_ff @ (posedge clk) begin
                   if (clear) begin
                        bit_out <=0;
                        end
                   else if (enable) begin
                        if (bit_out == 9) begin
                            bit_out<= 0;
                            end
                        else begin
                             bit_out <= bit_out+1;
                             end


                    end
            end


endmodule



module shift(
       input logic clk, load_byte, shift, reset,
       input logic [7:0]data,
       output logic serial_out);


       logic [8:0] data_in;


       always_ff @(posedge clk) begin
              if (reset) begin
                    serial_out<= 1; 
                    end
              else begin
                 if (load_byte==1) begin
                        data_in ={data,1'b0};
                        end
                 if (shift) begin
                        serial_out<= data_in[0];
                        data_in = {1'b1, data_in[8:1]};
                        end
              end
        end

endmodule





module datapath(
          input logic clk, reset, clear, clear_baud, start, shift, load_shift,
          input logic [7:0] data,
          output logic counter, counter_baud, Tx);

          logic [13:0]baud_out;
          logic [3:0] bit_out;
          logic serial_out;
          assign a = 1'b1;


          shift S1(.clk(clk), .load_byte(load_shift), .shift(shift), .reset(reset),
.data(data),.serial_out(serial_out));


          baud_counter BD(.clear(clear_baud), .clk(clk), .baud_out(baud_out));

          bit_counter BIT(.enable(shift), .clear(clear), .clk(clk), .bit_out(bit_out));

          mux21_1bit M0(.s(start), .a(a), .b(serial_out), .y(Tx));


          always_comb begin


             if (bit_out == 9) begin
                  counter = 1'b1;
                  end
             else begin
                  counter = 1'b0;
                  end


             if (baud_out == 5) begin 
                  counter_baud = 1'b1;
                  end
             else begin
                  counter_baud = 1'b0;
                  end


          end


endmodule


module FSM(
     input logic t_byte, byte_ready, counter, counter_baud, clk, reset,
     output logic clear, clear_baud, start, shift, load_shift, load_dreg);


     typedef enum logic [1:0] {S0, S1, S2) statetype;
     statetype cs, ns;


     always_ff @(posedge clk)
            if (reset) cs <= S0;
            else       cs <= ns;


         // next state and output logic
    always_comb begin
    clear = 1'b0;
    clear_baud =1'b0;
    start  = 1'b0;
    shift  = 1'b0;
    load_shift = 1'b0;
    load_dreg = 1'b0;



  case (cs)
        S0:     if (byte_ready==1'b1) begin
                       ns = S1;
                       clear = 1'b1;
                       clear_baud = 1'b1;
                       start = 1'b0;
                       shift=1'b0;
                       load_shift=1'b0;
                       load_dreg=1'b0;
                       end
                   else begin
                       ns = S0;
                       clear = 1'b1;
                       clear_baud = 1'b1;
                       start = 1'b0;
                       shift = 1'b0;
                       load_shift = 1'b0;
                       load_dreg = 1'b1;
                       end


         S1:     if (t_byte==1'b1) begin
                       ns = S2;
                       clear = 1'b1;
                       clear_baud = 1'b1;
                       start = 1'b0;
                       shift=1'b0;
                       load_shift=1'b0;
                       load_dreg=1'b0;
                       end
                   else begin
                       ns = S1;
                       clear = 1'b1;
                       clear_baud = 1'b1;
                       start = 1'b0;
                       shift = 1'b0;
                       load_shift = 1'b1;
                       load_dreg = 1'b0;
                       end


         S2:     if (counter==1'b1 & counter_baud==1'b1) begin
                       ns = S0;
                       clear = 1'b0;
                       clear_baud = 1'b0;
                       start = 1'b0;
                       shift=1'b0;
                       load_shift=1'b0;
                       load_dreg=1'b0;
                       end
                   else if (counter==1'b1 & counter_baud==1'b0) begin
                       ns = S2;
                       clear = 1'b0;
                       clear_baud = 1'b0;
                       start = 1'b1;
                       shift = 1'b0;
                       load_shift = 1'b0;
                       load_dreg = 1'b0;
                       end


                   else if (counter==1'b0 & counter_baud==1'b0) begin
                       ns = S2;
                       clear = 1'b0;
                       clear_baud = 1'b0;
                       start = 1'b1;
                       shift=1'b0;
                       load_shift=1'b0;
                       load_dreg=1'b0;
                       end


                   else if (counter==1'b0 & counter_baud==1'b1) begin
                       ns = S2;
                       clear = 1'b0;
                       clear_baud = 1'b0;
                       start = 1'b1;
                       shift = 1'b1;
                       load_shift = 1'b0;
                       load_dreg = 1'b1;
                       end


             default :   ns = S0;
       endcase
     end

endmodule





