`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Simple LUTRAM implementation of dual port buffer 
//////////////////////////////////////////////////////////////////////////////////


module Dual_Port_Buffer
#(  parameter DATA_WIDTH = 1,
    parameter BUFFER_SIZE = 146880)  // 2 * 306 * 240

(
    input clka, // pclk 24 MHz
    input we,
    input [17:0] addra,    //18-bit address covers 2*73440 = 146880 pixels
    input dina,    //1 bit white/black pixel data
    input clkb, // clk25 25 MHz
    input [17:0] addrb,     //18-bit address covers 2*73440 = 146880 pixels
    output reg doutb    //1 bit white/black pixel data
    );

    (* ram_style = "distributed" *)
    reg [DATA_WIDTH-1:0] lutram [BUFFER_SIZE-1:0];

    always @(posedge clka) begin
        if (we) begin
            lutram[addra] <= dina;
        end
    end

    always @(posedge clkb) begin
        doutb <= lutram[addrb];
    end

endmodule