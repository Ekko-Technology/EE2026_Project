`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 18.10.2025 03:18:31
// Design Name: 
// Module Name: FIFO_UDFS
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module FIFO_UDFS (
    // input wire clk,
    // input wire rst,

    // // write side
    // input wire wr_en,
    // input wire [WIDTH-1:0] wr_data,

    // // read side
    // input wire rd_en,
    // output wire [WIDTH-1:0] rd_data,
);

    // localparam WIDTH = 4,       // {fs, ls, fe, px}
    // localparam DEPTH = 32,      // power-of-two recommended
    // localparam ADDR  = 5        // log2(DEPTH)
    // reg [WIDTH-1:0] mem [0:DEPTH-1];
    // reg [ADDR-1:0] wptr;
    // reg [ADDR-1:0] rptr;
    // reg [ADDR:0] count;

    // assign rd_data = mem[rptr];

    // always @(posedge clk) begin
    //     if (rst) begin
    //         wptr  <= {ADDR{1'b0}};
    //         rptr  <= {ADDR{1'b0}};
    //         count <= {(ADDR+1){1'b0}};
    //     end else begin
    //         // write
    //         if (wr_en && !full) begin
    //             mem[wptr] <= wr_data;
    //             wptr <= wptr + 1'b1;
    //             count <= count + 1'b1;
    //         end
    //         // read
    //         if (rd_en && !empty) begin
    //             rptr <= rptr + 1'b1;
    //             count <= count - 1'b1;
    //         end
    //     end
    // end
endmodule
