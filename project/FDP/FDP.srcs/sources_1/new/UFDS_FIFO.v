`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 18.10.2025 03:18:31
// Design Name: 
// Module Name: UFDS_FIFO
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


// Asynchronous (dual-clock) FIFO: write at wr_clk (25 MHz), read at rd_clk (100 MHz)
module UFDS_FIFO (
    // write clock domain (25 MHz)
    input wire wr_clk,
    input wire wr_rst,
    input wire wr_en,
    input wire [WIDTH-1:0] wr_data,
    output wire wr_full,
    // read clock domain (100 MHz)
    input wire rd_clk,
    input wire rd_rst,
    input wire rd_en,
    output reg [WIDTH-1:0] rd_data,
    output wire rd_empty
);
    localparam integer WIDTH = 4; // {fs, ls, fe, px}
    localparam integer DEPTH = 1024; // power of two
    localparam integer ADDR_BITS = 10; // log2(DEPTH)

    reg [WIDTH-1:0] mem [0:DEPTH-1];

    // binary and Gray pointers include extra MSB for wrap detection
    reg [ADDR_BITS:0] wptr_bin=0, rptr_bin=0;
    reg [ADDR_BITS:0] wptr_gray=0, rptr_gray=0;

    // cross-domain synchronizers
    reg [ADDR_BITS:0] rptr_gray_w1=0, rptr_gray_w2=0;
    reg [ADDR_BITS:0] wptr_gray_r1=0, wptr_gray_r2=0;

    // write domain
    wire do_write = wr_en & ~wr_full;
    wire [ADDR_BITS:0] wptr_bin_n  = wptr_bin + do_write;
    wire [ADDR_BITS:0] wptr_gray_n = (wptr_bin_n >> 1) ^ wptr_bin_n;

    // full when next write == read with MSBs inverted
    wire [ADDR_BITS:0] rgray_w = rptr_gray_w2;
    assign wr_full = (wptr_gray_n == {~rgray_w[ADDR_BITS:ADDR_BITS-1], rgray_w[ADDR_BITS-2:0]});

    always @(posedge wr_clk) begin
        if (wr_rst) begin
            wptr_bin <= 0; wptr_gray <= 0;
            rptr_gray_w1 <= 0; rptr_gray_w2 <= 0;
        end else begin
            // sync read pointer into write domain
            rptr_gray_w1 <= rptr_gray;
            rptr_gray_w2 <= rptr_gray_w1;
            // write
            if (do_write) mem[wptr_bin[ADDR_BITS-1:0]] <= wr_data;
            // advance
            wptr_bin  <= wptr_bin_n;
            wptr_gray <= wptr_gray_n;
        end
    end

    // read domain
    wire do_read = rd_en & ~rd_empty;
    wire [ADDR_BITS:0] rptr_bin_n = rptr_bin + do_read;
    wire [ADDR_BITS:0] rptr_gray_n = (rptr_bin_n >> 1) ^ rptr_bin_n;

    // empty when read pointer equals synchronized write pointer
    wire [ADDR_BITS:0] wgray_r = wptr_gray_r2;
    assign rd_empty = (rptr_gray == wgray_r);

    always @(posedge rd_clk) begin
        if (rd_rst) begin
            rptr_bin <= 0; rptr_gray <= 0;
            wptr_gray_r1 <= 0; wptr_gray_r2 <= 0;
            rd_data <= {WIDTH{1'b0}};
        end else begin
            // sync write pointer into read domain
            wptr_gray_r1 <= wptr_gray;
            wptr_gray_r2 <= wptr_gray_r1;
            // read
            if (do_read) rd_data <= mem[rptr_bin[ADDR_BITS-1:0]];
            // advance
            rptr_bin  <= rptr_bin_n;
            rptr_gray <= rptr_gray_n;
        end
    end
endmodule
