`timescale 1ns / 1ps
// Simple dual-clock asynchronous FIFO for 19-bit words (addr[17:0] + bit)
// Depth must be a power of two.

module AsyncFifo19 #(
    parameter DEPTH = 32,
    parameter ADDR_BITS = 5 // log2(DEPTH)
)(
    // Write side (pclk domain)
    input  wire              wclk,
    input  wire              wrst,
    input  wire              w_en,
    input  wire [18:0]       w_data,
    output wire              w_full,

    // Read side (clk domain)
    input  wire              rclk,
    input  wire              rrst,
    input  wire              r_en,
    output reg  [18:0]       r_data,
    output wire              r_empty
);

    // Storage
    reg [18:0] mem [0:DEPTH-1];

    // Binary and Gray pointers
    reg [ADDR_BITS:0] wptr_bin = 0, wptr_gray = 0;
    reg [ADDR_BITS:0] rptr_bin = 0, rptr_gray = 0;

    // Synchronized pointers across domains
    reg [ADDR_BITS:0] rptr_gray_wclk = 0, rptr_gray_wclk_d = 0;
    reg [ADDR_BITS:0] wptr_gray_rclk = 0, wptr_gray_rclk_d = 0;

    // Utility: binary <-> gray
    function [ADDR_BITS:0] bin2gray(input [ADDR_BITS:0] b);
        bin2gray = (b >> 1) ^ b;
    endfunction

    // Write side
    wire [ADDR_BITS:0] rptr_gray_sync_w = rptr_gray_wclk_d;
    wire [ADDR_BITS:0] wptr_bin_next = wptr_bin + (w_en & ~w_full);
    wire [ADDR_BITS:0] wptr_gray_next = bin2gray(wptr_bin_next);

    // Full when next write pointer equals read pointer with MSBs inverted
    assign w_full = (wptr_gray_next == {~rptr_gray_sync_w[ADDR_BITS:ADDR_BITS-1], rptr_gray_sync_w[ADDR_BITS-2:0]});

    always @(posedge wclk) begin
        if (wrst) begin
            wptr_bin  <= 0;
            wptr_gray <= 0;
            rptr_gray_wclk   <= 0;
            rptr_gray_wclk_d <= 0;
        end else begin
            // sync read pointer from rclk domain
            rptr_gray_wclk   <= rptr_gray;
            rptr_gray_wclk_d <= rptr_gray_wclk;

            if (w_en && ~w_full) begin
                mem[wptr_bin[ADDR_BITS-1:0]] <= w_data;
            end
            wptr_bin  <= wptr_bin_next;
            wptr_gray <= wptr_gray_next;
        end
    end

    // Read side
    wire [ADDR_BITS:0] wptr_gray_sync_r = wptr_gray_rclk_d;
    wire [ADDR_BITS:0] rptr_bin_next = rptr_bin + (r_en & ~r_empty);
    wire [ADDR_BITS:0] rptr_gray_next = bin2gray(rptr_bin_next);

    assign r_empty = (rptr_gray == wptr_gray_sync_r);

    always @(posedge rclk) begin
        if (rrst) begin
            rptr_bin  <= 0;
            rptr_gray <= 0;
            wptr_gray_rclk   <= 0;
            wptr_gray_rclk_d <= 0;
            r_data <= 19'd0;
        end else begin
            // sync write pointer from wclk domain
            wptr_gray_rclk   <= wptr_gray;
            wptr_gray_rclk_d <= wptr_gray_rclk;

            if (r_en && ~r_empty) begin
                r_data <= mem[rptr_bin[ADDR_BITS-1:0]];
            end
            rptr_bin  <= rptr_bin_next;
            rptr_gray <= rptr_gray_next;
        end
    end

endmodule
