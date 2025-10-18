`timescale 1ns/1ps

module Median_Filter_tb;
    localparam KERNEL_SIZE  = 3;
    localparam PIXEL_DEPTH  = 12;   // RGB444
    localparam IMAGE_WIDTH  = 7;
    localparam IMAGE_HEIGHT = 7;

    reg clk = 0;
    reg reset = 1;
    reg frame_start = 0;
    reg [PIXEL_DEPTH-1:0] pixel_in = 12'd0; // RGB444
    reg we_in = 0;
    wire [11:0] pixel_out;
    wire [17:0] addr_out;

    Median_Filter #(
        .KERNEL_SIZE(KERNEL_SIZE),
        .PIXEL_DEPTH(PIXEL_DEPTH),
        .IMAGE_WIDTH(IMAGE_WIDTH),
        .IMAGE_HEIGHT(IMAGE_HEIGHT)
    ) dut (
        .clk(clk),
        .reset(reset),
        .frame_start(frame_start),
        .pixel_in(pixel_in),
        .we(we_in),
        .pixel_out(pixel_out),
        .addr_out(addr_out)
    );

    // Clock: 20ns period
    always #10 clk = ~clk;

    integer r, c;
    reg [9:0] tb_col;
    reg [8:0] tb_row;

    // Helper: form grayscale RGB444 from 4-bit value v
    function [11:0] gray444;
        input [3:0] v;
    begin
        gray444 = {v, v, v};
    end
    endfunction

    initial begin
        $display("Starting Median_Filter_tb...");
        tb_col = 0; tb_row = 0;
        // Reset
        reset = 1; frame_start = 0; we_in = 0; pixel_in = 12'd0;
        repeat (5) @(posedge clk);
        reset = 0; frame_start = 1; @(posedge clk); frame_start = 0;

        // Feed a 5x5 pattern: value = (row*5 + col) modulo 16, replicated to RGB444
        for (r = 0; r < IMAGE_HEIGHT; r = r + 1) begin
            for (c = 0; c < IMAGE_WIDTH; c = c + 1) begin
                @(posedge clk);
                we_in <= 1'b1;
                pixel_in <= gray444(((r*IMAGE_WIDTH + c) % 16));

                // Track row/col externally to know when outputs are valid
                if (tb_col == (IMAGE_WIDTH-1)) begin
                    tb_col <= 0;
                    if (tb_row == (IMAGE_HEIGHT-1)) tb_row <= 0; else tb_row <= tb_row + 1;
                end else begin
                    tb_col <= tb_col + 1;
                end

                // Display outputs when window is valid (center address row-1,col-1)
                if ((tb_row >= 2) && (tb_col >= 2)) begin
                    $display("time=%0t addr_out=%0d pixel_out(4b)=%0d", $time, addr_out, pixel_out[11:8]);
                end
            end
        end
    end

endmodule