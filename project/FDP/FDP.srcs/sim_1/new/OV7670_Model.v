`timescale 1ns/1ps

// Simple behavioral OV7670 camera model (byte-stream)
// - Generates PCLK, VSYNC, HREF, and D[7:0]
// - Sends two bytes per pixel while HREF=1
// - Frame: a small active area (WIDTH_PX x HEIGHT_LINES) with short blanking
//
// Byte mapping chosen to match OV7670_Capture's RGB444 extraction:
//   capture uses dout <= { d_latch[3:0], d_latch[7:4], d_latch[11:8] };
//   After two bytes (b1 then b2): d_latch = { b1, b2 }
//   Therefore dout = { b2[3:0], b2[7:4], b1[7:4] } = { B, G, R }.
//   To generate a target RGB444={R,G,B} send: b1 = {R, 4'h0}, b2 = {G, B}.

module OV7670_Model #(
    parameter PCLK_PERIOD_NS = 40, // 25 MHz (for sim, adjustable)
    parameter WIDTH_PX       = 40, // active pixels per line (<=640 ok)
    parameter HEIGHT_LINES   = 4,  // active lines per frame (<=480 ok)
    parameter HBLANK_BYTES   = 10, // horizontal blank bytes before/after line
    parameter VBLANK_LINES   = 2   // vertical blank lines before/after frame
) (
    output reg        pclk,
    output reg        vsync,
    output reg        href,
    output reg [7:0]  d
);

    // PCLK generation
    initial begin
        pclk = 1'b0;
        forever #(PCLK_PERIOD_NS/2) pclk = ~pclk;
    end

    // Simple pixel pattern (4-bit per channel)
    function [3:0] r4;
        input [9:0] x;
        input [8:0] y;
        r4 = x[3:0];
    endfunction
    function [3:0] g4;
        input [9:0] x;
        input [8:0] y;
        g4 = y[3:0];
    endfunction
    function [3:0] b4;
        input [9:0] x;
        input [8:0] y;
        b4 = (x[3:0] ^ y[3:0]);
    endfunction

    integer line;
    integer col;
    integer i;

    initial begin
        vsync = 1'b1; // idle high (as used by OV7670_Capture reset)
        href  = 1'b0;
        d     = 8'h00;

        // Vertical blank (pre-frame)
        repeat (VBLANK_LINES) begin
            @(posedge pclk);
        end

        // Start of frame: deassert VSYNC (active video)
        @(posedge pclk); vsync <= 1'b0;

        // Active lines
        for (line = 0; line < HEIGHT_LINES; line = line + 1) begin
            // Horizontal blanking before HREF
            for (i = 0; i < HBLANK_BYTES; i = i + 1) @(posedge pclk);

            href <= 1'b1;
            // Send two bytes per pixel for WIDTH_PX pixels
            for (col = 0; col < WIDTH_PX; col = col + 1) begin
                // First byte: {R, xxxx}
                d <= {r4(col[9:0], line[8:0]), 4'h0};
                @(posedge pclk);
                // Second byte: {G, B}
                d <= {g4(col[9:0], line[8:0]), b4(col[9:0], line[8:0])};
                @(posedge pclk);
            end
            href <= 1'b0;

            // Horizontal blanking after HREF
            for (i = 0; i < HBLANK_BYTES; i = i + 1) @(posedge pclk);
        end

        // Vertical blank (post-frame) and VSYNC high
        @(posedge pclk); vsync <= 1'b1;
        repeat (VBLANK_LINES) begin
            @(posedge pclk);
        end

        // End sim after one frame
        repeat (50) @(posedge pclk);
        $finish;
    end

endmodule
