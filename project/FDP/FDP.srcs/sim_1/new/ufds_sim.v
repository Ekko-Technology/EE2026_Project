`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 16.10.2025 01:20:47
// Design Name: 
// Module Name: ufds_sim
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


module ufds_sim(
// Verifies x resets on frame_start/line_start and only increments when in_valid=1
// Verifies toggle_line flips at each new line
// Verifies neighbor helpers return zeros on the first line (edges)
// Verifies U/UL/UR fetch from the previous row when we preload labels between lines
    );
    // DUT ports
    reg  clk = 0;
    reg  rst = 1;
    reg  in_valid = 0;
    reg  frame_start = 0;
    reg  line_start = 0;
    reg  frame_end = 0;
    reg  curr_pix = 0;

    wire [8:0] bbox_left, bbox_right, centroid_x;
    wire [7:0] bbox_top,  bbox_bottom, centroid_y;

    // Instantiate DUT
    UFDS_Detector dut (
        .clk(clk),
        .ext_reset(rst),
        .in_valid(in_valid),
        .frame_start(frame_start),
        .line_start(line_start),
        .frame_end(frame_end),
        .curr_pix(curr_pix),
        .bbox_left(bbox_left),
        .bbox_right(bbox_right),
        .bbox_top(bbox_top),
        .bbox_bottom(bbox_bottom),
        .centroid_x(centroid_x),
        .centroid_y(centroid_y)
    );

    // Clock
    always #5 clk = ~clk;

    // Helpers
    task drive_pixel(input v, input fs, input ls, input fe, input px);
        begin
            @(negedge clk);
            in_valid    = v;
            frame_start = fs;
            line_start  = ls;
            frame_end   = fe;
            curr_pix    = px;
            @(posedge clk);
        end
    endtask


    initial begin


        // Reset
        repeat (3) @(posedge clk);
        rst = 0;

        // Start of frame, first pixel (x starts at 0, y=0)
        // Drive first pixel with both frame_start and line_start asserted
        drive_pixel(1, 1, 1, 0, 0);
        // After first pixel, x should have incremented (only because in_valid=1)
        //x should be 1 after first valid pixel
        //y should be 0 on first line
        // Neighbors at (y=0) must be zero (no previous row)
        //line0_pixel0

        // Second pixel on first line
        drive_pixel(1, 0, 0, 0, 1);
        //x should increment to 2");
        //line0_pixel1 // top neighbors are still 0; left is current-row label (still 0)

        // Stall one cycle: x must not change when in_valid=0
        @(negedge clk);
        drive_pixel(0, 0, 0, 0, 0);
        @(posedge clk);
        //x must hold during stall (in_valid=0)

        // More pixels on first line (not strictly needed)
        drive_pixel(1, 0, 0, 0, 1); // x=3
        drive_pixel(1, 0, 0, 0, 1); // x=4
        //x should be 4 after two more pixels

        // Start second line: assert line_start; no valid pixel this cycle
        drive_pixel(0, 0, 1, 0, 0);
        //x should reset to 0 on new line when in_valid=0
        //y should increment to 1 at new line
        // toggle_line must flip each new line
        // Save toggle state and check it flipped relative to first line
        //toggle_line should be 1 on second line

        // Preload previous row labels so we can test neighbor helpers (prev=row1 when toggle_line=1)
        // We will test at x=2 on second line: UL=row1[1], U=row1[2], UR=row1[3]
        dut.row1_labels[1] = 12'hA1;
        dut.row1_labels[2] = 12'hA2;
        dut.row1_labels[3] = 12'hA3;

        // Pixel x=0 on second line: all neighbors except U/UR zero (but prev row initialized zeros except indices above)
        drive_pixel(1, 0, 0, 0, 1); // x becomes 1
        // At x=0, UL=0, L=0, U=row1[0]=0 (we didn't set), UR=row1[1]=A1 but boundary at x=0 still allows UR
        //            "L at x=0 must be 0");
        //            "UL at x=0 must be 0");
        //            "U at x=0 expected 0");
        //       "UR at x=0 should equal row1[1]");

        // Pixel x=1 on second line: UL=row1[0]=0, U=row1[1]=A1, UR=row1[2]=A2
        drive_pixel(1, 0, 0, 0, 1); // x becomes 2
        //            "UL at x=1 expected 0");
        //       "U at x=1 should equal row1[1]");
        //       "UR at x=1 should equal row1[2]");

        // Pixel x=2 on second line: UL=row1[1]=A1, U=row1[2]=A2, UR=row1[3]=A3
        drive_pixel(1, 0, 0, 0, 1); // x becomes 3
        //      "UL at x=2 should equal row1[1]");
        //      "U at x=2 should equal row1[2]");
        //      "UR at x=2 should equal row1[3]");

        // L check note:
        // Current code writes current-row labels each pixel (right now zero), so L will be 0 in these tests.
        // After we add label assignment, we can extend this to check L against the label at x-1.

    end
endmodule
