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


module ufds_sim(    );
// Verifies x resets on frame_start/line_start and only increments when in_valid=1
// Verifies toggle_line flips at each new line
// Verifies neighbor helpers return zeros on the first line (edges)
// Verifies U/UL/UR fetch from the previous row when we preload labels between lines

    // localparam integer IMG_W = 24;
    // localparam integer IMG_H = 12;

    // // clocks
    // reg pclk=0, clk=0, rst=1;
    // always #20 pclk = ~pclk; // 25 MHz
    // always #5  clk  = ~clk;  // 100 MHz

    // // producer (no backpressure)
    // reg p_valid=0, p_fs=0, p_ls=0, p_fe=0, p_px=0;

    // wire [2:0] comp_count;
    // wire [8:0] comp0_left,  comp1_left,  comp2_left,  comp3_left;
    // wire [8:0] comp0_right, comp1_right, comp2_right, comp3_right;
    // wire [7:0] comp0_top,   comp1_top,   comp2_top,   comp3_top;
    // wire [7:0] comp0_bottom,comp1_bottom,comp2_bottom,comp3_bottom;
    // wire [8:0] comp0_cx,    comp1_cx,    comp2_cx,    comp3_cx;
    // wire [7:0] comp0_cy,    comp1_cy,    comp2_cy,    comp3_cy;
    // wire [15:0]comp0_area,  comp1_area,  comp2_area,  comp3_area;

    // UFDS_Bridge #(.FIFO_DEPTH(1024), .FIFO_ABITS(10)) dut (
    //     .pclk(pclk), .p_rst(rst),
    //     .p_valid(p_valid), .p_fs(p_fs), .p_ls(p_ls), .p_fe(p_fe), .p_px(p_px),
    //     .clk(clk), .ext_reset(rst),

    //     // .bbox_left(), .bbox_right(), .centroid_x(),
    //     // .bbox_top(),  .bbox_bottom(), .centroid_y(),

    //     .comp_count(comp_count),
    //     .comp0_left(comp0_left), .comp1_left(comp1_left), .comp2_left(comp2_left), .comp3_left(comp3_left),
    //     .comp0_right(comp0_right), .comp1_right(comp1_right), .comp2_right(comp2_right), .comp3_right(comp3_right),
    //     .comp0_top(comp0_top), .comp1_top(comp1_top), .comp2_top(comp2_top), .comp3_top(comp3_top),
    //     .comp0_bottom(comp0_bottom), .comp1_bottom(comp1_bottom), .comp2_bottom(comp2_bottom), .comp3_bottom(comp3_bottom),
    //     .comp0_cx(comp0_cx), .comp1_cx(comp1_cx), .comp2_cx(comp2_cx), .comp3_cx(comp3_cx),
    //     .comp0_cy(comp0_cy), .comp1_cy(comp1_cy), .comp2_cy(comp2_cy), .comp3_cy(comp3_cy),
    //     .comp0_area(comp0_area), .comp1_area(comp1_area), .comp2_area(comp2_area), .comp3_area(comp3_area)
    // );

    // integer x, y;
    // reg started = 0;

    // // fixed-rate raster source
    // always @(posedge pclk) begin
    //     if (rst) begin
    //         started <= 1'b0;
    //         p_valid <= 1'b0; p_fs<=1'b0; p_ls<=1'b0; p_fe<=1'b0; p_px<=1'b0;
    //         x <= 0; y <= 0;
    //     end else if (!started) begin
    //         // wait until UFDS reports ready (done with init)
    //         if (dut.ready_o) begin
    //             started <= 1'b1;
    //             p_valid <= 1'b1;
    //             p_fs    <= 1'b1; // first pixel will assert fs & ls
    //             p_ls    <= 1'b1;
    //             p_fe    <= 1'b0;
    //             p_px    <= 1'b0;
    //             x <= 0; y <= 0;
    //         end else begin
    //             p_valid <= 1'b0; p_fs<=1'b0; p_ls<=1'b0; p_fe<=1'b0; p_px<=1'b0;
    //         end
    //     end else begin
    //         p_valid <= 1'b1;
    //         p_fs    <= (x==0 && y==0);
    //         p_ls    <= (x==0);
    //         p_fe    <= (x==IMG_W-1 && y==IMG_H-1);
    //         // 4 components A,B,C,D
    //         p_px <= ((x>=2  && x<=5 ) && (y>=1 && y<=3)) ? 1'b1 :
    //                 ((x>=9  && x<=12) && (y>=0 && y<=1)) ? 1'b1 :
    //                 ((x>=15 && x<=17) && (y>=5 && y<=8)) ? 1'b1 :
    //                 (((x==20) && (y>=5 && y<=7)) || ((x>=19 && x<=21) && (y==6))) ? 1'b1 : 1'b0;

    //         if (x == IMG_W-1) begin
    //             x <= 0;
    //             if (y == IMG_H-1) y <= 0; else y <= y + 1;
    //         end else begin
    //             x <= x + 1;
    //         end
    //     end
    // end

    // // helper: check presence in any slot
    // task expect_box(input [8:0] L, input [8:0] R, input [7:0] T, input [7:0] B,
    //                 input [8:0] CX, input [7:0] CY);
    //     integer hits;
    //     begin
    //         hits = 0;
    //         if (comp0_left==L && comp0_right==R && comp0_top==T && comp0_bottom==B && comp0_cx==CX && comp0_cy==CY) hits = hits + 1;
    //         if (comp1_left==L && comp1_right==R && comp1_top==T && comp1_bottom==B && comp1_cx==CX && comp1_cy==CY) hits = hits + 1;
    //         if (comp2_left==L && comp2_right==R && comp2_top==T && comp2_bottom==B && comp2_cx==CX && comp2_cy==CY) hits = hits + 1;
    //         if (comp3_left==L && comp3_right==R && comp3_top==T && comp3_bottom==B && comp3_cx==CX && comp3_cy==CY) hits = hits + 1;
    //         if (hits == 0) begin
    //             $display("ERROR: bbox [L%0d R%0d T%0d B%0d] cx=%0d cy=%0d not found", L,R,T,B,CX,CY);
    //             $fatal;
    //         end
    //     end
    // endtask

    // initial begin
    //     repeat (8) @(posedge clk);
    //     rst = 0;

    //     // Let sim run long enough: frame is 11.52us; add init margin (~13.5us) + slack
    //     #(120_000);

    //     if (comp_count < 3'd4) begin
    //         $display("ERROR: comp_count=%0d expected 4", comp_count); $fatal;
    //     end
    //     // A, B, C, D (order free)
    //     expect_box(9'd2,  9'd5,  8'd1, 8'd3,  9'd3,  8'd2);
    //     expect_box(9'd9,  9'd12, 8'd0, 8'd1,  9'd10, 8'd0);
    //     expect_box(9'd15, 9'd17, 8'd5, 8'd8,  9'd16, 8'd6);
    //     expect_box(9'd19, 9'd21, 8'd5, 8'd7,  9'd20, 8'd6);

    //     $display("PASS: all 4 components detected and bbox/centroid correct.");
    //     $finish;
    // end

     //Successfully tested multi-component detection on 24x12 image with 4 separated objects, but poor sync
    // Test image size
    localparam integer IMG_W = 24;
    localparam integer IMG_H = 12;

    reg  clk=0, rst=1;
    reg  in_valid=0, frame_start=0, line_start=0, frame_end=0, curr_pix=0;

    wire [8:0] bbox_left, bbox_right, centroid_x;
    wire [7:0] bbox_top,  bbox_bottom, centroid_y;


    // DUT
    // If UFDS_Detector has WIDTH/HEIGHT parameters, pass .WIDTH(IMG_W), .HEIGHT(IMG_H)
    UFDS_Detector dut(
        .clk(clk), .ext_reset(rst),
        .in_valid(in_valid),
        .frame_start(frame_start),
        .line_start(line_start),
        .frame_end(frame_end),
        .curr_pix(curr_pix),
        .bbox_left(bbox_left), .bbox_right(bbox_right),
        .bbox_top(bbox_top),   .bbox_bottom(bbox_bottom),
        .centroid_x(centroid_x), .centroid_y(centroid_y)
    );

    // 100 MHz
    always #5 clk = ~clk;
    



    // Foreground map: 4 separated components
    // A: x=2..5,  y=1..3  (area 12)
    // B: x=9..12, y=0..1  (area 8)
    // C: x=15..17,y=5..8  (area 12)
    // D: plus centered at (20,6), arms len=1 (area 5)
    function   fg(input integer x, input integer y);
        begin
            fg = 0;
            if ((x>=2  && x<=5 ) && (y>=1 && y<=3)) fg = 1; // A
            if ((x>=9  && x<=12) && (y>=0 && y<=1)) fg = 1; // B
            if ((x>=15 && x<=17) && (y>=5 && y<=8)) fg = 1; // C
            if ((x==20 && (y>=5 && y<=7)) || ((x>=19 && x<=21) && y==6)) fg = 1; // D
        end
    endfunction

    // Push one pixel: assert on posedge, deassert on negedge (matches DUT sampling)
    task push_pixel(input   fs, input   ls, input   px);
    begin
        wait (dut.state == dut.S_READY);
        @(posedge clk);
        curr_pix    = px;
        in_valid   = 1'b1;
        frame_start = fs;
        line_start  = ls;
        @(negedge clk);
        in_valid    = 1'b0;
        frame_start = 1'b0;
        frame_end  = 1'b0;
        line_start  = 1'b0;
    end
    endtask

    // Helpers: scan active roots by bbox and simple checks
    function integer find_by_bbox(input integer L, input integer R, input integer T, input integer B);
        integer i;
        begin
            find_by_bbox = 0;
            for (i=1; i<dut.next_label; i=i+1) begin
                if (dut.active_root[i] &&
                    dut.min_x[i]==L && dut.max_x[i]==R &&
                    dut.min_y[i]==T && dut.max_y[i]==B) begin
                    find_by_bbox = i;
                end
            end
        end
    endfunction

    task check_component(input integer expected_area,
                         input integer L, input integer R,
                         input integer T, input integer B);
        integer lbl;
        begin
            lbl = find_by_bbox(L,R,T,B);
            if (lbl == 0) begin
                $display("ERROR: bbox [L%0d R%0d T%0d B%0d] not found", L,R,T,B);
                $fatal;
            end
            if (dut.area[lbl] !== expected_area) begin
                $display("ERROR: area[%0d]=%0d expected %0d", lbl, dut.area[lbl], expected_area);
                $fatal;
            end
            $display("OK: label %0d area=%0d bbox=[L%0d R%0d T%0d B%0d]", lbl, dut.area[lbl], L,R,T,B);
        end
    endtask

    task check_counts(input integer expected_active, input integer expected_total_area);
        integer i, cnt, sum_area;
        begin
            cnt = 0; sum_area = 0;
            for (i=1; i<dut.next_label; i=i+1) begin
                if (dut.active_root[i]) begin
                    cnt = cnt + 1;
                    sum_area = sum_area + dut.area[i];
                end
            end
            if (cnt !== expected_active) begin
                $display("ERROR: active=%0d expected %0d", cnt, expected_active); $fatal;
            end
            if (sum_area !== expected_total_area) begin
                $display("ERROR: total_area=%0d expected %0d", sum_area, expected_total_area); $fatal;
            end
            $display("OK: active=%0d total_area=%0d", cnt, sum_area);
        end
    endtask

    // Optional: assert no X on neighbors when accepting a pixel
always @(posedge clk) begin
    if (dut.state==dut.S_READY && in_valid) begin
        if ((^dut.left_label)    === 1'bx ||
            (^dut.up_label)      === 1'bx ||
            (^dut.upleft_label)  === 1'bx ||
            (^dut.upright_label) === 1'bx) begin
            $display("ERROR: neighbor X at x=%0d y=%0d", dut.x, dut.y);
            $stop; // or $finish;
        end
    end
end

    integer x, y;
    initial begin
        // Reset
        repeat (4) @(posedge clk);
        rst = 0;

        // Wait DUT ready
        #44150;

        // Stream the 24x12 frame
        for (y=0; y<IMG_H; y=y+1) begin
            for (x=0; x<IMG_W; x=x+1) begin
                push_pixel((y==0 && x==0), (x==0), fg(x,y));
                wait (dut.state == dut.S_READY);
            end
        end

        // Drain merges
        repeat (100) @(posedge clk);

        // Checks
        check_component(12, 2, 5, 1, 3);   // A
        check_component(8,  9,12, 0, 1);   // B
        check_component(12,15,17, 5, 8);   // C
        check_component(5, 19,21, 5, 7);   // D
        check_counts(4, 12+8+12+5);

        // Validate single-bbox outputs (largest component = A)
        // Give a couple cycles for any last updates to settle
        repeat (4) @(posedge clk);
        if (bbox_left   !== 9'd2 ||
            bbox_right  !== 9'd5 ||
            bbox_top    !== 8'd1 ||
            bbox_bottom !== 8'd3 ||
            centroid_x  !== 9'd3 ||
            centroid_y  !== 8'd2) begin
            $display("ERROR: bbox outputs mismatch. got L%0d R%0d T%0d B%0d CX=%0d CY=%0d, expected L2 R5 T1 B3 CX=3 CY=2",
                     bbox_left, bbox_right, bbox_top, bbox_bottom, centroid_x, centroid_y);
            $fatal;
        end else begin
            $display("OK: single-bbox outputs match largest component A.");
        end

        $display("PASS: multi-component test");
        $finish;
    end


    

    /* // Successfully tested 3x8 pixel block with 2x3 object
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
    task push_pixel(input fs, input ls, input px);
        begin
            wait (dut.state == dut.S_READY);
            @(posedge clk);
            curr_pix    = px;
            in_valid    = 1;
            frame_start = fs;
            line_start  = ls;
            @(negedge clk);
            in_valid    = 0;
            frame_start = 0;
            frame_end   = 0;
            line_start  = 0;
        end
    endtask

    integer xi, yi;
    initial begin


        // reset
        repeat (4) @(posedge clk);
        rst = 0;

        // wait for init sweeps to complete
        #44150;

        // feed 3 lines of 8 pixels with a simple 2x3 block at (x=2..3,y=0..2)
        for (yi=0; yi<3; yi=yi+1) begin
            for (xi=0; xi<8; xi=xi+1) begin
                push_pixel((yi==0 && xi==0), (xi==0), ((xi>=2 && xi<=3) ? 1:0));
                wait (dut.state == dut.S_READY);
            end
        end

    end
    */

    /*
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
    */
endmodule
