`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 18.10.2025 23:10:50
// Design Name: 
// Module Name: UFDS_Bridge
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


module UFDS_Bridge 
(
    // producer @ 25 MHz
    input  wire pclk,
    input  wire p_rst,
    input  wire p_valid,
    input  wire p_fs,
    input  wire p_ls,
    input  wire p_fe,
    input  wire p_px,
    // UFDS @ 100 MHz
    input  wire clk,
    input  wire ext_reset,
    // UFDS outputs
    // output wire [8:0] bbox_left, bbox_right, centroid_x,
    // output wire [7:0] bbox_top,  bbox_bottom, centroid_y
    output wire ready_o,
    output wire [2:0] comp_count,
    output wire [8:0] comp0_left,  comp1_left,  comp2_left,  comp3_left,
    output wire [8:0] comp0_right, comp1_right, comp2_right, comp3_right,
    output wire [7:0] comp0_top,   comp1_top,   comp2_top,   comp3_top,
    output wire [7:0] comp0_bottom,comp1_bottom,comp2_bottom,comp3_bottom,
    output wire [8:0] comp0_cx,    comp1_cx,    comp2_cx,    comp3_cx,
    output wire [7:0] comp0_cy,    comp1_cy,    comp2_cy,    comp3_cy,
    output wire [15:0]comp0_area,  comp1_area,  comp2_area,  comp3_area
);
    wire       rd_empty;
    wire [3:0] rd_data;
    reg        rd_en;
    reg        in_valid_q;
    reg  fs_q, ls_q, fe_q, px_q;
    wire ready_i;
    assign ready_o = ready_i;

    UFDS_FIFO u_fifo (
        .wr_clk (pclk),
        .wr_rst (p_rst | ext_reset),
        .wr_en  (p_valid),
        .wr_data({p_fs, p_ls, p_fe, p_px}),
        .wr_full(), // unused
        .rd_clk (clk),
        .rd_rst (ext_reset | p_rst),
        .rd_en  (rd_en),
        .rd_data(rd_data),
        .rd_empty(rd_empty)
    );

    UFDS_Detector u_ufds (
        .clk(clk), .ext_reset(ext_reset),
        .in_valid(in_valid_q),
        .frame_start(fs_q), .line_start(ls_q), .frame_end(fe_q), .curr_pix(px_q),
        // .bbox_left(bbox_left), .bbox_right(bbox_right),
        // .bbox_top(bbox_top), .bbox_bottom(bbox_bottom),
        // .centroid_x(centroid_x), .centroid_y(centroid_y),
        .ready_to_read(ready_i),
        .comp_count(comp_count),
        .comp0_left(comp0_left),   .comp1_left(comp1_left),   .comp2_left(comp2_left),   .comp3_left(comp3_left),
        .comp0_right(comp0_right), .comp1_right(comp1_right), .comp2_right(comp2_right), .comp3_right(comp3_right),
        .comp0_top(comp0_top),     .comp1_top(comp1_top),     .comp2_top(comp2_top),     .comp3_top(comp3_top),
        .comp0_bottom(comp0_bottom),.comp1_bottom(comp1_bottom),.comp2_bottom(comp2_bottom),.comp3_bottom(comp3_bottom),
        .comp0_cx(comp0_cx),       .comp1_cx(comp1_cx),       .comp2_cx(comp2_cx),       .comp3_cx(comp3_cx),
        .comp0_cy(comp0_cy),       .comp1_cy(comp1_cy),       .comp2_cy(comp2_cy),       .comp3_cy(comp3_cy),
        .comp0_area(comp0_area),   .comp1_area(comp1_area),   .comp2_area(comp2_area),   .comp3_area(comp3_area)
    );

    reg        have;
    reg [3:0]  hold;
    reg        pop_inflight;

    always @(posedge clk) begin
        if (ext_reset | p_rst) begin
            rd_en        <= 1'b0;
            pop_inflight <= 1'b0;
            have         <= 1'b0;
            hold         <= 4'b0;
            in_valid_q   <= 1'b0;
            {fs_q,ls_q,fe_q,px_q} <= 4'b0;
        end else begin
            // start a pop only if we don't already hold a word and no pop is inflight
            if (!have && !pop_inflight && !rd_empty) begin
                rd_en        <= 1'b1;
                pop_inflight <= 1'b1;
            end else begin
                rd_en <= 1'b0;
            end

            // capture FIFO word one cycle after rd_en (when pop_inflight is set)
            if (pop_inflight) begin
                hold         <= rd_data;
                have         <= 1'b1;
                pop_inflight <= 1'b0;
            end

            // present one cycle when UFDS is ready
            if (have && ready_i) begin
                {fs_q,ls_q,fe_q,px_q} <= hold;
                in_valid_q <= 1'b1;
                have <= 1'b0;
            end else begin
                in_valid_q <= 1'b0;
            end
        end
    end
endmodule
