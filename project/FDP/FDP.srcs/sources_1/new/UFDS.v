`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Inspirations taken from AprilTagv1.0, AprilTagv2.0, Suzuki-Abe
// Given a bitmap of size 320 x 240, but there is a limit to vertical traversal speed.
// Use reduced neighbour set: check if left, up-left, up, up-right are part of a component
// Search from left to right, top to bottom
// Every component found given an ID, then use minDistBtnBlobs to merge smaller blobs close together
//
// 1. Grouping: Use UFDS to find connected components from bitmap and assign IDs
// 2. Merging: Merge small components into larger ones based on bounding box overlap
// 3. Output: All final bounding boxes which are top bottom left right coordinates
//
// Personal implementation of a connceted-component labelling using UFDS
//////////////////////////////////////////////////////////////////////////////////


module UFDS_Detector(
    input clk,
    input ext_reset,

    // since image comes in serially, one pixel per clock. This is to ensure
    // that there is handshake in signals to know when a new frame starts and ends
    // when a new line starts
    // when each pixel is valid
    input  wire in_valid,      // 1 during active pixels
    input  wire frame_start,   // pulse at first active pixel
    input  wire line_start,    // pulse at first pixel of each line
    input  wire frame_end,     // pulse after last active pixel
    input  wire curr_pix,      // every pixel that is coming in

    output reg [8:0] bbox_left,
    output reg [8:0] bbox_right,
    output reg [7:0] bbox_top,
    output reg [7:0] bbox_bottom,
    output reg [8:0] centroid_x,
    output reg [7:0] centroid_y
    // output reg out_valid

    );

localparam integer WIDTH = 320;
localparam integer HEIGHT = 240;
localparam integer x_bitsize = 9;
localparam integer y_bitsize = 8;
localparam integer label_bits = 12; // rmb label == 0 is the background and supports up to 4095 labels (can reduce if dont need this many labels)

// current pixel coordinates (increment x each pixel, on line_start x=0 and y increment by 1)
reg [x_bitsize-1:0] x;
reg [y_bitsize-1:0] y;

// UFDS need to check neighbours' labels, we use 2 lines of buffer (reduced neighbours) 
// to store labels temporarily for comparison
// get away using only two (inspiration from mergesort haha to reduce space complexity)
// just swap the roles of these two after every row (using toggle_line 0: prev=buff0, curr=buff1; 1: prev=buff1, curr=buff0)
// each entry stores the component ID
reg [label_bits-1:0] row0_labels[0:WIDTH-1];
reg [label_bits-1:0] row1_labels[0:WIDTH-1];
reg toggle_line; // 0: prev=buff0, curr=buff1; 1: prev=buff1, curr=buff0

// helpers to get neighbour's labels of current x (since no pointers), no need y
reg [label_bits-1:0] L, UL, U, UR; // reduced neighbour set

function [label_bits-1:0] label_top(input [x_bitsize-1:0] x);
    if (y == 0) begin
        label_top = 0; // first line has no previous line
    end else begin
        label_top = toggle_line ? row1_labels[x] : row0_labels[x]; 
    end
endfunction

function [label_bits-1:0] label_top_left(input [x_bitsize-1:0] x);
    if (y == 0 || x == 0) begin
        label_top_left = 0; // first line or first pixel has no previous line or left pixel
    end else begin
        label_top_left = toggle_line ? row1_labels[x-1] : row0_labels[x-1]; 
    end
endfunction

function [label_bits-1:0] label_top_right(input [x_bitsize-1:0] x);
    if (y == 0 || x == WIDTH-1) begin
        label_top_right = 0; // first line or last pixel has no previous line or right pixel
    end else begin
        label_top_right = toggle_line ? row1_labels[x+1] : row0_labels[x+1]; 
    end
endfunction

function [label_bits-1:0] label_curr_left(input [x_bitsize-1:0] x);
    if (y == 0 || x == 0) begin
        label_curr_left = 0; // first line or first pixel has no previous line or left pixel
    end else begin
        label_curr_left = toggle_line ? row0_labels[x-1] : row1_labels[x-1]; 
    end
endfunction

task set_curr_label(input [x_bitsize-1:0] x, input [label_bits-1:0] L);
    begin
        if (toggle_line) begin
            row0_labels[x] <= L;
        end else begin
            row1_labels[x] <= L;
        end
    end
endtask

integer i; // used for iterations of for loops later

always @(posedge clk) begin
    if (ext_reset) begin
        x <= 0; y <= 0; toggle_line <= 0;
        bbox_left <= 0; bbox_right <= 0; bbox_top <= 0; bbox_bottom <= 0; centroid_x <= 0; centroid_y <= 0;
        for (i = 0; i < WIDTH; i = i + 1) begin
            row0_labels[i] <= 0;
            row1_labels[i] <= 0;
        end
        //out_valid <= 1'b0;
    end else begin
        // new frame: reset coordinates and clear both line buffers
        if (frame_start) begin
            x <= 0; y <= 0; toggle_line <= 0;
            for (i = 0; i < WIDTH; i = i + 1) begin
                row0_labels[i] <= 0;
                row1_labels[i] <= 0;
            end
        end 
        
        // new line except first pixel of a frame
        if (line_start && !frame_start) begin
            // reset x, increment y, toggle line buffers
            x <= 0;
            y <= y + 1;
            toggle_line <= ~toggle_line;
        end 
        
        if (in_valid) begin
            // during active pixels
            // get reduced neighbour labels
            L  = label_curr_left(x);
            UL = label_top_left(x);
            U  = label_top(x);
            UR = label_top_right(x);

            if (!curr_pix) begin
                set_curr_label(x, 0); // background pixel, label = 0
            end else begin
                // Placeholder; Step 2 assigns/merges a label here
                // set_curr_label(x, some_label);
                set_curr_label(x, 0);
            end

            // increment x within the line (reminder y is handled at the start)
            x <= x + 1;
        end

        // frame_end here: outputs to be computed
        // if (frame_end) out_valid <= 1;
    end
end
    
endmodule
