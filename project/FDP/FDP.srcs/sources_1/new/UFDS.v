`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Inspirations taken from AprilTagv1.0, AprilTagv2.0, Suzuki-Abe
// AND from CS2040C for the Union Find Disjoint Set algorithm  
// I have decided to use weighted quick-union (weight is size of the connected component) for space and time complexity
// - find: follow parent "pointers" to the root. same root == same ID
// - Union: Attach the smaller tree to the larger (by size) and merge the stats we stored
// quick-find didnt seem right coz we need to scan and rewrite the entire componentID array every union() call == slow and resource heavy 
// made it weighted so that find is faster (shorter tree height) - O(log n) for both find and union
// path compression i give up and idt we need for now?
//
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
// labels here refers to the component ID that we know (shorter name lol)
//////////////////////////////////////////////////////////////////////////////////


module UFDS_Detector(
    input clk,
    input ext_reset,

    // since image comes in serially, one pixel per clock. I put these as some sort of
    // handshake in signals to know when a new frame starts and ends
    // and also when a new line starts and when each pixel is valid
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

/* 
 *UFDS need to check neighbours' labels, we use 2 lines of buffer (reduced neighbours) 
 * to store labels temporarily for comparison
 * get away using only two (inspiration from mergesort haha to reduce space complexity)
 * just swap the roles of these two after every row (using toggle_line 0: prev=buff0, curr=buff1; 1: prev=buff1, curr=buff0)
 * each entry stores the component ID
*/
reg [label_bits-1:0] row0_labels[0:WIDTH-1];
reg [label_bits-1:0] row1_labels[0:WIDTH-1];
reg toggle_line; // 0: prev=buff0, curr=buff1; 1: prev=buff1, curr=buff0

/* 
 * helpers to get neighbour's labels of current x (since no pointers), no need y
 */
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
        label_curr_left = toggle_line ? row1_labels[x-1] : row0_labels[x-1]; 
    end
endfunction

// helper union() which is to replace sets containing p and q with their union
task set_curr_label(input [x_bitsize-1:0] x, input [label_bits-1:0] L);
    begin
        if (toggle_line) begin
            row0_labels[x] <= L;
        end else begin
            row1_labels[x] <= L;
        end
    end
endtask

/*
 * Quick-Union Disjoint Set ADT implementation (specifically weighted union) 
 * and the stats to store for each label
 */
localparam integer MAX_LABELS = 4096; // can reduce if dont need this many labels
localparam integer FIND_MAX_ITERATIONS = 4; // to prevent infinite loop in find, since loops in functions are combinational. Height of tree is O(log n) = 4

reg [label_bits-1:0] parent [0:MAX_LABELS-1]; // C++ eq: vector<int> parent (MAX_LABELS);
reg [3:0] weight [0:MAX_LABELS-1]; // C++ eq: vector<int> weight (MAX_LABELS); 
reg active_root [0:MAX_LABELS-1]; // to mark if this label is active (used) or not
reg [label_bits-1:0] next_label; // next unused label n0.

// stats for each component (hence only store for root pixel, where parent[label] == label)
reg [23:0] area [0:MAX_LABELS-1]; // number of pixels in this component
reg [31:0] sum_x [0:MAX_LABELS-1]; // sum of x coordinates of pixels in this component
reg [23:0] sum_y [0:MAX_LABELS-1]; // sum of y coordinates of pixels in this component
reg [x_bitsize-1:0] min_x [0:MAX_LABELS-1]; 
reg [x_bitsize-1:0] max_x [0:MAX_LABELS-1]; 
reg [y_bitsize-1:0] min_y [0:MAX_LABELS-1]; 
reg [y_bitsize-1:0] max_y [0:MAX_LABELS-1]; 


// C++ eq: find(root of a) by following parent[]; bounded to keep logic small
function [label_bits-1:0] uf_find(input [label_bits-1:0] a);
    reg [label_bits-1:0] r;
    integer k;
    begin
        r = a;
        if (r == 0) begin uf_find = 0; end
        else begin
            for (k = 0; k < FIND_MAX_ITERS; k = k + 1) begin
                if (parent[r] == r) begin uf_find = r; disable for; end
                r = parent[r];
            end
            uf_find = r; // fallback if loop cap hit
        end
    end
endfunction

// C++: union by rank; attach smaller tree, update rank, merge stats rb -> ra
task merge_stats(input [label_bits-1:0] ra, input [label_bits-1:0] rb);
    begin
        area[ra] <= area[ra] + area[rb];
        sumx[ra] <= sumx[ra] + sumx[rb];
        sumy[ra] <= sumy[ra] + sumy[rb];
        if (minx[rb] < minx[ra]) minx[ra] <= minx[rb];
        if (miny[rb] < miny[ra]) miny[ra] <= miny[rb];
        if (maxx[rb] > maxx[ra]) maxx[ra] <= maxx[rb];
        if (maxy[rb] > maxy[ra]) maxy[ra] <= maxy[rb];
        active[rb] <= 1'b0;
        area[rb] <= 0; sumx[rb] <= 0; sumy[rb] <= 0;
        minx[rb] <= {x_bitsize{1'b1}}; miny[rb] <= {y_bitsize{1'b1}};
        maxx[rb] <= 0; maxy[rb] <= 0;
    end
endtask

task uf_union(input [label_bits-1:0] a, input [label_bits-1:0] b);
    reg [label_bits-1:0] ra, rb;
    begin
        if (a==0 || b==0) disable uf_union;
        ra = uf_find(a); rb = uf_find(b);
        if (ra == rb) disable uf_union;
        if (rank_[ra] < rank_[rb]) begin
            parent[ra] <= rb; merge_stats(rb, ra);
        end else if (rank_[ra] > rank_[rb]) begin
            parent[rb] <= ra; merge_stats(ra, rb);
        end else begin
            parent[rb] <= ra; rank_[ra] <= rank_[ra] + 1'b1; merge_stats(ra, rb);
        end
    end
endtask

task init_label(input [label_bits-1:0] L);
    begin
        parent[L] <= L; rank_[L] <= 0; active[L] <= 1'b1;
        area[L] <= 0; sumx[L] <= 0; sumy[L] <= 0;
        minx[L] <= {x_bitsize{1'b1}}; miny[L] <= {y_bitsize{1'b1}};
        maxx[L] <= 0; maxy[L] <= 0;
    end
endtask

task add_pixel_to_root(input [label_bits-1:0] R);
    begin
        area[R] <= area[R] + 1;
        sumx[R] <= sumx[R] + x;
        sumy[R] <= sumy[R] + y;
        if (x < minx[R]) minx[R] <= x; if (x > maxx[R]) maxx[R] <= x;
        if (y < miny[R]) miny[R] <= y; if (y > maxy[R]) maxy[R] <= y;
    end
endtask

/*
 * run-time main-loop by per-pixel UFDS sequence
 */
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
                // 1. map neighbours to current roots
                reg [label_bits-1:0] R_left;
                reg [label_bits-1:0] R_up;
                reg [label_bits-1:0] R_ul;
                reg [label_bits-1:0] R_ur;
                reg [label_bits-1:0] R;
                R_left = (L  != 0) ? uf_find(L)  : 0;
                R_up   = (U  != 0) ? uf_find(U)  : 0;
                R_ul   = (UL != 0) ? uf_find(UL) : 0;
                R_ur   = (UR != 0) ? uf_find(UR) : 0;

                /// 2. Choose representative: smallest non-zero root label
                R = 0;
                if (R_left) begin
                    R = R_left;
                end
                if (R_up && (R == 0 || R_up < R)) begin
                    R = R_up;
                end
                if (R_ul && (R == 0 || R_ul < R)) begin
                    R = R_ul;
                end
                if (R_ur && (R == 0 || R_ur < R)) begin
                    R = R_ur;
                end

                // 3. If no neighbours, new label
                if (R == 0) begin
                    R <= next_label;
                    init_label(next_label);
                    active[next_label] <= 1'b1;
                    parent[next_label] <= next_label;
                    if (next_label != MAX_LABELS - 1) begin
                        next_label <= next_label + 1'b1;
                    end
                end

                // 4. Union with any neighbour roots that are different
                if (R_left != 0 && R_left != R) begin
                    uf_union(R, R_left);
                end
                if (R_up != 0 && R_up != R) begin
                    uf_union(R, R_up);
                end
                if (R_ul != 0 && R_ul != R) begin
                    uf_union(R, R_ul);
                end
                if (R_ur != 0 && R_ur != R) begin
                    uf_union(R, R_ur);
                end

                // 5. Final root and update
                R = uf_find(R);
                set_curr_label(x, R);
                add_pixel_to_root(R);
            end

            // increment x within the line (reminder y is handled at the start)
            x <= x + 1;
        end

        // frame_end here: outputs to be computed
        // if (frame_end) out_valid <= 1;
    end
end
    
endmodule
