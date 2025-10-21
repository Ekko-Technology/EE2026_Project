`timescale 1ns/1ps
module UFDS_FIFO #(
    parameter WIDTH = 4,
    parameter ADDR_BITS = 9 // depth = 2**ADDR_BITS (512 entries)
)(
    input  wire              wr_clk,
    input  wire              wr_rst,
    input  wire              wr_en,
    input  wire [WIDTH-1:0]  wr_data,
    output wire              wr_full,

    input  wire              rd_clk,
    input  wire              rd_rst,
    input  wire              rd_en,
    output reg  [WIDTH-1:0]  rd_data,
    output wire              rd_empty
);
    localparam DEPTH = (1 << ADDR_BITS);

    // Memory
    (* ram_style = "distributed" *) reg [WIDTH-1:0] mem [0:DEPTH-1];

    // Binary and Gray pointers
    (* ram_style = "distributed" *) reg [ADDR_BITS:0] wr_bin = 0, rd_bin = 0;
    (* ram_style = "distributed" *) reg [ADDR_BITS:0] wr_gray = 0, rd_gray = 0;

    // Synchronizers for cross-domain Gray pointers
    (* ram_style = "distributed" *) reg [ADDR_BITS:0] rd_gray_w1=0, rd_gray_w2=0;
    (* ram_style = "distributed" *) reg [ADDR_BITS:0] wr_gray_r1=0, wr_gray_r2=0;

    // Bin<->Gray helpers
    wire [ADDR_BITS:0] wr_bin_n = wr_bin + (wr_en & ~wr_full);
    wire [ADDR_BITS:0] rd_bin_n = rd_bin + (rd_en & ~rd_empty);
    wire [ADDR_BITS:0] wr_gray_n = (wr_bin_n >> 1) ^ wr_bin_n;
    wire [ADDR_BITS:0] rd_gray_n = (rd_bin_n >> 1) ^ rd_bin_n;

    // Full/Empty
    // Full when next write Gray equals read Gray with MSBs inverted
    wire full_nxt = (wr_gray_n == {~rd_gray_w2[ADDR_BITS:ADDR_BITS-1], rd_gray_w2[ADDR_BITS-2:0]});
    assign wr_full = full_nxt;

    // Empty when write Gray equals current read Gray
    assign rd_empty = (wr_gray_r2 == rd_gray);

    // Write domain
    always @(posedge wr_clk) begin
        if (wr_rst) begin
            wr_bin  <= 0;
            wr_gray <= 0;
            rd_gray_w1 <= 0;
            rd_gray_w2 <= 0;
        end else begin
            // synchronize read pointer into write clock
            rd_gray_w1 <= rd_gray;
            rd_gray_w2 <= rd_gray_w1;

            if (wr_en && ~wr_full) begin
                mem[wr_bin[ADDR_BITS-1:0]] <= wr_data;
                wr_bin  <= wr_bin_n;
                wr_gray <= wr_gray_n;
            end
        end
    end

    // Read domain (1-cycle read latency)
    always @(posedge rd_clk) begin
        if (rd_rst) begin
            rd_bin  <= 0;
            rd_gray <= 0;
            wr_gray_r1 <= 0;
            wr_gray_r2 <= 0;
            rd_data <= {WIDTH{1'b0}};
        end else begin
            // synchronize write pointer into read clock
            wr_gray_r1 <= wr_gray;
            wr_gray_r2 <= wr_gray_r1;

            // synchronous read: output updates one cycle after rd_en
            if (rd_en && ~rd_empty) begin
                rd_data <= mem[rd_bin[ADDR_BITS-1:0]];
                rd_bin  <= rd_bin_n;
                rd_gray <= rd_gray_n;
            end
        end
    end
endmodule