`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/23/2025 02:24:55 PM
// Design Name: 
// Module Name: uart_sim
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


module uart_sim(

    );

    reg clk50_1 = 0;
    reg btnU_1 = 0; // reset
    reg [79:0] tx_fifo_payload_1;
    reg tx_fifo_wr_en_1;
    wire uart_tx_1;
    wire [79:0] rx_fifo_payload_1;
    wire rx_fifo_rd_en_1;
    reg uart_rx_1;

    UART_Controller #(.UART_CLKS_PER_BIT(4))
    uart_sender (
        .clk(clk50_1),
        .rst(btnU_1),
        .tx_fifo(tx_fifo_payload_1),
        .tx_fifo_wr_en(tx_fifo_wr_en_1),
        .tx_pin(uart_tx_1),
        .rx_fifo(rx_fifo_payload_1),
        .rx_fifo_rd_en(rx_fifo_rd_en_1),
        .rx_pin(uart_rx_1)
    );


    // Second UART instance to receive data sent by the first UART
    reg clk50 = 0;
    reg btnU = 0; // reset
    reg [79:0] tx_fifo_payload;
    reg tx_fifo_wr_en;
    wire uart_tx;
    wire [79:0] rx_fifo_payload;
    wire rx_fifo_rd_en;
    reg uart_rx;

    always @(posedge clk50_1) begin
        uart_rx <= uart_tx_1; // connect TX of first UART to RX of second UART
    end

    UART_Controller #(.UART_CLKS_PER_BIT(4))
    uart_receiver (
        .clk(clk50),
        .rst(btnU),
        .tx_fifo(tx_fifo_payload),
        .tx_fifo_wr_en(tx_fifo_wr_en),
        .tx_pin(uart_tx),
        .rx_fifo(rx_fifo_payload),
        .rx_fifo_rd_en(rx_fifo_rd_en),
        .rx_pin(uart_rx)
    );

    always #1 clk50_1 = ~clk50_1; // 50 MHz clock
    always #1 clk50 = ~clk50; // 50 MHz clock
    initial begin
        // Initialize inputs
        btnU_1 = 0;
        btnU = 0;
        tx_fifo_payload_1 = 80'h00010203040506070809; // example data
        tx_fifo_wr_en_1 = 0;
        uart_rx_1 = 1; // idle state

        // Send data
        #10;
        tx_fifo_wr_en_1 = 1; // pulse to send data
        #2;
        tx_fifo_wr_en_1 = 0;
    end

endmodule
