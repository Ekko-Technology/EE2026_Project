`timescale 1ns / 1ps

module Top(
    input clk, btnU,

    output ov7670_pwdn, ov7670_reset, ov7670_xclk,
    input ov7670_href, ov7670_pclk_raw, ov7670_vsync,
    inout ov7670_siod,
    output ov7670_sioc,
    input [7:0] ov7670_d,
    output [15:0] led,
    input [1:0] sw,

    output vga_Hsync,
    output vga_Vsync,
    output [11:0] vga_RGB    //4-bit red, 4-bit green, 4-bit blue
    );

    // Generate 25MHz clock for VGA from 100MHz system clock
    // reg clk_div = 0;
    // reg clk25 = 0;
    // always @(posedge clk) begin
    //     clk_div <= clk_div + 1;
    //     if (clk_div == 1'b1) begin
    //         clk25 <= ~clk25;  // Toggle every 2 cycles = 25MHz
    //     end
    // end

    wire clk_status, clk25, clk24;
    display_clocks disp_clocks(
        .clk_in1(clk),
        .clk_out1(clk25),
        .clk_out2(clk24),
        .reset(btnU),
        .locked(clk_status)
    );

    // Wire for BRAM address from VGA controller
    wire [16:0] frame_addr;
    wire [14:0] frame_pixel_raw; // 15-bit RGB555 from BRAM
    wire [11:0] frame_pixel;     // Down-converted RGB444 to VGA
    wire active_area;
    
    //global clock buffer for camera pixel clock
    wire ov7670_pclk;
    BUFG pclk_bufg (
        .I(ov7670_pclk_raw),
        .O(ov7670_pclk)
    );

    VGA_Controller vga(
        .clk25(clk25),
        .vga_red(vga_RGB[11:8]),
        .vga_green(vga_RGB[7:4]), 
        .vga_blue(vga_RGB[3:0]),
        .vga_hsync(vga_Hsync),
        .vga_vsync(vga_Vsync),
        .frame_addr(frame_addr),
        .frame_pixel(frame_pixel),
        .bitmap_pixel(bitmap_pixel),
        .display_switch(sw[1]),
        .active_area(active_area)
    );
    OV7670_Controller ov7670(
        .clk(clk24),
        .resend(btnU),
        .config_finished(),
        .sioc(ov7670_sioc),
        .siod(ov7670_siod),
        .reset(ov7670_reset),
        .pwdn(ov7670_pwdn),
        .xclk(ov7670_xclk)
    );

    wire we;
    wire [16:0] addr;
    wire [14:0] dout; // RGB555 from capture

    // Debounce / synchronize btnU to camera pixel clock domain for capture reset
    reg [2:0] reset_sync = 3'b111;
    always @(posedge ov7670_pclk) begin
        reset_sync <= {reset_sync[1:0], btnU};
    end
    wire cap_reset = reset_sync[2];

    OV7670_Capture ov7670_capture(
        .pclk(ov7670_pclk),
        .vsync(ov7670_vsync),
        .href(ov7670_href),
        .ext_reset(cap_reset),
        .avg_enable(sw[0]),
        .d(ov7670_d),
        .addr(addr),
        .dout(dout),
        .we(we)
    );
    
    image_mem frame_buffer(
        .clka(ov7670_pclk),
        .wea(we),
        .addra(addr),
        .dina(dout),          // write RGB565
        .clkb(clk25),
        .addrb(frame_addr),
        .doutb(frame_pixel_raw) // read RGB565
    );

    // Image_Buffer frame_buffer(
    //     .clk_write(ov7670_pclk), // 24 MHz
    //     .write_addr(addr),
    //     .write_data(dout),    //15 bit RGB555 pixel data
    //     .write_done(vga_Vsync),
    //     .clk_read(clk25), // 25 MHz
    //     .read_addr(frame_addr),
    //     .read_data(frame_pixel_raw),    //15 bit RGB555 pixel data
    //     .frame_ready(led[1])
    // );

    // RGB555 -> RGB444 with simple LSB rounding (improves brightness balance)
    wire [4:0] r5 = frame_pixel_raw[14:10];
    wire [4:0] g5 = frame_pixel_raw[9:5];
    wire [4:0] b5 = frame_pixel_raw[4:0];

    wire [3:0] r4 = (r5[4:1] == 4'hF) ? 4'hF : (r5[4:1] + r5[0]);
    wire [3:0] g4 = (g5[4:1] == 4'hF) ? 4'hF : (g5[4:1] + g5[0]);
    wire [3:0] b4 = (b5[4:1] == 4'hF) ? 4'hF : (b5[4:1] + b5[0]);
    assign frame_pixel = { r4, g4, b4 };

    wire bitmap_pixel;
    // orange ball: 00111 01011 11101
    assign bitmap_pixel = (r5 >= 5'd5 && r5 <=5'd9 && g5 >= 5'd9 && g5 <= 5'd13 && b5 >= 5'd27 && b5 <= 5'd31) ? 1'b1 : 1'b0;
    bitmap_mem bitmap_buffer(
        .clka(clk25),
        .wea(we),
        .addra(frame_addr),
        .dina(bitmap_pixel),
        .clkb(clk25),
        .addrb(frame_addr),
        .doutb() // read RGB565
    );

    assign led = (frame_addr == 38240) ? {1'b0, r5, g5, b5} : led;

endmodule
