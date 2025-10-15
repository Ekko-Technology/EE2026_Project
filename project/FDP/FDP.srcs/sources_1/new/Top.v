`timescale 1ns / 1ps

module Top(
    input clk, btnU,

    output ov7670_pwdn, ov7670_reset, ov7670_xclk,
    input ov7670_href, ov7670_pclk, ov7670_vsync,
    inout ov7670_siod,
    output ov7670_sioc,
    input [7:0] ov7670_d,
    output [15:0] led,
    input [4:0] sw,

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
    wire [11:0] image_pixel; // 12-bit RGB444 from BRAM
    wire [11:0] frame_pixel;     // RGB444 to VGA
    wire active_area;

    VGA_Controller vga(
        .clk25(clk25),
        .vga_red(vga_RGB[11:8]),
        .vga_green(vga_RGB[7:4]), 
        .vga_blue(vga_RGB[3:0]),
        .vga_hsync(vga_Hsync),
        .vga_vsync(vga_Vsync),
        .frame_addr(frame_addr),
        .frame_pixel(frame_pixel),
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
    wire [11:0] dout; // RGB444 from capture

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
        .dina(dout),          // write RGB444
        .clkb(clk25),
        .addrb(frame_addr),
        .doutb(image_pixel) // read RGB444
    );

    wire bitmap_pixel;
    wire threshold_pixel;
    assign threshold_pixel = ((dout[3:0] >= 4'hB) && (dout[3:0] <= 4'hF) && //Red
                              (dout[7:4] >= 4'h6) && (dout[7:4] <= 4'hF) && //Green
                              (dout[11:8] >= 4'h0) && (dout[11:8] <= 4'h4)) //Blue
                              ? 1'b1 : 1'b0;
    bitmap_mem bitmap_buffer(
        .clka(ov7670_pclk),
        .wea(we),
        .addra(addr),
        .dina(threshold_pixel),          // write bitmap pixel
        .clkb(clk25),
        .addrb(frame_addr),
        .doutb(bitmap_pixel) // read bitmap pixel
    );

    // Direct RGB444 path
    assign frame_pixel = (~sw[1])
                            ? image_pixel
                            : (bitmap_pixel ? 12'hFFF : 12'h000);

    // Show middle pixel value on LEDs for debugging
    assign led = (frame_addr == 38240) ? {4'b0000, image_pixel} : led;

endmodule
