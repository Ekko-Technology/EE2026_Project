`timescale 1ns / 1ps

module Top(
    input clk, btnU,

    output ov7670_pwdn, ov7670_reset, ov7670_xclk,
    input ov7670_href, ov7670_pclk, ov7670_vsync,
    inout ov7670_siod,
    output ov7670_sioc,
    input [7:0] ov7670_d,
    output reg [15:0] led,
    input [4:0] sw,

    output vga_Hsync,
    output vga_Vsync,
    output [11:0] vga_RGB    //4-bit red, 4-bit green, 4-bit blue
    );

    // ----------- CLOCKS ----------- //
    // Generate 25 MHz (for VGA) and 24 MHz (for camera) clocks from 100 MHz input
    wire clk_status, clk25, clk24;
    display_clocks disp_clocks(
        .clk_in1(clk),
        .clk_out1(clk25),
        .clk_out2(clk24),
        .reset(btnU),
        .locked(clk_status)
    );

    // ----------- VGA CONTROLLER ----------- //
    // Wire for BRAM address from VGA controller
    wire [16:0] frame_addr;             // logical 0..(306*240-1)
    wire [11:0] image_pixel;            // 12-bit RGB444 from BRAM
    wire [11:0] frame_pixel;            // RGB444 to VGA
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
        .active_area(active_area)
    );

    // ----------- OV7670 CAMERA CONTROLLER ----------- //
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
    wire [16:0] addr;          // logical capture address within a frame (0..73439)
    wire [11:0] dout;          // RGB444 from capture

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
        .d(ov7670_d),
        .addr(addr),
        .dout(dout),
        .we(we)
    );

    //----------- PING PONG BUFFERS ----------- //
    //No hard guards or syncs now, both just triggering the BRAM upper/lower swap on Vsync
    //Seems to be working fine and no tears for now so f it we ball I guess
    
    //Ping-pong buffer parameters (RGB444, 12-bit):
    localparam [17:0] FRAME_PIXELS   = 18'd73440;
    localparam [17:0] TOTAL_PIXELS   = 18'd146880; // 2*73440

    //PCLK domain: toggle write buffer on rising edge of camera VSYNC
    reg vsync_d1 = 1'b0;
    reg vsync_d2 = 1'b0;
    always @(posedge ov7670_pclk) begin
        vsync_d1 <= ov7670_vsync;
        vsync_d2 <= vsync_d1;
    end
    wire cam_vsync_rise = (vsync_d1 & ~vsync_d2);

    //WRITE to top half at start (1 = TOP half, 0 = BOTTOM half)
    reg wr_sel = 1'b1; // start writing TOP
    always @(posedge ov7670_pclk) begin
        if (cap_reset) wr_sel <= 1'b1;  // reset to TOP
        else if (cam_vsync_rise) wr_sel <= ~wr_sel;
    end

    //clk25 (VGA) domain: toggle read buffer on rising edge of VGA VSYNC (technically vsync is active low but catching the posedge is fine too)
    reg [1:0] vga_rst_sync = 2'b00;
    reg vga_vsync_d1 = 1'b0, vga_vsync_d2 = 1'b0;
    always @(posedge clk25) begin
        vga_rst_sync <= {vga_rst_sync[0], btnU};
        vga_vsync_d1 <= vga_Vsync;
        vga_vsync_d2 <= vga_vsync_d1;
    end
    wire vga_reset = vga_rst_sync[1];
    wire vga_vsync_rise = (vga_vsync_d1 & ~vga_vsync_d2);

    //READ from bottom half at start (0 = BOTTOM half, 1 = TOP half)
    reg rd_sel = 1'b0; // start reading BOTTOM
    always @(posedge clk25) begin
        if (vga_reset) rd_sel <= 1'b0;  // reset to BOTTOM
        else if (vga_vsync_rise) rd_sel <= ~rd_sel;
    end

    // Compute physical addresses into 2x frame BRAM (18-bit addressing)
    wire [17:0] wr_base = wr_sel ? FRAME_PIXELS : 18'd0;
    wire [17:0] rd_base = rd_sel ? FRAME_PIXELS : 18'd0;
    wire [17:0] addra18 = {1'b0, addr} + wr_base;
    wire [17:0] addrb18 = {1'b0, frame_addr} + rd_base;

    image_mem frame_buffer(
        .clka(ov7670_pclk),
        .wea(we),
        .addra(addra18),
        .dina(dout),          // write RGB444
        .clkb(clk25),
        .addrb(addrb18),
        .doutb(image_pixel)   // read RGB444
    );

    //----------- BITMAP BUFFER ----------- //
    // Thresholding to create 1-bit bitmap from RGB444 input
    wire bitmap_pixel;
    wire threshold_pixel;
    assign threshold_pixel = ((dout[3:0] >= 4'hB) && (dout[3:0] <= 4'hF) && //Red
                              (dout[7:4] >= 4'h6) && (dout[7:4] <= 4'hF) && //Green
                              (dout[11:8] >= 4'h0) && (dout[11:8] <= 4'h4)) //Blue
                              ? 1'b1 : 1'b0;

    Dual_Port_Buffer bitmap_buffer(
        .clka(ov7670_pclk),
        .we(we),
        .addra(addra18),
        .dina(threshold_pixel), // write bitmap pixel
        .clkb(clk25),
        .addrb(addrb18),
        .doutb(bitmap_pixel) // read bitmap pixel
    );

    // Direct RGB444 path
    assign frame_pixel = (~sw[0]) ? image_pixel :
                         (bitmap_pixel ? 12'hFFF : 12'h000);

    // Show middle pixel value on LEDs for debugging
    always @(posedge clk25) begin
        if (active_area && (frame_addr == 38240)) begin
            led[11:0] <= image_pixel;
        end else begin
            led[11:0] <= led[11:0];
        end
    end

endmodule
