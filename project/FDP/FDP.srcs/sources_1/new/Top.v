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

    //----------- MEDIAN FILTER ----------- //
    wire pixel_valid;
    wire [11:0] filtered_pixel;
    wire [17:0] filtered_addr;

    Median_Filter #(
        .KERNEL_SIZE(3),
        .PIXEL_DEPTH(12),
        .IMAGE_WIDTH(306),
        .IMAGE_HEIGHT(240)
    )
    median_filter(
        .clk(ov7670_pclk),
        .reset(cap_reset),
        .frame_start(ov7670_vsync),
        .pixel_in(dout),
        .we(we),
        .pixel_out(filtered_pixel),
        .addr_out(filtered_addr),
        .pixel_valid(pixel_valid)
    );

    // Unified write controls for both RGB frame buffer and 1-bit bitmap buffer
    // Share write enable and address; keep separate data for RGB (12-bit) and bitmap (1-bit)
    reg        we_w;            // common write enable for both memories
    reg [17:0] waddr18_r;       // common write address for both memories (absolute with base)
    reg [11:0] rgb_dina_r;      // RGB data to image_mem
    reg        bmp_dina_r;      // bitmap data to Dual_Port_Buffer
    // Single pending entry for filtered overwrite (affects both memories)
    reg        pend;            // pending filtered write
    reg [17:0] pend_addr_q;     // absolute address for filtered center
    reg [11:0] fpix_q;          // latched filtered RGB pixel
    reg        bdin_q;          // latched filtered bitmap bit

    
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
    wire [17:0] addrb18 = {1'b0, frame_addr} + rd_base;

    // PCLK-domain writer:
    // - Image RGB: raw on we==1, filtered overwrite on we==0
    // - Bitmap 1-bit: raw-threshold on we==1, filtered-threshold overwrite on we==0
    always @(posedge ov7670_pclk) begin
        if (cap_reset) begin
            we_w <= 1'b0;
            waddr18_r <= 18'd0;
            rgb_dina_r <= 12'd0;
            bmp_dina_r <= 1'b0;
            pend <= 1'b0;
            pend_addr_q <= 18'd0;
            fpix_q <= 12'd0;
            bdin_q <= 1'b0;
        end else begin
            we_w <= 1'b0; // default no write; assert exactly once per cycle below

            // Latch a new filtered center write when available
            if (sw[1] && we && pixel_valid) begin
                pend <= 1'b1;
                pend_addr_q <= filtered_addr + wr_base; // absolute address in the active write half
                fpix_q <= filtered_pixel;
                // bitmap bit from filtered pixel (same address as RGB)
                bdin_q <= ((filtered_pixel[3:0]  >= 4'h9) && (filtered_pixel[3:0]  <= 4'hF) &&
                           (filtered_pixel[7:4]  >= 4'h6) && (filtered_pixel[7:4]  <= 4'hF) &&
                           (filtered_pixel[11:8] >= 4'h0) && (filtered_pixel[11:8] <= 4'h5)) ? 1'b1 : 1'b0;
            end

            // On the non-pixel byte cycles (we==0), perform the pending filtered write
            if (!we && pend) begin
                waddr18_r  <= pend_addr_q;
                rgb_dina_r <= fpix_q;
                bmp_dina_r <= bdin_q;
                we_w <= 1'b1;
                pend <= 1'b0;
            end

            // On pixel-complete cycles (we==1), always write the RAW pixel to its address
            if (we) begin
                waddr18_r  <= {1'b0, addr} + wr_base;
                rgb_dina_r <= dout;
                bmp_dina_r <= ((dout[3:0]  >= 4'h8) && (dout[3:0]  <= 4'hF) &&
                                   (dout[7:4]  >= 4'h6) && (dout[7:4]  <= 4'hF) &&
                                   (dout[11:8] >= 4'h0) && (dout[11:8] <= 4'h5)) ? 1'b1 : 1'b0;
                we_w <= 1'b1;
            end
        end
    end

    image_mem frame_buffer(
        .clka(ov7670_pclk),
        .wea(we_w),
        .addra(waddr18_r),
        .dina(rgb_dina_r),          // write RGB444 (raw on we cycles, filtered on alt cycles)
        .clkb(clk25),
        .addrb(addrb18),
        .doutb(image_pixel)   // read RGB444
    );

    //----------- BITMAP BUFFER (1-bit) ----------- //
    wire bitmap_pixel;

    Dual_Port_Buffer bitmap_buffer(
        .clka(ov7670_pclk),
        .we(we_w),
        .addra(waddr18_r),
        .dina(bmp_dina_r), // write bitmap pixel
        .clkb(clk25),
        .addrb(addrb18),
        .doutb(bitmap_pixel) // read bitmap pixel
    );

    // Direct RGB444 path
    // assign frame_pixel = (~sw[0]) ? image_pixel :
    //                      (bitmap_pixel ? 12'hFFF : 12'h000);

    // Show middle pixel value on LEDs for debugging
    // always @(posedge clk25) begin
    //     if (active_area && (frame_addr == 38240)) begin
    //         led[11:0] <= image_pixel;
    //     end else begin
    //         led[11:0] <= led[11:0];
    //     end
    // end




    wire [15:0] mouse_led;
    wire [11:0] mouse_vga_color;
    wire [15:0] servo_x_pwm; 
    wire [15:0] servo_y_pwm;

    // temporary holder for screen pixel coordinates
    wire [9:0] x_coord;
    wire [8:0] y_coord;

    // outputs PWM signals, crosshair overlay and LED bullets remaining
    mouse_movement mouse_ctrl (
        .clk(clk),
        .btnU(btnU),
        .x_coord(x_coord), // pass VGA horizontal pixel
        .y_coord(y_coord), // pass VGA vertical pixel
        .Mouse_Clk(Mouse_Clk),
        .Mouse_Data(Mouse_Data),
        .servo_x_pwm(servo_x_pwm),
        .servo_y_pwm(servo_y_pwm),
        .led(mouse_led),
        .vga_RGB(mouse_vga_color)
    );


    // Choose which value drives Top's led output
    always @(posedge clk25) begin
        if (active_area && frame_addr == 38240)
            led <= image_pixel;    // debug pixel
        else
            led <= mouse_led;      // mouse LEDs
    end


    // Combine camera and crosshair overlay colors
    assign frame_pixel = ((~sw[1])
                            ? image_pixel
                            : (bitmap_pixel ? 12'hFFF : 12'h000))
                          | mouse_vga_color;

endmodule
