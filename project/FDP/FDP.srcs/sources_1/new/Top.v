`timescale 1ns / 1ps

module Top(
    input clk, btnU, btnC,

    output ov7670_pwdn, ov7670_reset, ov7670_xclk,
    input ov7670_href, ov7670_pclk, ov7670_vsync,
    inout ov7670_siod,
    output ov7670_sioc,
    input [7:0] ov7670_d,
    output reg [15:0] led,
    input [4:0] sw,

    inout mouse_clk,
    inout mouse_data,

    output [7:0] seg,
    output [3:0] an,

    output vga_Hsync,
    output vga_Vsync,
    output [11:0] vga_RGB    //4-bit red, 4-bit green, 4-bit blue
    );

    localparam [23:0] RGB_THRESHOLD = {
        4'hF, 4'hF, //B_MIN, B_MAX
        4'hF, 4'hF, //G_MIN, G_MAX
        4'hF, 4'hF  //R_MIN, R_MAX
    };

    // ----------- CLOCKS ----------- //
    // Generate 25 MHz (for VGA) and 24 MHz (for camera) clocks from 100 MHz input
    wire clk_status, clk25, clk24, clk50;
    display_clocks disp_clocks(
        .clk_in1(clk),
        .clk_out1(clk25),
        .clk_out2(clk24),
        .clk_out3(clk50),
        .reset(btnU),
        .locked(clk_status)
    );

    // ----------- VGA CONTROLLER ----------- //
    // Wire for BRAM address from VGA controller
    wire [16:0] frame_addr;             // logical 0..(306*240-1)
    wire [11:0] image_pixel;            // 12-bit RGB444 from BRAM
    reg [11:0] frame_pixel;            // RGB444 to VGA
    wire [9:0] frame_x;  // current x coord in frame (0..639)
    wire [9:0] frame_y;  // current y coord in frame (0..479)
    wire active_area;

    VGA_Controller vga(
        .clk25(clk25),
        .vga_red(vga_RGB[11:8]),
        .vga_green(vga_RGB[7:4]), 
        .vga_blue(vga_RGB[3:0]),
        .vga_hsync(vga_Hsync),
        .vga_vsync(vga_Vsync),
        .frame_addr(frame_addr),
        .frame_x(frame_x),
        .frame_y(frame_y),
        .frame_pixel(frame_pixel),
        .active_area(active_area)
    );

    // ----------- OV7670 CAMERA CONTROLLER ----------- //
    // Generate a single-cycle resend pulse in the 24 MHz camera clock domain
    // so a button press replays the full I2C register sequence once.
    reg [2:0] btnU_sync24 = 3'b000;
    always @(posedge clk24) begin
        btnU_sync24 <= {btnU_sync24[1:0], btnU};
    end
    wire cam_resend_pulse = btnU_sync24[0] & ~btnU_sync24[1];

    OV7670_Controller ov7670(
        .clk(clk24),
        .resend(cam_resend_pulse),
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

    //----------- MEDIAN FILTERS (3x3 and 5x5) ----------- //
    // // 3x3 instance
    // wire pixel_valid_3x3;
    // wire [11:0] filtered_pixel_3x3;
    // wire [17:0] filtered_addr_3x3;
    // Median_Filter #(
    //     .KERNEL_SIZE(3),
    //     .PIXEL_DEPTH(12),
    //     .IMAGE_WIDTH(306),
    //     .IMAGE_HEIGHT(240)
    // )
    // median_filter(
    //     .clk(ov7670_pclk),
    //     .reset(cap_reset),
    //     .frame_start(ov7670_vsync),
    //     .pixel_in(dout),
    //     .we(we),
    //     .pixel_out(filtered_pixel_3x3),
    //     .addr_out(filtered_addr_3x3),
    //     .pixel_valid(pixel_valid_3x3)
    // );
    
    // 5x5 instance (stub for A/B testing)
    // wire pixel_valid_5x5;
    // wire [11:0] filtered_pixel_5x5;
    // wire [17:0] filtered_addr_5x5;
    // Median_Filter_5x5 #(
    //     .PIXEL_DEPTH(12),
    //     .IMAGE_WIDTH(306),
    //     .IMAGE_HEIGHT(240)
    // )
    // median_filter_5x5(
    //     .clk(ov7670_pclk),
    //     .reset(cap_reset),
    //     .frame_start(ov7670_vsync),
    //     .pixel_in(dout),
    //     .we(we),
    //     .pixel_out(filtered_pixel_5x5),
    //     .addr_out(filtered_addr_5x5),
    //     .pixel_valid(pixel_valid_5x5)
    // );

    //----------- GAUSSIAN 3x3 via Convolution3x3 ----------- //
    // Kernel: [1 2 1; 2 4 2; 1 2 1] with SCALE=4 (divide by 16)
    wire                    pixel_valid_gauss_3x3;
    wire [11:0]             filtered_pixel_gauss_3x3;
    wire [17:0]             filtered_addr_gauss_3x3;
    Convolution_3x3 #(
        .IMAGE_WIDTH(306),
        .IMAGE_HEIGHT(240),
        .PIXEL_DEPTH(12),
        .COEF_WIDTH(8),
        .k00(1), .k01(2), .k02(1),
        .k10(2), .k11(4), .k12(2),
        .k20(1), .k21(2), .k22(1),
        .BIAS(0),
        .SCALE(4)
    ) gaussian3x3 (
        .clk(ov7670_pclk),
        .reset(cap_reset),
        .frame_start(ov7670_vsync),
        .we(we),
        .mode_rgb(1'b1),             // camera provides RGB444
        .pixel_rgb_in(dout),
        .pixel_bin_in(1'b0),
        .pixel_out(filtered_pixel_gauss_3x3),
        .addr_out(filtered_addr_gauss_3x3),
        .pixel_valid(pixel_valid_gauss_3x3)
    );

    //----------- MORPHOLOGY (ERODE/DILATE) ----------- //
    // Explicit binary thresholds (reuse for writer and morphology)
    wire threshold_bin_gauss = (((filtered_pixel_gauss_3x3[3:0] >= RGB_THRESHOLD[23:20]) && (filtered_pixel_gauss_3x3[3:0] <= RGB_THRESHOLD[19:16]) &&
                                 (filtered_pixel_gauss_3x3[7:4] >= RGB_THRESHOLD[15:12]) && (filtered_pixel_gauss_3x3[7:4] <= RGB_THRESHOLD[11:8]) &&
                                 (filtered_pixel_gauss_3x3[11:8] >= RGB_THRESHOLD[7:4]) && (filtered_pixel_gauss_3x3[11:8] <= RGB_THRESHOLD[3:0])) ? 1'b1 : 1'b0);
    wire threshold_bin_raw   = (((dout[3:0]  >= RGB_THRESHOLD[23:20]) && (dout[3:0]  <= RGB_THRESHOLD[19:16]) &&
                                 (dout[7:4]  >= RGB_THRESHOLD[15:12]) && (dout[7:4]  <= RGB_THRESHOLD[11:8]) &&
                                 (dout[11:8] >= RGB_THRESHOLD[7:4]) && (dout[11:8] <= RGB_THRESHOLD[3:0])) ? 1'b1 : 1'b0);

    wire pixel_valid_erode_3x3;
    wire filtered_pixel_erode_3x3;
    wire [17:0] filtered_addr_erode_3x3;
    Morphology_3x3 #(
        .IMAGE_WIDTH(306),
        .IMAGE_HEIGHT(240)
    ) erode3x3 (
        .clk(ov7670_pclk),
        .reset(cap_reset),
        .frame_start(ov7670_vsync),
        .we(pixel_valid_gauss_3x3),
        // morphology operates on gaussian-thresholded pixels
        .pixel_in(threshold_bin_gauss),
        .op_dilate(1'b0),
        .pixel_out(filtered_pixel_erode_3x3),
        .addr_out(filtered_addr_erode_3x3),
        .pixel_valid(pixel_valid_erode_3x3)
    );

    wire pixel_valid_dilate_3x3;
    wire filtered_pixel_dilate_3x3;
    wire [17:0] filtered_addr_dilate_3x3;
    Morphology_3x3 #(
        .IMAGE_WIDTH(306),
        .IMAGE_HEIGHT(240)
    ) dilate3x3 (
        .clk(ov7670_pclk),
        .reset(cap_reset),
        .frame_start(ov7670_vsync),
        .we(pixel_valid_erode_3x3),
        .pixel_in(filtered_pixel_erode_3x3),
        .op_dilate(1'b1),
        .pixel_out(filtered_pixel_dilate_3x3),
        .addr_out(filtered_addr_dilate_3x3),
        .pixel_valid(pixel_valid_dilate_3x3)
    );

    // Select which binary pipeline to write into the bitmap buffer:
    // sw[1] enables filtered pipeline in writer; sw[2] selects morphology; sw[3] chooses dilate(1)/erode(0)
    wire bin_filtered_valid = (sw[2]) ? (sw[3] ? pixel_valid_dilate_3x3 : pixel_valid_erode_3x3)
                                      : pixel_valid_gauss_3x3;
    // centered write address (defined below after FRAME_PIXELS parameter)
    wire [17:0] bin_filtered_addr;

    wire bin_filtered_bit   = (sw[2]) ? (sw[3] ? filtered_pixel_dilate_3x3 : filtered_pixel_erode_3x3)
                                      : threshold_bin_gauss;


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

    // ----------- ADDRESS RECENTERING FOR 3x3 STAGES ----------- //
    // Cancel the inherent (-1,-1) window-center shift per stage by adding
    // (+1 row, +1 col) = +307 addresses for a 306x240 frame. Apply per-stage:
    //   Gaussian: +307; Erode(after Gauss): +614; Dilate(after Erode): +921
    localparam [17:0] ADDR_SHIFT1 = 18'd307;
    localparam [17:0] ADDR_SHIFT2 = 18'd614;
    localparam [17:0] ADDR_SHIFT3 = 18'd921;

    // Add shift and clamp to the last pixel to avoid OOB at bottom/right borders
    wire [18:0] gauss_tmp  = {1'b0, filtered_addr_gauss_3x3}  + ADDR_SHIFT1;
    wire [18:0] erode_tmp  = {1'b0, filtered_addr_erode_3x3}  + ADDR_SHIFT2;
    wire [18:0] dilate_tmp = {1'b0, filtered_addr_dilate_3x3} + ADDR_SHIFT3;

    wire [17:0] addr_gauss_centered  = (gauss_tmp  >= FRAME_PIXELS) ? (FRAME_PIXELS - 1) : gauss_tmp[17:0];
    wire [17:0] addr_erode_centered  = (erode_tmp  >= FRAME_PIXELS) ? (FRAME_PIXELS - 1) : erode_tmp[17:0];
    wire [17:0] addr_dilate_centered = (dilate_tmp >= FRAME_PIXELS) ? (FRAME_PIXELS - 1) : dilate_tmp[17:0];

    assign bin_filtered_addr = (sw[2]) ? (sw[3] ? addr_dilate_centered : addr_erode_centered)
                                       : addr_gauss_centered;

    // Latch the write-base per frame so all writes for a frame (including padding flush)
    // target the same half. Arm on VSYNC rise, capture on the first incoming pixel (we==1)
    // of the new frame to avoid any overlap hazards.
    reg [17:0] wr_base_frame = FRAME_PIXELS; // consistent with wr_sel reset to 1 (TOP) on cap_reset
    reg        wrb_arm = 1'b0;
    always @(posedge ov7670_pclk) begin
        if (cap_reset) begin
            wr_base_frame <= FRAME_PIXELS;
            wrb_arm <= 1'b0;
        end else begin
            if (cam_vsync_rise) begin
                wrb_arm <= 1'b1; // prepare to latch base for the next frame
            end
            if (wrb_arm && we) begin
                // wr_sel has already toggled on cam_vsync_rise; use current wr_base
                wr_base_frame <= wr_base;
                wrb_arm <= 1'b0;
            end
        end
    end

    // write control registers for both RGB frame buffer and 1-bit bitmap buffer
    // reg rgb_we_w;   // write enable for RGB frame buffer
    // reg bmp_we_w;   // write enable for bitmap buffer
    // reg [17:0] rgb_waddr18_r; // write address for RGB frame buffer (absolute with base)
    // reg [17:0] bmp_waddr18_r; // write address for bitmap buffer (absolute with base)
    // reg [11:0] rgb_dina_r; // RGB data to image_mem
    // reg bmp_dina_r; // bitmap data to Dual_Port_Buffer
    // reg we_latch; // latch to detect first pclk high after we

    // always @(posedge clk50) begin
    //     if (cap_reset) begin
    //         rgb_we_w <= 1'b0;
    //         bmp_we_w <= 1'b0;
    //         rgb_waddr18_r <= 18'd0;
    //         bmp_waddr18_r <= 18'd0;
    //         rgb_dina_r <= 12'd0;
    //         bmp_dina_r <= 1'b0;
    //         we_latch <= 1'b0;
    //     end else begin
    //         rgb_we_w <= 1'b0; // default no write
    //         bmp_we_w <= 1'b0; // default no write

    //         if (we && ~we_latch) begin
    //             // new pixel data and first time polled high pclk, so just write raw thresholded pixel
    //             we_latch <= 1'b1;
    //             rgb_we_w <= 1'b1;
    //             rgb_waddr18_r <= {1'b0, addr} + wr_base_frame;
    //             rgb_dina_r <= dout;
    //             // bitmap bit from raw pixel (same address as RGB)
    //             bmp_we_w <= 1'b1;
    //             bmp_waddr18_r <= {1'b0, addr} + wr_base_frame;
    //             bmp_dina_r <= threshold_pixel;
    //         end
    //         else begin
    //             if (!we && we_latch) begin
    //                 // reset latch on pclk low
    //                 we_latch <= 1'b0;
    //             end
    //             // either no new pixel data, or already read in current pclk high, so check convolution filters to overwrite old pixels
    //             if (sw[1] && pixel_valid_gauss_3x3) begin
    //                 //only edit bitmap based on filtered pixel
    //                 bmp_we_w <= 1'b1;
    //                 bmp_waddr18_r <= filtered_addr_gauss_3x3 + wr_base_frame;
    //                 bmp_dina_r <= threshold_pixel;
    //             end
    //             // if (sw[2] && pixel_valid_erode_3x3) begin
    //             //     //only edit bitmap based on filtered pixel
    //             //     bmp_we_w <= 1'b1;
    //             //     bmp_waddr18_r <= filtered_addr_erode_3x3 + wr_base_frame;
    //             //     bmp_dina_r <= filtered_pixel_erode_3x3;
    //             // end
    //         end
    //     end
    // end

    // Unified write controls for both RGB frame buffer and 1-bit bitmap buffer
    // Share write enable and address; keep separate data for RGB (12-bit) and bitmap (1-bit)
    reg        we_w;            // common write enable for both memories
    reg [17:0] waddr18_r;       // common write address for both memories (absolute with base)
    reg [11:0] rgb_dina_r;      // RGB data to image_mem
    reg        bmp_dina_r;      // bitmap data to Dual_Port_Buffer
    // Single pending entry for filtered overwrite (affects both memories)
    reg        pend;            // pending filtered write
    reg [17:0] pend_addr_q;     // absolute address for filtered center
    reg        bdin_q;          // latched filtered bitmap bit

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
            bdin_q <= 1'b0;
        end else begin
            we_w <= 1'b0; // default no write; assert exactly once per cycle below

            // Latch a pending filtered write only when we==1 (raw write occupies the port).
            if (we && sw[1] && bin_filtered_valid && !pend) begin
                pend <= 1'b1;
                pend_addr_q <= bin_filtered_addr + wr_base_frame; // absolute address in the latched frame half
                // bitmap bit from selected filtered pipeline (same address as RGB)
                bdin_q <= bin_filtered_bit;
            end

            // On the non-pixel cycles (we==0):
            //  1) If a pending entry exists (from a prior we==1), commit it first to preserve order.
            //  2) Else, if a current filtered pixel is valid, write it immediately (padding flush support).
            if (!we) begin
                if (pend) begin
                    waddr18_r  <= pend_addr_q;
                    // rgb_dina_r <= fpix_q;    // choose not to overwrite rgb frame
                    bmp_dina_r <= bdin_q;
                    we_w <= 1'b1;
                    pend <= 1'b0;
                end else if (sw[1] && bin_filtered_valid) begin
                    waddr18_r  <= bin_filtered_addr + wr_base_frame;
                    // rgb_dina_r <= filtered_pixel;    // choose not to overwrite rgb frame
                    bmp_dina_r <= bin_filtered_bit;
                    we_w <= 1'b1;
                end
            end

            // On pixel-complete cycles (we==1), always write the RAW pixel to its address
            if (we) begin
                waddr18_r  <= {1'b0, addr} + wr_base_frame;
                rgb_dina_r <= dout;
                bmp_dina_r <= threshold_bin_raw;
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
    //     if (active_area && (frame_addr == 36567)) begin
    //         ss_output[11:0] <= image_pixel;
    //     end else begin
    //         ss_output[11:0] <= ss_output[11:0];
    //     end
    // end


    // ----------- UFDS BRIDGE FOR FIND CONTOURS ----------- //

    wire [9:0] bbox_left, bbox_right, centroid_x;
    wire [8:0] bbox_top, bbox_bottom, centroid_y;
    wire ready_o;
    // Only feed UFDS within the 306x240 cropped active area (x in [14,319], y in [0,239])
    wire in_roi = active_area && (frame_x[9:1] >= 10'd14) && (frame_x[9:1] < 10'd320) && (frame_y[9:1] < 10'd240);
    // Decimate the VGA-doubled raster (640x480) to source grid (320x240):
    // take only even hCounter/vCounter pixels so each source pixel is enqueued once
    wire decim_hv = (~frame_x[0]) && (~frame_y[0]);
    UFDS_Bridge ufds_bridge (
        .pclk(clk25),
        .p_rst(cap_reset),
    // Gate valid to ROI AND decimate by 2x2 so UFDS sees exactly 306x240 unique pixels per frame
    .p_valid(in_roi && decim_hv),
        .p_x(frame_x[9:1] - 14),
        .p_y(frame_y[9:1]),
        .p_px(bitmap_pixel), // use same bitmap data as pixel input
        .clk(clk50),
        .ext_reset(cap_reset),
        .bbox_left(bbox_left),
        .bbox_right(bbox_right),
        .bbox_top(bbox_top),
        .bbox_bottom(bbox_bottom),
        .centroid_x(centroid_x),
        .centroid_y(centroid_y),
        .ready_o(ready_o)
    );

    // ----------- MOUSE CONTROLLER ----------- //
    wire [15:0] mouse_led;
    wire [11:0] mouse_vga_color;

    // Servo PWM outputs that are one bit, toggled high/low depending on pwm signal
    wire servo_x_pwm;
    wire servo_y_pwm;

    // temporary holder for screen pixel coordinates (to be replaced)
    wire [9:0] x_coord;
    wire [8:0] y_coord;

    // wire [7:0] cooldown_progress;

    // mouse controller module
    mouse_movement mouse_ctrl (
        .clk(clk),
        .btnU(btnU),
        .x_coord(x_coord),
        .y_coord(y_coord),
        .mouse_clk(mouse_clk),
        .mouse_data(mouse_data),
        .servo_x_pwm(servo_x_pwm),
        .servo_y_pwm(servo_y_pwm),
        .led(mouse_led),
        .cooldown_progress(cooldown_progress)
    );

    // always @(*) begin
    //     led = mouse_led;
    // end


    // ----------- DISPLAY OUTPUTS ----------- //
    // reg [15:0] ss_output = 16'h0000;
    // Seven_Seg ssd (
    //     .clk(clk),
    //     .num(ss_output),
    //     .dd(4'b0000),
    //     .seg(seg),
    //     .an(an)
    // );



    // Gradual color change feature
    localparam GREEN = 12'h0F0;
    localparam RED = 12'hF00;
    localparam COOLDOWN = 200_000_000;


    localparam CROSSHAIR_HEIGHT = 11;
    wire [7:0] fill_height = (cooldown_progress * CROSSHAIR_HEIGHT) / 255;
    wire [7:0] green_start_y = 120 + 11;        // bottom of the stem
    wire [7:0] green_top_y = green_start_y - fill_height;    // current row index of green 

    reg [8:0] bbox_left_latch, bbox_right_latch;
    reg [7:0] bbox_top_latch, bbox_bottom_latch;
    reg [8:0] centroid_x_latch;
    reg [7:0] centroid_y_latch;
    wire [15:0] crosshair_radius_squared = (frame_x[9:1]-14-153)*(frame_x[9:1]-14-153) + (frame_y[9:1]-120)*(frame_y[9:1]-120);

    integer crosshair_radius = 7;

    always @(posedge clk25) begin
        if (frame_addr == 73439) begin
            bbox_left_latch   <= bbox_left;
            bbox_right_latch  <= bbox_right;
            bbox_top_latch    <= bbox_top;
            bbox_bottom_latch <= bbox_bottom;
            centroid_x_latch  <= centroid_x;
            centroid_y_latch  <= centroid_y;

        end else begin
            // --- Crosshair drawing with cooldown-based bottom fill ---
            // === Bottom vertical arm ===
            if (frame_x[9:1]-14 == 153 &&
                frame_y[9:1] >= 120+2 && frame_y[9:1] <= 120+11) begin

                // Green portion rises upward from bottom
                if (frame_y[9:1] >= green_top_y)
                    frame_pixel <= GREEN;
                else
                    frame_pixel <= RED;
            end

            // === Top vertical arm ===
            else if (frame_x[9:1]-14 == 153 &&
                    frame_y[9:1] >= 120-11 && frame_y[9:1] <= 120-2) begin

                // Mirror the same cooldown progress upward
                if (frame_y[9:1] <= (120 - CROSSHAIR_HEIGHT + fill_height))
                    frame_pixel <= GREEN;
                else
                    frame_pixel <= RED;
            end

            // === Left horizontal arm ===
            else if (frame_y[9:1] == 120 &&
                    frame_x[9:1]-14 >= 153-11 && frame_x[9:1]-14 <= 153-2) begin

                // Turn green once cooldown crosses midpoint
                if (fill_height >= CROSSHAIR_HEIGHT / 2)
                    frame_pixel <= GREEN;
                else
                    frame_pixel <= RED;
            end

            // === Right horizontal arm ===
            else if (frame_y[9:1] == 120 &&
                    frame_x[9:1]-14 >= 153+2 && frame_x[9:1]-14 <= 153+11) begin

                if (fill_height >= CROSSHAIR_HEIGHT / 2)
                    frame_pixel <= GREEN;
                else
                    frame_pixel <= RED;
            end
            
            // Draw bbox overlay only inside ROI to avoid artifacts outside the cropped area
            // if (frame_x[9:1]-14 == 153 && frame_y[9:1] >= 120+2 && frame_y[9:1] <= 120+11 || //bottom long vertical line
            //     frame_x[9:1]-14 == 153 && frame_y[9:1] >= 120-11 && frame_y[9:1] <= 120-2 || //top long vertical line
            //     frame_y[9:1] == 120 && frame_x[9:1]-14 >= 153-11 && frame_x[9:1]-14 <= 153-2 || //left long horizontal line
            //     frame_y[9:1] == 120 && frame_x[9:1]-14 >= 153+2 && frame_x[9:1]-14 <= 153+11 //right long horizontal line
            // ) begin
            //     //draw red circle and crosshair at centre of screen
            //     frame_pixel <= 12'h0F0;
            // end
            else if (in_roi && 
                (
                    frame_x[9:1]-14 == bbox_left_latch && frame_y[9:1] >= bbox_top_latch && frame_y[9:1] <= bbox_bottom_latch ||   //bbox left
                    frame_x[9:1]-14 == bbox_right_latch && frame_y[9:1] >= bbox_top_latch && frame_y[9:1] <= bbox_bottom_latch ||   //bbox right
                    frame_y[9:1] == bbox_top_latch && frame_x[9:1]-14 >= bbox_left_latch && frame_x[9:1]-14 <= bbox_right_latch ||  //bbox top
                    frame_y[9:1] == bbox_bottom_latch && frame_x[9:1]-14 >= bbox_left_latch && frame_x[9:1]-14 <= bbox_right_latch ||  //bbox bottom
                    frame_x[9:1]-14 == centroid_x_latch && frame_y[9:1] >= centroid_y_latch-2 && frame_y[9:1] <= centroid_y_latch+2 || //centroid vertical line
                    frame_y[9:1] == centroid_y_latch && frame_x[9:1]-14 >= centroid_x_latch-2 && frame_x[9:1]-14 <= centroid_x_latch+2    //centroid horizontal line
                )
            ) begin
                frame_pixel <= 12'h00F;
            end
            else begin
                if (sw[0]) frame_pixel <= (bitmap_pixel ? 12'hFFF : 12'h000);
                else frame_pixel <= image_pixel;
            end
        end
    end


    // Sets timer
    Time_Countdown timer_inst (
        .clk(clk),
        .sw(sw[3:0]),
        .btnC(btnC),
        .btnU(btnU),
        .seg(seg[7:0]),
        .an(an[3:0])
    );



endmodule




            // // bottom vertical y
            // if (frame_y[9:1] >= 122 && frame_y[9:1] <= 131) begin
            //     if (frame_x[9:1]-14 == 153 && frame_y[9:1] >= green_top_y && frame_y[9:1] <= 120+11) begin
            //         // Bottom vertical line (cooldown fill rising from bottom)
            //         frame_pixel <= GREEN;
            //     end
            //     if (frame_x[9:1]-14 == 153 && frame_y[9:1] >= 120+2 && frame_y[9:1] <= green_top_y) begin
            //         // Bottom vertical line (cooldown fill rising from bottom)
            //         frame_pixel <= RED;
            //     end
            // end

            // // horizontal lines
            // if (frame_y[9:1] >= 116 && frame_y[9:1] <= 124)
            //     if (frame_y[9:1] >= green_top_y && frame_y[9:1] <= 124 && frame_x[9:1]-14 >= 153-11 && frame_x[9:1]-14 <= 153-2) begin
            //         frame_pixel <= GREEN;

            //     if (frame_y[9:1] >= 116 && frame_y[9:1] <= green_top_y && frame_x[9:1]-14 >= 153-11 && frame_x[9:1]-14 <= 153-2) begin
            //         frame_pixel <= RED;
            // end 

            // // top vertical y
            // if (frame_y[9:1] >= 109 && frame_y[9:1] <= 118) begin
            //     if (frame_x[9:1]-14 == 153 && frame_y[9:1] >= green_top_y && frame_y[9:1] <= 120-2) begin
            //         // Bottom vertical line (cooldown fill rising from bottom)
            //         frame_pixel <= GREEN;
            //     end
            //     if (frame_x[9:1]-14 == 153 && frame_y[9:1] >= 120-11 && frame_y[9:1] <= green_top_y) begin
            //         // Bottom vertical line (cooldown fill rising from bottom)
            //         frame_pixel <= RED;
            //     end
            // end