`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 17.10.2025 14:37:16
// Design Name: 
// Module Name: mouse_movement
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


module mouse_movement(
    input clk,
    input btnU, // reset
    input [13:0] x_coord,  // pixel index width
    input [13:0] y_coord, // pixel index height
    inout mouse_clk,  //PS2 mouse clock
    inout mouse_data,  //PS2 data packets
    `ifdef SIMULATION
        input left_sim, // simulation-only input to simulate left clicks
    `endif
    output servo_x_pwm, // PWM signal for X servo
    output servo_y_pwm, // PWM signal for Y servo
    output reg [15:0] led,  //Bullet counts
    output reg [7:0] cooldown_progress
    );
    
    wire [13:0] xpos;
    wire [13:0] ypos;
    wire [3:0] zpos;

// for simulation purposes
    wire left, right, middle, new_event;
    
    MouseCtl mouse_instance (
        .clk(clk),
        .rst(btnU),
        .ps2_clk(mouse_clk),
        .ps2_data(mouse_data),
        .xpos(xpos),
        .ypos(ypos),
        .zpos(zpos),
        .left(left),
        .middle(middle),
        .right(right),
        .new_event(new_event),
        // .setx(1'b0),
        // .sety(1'b0),
        // .setmax_x(10'd640),
        // .setmax_y(9'd480),
        .setx(),
        .sety(),
        .setmax_x(),
        .setmax_y(),
        // .value(12'd1024)
        .value()
    );

    // capture the overall x and y movements of the mouse every clock edge
    reg signed [13:0] prev_xpos;
    reg signed [13:0] prev_ypos;
    // store deltas with more bits since we are using signed magnitudes 
    reg signed [15:0] delta_x;
    reg signed [15:0] delta_y;

    // If input resolution increased (14-bit) but original behavior assumed 12-bit,
    // shift deltas down by COORD_SCALE_SHIFT bits to keep the same magnitude range.
    localparam COORD_SCALE_SHIFT = 2; // 14-bit -> 12-bit: shift right by 2
    wire signed [15:0] scaled_delta_x = delta_x >>> COORD_SCALE_SHIFT;
    wire signed [15:0] scaled_delta_y = delta_y >>> COORD_SCALE_SHIFT;

    // Servo angles
    reg signed [15:0] servo_x_angle;
    reg signed [15:0] servo_y_angle;

    // use magnitude of deltas to filter sharp or subtle movements
    reg signed [15:0] filtered_x;
    reg signed [15:0] filtered_y;

    // Parameters to ignore sharp or subtle movements (must experiment with)
    localparam MIN_MOVE = 2; // ignore slow delta movements
    localparam MAX_MOVE = 20;// ignore fast delta movements

    // Update servo angles every smooth_tick
    always @(posedge clk) begin
        if (btnU) begin
            prev_xpos <= xpos;
            prev_ypos <= ypos;
            delta_x <= 0;
            delta_y <= 0;
            servo_x_angle <= 90;
            servo_y_angle <= 90;
        end else if (new_event) begin
            delta_x <= $signed(xpos) - $signed(prev_xpos);
            delta_y <= $signed(ypos) - $signed(prev_ypos);

            prev_xpos <= xpos;
            prev_ypos <= ypos;
        end

    
        if (new_event) begin
            // Filter X (use scaled deltas so behavior matches original 12-bit inputs)
            if (scaled_delta_x > MIN_MOVE)
                filtered_x = (scaled_delta_x > MAX_MOVE) ? MAX_MOVE : scaled_delta_x;
            else if (scaled_delta_x < -MIN_MOVE)
                filtered_x = (scaled_delta_x < -MAX_MOVE) ? -MAX_MOVE : scaled_delta_x;
            else
                filtered_x = 0;

            // Filter Y
            if (scaled_delta_y > MIN_MOVE)
                filtered_y = (scaled_delta_y > MAX_MOVE) ? MAX_MOVE : scaled_delta_y;
            else if (scaled_delta_y < -MIN_MOVE)
                filtered_y = (scaled_delta_y < -MAX_MOVE) ? -MAX_MOVE : scaled_delta_y;
            else
                filtered_y = 0;

            // Apply filtered deltas
            servo_x_angle <= servo_x_angle + filtered_x;
            servo_y_angle <= servo_y_angle + filtered_y;

            // Clamp angles
            if ((servo_x_angle + filtered_x) < 0) servo_x_angle <= 1;
            else if ((servo_x_angle + filtered_x) > 180) servo_x_angle <= 179;

            if ((servo_y_angle + filtered_y) < 0) servo_y_angle <= 1;
            else if ((servo_y_angle + filtered_y) > 180) servo_y_angle <= 179;
        end
    end
    
    // Servo PWM generation (50 Hz, 20 ms period)
    reg [20:0] pwm_counter = 0;  // counts up to 1,000,000 for 20 ms at 50 MHz clock
    reg servo_x_out;
    reg servo_y_out;

    // reg [20:0] pulse_widths [0:180];

    // integer i = 0;
    // initial begin
    //     for (i = 0; i <= 180; i = i + 1) begin
    //         pulse_widths[i] = 100_000 + (i * 100_000 / 180);
    //     end
    // end
    // Map raw mouse coordinate directly to pulse width (no runtime division):
    // raw in 0..IN_MAX maps to 100_000..200_000
    // For X we keep previous inversion (0->max pulse, IN_MAX->min pulse)
    // Now using full xpos/ypos width (14 bits). Map 0..16383 -> 100_000..200_000
    localparam integer IN_MAX = 14'd16383; // raw input max (14-bit)
    localparam integer FP_N  = 16; // fractional bits for fixed-point reciprocal
    // FP_K = round(100000 * 2^FP_N / IN_MAX)
    // 100000 * 65536 / 16383 ≈ 400024.415 -> use 400024
    localparam integer FP_K  = 400024; // precomputed constant for 100000/16383 * 2^16

    // clamp raw inputs to IN_MAX (now 14 bits)
    wire [13:0] raw_x = (xpos > IN_MAX) ? IN_MAX : xpos[13:0];
    wire [13:0] raw_y = (ypos > IN_MAX) ? IN_MAX : ypos[13:0];

    // reverse X like previous code: use IN_MAX - raw_x
    wire [13:0] raw_x_rev = IN_MAX - raw_x;

    // multiply then shift to divide by IN_MAX (using FP_K)
    // widen multiply to avoid overflow: raw(14) * FP_K(~19) => ~33 bits; use 40 bits for headroom
    wire [39:0] mult_x = raw_x_rev * FP_K;
    wire [39:0] mult_y = raw_y * FP_K;

    // add rounding and shift down by FP_N fractional bits
    wire [20:0] pulse_width_x = 100_000 + ((mult_x + (1 << (FP_N-1))) >> FP_N);
    wire [20:0] pulse_width_y = 100_000 + ((mult_y + (1 << (FP_N-1))) >> FP_N);

    always @(posedge clk) begin
        // Reset counter every 20 ms as most servo motors works with such
        if (pwm_counter >= 2_000_000 - 1)
            pwm_counter <= 0;
        else
            pwm_counter <= pwm_counter + 1;
        servo_x_out <= (pwm_counter < pulse_width_x);
        servo_y_out <= (pwm_counter < pulse_width_y);
    end

    // wire [20:0] pulse_width_x = pulse_widths[(180 - servo_x_angle)];
    // wire [20:0] pulse_width_y = pulse_widths[servo_y_angle];

    assign servo_x_pwm = servo_x_out;
    assign servo_y_pwm = servo_y_out;



    // Logic for Crosshair overlay and LED for ammo rounds 
    // Screen parameters
    localparam WIDTH  = 306;
    localparam HEIGHT = 240;
    localparam CENTER_X = WIDTH / 2;  // 153
    localparam CENTER_Y = HEIGHT / 2;  // 120
    
    // // Crosshair parameters
    // localparam CH_HEIGHT = 20;
    // localparam CH_WIDTH  = 20;
    // localparam CH_THICKNESS = 3;
    // localparam CH_CENTER_DOT_THICKNESS = 4;
   
    // localparam GAP_FROM_CENTER_DOT = 5;
    
    // COLOR OUTPUT
    localparam GREEN = 12'h0F0;
    localparam RED   = 12'hF00;
    
    reg [27:0] cooldown_timer = 0;
    reg shot_enabled = 1;
    reg [4:0] bullet_count = 5'd16;

    // Add 2 seconds cooldown on each shot
`ifdef SIMULATION
    localparam COOLDOWN = 10; // very short cooldown for simulation
`else
    localparam COOLDOWN = 200_000_000; // 1s
`endif


    always @(posedge clk) begin
        // Reset game 
        if (btnU) begin
            cooldown_timer <= 0;
            shot_enabled <= 1;
            bullet_count <= 16;
            cooldown_progress <= 8'd255;
            led <= 16'hFFFF;
        end
        else begin
            // Decrement cooldown timer if not zero
            if (cooldown_timer > 0)
                cooldown_timer <= cooldown_timer - 1;

            // compute cooldown progress
            cooldown_progress <= (cooldown_timer == 0) ? 8'd255 : (cooldown_timer * 255 / COOLDOWN);

            if (cooldown_timer == 0)
                shot_enabled <= 1;

            // Fire if left is held, bullets remain, and cooldown expired
            if (left && bullet_count > 0 && shot_enabled) begin
                cooldown_timer <= COOLDOWN;   // reset cooldown
                bullet_count <= bullet_count - 1;
                led <= led >> 1; // shift LED to the right to indicate ONE bullet used
                shot_enabled <= 0; // disable shooting until cooldown expires
            end            
        end
    end
   
//    wire crosshair_pixel;
//    // Crosshair horizontal and vertical arms centered on screen
//    assign crosshair_pixel = (
//        // left horizontal crosshair
//        (y_coord >= CENTER_Y - CH_THICKNESS/2 && y_coord <= CENTER_Y + CH_THICKNESS/2 &&
//         x_coord >= CENTER_X - GAP_FROM_CENTER_DOT - CH_WIDTH && x_coord <= CENTER_X - GAP_FROM_CENTER_DOT) ||
         
//        // right horizontal crosshair
//         (y_coord >= CENTER_Y - CH_THICKNESS/2 && y_coord <= CENTER_Y + CH_THICKNESS/2 &&
//          x_coord >= CENTER_X + GAP_FROM_CENTER_DOT && x_coord <= CENTER_X + GAP_FROM_CENTER_DOT + CH_WIDTH) ||
   
//        // top vertical crosshair
//        (x_coord >= CENTER_X - CH_THICKNESS/2 && x_coord <= CENTER_X + CH_THICKNESS/2 &&
//         y_coord >= CENTER_Y - GAP_FROM_CENTER_DOT - CH_HEIGHT && y_coord <= CENTER_Y - GAP_FROM_CENTER_DOT) ||
         
//        // bottom vertical crosshair
//         (x_coord >= CENTER_X - CH_THICKNESS/2 && x_coord <= CENTER_X + CH_THICKNESS/2 &&
//          y_coord >= CENTER_Y + GAP_FROM_CENTER_DOT && y_coord <= CENTER_Y + GAP_FROM_CENTER_DOT + CH_HEIGHT) ||
   
//        // Center dot
//        (x_coord >= CENTER_X - CH_CENTER_DOT_THICKNESS/2 && x_coord <= CENTER_X + CH_CENTER_DOT_THICKNESS/2 &&
//         y_coord >= CENTER_Y - CH_CENTER_DOT_THICKNESS/2 && y_coord <= CENTER_Y + CH_CENTER_DOT_THICKNESS/2)
//    );
   
   
    // always @(*) begin
    //     if (crosshair_pixel) begin
    //         if (shot_enabled)
    //             vga_RGB = GREEN;
    //         else
    //             vga_RGB = RED;
    //     end else begin
    //         vga_RGB = 12'h000; // background
    //     end
    // end
    
endmodule
