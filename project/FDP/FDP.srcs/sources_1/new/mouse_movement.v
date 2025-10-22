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
    input [9:0] x_coord,  // pixel index width
    input [8:0] y_coord, // pixel index height
    inout mouse_clk,  //PS2 mouse clock
    inout mouse_data,  //PS2 data packets
    output servo_x_pwm, // PWM signal for X servo
    output servo_y_pwm, // PWM signal for Y servo
    output reg [15:0] led,  //Bullet counts
    output reg [7:0] cooldown_progress
    );
    
    wire [11:0] xpos;
    wire [11:0] ypos;
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
        .setx(1'b0),
        .sety(1'b0),
        .setmax_x(10'd640),
        .setmax_y(9'd480),
        .value(12'd1024)
    );

    // capture the overall x and y movements of the mouse every clock edge
    reg signed [11:0] prev_xpos;
    reg signed [11:0] prev_ypos;
    // store deltas with more bits since we are using signed magnitudes 
    reg signed [15:0] delta_x;
    reg signed [15:0] delta_y;

    always @(posedge clk) begin
        if (btnU) begin
            prev_xpos <= xpos;
            prev_ypos <= ypos;
            delta_x <= 0;
            delta_y <= 0;
        end else if (new_event) begin
            delta_x <= $signed(xpos) - $signed(prev_xpos);
            delta_y <= $signed(ypos) - $signed(prev_ypos);

            prev_xpos <= xpos;
            prev_ypos <= ypos;
        end
    end


    // Servo angles
    reg signed [15:0] servo_x_angle;
    reg signed [15:0] servo_y_angle;

    // use magnitude of deltas to filter sharp or subtle movements
    reg signed [15:0] filtered_x;
    reg signed [15:0] filtered_y;

    // Parameters to ignore sharp or subtle movements (must experiment with)
    localparam MIN_MOVE = 2; // ignore slow delta movements
    localparam MAX_MOVE = 20;// ignore fast delta movements

    // Servo update smoothing, where servo angles are only updated every SMOOTH_DIV cycles
    // localparam SMOOTH_DIV = 100_000; // 1 ms update at 100 MHz
    // reg [16:0] smooth_counter = 0;
    // reg smooth_tick = 0;

    // always @(posedge clk) begin
    //     if (smooth_counter >= SMOOTH_DIV-1) begin
    //         smooth_counter <= 0;
    //         smooth_tick <= 1;
    //     end else begin
    //         smooth_counter <= smooth_counter + 1;
    //         smooth_tick <= 0;
    //     end
    // end

    // Update servo angles every smooth_tick
    always @(posedge clk) begin
        if (btnU) begin
            servo_x_angle <= 90;
            servo_y_angle <= 90;
        // end else if (new_event && smooth_tick) begin
        end
        else if (new_event) begin
            // Filter X
            if (delta_x > MIN_MOVE)
                filtered_x = (delta_x > MAX_MOVE) ? MAX_MOVE : delta_x;
            else if (delta_x < -MIN_MOVE)
                filtered_x = (delta_x < -MAX_MOVE) ? -MAX_MOVE : delta_x;
            else
                filtered_x = 0;

            // Filter Y
            if (delta_y > MIN_MOVE)
                filtered_y = (delta_y > MAX_MOVE) ? MAX_MOVE : delta_y;
            else if (delta_y < -MIN_MOVE)
                filtered_y = (delta_y < -MAX_MOVE) ? -MAX_MOVE : delta_y;
            else
                filtered_y = 0;

            // Apply filtered deltas
            servo_x_angle <= servo_x_angle + filtered_x;
            servo_y_angle <= servo_y_angle + filtered_y;

            // Clamp angles
            if (servo_x_angle < 0) servo_x_angle <= 1;
            else if (servo_x_angle > 180) servo_x_angle <= 179;

            if (servo_y_angle < 0) servo_y_angle <= 1;
            else if (servo_y_angle > 180) servo_y_angle <= 179;
        end
    end
    
    // Servo PWM generation (50 Hz, 20 ms period)
    reg [20:0] pwm_counter = 0;  // counts up to 1,000,000 for 20 ms at 50 MHz clock
    reg servo_x_out;
    reg servo_y_out;

    always @(posedge clk) begin
        // Reset counter every 20 ms as most servo motors works with such
        if (pwm_counter >= 2_000_000 - 1)
            pwm_counter <= 0;
        else
            pwm_counter <= pwm_counter + 1;
    end

    // Convert servo angle (0–180 degrees) to pulse width (1 ms – 2 ms)
    wire [20:0] pulse_width_x = 100_000 + (servo_x_angle * 100_000 / 180);
    wire [20:0] pulse_width_y = 100_000 + (servo_y_angle * 100_000 / 180);

    // Generate PWM
    always @(posedge clk) begin
        servo_x_out <= (pwm_counter < pulse_width_x);
        servo_y_out <= (pwm_counter < pulse_width_y);
    end

    assign servo_x_pwm = servo_x_out;
    assign servo_y_pwm = servo_y_out;



    // Logic for Crosshair overlay and LED for ammo rounds 
    // Screen parameters
<<<<<<< HEAD
    // localparam WIDTH  = 306;
    // localparam HEIGHT = 240;
    // localparam CENTER_X = WIDTH / 2;  // 153
    // localparam CENTER_Y = HEIGHT / 2;  // 120
=======
    localparam WIDTH  = 306;
    localparam HEIGHT = 240;
    localparam CENTER_X = WIDTH / 2;  // 153
    localparam CENTER_Y = HEIGHT / 2;  // 120
>>>>>>> 3dc90a101e5a55603eab2c9b2cbf1d18b5f4457c
    
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
