`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 21.10.2025 21:20:34
// Design Name: 
// Module Name: Time_Countdown
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


// Display countdown timer on 7 segment display with 30 secs, 1min and 1min 30 secs countdown options

module Time_Countdown(
    input clk,
    input [3:0] sw,
    input btnC, // start timer
    input btnU, // reset timer
    output reg [7:0] seg,
    output reg [3:0] an
    );

    localparam countdown_30s = 8'd30; // 30 seconds countdown
    localparam countdown_60s = 8'd60; // 1 minute countdown
    localparam countdown_90s = 8'd90; // 1 minute 30 seconds countdown
    localparam countdown_120s = 8'd120; // 2 minutes countdown

    reg [26:0] clk_counter = 27'd0; // Clock divider for 1 second tick
    reg one_sec_tick = 1'b0; // 1 second tick signal

    // Logic to handle countdown and display on 7-segment
    always @(posedge clk) begin 
        if (clk_counter == 100_000_000) begin // Assuming 100MHz clock for 1 second tick
            clk_counter <= 0;
            one_sec_tick <= 1'b1;
        end else begin
            clk_counter <= clk_counter + 1;
            one_sec_tick <= 1'b0;
        end
    end


    // split 4 bits each for each segment LED to display MM:SS
    reg [10:0] countdown_value;
    reg [6:0] minutes;
    reg [6:0] seconds;
    reg [3:0] lhs_min;
    reg [3:0] rhs_min;
    reg [3:0] lhs_sec;
    reg [3:0] rhs_sec;

    reg [24:0] btnC_Counter = 0;
    reg [25:0] blink_counter = 0;
    reg blink_state = 0;


    // initialize still_winning as one as user has not bust timer
    reg still_winning = 1'b1;
    reg paused = 1'b1;
    reg initialized = 1'b1;

    always @(posedge clk) begin
        // make sure counter is no longer counting or paused before allowing to reset user to has NOT lost and original timing
        if (btnU && paused) begin
            still_winning <= 1;
            paused <= 1;
            initialized <= 1;
            // countdown_value <= 0; // reset timer to 0
        end 
        else begin
            // Handle switch initialization when user has not lost
            if (initialized && paused && still_winning) begin
                if (sw[3]) begin 
                    countdown_value <= countdown_120s;
                end
                else if (sw[2]) begin
                    countdown_value <= countdown_90s;
                end
                else if (sw[1]) begin 
                    countdown_value <= countdown_60s;
                end
                else if (sw[0]) begin
                    countdown_value <= countdown_30s;
                end
                else begin
                    countdown_value <= 0;
                end
            end
        end

        // Button C toggle logic for debouncing
        if (btnC_Counter > 0) begin
            btnC_Counter <= btnC_Counter - 1;
        end
        else if (btnC) begin
            paused <= ~paused;
            btnC_Counter <= 25'd20000000; // 0.2s
            initialized <= 0;
        end
        else if (~paused && one_sec_tick) begin
            //  if paused is toggled by btnC, means it is paused
            if (countdown_value > 0)
                countdown_value <= countdown_value - 1;
            // once countdown_value reaches zero, signalling that user has lost 
            else if (countdown_value == 0) begin
                still_winning <= 1'b0;
                paused <= 1'b1;
                initialized <= 0; // only initialized to 1 by btnU(reset) is pressed
            end
        end
    end

    always @(posedge clk) begin 
        blink_counter <= blink_counter + 1;
        if (blink_counter == 50_000_000 - 1) begin
            blink_counter <= 0;
            blink_state <= ~blink_state;
        end
    end

    always @(posedge clk) begin
        minutes = countdown_value / 60;
        seconds = countdown_value % 60;

        lhs_min = minutes / 10;
        rhs_min = minutes % 10;
        lhs_sec = seconds / 10;
        rhs_sec = seconds % 10;
    end


    reg [1:0] refresh_counter = 0;
    reg [4:0] current_digit; //[4] = decimal point, [3:0] = hex digit
    reg [17:0] counter_381hz = 0; //18 bit counter, for 381hz clock enable signal
    reg clock_enable_381hz = 0;

    always @(posedge clk) begin

        counter_381hz <= counter_381hz + 1;
        if (counter_381hz == 0) 
            clock_enable_381hz = 1;
        else 
            clock_enable_381hz = 0;


    
        if (clock_enable_381hz) begin
            refresh_counter <= refresh_counter + 1;

            case (refresh_counter)
                2'b00: begin
                    an = 4'b1110; 
                    current_digit = {1'b0, rhs_sec};
                end
                2'b01: begin
                    an = 4'b1101;       
                    current_digit = {1'b0, lhs_sec};
                end
                2'b10: begin
                    an = 4'b1011;       
                    current_digit = {1'b0, rhs_min};
                end
                2'b11: begin
                    an = 4'b0111;
                    current_digit = {1'b0, lhs_min};
                end
            endcase

            // When countdown ends, blink zeros
            if (~still_winning && ~blink_state) begin
                seg[6:0] = 7'b1111111; // all off during blink off phase
            end else begin
                case (current_digit[3:0])   //hex to 7-seg encoding
                    4'h0: seg[6:0] = 7'b1000000;
                    4'h1: seg[6:0] = 7'b1111001;
                    4'h2: seg[6:0] = 7'b0100100;
                    4'h3: seg[6:0] = 7'b0110000;
                    4'h4: seg[6:0] = 7'b0011001;
                    4'h5: seg[6:0] = 7'b0010010;
                    4'h6: seg[6:0] = 7'b0000010;
                    4'h7: seg[6:0] = 7'b1111000;
                    4'h8: seg[6:0] = 7'b0000000;
                    4'h9: seg[6:0] = 7'b0010000;
                    default: seg[6:0] = 7'b1111111; //turn off all segments
                endcase
            end
            seg[7] = ~current_digit[4]; //decimal point, active low
        end
    end
endmodule
