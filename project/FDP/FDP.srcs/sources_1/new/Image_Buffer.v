`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Custom inferred BRAM block for image buffering
//////////////////////////////////////////////////////////////////////////////////


module Image_Buffer(
    input clk_write, // 24 MHz
    input [16:0] write_addr,    //17 bit address covers 320*240 = 76800 pixels
    input [14:0] write_data,    //15 bit RGB555 pixel data
    input write_done,
    input clk_read, // 25 MHz
    input [16:0] read_addr,     //17 bit address covers 320*240 = 76800 pixels
    output reg [14:0] read_data,    //15 bit RGB555 pixel data
    output frame_ready
    );

    localparam integer IMAGE_SIZE = 76800;       //320*240
    localparam integer DATA_WIDTH = 15;        //RGB555
    localparam integer BUFFER_SIZE = IMAGE_SIZE*DATA_WIDTH*1.55;   //320*240*15*1.55=1785600

    reg [DATA_WIDTH-1:0] bram [IMAGE_SIZE*1.55-1:0];

    reg [20:0] read_addr_offset = BUFFER_SIZE - IMAGE_SIZE*DATA_WIDTH;  // make read addr offset start one frame before write offset
    wire [20:0] read_addr_actual;
    assign read_addr_actual = (read_addr + read_addr_offset) % BUFFER_SIZE;

    reg [20:0] write_addr_offset = 21'd0; // offset for writing current image of buffer
    wire [20:0] write_addr_actual;
    assign write_addr_actual = (write_addr + write_addr_offset) % BUFFER_SIZE;

    wire write_enable;
    assign write_enable = ~(write_addr ^ read_addr_actual); // write only when addresses are different

    always @(posedge write_done) begin  //sync to VGA Vsync??
        read_addr_offset <= (read_addr_offset + IMAGE_SIZE*DATA_WIDTH) % BUFFER_SIZE;
        write_addr_offset <= (write_addr_offset + IMAGE_SIZE*DATA_WIDTH) % BUFFER_SIZE;
    end

    always @(posedge clk_write) begin
        if (write_enable) begin
            bram[write_addr_actual] <= write_data;
        end
    end

    always @(posedge clk_read) begin
        read_data <= bram[read_addr_actual];
    end

endmodule
