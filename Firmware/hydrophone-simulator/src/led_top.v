//Copyright (C)2014-2021 Gowin Semiconductor Corporation.
//All rights reserved.
//File Title: Template file for instantiation
//GOWIN Version: V1.9.8
//Part Number: GW1N-LV1QN48C6/I5
//Device: GW1N-1
//Created Time: Sat Jan  1 12:01:54 2022

module led (
    input XTAL_IN,
    input USER_BTN_A,
    input USER_BTN_B,
    output reg [1:0] pulse45,
    output reg [1:0] pulse37_5,
    output pulseC,
    output reg [2:0] led // 110 B, 101 R, 011 G
);

//Change the instance name and port connections to the signal names
//-------- Clock --------

// 24MHz * 3 = 72MHz

wire clkout_o;
assign clkin_i = XTAL_IN;

Gowin_rPLL your_instance_name(
    .clkout(clkout_o), //output clkout
    .clkin(clkin_i) //input clkin
);

//------- Definitions -------------------

parameter integer TICKS_ONE_MICROSECOND = 72;
parameter integer TICKS_ONE_MILLISECOND = TICKS_ONE_MICROSECOND * 1000;
parameter integer TICKS_ONE_SECOND = TICKS_ONE_MILLISECOND * 1000;

parameter integer TICKS_FULL_45KHZ = TICKS_ONE_MICROSECOND * 200 / 9;
parameter integer TICKS_FULL_37_5KHZ = TICKS_ONE_MICROSECOND * 80 / 3;

initial begin
    led <= 3'b111; // active low (all off)
    pulse45 = 0;
    pulse37_5 = 0;
end

//------- Modulator -------------------

wire mod45;
wire mod37_5;

modulator #(
    .TICKS(TICKS_FULL_45KHZ)
) (
    .clock(clkout_o),
    .modulator(mod45)
);

modulator #(
    .TICKS(TICKS_FULL_37_5KHZ)
)(
    .clock(clkout_o),
    .modulator(mod37_5)
);

//-------- Pulser -------------------

// 100us => 15cm
parameter integer CENTIMETERS = 40;
parameter integer OFFSET_A1 = TICKS_ONE_MICROSECOND * (100);
parameter integer OFFSET_A2 = TICKS_ONE_MICROSECOND * (100+ 100);

parameter integer OFFSET_B1 = TICKS_ONE_MICROSECOND * (105);
parameter integer OFFSET_B2 = TICKS_ONE_MICROSECOND * (105+ 100);


reg [31:0] counter = 0;
always @(posedge clkout_o) begin
    if (counter < TICKS_ONE_SECOND) begin
        counter <= counter + 1;
    end else begin
        counter <= 0;
    end

    if (OFFSET_A1 < counter && counter < (OFFSET_A1 + TICKS_ONE_MILLISECOND*10)) begin
        pulse45[0] <= mod45; // high for first 10ms
    end else begin
        pulse45[0] <= 0; // low until 1 second
    end

    if (OFFSET_A2 < counter && counter < (OFFSET_A2 + TICKS_ONE_MILLISECOND*10)) begin
        pulse45[1] <= mod45; // high for first 10ms
    end else begin
        pulse45[1] <= 0; // low until 1 second
    end

    if (OFFSET_B1 < counter && counter < (OFFSET_B1 + TICKS_ONE_MILLISECOND*10)) begin
        pulse37_5[0] <= mod37_5; // high for first 10ms
    end else begin
        pulse37_5[0] <= 0; // low until 1 second
    end

    if (OFFSET_B2 < counter && counter < (OFFSET_B2 + TICKS_ONE_MILLISECOND*10)) begin
        pulse37_5[1] <= mod37_5; // high for first 10ms
    end else begin
        pulse37_5[1] <= 0; // low until 1 second
    end


    led[2] <= !pulse37_5[0];
end

endmodule
