`timescale 1ns / 1ps

module tang_bootrom ( doa, addra, clka, rsta );

   output [15:0] doa;

   input  [8:0] addra;
   input  clka;
   input  rsta;

   EG_LOGIC_BRAM #( .DATA_WIDTH_A(16),
            .ADDR_WIDTH_A(9),
            .DATA_DEPTH_A(512),
            .DATA_WIDTH_B(16),
            .ADDR_WIDTH_B(9),
            .DATA_DEPTH_B(512),
            .MODE("SP"),
            .REGMODE_A("NOREG"),
            .RESETMODE("SYNC"),
            .IMPLEMENT("9K"),
            .DEBUGGABLE("NO"),
            .PACKABLE("NO"),
            .INIT_FILE("../../../rom/m9312/bootrom.mif"),
            .FILL_ALL("NONE"))
         inst(
            .dia({16{1'b0}}),
            .dib({16{1'b0}}),
            .addra(addra),
            .addrb({9{1'b0}}),
            .cea(1'b1),
            .ceb(1'b0),
            .ocea(1'b0),
            .oceb(1'b0),
            .clka(clka),
            .clkb(1'b0),
            .wea(1'b0),
            .web(1'b0),
            .bea(1'b0),
            .beb(1'b0),
            .rsta(rsta),
            .rstb(1'b0),
            .doa(doa),
            .dob());


endmodule
