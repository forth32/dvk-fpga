//********************************************************************
//*   Модуль статической памяти с управляющей программой терминала
//********************************************************************
module vtram (
   input          wb_clk_i,
   input  [15:0]  wb_adr_i,
   input  [15:0]  wb_dat_i,
   output [15:0]  wb_dat_o,
   input          wb_cyc_i,
   input          wb_we_i,
   input  [1:0]   wb_sel_i,
   input          wb_stb_i,
   output         wb_ack_o
);
wire [1:0] ena;
reg  [1:0] ack;

vtmem ram(
   .address(wb_adr_i[11:1]),
   .byteena(ena),
   .clock(wb_clk_i),
   .data(wb_dat_i),
   .rden(~wb_we_i & wb_cyc_i & wb_stb_i),
   .wren( wb_we_i & wb_cyc_i & wb_stb_i),
   .q(wb_dat_o));

assign ena = wb_we_i ? wb_sel_i : 2'b11;
assign wb_ack_o = wb_cyc_i & wb_stb_i & (ack[1] | wb_we_i);
always @ (posedge wb_clk_i)
begin
   ack[0] <= wb_cyc_i & wb_stb_i;
   ack[1] <= wb_cyc_i & ack[0];
end
endmodule
