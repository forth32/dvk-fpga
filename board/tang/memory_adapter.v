//*********************************************************
//*   Теневое ПЗУ платы МС1201.02
//*********************************************************
module rom055(
 input [11:0] address,
 input clock,
 output [15:0] q
 );

tang_rom055 rom(
  .addra(address), 
  .doa(q),
  .clka(clock),
  .rsta(1'b0)
);  
	
endmodule


//*********************************************************
//*   ОЗУ контроллера КСМ с управляющей микропрограммой
//*********************************************************
module vtmem (
   input   [10:0]  address,
   input   [1:0]  byteena,
   input   clock,
   input   [15:0]  data,
   input   rden,
   input   wren,
   output  [15:0]  q
);

wire [1:0] writeenable;
assign writeenable[0]=wren & byteena[0];
assign writeenable[1]=wren & byteena[1];

tang_vtmem vram(
	.doa(q),
	.dia(data),
	.addra(address),
	.wea(writeenable),
	.clka(clock)
);	

endmodule

//*********************************************************
//*   ПЗУ знакогенератора со шрифтами
//*********************************************************
module fontrom(
 input [14:0] address,
 input clock,
 output q
 );

tang_fontrom rom(
  .addra(address), 
  .doa(q),
  .clka(clock),
  .rsta(1'b0)
);  
	
endmodule
