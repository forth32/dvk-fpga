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
//*   Теневое ПЗУ платы МС1201.01
//*********************************************************
module rom000(
 input [11:0] address,
 input clock,
 output [15:0] q
 );

tang_rom000 rom(
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



//*********************************************************
//*   Двухпортовое ОЗУ графической подсистемы КГД
//*********************************************************
module kgdvram (
   input   [13:0]  address_a,
   input   [16:0]  address_b,
   input   clock_a,
   input   clock_b,
   input   [7:0]  data_a,
   input   [0:0]  data_b,
   input   wren_a,
   input   wren_b,
   output  [7:0]  q_a,
   output  [0:0]  q_b
);


tang_kgdvram vram( 
.doa(q_a),
.dob(q_b),
.dia(data_a),
.dib(data_b),
.addra(address_a),
.addrb(address_b),
.wea(wren_a),
.web(wren_b),
.clka(clock_a),
.clkb(clock_b)
);

endmodule
