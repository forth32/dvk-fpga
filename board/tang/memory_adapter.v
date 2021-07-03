//
// Этот файл - набор интерфейсных модулей для адаптации TANG IP-компонентов к стандарту Altera.
// Модули являются переходниками к мегафункциям tang-среды.
//

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

//*********************************************************
//*   Регистровый файл процессора 1801ВМ1
//*********************************************************
module vm1_vcram (
   input [5:0]    address_a,
   input [5:0]    address_b,
   input [1:0]    byteena_a,
   input          clock,
   input [15:0]   data_a,
   input [15:0]   data_b,
   input          wren_a,
   input          wren_b,
   output [15:0]  q_a,
   output [15:0]  q_b
);
wire [1:0] wea;
wire [1:0] web;

assign wea[0] = wren_a & byteena_a[0];
assign wea[1] = wren_a & byteena_a[1];

assign web[0] = wren_b;
assign web[1] = wren_b;


tang_vm1_vcram vcram(
   .clka(clock),
   .clkb(clock),
   .wea(wea),
   .addra(address_a),
   .dia(data_a),
   .web(web),
   .addrb(address_b),
   .dib(data_b),
   .doa(q_a),
   .dob(q_b)
);

endmodule

//*********************************************************
//*   ПЗУ пользователя 140000-157777 для всех плат
//*********************************************************
module user_rom(
 input [11:0] address,
 input clock,
 output [15:0] q
 );

tang_user_rom rom(
  .addra(address), 
  .doa(q),
  .clka(clock),
  .rsta(1'b0)
);  
   
endmodule

//*********************************************************
//*   ПЗУ эмулятора пульта-загрузчика М9312
//*********************************************************
module boot_rom(
 input [8:0] address,
 input clock,
 output [15:0] q
 );

tang_bootrom brom(
  .addra(address), 
  .doa(q),
  .clka(clock),
  .rsta(1'b0)
);  
   
endmodule


//*********************************************************
//* Буфер сектора для модуля sdspi
//*********************************************************
module sectorbuf (
   input   [7:0]  address_a,
   input   [7:0]  address_b,
   input     clock_a,
   input     clock_b,
   input   [15:0]  data_a,
   input   [15:0]  data_b,
   input     wren_a,
   input     wren_b,
   output   [15:0]  q_a,
   output   [15:0]  q_b
);

tang_sectorbuf sbuf( 
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
