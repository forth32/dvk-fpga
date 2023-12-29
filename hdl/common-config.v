//
//  Вспомогательные определения, общие для всех отладочных плат
//  Этот файл подключается в конце файла config.v, уникального для каждой платы
//------------------------------------------------------------------------------------------------------------------------

// тактовая частота процессора в герцах
`define clkref 50000000/`PLL_DIV*`PLL_MUL

// Определение типа модуля соединительной платы-корзины
`ifdef adr22
   `define TOPBOARD topboard22 // 22-битный
`else   
   `define TOPBOARD topboard16 // 16-битный
`endif

// Для платы KDF11B выкидываем монитор-загрузчик, поскольку там есть свой родной 
`ifdef kdf11_board
   `undef bootrom_module
`endif	

// фиктивный CPUSLOW для неподдерживаемых процессорных плат
`ifndef CPUSLOW
	`define CPUSLOW 1          
`endif

// удаление графического модуля при отсутствии текcтового терминала
`ifndef KSM_module
	`undef KGD_module
`endif

// Выбор ведущего и ведомых SDSPI
`ifdef RK_module
  `define RK_sdmode 1'b1  
  `define DM_sdmode 1'b0  
  `define MY_sdmode 1'b0
  `define DX_sdmode 1'b0
  `define DW_sdmode 1'b0
  `define DB_sdmode 1'b0
  `define def_mosi  rk_mosi
  `define def_cs    rk_cs
  `define def_sclk  rk_sclk

  `elsif DM_module
  `define DM_sdmode 1'b1  
  `define RK_sdmode 1'b0  
  `define MY_sdmode 1'b0
  `define DX_sdmode 1'b0
  `define DW_sdmode 1'b0
  `define DB_sdmode 1'b0
  `define def_mosi  dm_mosi
  `define def_cs    dm_cs
  `define def_sclk  dm_sclk
  
`elsif MY_module
  `define MY_sdmode 1'b1
  `define RK_sdmode 1'b0  
  `define DM_sdmode 1'b0  
  `define DX_sdmode 1'b0
  `define DW_sdmode 1'b0
  `define DB_sdmode 1'b0
  `define def_mosi  my_mosi
  `define def_cs    my_cs
  `define def_sclk  my_sclk

`elsif DX_module
  `define DX_sdmode 1'b1
  `define MY_sdmode 1'b0
  `define DM_sdmode 1'b0  
  `define RK_sdmode 1'b0  
  `define DW_sdmode 1'b0
  `define DB_sdmode 1'b0
  `define def_mosi  dx_mosi
  `define def_cs    dx_cs
  `define def_sclk  dx_sclk

`elsif DB_module
  `define DB_sdmode 1'b1  
  `define DX_sdmode 1'b0
  `define MY_sdmode 1'b0
  `define DM_sdmode 1'b0  
  `define RK_sdmode 1'b0  
  `define DW_sdmode 1'b0
  `define def_mosi  db_mosi
  `define def_cs    db_cs
  `define def_sclk  db_sclk
  
`else
  `define DW_sdmode 1'b1
  `define DX_sdmode 1'b0
  `define DM_sdmode 1'b0  
  `define MY_sdmode 1'b0
  `define RK_sdmode 1'b0  
  `define def_mosi  dw_mosi
  `define def_cs    dw_cs
  `define def_sclk  dw_sclk
  
`endif  
  
// Перенос вектора ИРПС2 на нестандартный адрес при наличии DW
`ifdef DW_module
	`define irps2_ri 16'o000330
	`define irps2_ti 16'o000334
`else
	`define irps2_ri 16'o000300
	`define irps2_ti 16'o000304
`endif
  
