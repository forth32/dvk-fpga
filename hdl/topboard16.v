//
// FPGA-версия советских PDP-11-совместимых микро-ЭВМ
//
//================================================================================
// Этот модуль - верхний модуль ядра проекта. Представляет собой корзину с общей шиной Wishbone,
// в которую вставлена процессорная плата,модуль ОЗУ и модули периферийных устройств.
// 
// Процессорная плата выбирается из нескольких доступных, остальные модули общие для всех используемых процессоров
// и включаются в конфигурацию по выбору, сделанному в файле конфигурации config.v
//
//  Между этим модулем и физическими выводами FPGA находится интерфейсный  модуль под конкретную плату.
//  Интерфейсный модуль объявляется корневым модулем проекта.
//===============================================================================================
//!  Этот вариант модуля предназначен для подключения процессорных плат с 16-битным адресом
//

module topboard16 (

   input          clk50,        // Тактовый сигнал 50 MHz
   input          clk_p,        // основной синхросигнал, прямая фаза
   input          clk_n,        // основной синхросигнал, инверсная фаза
   input          sdclock,      // синхросигнал Sd-карты
   input          clkrdy,       // готовность PLL
   
   // кнопки
   input          bt_reset,         // общий сброс
   input          bt_halt,          // пультовое прерывание
   input          bt_terminal_rst,  // сброс терминальной подсистемы
   input          bt_timer,         // выключатель таймера
   
   // переключатели конфигурации
   input [3:0]    sw_diskbank,  // дисковый банк
   input          sw_console,   // выбор консольного порта: 0 - терминальный модуль, 1 - ИРПС 2
   input          sw_cpuslow,   // замедление процессора

   // индикаторные светодиоды      
   output         rk_led,               // запрос обмена диска RK
   output         dm_led,               // запрос обмена диска RK
   output         dw_led,               // запрос обмена диска DW
   output         my_led,               // запрос обмена диска MY
   output         dx_led,               // запрос обмена диска DX
   output         timer_led,            // индикация включения таймера
   
   // Интерфейс SDRAM
   output         sdram_reset,       // сброс/переинициализация SDRAM
   output         sdram_stb,         // строб транзакции
   output         sdram_we,          // разрешение записи
   output [1:0]   sdram_sel,         // выбор байтов
   input          sdram_ack,         // подтверждение транзакции
   output [21:1]  sdram_adr,         // шина адреса
   output [15:0]  sdram_out,         // шина данных хост -> память
   input  [15:0]  sdram_dat,         // шина данных память -> хост  
   input          sdram_ready,       // готовность SDRAM
   
   // интерфейс SD-карты
   output         sdcard_cs, 
   output         sdcard_mosi, 
   output         sdcard_sclk, 
   input          sdcard_miso, 
   
   // VGA
   output         vgah,         // горизонтальная синхронизация
   output         vgav,         // вертикакльная синхронизация
   output         vgared,       // красный видеосигнал
   output         vgagreen,     // зеленый видеосигнал
   output         vgablue,      // синий видеосигнал

   // PS/2
   input          ps2_clk, 
   input          ps2_data,
   
   // пищалка    
   output         buzzer, 
    
   // дополнительный UART 
   output         irps_txd,
   input          irps_rxd,
   
   // LPT
   output [7:0]   lp_data,    // данные для передачи к принтеру
   output         lp_stb_n,   // строб записи в принтер
   output         lp_init_n,  // строб сброса
   input          lp_busy,    // сигнал занятости принтера
   input          lp_err_n    // сигнал ошибки
   
);

wire [2:0] vspeed;   // индекс скорости порта

wire        sys_init;         // общий сброс

// шина WISHBONE                                       
wire        wb_clk;                    
wire [15:0] wb_adr;                    
wire [15:0] wb_out;                    
wire [15:0] wb_mux;                    
wire        wb_we;                     
wire [1:0]  wb_sel;                    
wire        wb_stb;                    
wire        global_ack;                    

// Основная шина процессора
wire        cpu_access_req;          // разрешение доступа к шине
wire [15:0] cpu_adr;                 // шина адреса
wire [15:0] cpu_data_out;            // выход шины данных
wire        cpu_we;                  // направление передачи (1 - от процессора)
wire [1:0]  cpu_bsel;                // выбор байтов из слова
wire        cpu_stb;                 // строб обмена по шине
wire        cpu_ack;                 // подтверждение транзакции

// шина векторов прерываний                                       
wire        vm_una;                    // запрос безадресного чтения
wire        vm_istb;                   // Строб приема вектора прерывания 
wire        vm_iack;                   // подтверждение прерывания
wire [15:0] vm_ivec;                   // вектор прерывания

// сигналы выбора периферии
wire uart1_stb;
wire uart2_stb;
wire sysram_stb;
wire rom_stb;
wire rk11_stb;
wire rk611_stb;
wire lpt_stb;
wire dw_stb;
wire rx_stb;
wire my_stb;
wire kgd_stb;

// линии подтверждения обмена, исходяшие из устройства
wire uart1_ack;
wire uart2_ack;
wire rom_ack;
wire rk11_ack;
wire rk611_ack;
wire lpt_ack;
wire dw_ack;
wire rx_ack;
wire my_ack;
wire kgd_ack;

// линии подтверждения, входящие в DMA-контроллеры устройств
wire rk11_dma_ack;
wire rk611_dma_ack;
wire my_dma_ack;

//  Шины данных от периферии
wire [15:0] uart1_dat;
wire [15:0] uart2_dat;
wire [15:0] rom_dat;
wire [15:0] rk11_dat;
wire [15:0] rk611_dat;
wire [15:0] lpt_dat;
wire [15:0] dw_dat;
wire [15:0] rx_dat;
wire [15:0] my_dat;
wire [15:0] kgd_dat;


// линии процессорных сбросов и прерываний                                       
wire        vm_init_out;               // выход сброса от процессора к устройствам на шине
wire        vm_dclo_in;                // вход сброса
wire        vm_aclo_in;                // прерывание по аварии питания
wire        vm_virq;                   // запрос векторного прерывания

// линии прерывания внешних устройств                                       
wire        irpstx_irq, irpstx_iack;            
wire        irpsrx_irq, irpsrx_iack;            
wire        irpstx2_irq, irpstx2_iack;            
wire        irpsrx2_irq, irpsrx2_iack;            
wire        rk11_irq, rk11_iack;
wire        rk611_irq, rk611_iack;
wire        lpt_irq, lpt_iack;
wire        dw_irq, dw_iack;
wire        rx_irq, rx_iack;
wire        my_irq, my_iack;

wire        global_reset;   // кнопка сброса

// Линии обмена с SD-картой от разных контроллеров
wire         rk_mosi;       // mosi от RK11
wire         rk_cs;         // cs от RK11
wire         rk_sclk;       // sclk от RK11
wire         dm_mosi;       // mosi от RK611
wire         dm_cs;         // cs от RK611
wire         dm_sclk;       // sclk от RK611
wire         dw_mosi;       // mosi от DW
wire         dw_cs;         // cs от DW
wire         dw_sclk;
wire         dx_mosi;       // mosi от DW
wire         dx_cs;         // cs от DW
wire         dx_sclk;
wire         my_mosi;       // mosi от MY
wire         my_cs;         // cs от MY
wire         my_sclk;

// Сигналы диспетчера доступа к SD-карте
wire        rk_sdreq;       // запрос доступа
reg         rk_sdack;       // разрешение доступа
wire        dw_sdreq;
reg         dw_sdack; 
wire        dm_sdreq;
reg         dm_sdack; 
wire        dx_sdreq;
reg         dx_sdack; 
wire        my_sdreq;
reg         my_sdack; 

wire        timer_on;       // разрешение таймера

// линии невекторных прерываний 
assign      sys_init = vm_init_out;   // сброс

// пищалка
wire nbuzzer;
assign buzzer=~nbuzzer;

assign wb_clk=clk_p;

//************************************
//*            VGA
//************************************
// Линии текстового дисплея
wire vgared_t,vgagreen_t,vgablue_t;  // видеосигналы

// Линии графического дисплея
wire vgavideo_g;    // видеовыход 
wire genable;       // включение графического видеовыхода
wire tdisable;      // отключение текстового видеовыхода

// Селектор источника видео
// складываем видеопотоки от обоих видеоконтроллеров
assign vgagreen = ((genable == 1)? vgavideo_g: 1'b0) | ((tdisable == 1'b0)? vgagreen_t: 1'b0);
assign vgared   = ((genable == 1)? vgavideo_g: 1'b0) | ((tdisable == 1'b0)? vgared_t: 1'b0);
assign vgablue  = ((genable == 1)? vgavideo_g: 1'b0) | ((tdisable == 1'b0)? vgablue_t: 1'b0);

 
//********************************************
//* Светодиоды
//********************************************
assign rk_led = ~rk_sdreq;   // запрос обмена диска RK
assign dw_led = ~dw_sdreq;   // запрос обмена диска DW
assign dm_led = ~dm_sdreq;   // запрос обмена диска DM
assign my_led = ~my_sdreq;   // запрос обмена диска MY
assign dx_led = ~dx_sdreq;   // запрос обмена диска DX
assign timer_led = ~timer_on;   // индикация включения таймера


//**************************************************************
//*   Модуль формирования сбросов
//**************************************************************

wbc_rst reset
(
   .osc_clk(clk50),             // основной клок 50 МГц
   .sys_clk(wb_clk),            // сигнал синхронизации  wishbone
   .pll_lock(clkrdy),           // сигнал готовности PLL
   .button(~bt_reset),          // кнопка сброса
   .sys_ready(sdram_ready),     // вход готовности системных компонентов (влияет на sys_rst)
   .sys_dclo(vm_dclo_in),   
   .sys_aclo(vm_aclo_in),
   .global_reset(global_reset)  // выход кнопки сброса 
);

//*********************************************
//*  Интерфейс к модулю SDRAM
//*********************************************

assign sdram_reset=global_reset;       // сигнал сброса модуля SDRAM
assign sdram_we=wb_we;                 // признак транзакции записи
assign sdram_sel=wb_sel;               // выбор байтов
assign sdram_adr={6'o0, wb_adr[15:1]}; // шина адреса
assign sdram_out=wb_out;               // выходная шина данных

//**********************************************************
//*       Процессорная плата
//**********************************************************
`BOARD cpu(
// Синхросигналы  
   .clk_p(clk_p),
   .clk_n(clk_n),
   .cpuslow(sw_cpuslow),              // Режим замедления процессора

// Шина Wishbone                                       
   .cpu_gnt_i(cpu_access_req),     // 1 - разрешение cpu работать с шиной
                                   // 0 - DMA с внешними устройствами, cpu отключен от шины и бесконечно ждет ответа  ack
   .cpu_adr_o(cpu_adr),            // выход шины адреса
   .cpu_dat_o(cpu_data_out),       // выход шины данных
   .cpu_dat_i(wb_mux),             // вход шины данных
   .cpu_we_o(cpu_we),              // разрешение записи
   .cpu_sel_o(cpu_bsel),           // выбор байтов для передачи
   .cpu_stb_o(cpu_stb),            // строб данных

   .sysram_stb(sysram_stb),        // строб обращения к системной памяти
   .global_ack(cpu_ack),           // подтверждение обмена от памяти и устройств страницы ввода-вывода
   
// Сбросы и прерывания
   .vm_init(vm_init_out),          // Выход сброса для периферии
   .dclo(vm_dclo_in),              // Вход сброса процессора
   .aclo(vm_aclo_in),              // Сигнал аварии питания
   .halt(bt_halt),                 // Прерывание входа в пультовоый режим
   .virq(vm_virq),                 // Векторное прерывание

// Шины обработки прерываний                                       
   .ivec(vm_ivec),                 // Шина приема вектора прерывания
   .istb(vm_istb),                 // Строб приема вектора прерывания
   .iack(vm_iack),                 // Подтверждение приема вектора прерывания
   
   .timer_button(bt_timer),    // кнопка включения-отключения таймера
   .timer_status(timer_on)         // линия индикатора состояния таймера
   
);

//**********************************
//* Пзу пользователя 140000-157777
//**********************************
`ifdef userrom
reg rom_ack0;
reg rom_ack1;

user_rom rom(
   .address(wb_adr[12:1]),
   .clock(wb_clk),
   .q(rom_dat)
);
// формирователь cигнала подверждения транзакции с задержкой на 1 такт
always @ (posedge wb_clk)  begin
   rom_ack0 <= rom_stb & ~wb_we;
   rom_ack1 <= rom_ack0 & ~wb_we;
end
assign rom_ack=rom_ack1;
`else
assign rom_ack=1'b0;
`endif

//**********************************
// Выбор консольного порта
//**********************************
wire  uart1_txd, uart1_rxd;   // линии ИРПС 1
wire  uart2_txd, uart2_rxd;   // линии ИРПС 2
wire  terminal_tx,terminal_rx;// линии аппаратного терминала

`ifdef KSM_module
assign irps_txd = (sw_console == 0)? uart2_txd : uart1_txd;
assign terminal_rx = (sw_console == 0)? uart1_txd : uart2_txd;
assign uart1_rxd = (sw_console == 0)? terminal_tx : irps_rxd;
assign uart2_rxd = (sw_console == 0)? irps_rxd : terminal_tx;
`else
assign irps_txd = uart1_txd;
assign uart1_rxd = irps_rxd;
`endif

//**********************************************
// Выбор скорости последовательных портов
//**********************************************
wire [31:0] uart1_speed;  // скорость ИРПС 1
wire [31:0] uart2_speed;  // скорость ИРПС 2
wire [31:0] baud2;        // делитель скорости второго порта ИРПС

// Согласование скорости с терминальным модулем
wire [31:0]   terminal_baud;    // делитель, соответствующий текущей скорости терминала                     
assign  terminal_baud = 
  (vspeed == 3'd0)   ? 32'd767: 32'D0 | // 1200
  (vspeed == 3'd1)   ? 32'd383: 32'D0 | // 2400
  (vspeed == 3'd2)   ? 32'd191: 32'D0 | // 4800
  (vspeed == 3'd3)   ? 32'd95: 32'D0 |  // 9600
  (vspeed == 3'd4)   ? 32'd47: 32'D0 |  // 19200
  (vspeed == 3'd5)   ? 32'd23: 32'D0 |  // 38400
  (vspeed == 3'd6)   ? 32'd15: 32'D0 |  // 57600
  (vspeed == 3'd6)   ? 32'd7:  32'D0 ;  // 115200
                       
// Выбор скорости второго UART                        
// assign  baud2 = 921600/`UART2SPEED-1;
assign baud2 = 
  (`UART2SPEED == 3'd0)   ? 32'd767: // 1200
  (`UART2SPEED == 3'd1)   ? 32'd383: // 2400
  (`UART2SPEED == 3'd2)   ? 32'd191: // 4800
  (`UART2SPEED == 3'd3)   ? 32'd95:  // 9600
  (`UART2SPEED == 3'd4)   ? 32'd47:  // 19200
  (`UART2SPEED == 3'd5)   ? 32'd23:  // 38400
  (`UART2SPEED == 3'd6)   ? 32'd15:  // 57600
                            32'd7;   // 115200

// Селектор делителей скорости обоих портов в зависимости от того, кто из них подключен к терминалу
`ifdef KSM_module
assign uart1_speed = (sw_console == 0)? terminal_baud : baud2;
assign uart2_speed = (sw_console == 0)? baud2 : terminal_baud;
`else
assign uart1_speed = baud2;
assign uart2_speed = baud2;
`endif

//**********************************
//*     ирпс1 (консоль)
//**********************************
wbc_uart #(.REFCLK(`clkref)) uart1
(
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_adr_i(wb_adr[2:0]),
   .wb_dat_i(wb_out),
   .wb_dat_o(uart1_dat),
   .wb_cyc_i(1'b1),
   .wb_we_i(wb_we),
   .wb_stb_i(uart1_stb),
   .wb_ack_o(uart1_ack),

   .txd(uart1_txd),
   .rxd(uart1_rxd),

   .tx_cts_i(1'b0),
   .tx_irq_o(irpstx_irq),
   .tx_iack_i(irpstx_iack),
   .rx_irq_o(irpsrx_irq),
   .rx_iack_i(irpsrx_iack),

   .cfg_bdiv(uart1_speed),
   .cfg_nbit(2'b11),
   .cfg_nstp(1'b1),
   .cfg_pena(1'b0),
   .cfg_podd(1'b0)
);

//**********************************
//*     ирпс2
//**********************************
`ifdef IRPS2_module
wbc_uart #(.REFCLK(`clkref)) uart2
(
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_adr_i(wb_adr[2:0]),
   .wb_dat_i(wb_out),
   .wb_dat_o(uart2_dat),
   .wb_cyc_i(1'b1),
   .wb_we_i(wb_we),
   .wb_stb_i(uart2_stb),
   .wb_ack_o(uart2_ack),

   .tx_cts_i(1'b0),
   .txd(uart2_txd),
   .rxd(uart2_rxd),

   .tx_irq_o(irpstx2_irq),
   .tx_iack_i(irpstx2_iack),
   .rx_irq_o(irpsrx2_irq),
   .rx_iack_i(irpsrx2_iack),

   .cfg_bdiv(uart2_speed),
   .cfg_nbit(2'b11),
   .cfg_nstp(1'b1),
   .cfg_pena(1'b0),
   .cfg_podd(1'b0)
);
`else 
assign uart2_txd=1'b1;
assign irpstx2_irq=1'b0;
assign irpsrx2_irq=1'b0;
`endif

//**********************************
//*   Текстовый терминал КСМ
//**********************************
wire [10:0] col;  // колонка X, 0-1055
wire [9:0]  row;  // строка Y, 0-627

`ifdef KSM_module

ksm terminal(
   // VGA
   .vgahs(vgah), 
   .vgavs(vgav), 
   .vgared(vgared_t),
   .vgagreen(vgagreen_t),
   .vgablue(vgablue_t),
   // Последовательный порт
   .tx(terminal_tx), 
   .rx(terminal_rx), 
   // Клавиатура
   .ps2_clk(ps2_clk), 
   .ps2_data(ps2_data), 
   
   .buzzer(nbuzzer),            // пищалка
   
   .vspeed(vspeed),             // текущая скорость порта
   .initspeed(`TERMINAL_SPEED), // начальная скорость порта
   
   .col(col),
   .row(row),
   
   .clk50(clk50), 
   .reset(bt_terminal_rst | ~clkrdy)         // сброс видеоподсистемы
);
`else
assign nbuzzer=1'b0;
assign vgah=1'b0;
assign vgav=1'b0;
assign vgared_t=1'b0;
assign vgagreen_t=1'b0;
assign vgablue_t=1'b0;
`endif

//**********************************
//*  Графическая подсистема КГД
//**********************************
`ifdef KGD_module
kgd graphics(
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_adr_i(wb_adr[2:0]),
   .wb_dat_i(wb_out),
   .wb_dat_o(kgd_dat),
   .wb_cyc_i(1'b1),
   .wb_we_i(wb_we),
   .wb_stb_i(kgd_stb),
   .wb_sel_i(wb_sel), 
   .wb_ack_o(kgd_ack),
   
   .clk50 (clk50),
   
   .vreset(bt_terminal_rst | ~clkrdy),  // сброс графической подсистемы
   .vgavideo(vgavideo_g),  // видеовыход 
   .col(col),              // счетчик видеостолбцов
   .row(row),              // счетчик видеострок
   .tdisable(tdisable),    // отключение тектового экрана
   .genable(genable)       // подключение графического экрана
);
`else 
assign kgd_ack=1'b0;
assign tdisable=1'b0;
assign genable=1'b0;
assign vgavideo_g=1'b0;
`endif

//**********************************
//*  ИРПР
//**********************************
`ifdef IRPR_module
irpr printer (
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_adr_i(wb_adr[1:0]),
   .wb_dat_i(wb_out),
   .wb_dat_o(lpt_dat),
   .wb_cyc_i(1'b1),
   .wb_we_i(wb_we),
   .wb_stb_i(lpt_stb),
   .wb_ack_o(lpt_ack),
   .irq(lpt_irq),
   .iack(lpt_iack),
   // интерфейс к принтеру
   .lp_data(lp_data),     // данные для передачи к принтеру
   .lp_stb_n(lp_stb_n),   // строб записи в принтер
   .lp_init_n(lp_init_n), // строб сброса
   .lp_busy(lp_busy),     // сигнал занятости принтера
   .lp_err_n(lp_err_n)    // сигнал ошибки
);
`else 
assign lpt_ack=1'b0;
assign lpt_irq=1'b0;
`endif



//****************************************************
//*  Дисковый контроллер RK11D
//****************************************************

// Сигналы запроса-подтверждения DMA
wire rk11_dma_req;
wire rk11_dma_gnt;

// выходная шина DMA
wire [15:0] rk11_adr;                     
wire        rk11_dma_stb;
wire        rk11_dma_we;
wire [15:0] rk11_dma_out;

wire [3:0]  rksddebug;

`ifdef RK_module

rk11 rkdisk (

// шина wishbone
   .wb_clk_i(wb_clk),      // тактовая частота шины
   .wb_rst_i(sys_init),    // сброс
   .wb_adr_i(wb_adr[3:0]), // адрес 
   .wb_dat_i(wb_out),      // входные данные
   .wb_dat_o(rk11_dat),    // выходные данные
   .wb_cyc_i(1'b1),      // начало цикла шины
   .wb_we_i(wb_we),        // разрешение записи (0 - чтение)
   .wb_stb_i(rk11_stb),    // строб цикла шины
   .wb_sel_i(wb_sel),      // выбор конкретных байтов для записи - старший, младший или оба
   .wb_ack_o(rk11_ack),    // подтверждение выбора устройства

// обработка прерывания   
   .irq(rk11_irq),         // запрос
   .iack(rk11_iack),       // подтверждение
   
// DMA
   .dma_req(rk11_dma_req), // запрос DMA
   .dma_gnt(rk11_dma_gnt), // подтверждение DMA
   .dma_adr_o(rk11_adr),   // выходной адрес при DMA-обмене
   .dma_dat_i(wb_mux),     // входная шина данных DMA
   .dma_dat_o(rk11_dma_out), // выходная шина данных DMA
   .dma_stb_o(rk11_dma_stb), // строб цикла шины DMA
   .dma_we_o(rk11_dma_we),   // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   .dma_ack_i(rk11_dma_ack), // Ответ от устройства, с которым идет DMA-обмен
   
// интерфейс SD-карты
   .sdcard_cs(rk_cs), 
   .sdcard_mosi(rk_mosi), 
   .sdcard_miso(sdcard_miso), 
   .sdcard_sclk(rk_sclk),

   .sdclock(sdclock),
   .sdreq(rk_sdreq),
   .sdack(rk_sdack),
   .sdmode(`RK_sdmode),           // режим ведущего-ведомого
   
// Адрес массива дисков на карте
   .start_offset({1'b0,sw_diskbank,22'h0}),

// отладочные сигналы
   .sdcard_debug(rksddebug)
   ); 

`else 
assign rk11_ack=1'b0;
assign rk11_dma_req=1'b0;
assign rk_sdreq = 1'b0;
assign rk11_irq=1'b0;
`endif

  
//****************************************************
//*  Дисковый контроллер RK611
//****************************************************

// Сигналы запроса-подтверждения DMA
wire rk611_dma_req;
wire rk611_dma_gnt;

// выходная шина DMA
wire [17:0] rk611_adr;                     
wire        rk611_dma_stb;
wire        rk611_dma_we;
wire [15:0] rk611_dma_out;

`ifdef DM_module

rk611 dmdisk (

// шина wishbone
   .wb_clk_i(wb_clk),      // тактовая частота шины
   .wb_rst_i(sys_init),    // сброс
   .wb_adr_i(wb_adr[4:0]), // адрес 
   .wb_dat_i(wb_out),      // входные данные
   .wb_dat_o(rk611_dat),    // выходные данные
   .wb_cyc_i(1'b1),      // начало цикла шины
   .wb_we_i(wb_we),        // разрешение записи (0 - чтение)
   .wb_stb_i(rk611_stb),    // строб цикла шины
   .wb_sel_i(wb_sel),      // выбор конкретных байтов для записи - старший, младший или оба
   .wb_ack_o(rk611_ack),    // подтверждение выбора устройства

// обработка прерывания   
   .irq(rk611_irq),         // запрос
   .iack(rk611_iack),       // подтверждение
   
// DMA
   .dma_req(rk611_dma_req), // запрос DMA
   .dma_gnt(rk611_dma_gnt), // подтверждение DMA
   .dma_adr_o(rk611_adr),   // выходной адрес при DMA-обмене
   .dma_dat_i(wb_mux),     // входная шина данных DMA
   .dma_dat_o(rk611_dma_out), // выходная шина данных DMA
   .dma_stb_o(rk611_dma_stb), // строб цикла шины DMA
   .dma_we_o(rk611_dma_we),   // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   .dma_ack_i(rk611_dma_ack), // Ответ от устройства, с которым идет DMA-обмен
   
// интерфейс SD-карты
   .sdcard_cs(dm_cs), 
   .sdcard_mosi(dm_mosi), 
   .sdcard_miso(sdcard_miso), 
   .sdcard_sclk(dm_sclk),

   .sdclock(sdclock),
   .sdreq(dm_sdreq),
   .sdack(dm_sdack),
   .sdmode(`DM_sdmode),           // режим ведущего-ведомого
   
// Адрес массива дисков на карте
//   .start_offset({1'b0,sw_diskbank,18'h0}),
   .start_offset({1'b0, sw_diskbank,22'h330000})

   ); 

`else 
assign rk611_ack=1'b0;
assign rk611_dma_req=1'b0;
assign dm_sdreq = 1'b0;
assign rk611_irq=1'b0;
`endif

  
//**********************************
//*   Дисковый контроллер DW
//**********************************
wire [3:0] dwsddebug;

`ifdef DW_module

dw hdd(
// шина wishbone
   .wb_clk_i(wb_clk),   // тактовая частота шины
   .wb_rst_i(sys_init),   // сброс
   .wb_adr_i(wb_adr[4:0]),   // адрес 
   .wb_dat_i(wb_out),   // входные данные
   .wb_dat_o(dw_dat),   // выходные данные
   .wb_cyc_i(1'b1),   // начало цикла шины
   .wb_we_i(wb_we),     // разрешение записи (0 - чтение)
   .wb_stb_i(dw_stb),   // строб цикла шины
   .wb_sel_i(wb_sel),   // выбор конкретных байтов для записи - старший, младший или оба
   .wb_ack_o(dw_ack),   // подтверждение выбора устройства

// обработка прерывания   
   .irq(dw_irq),        // запрос
   .iack(dw_iack),      // подтверждение
   
   
// интерфейс SD-карты
   .sdcard_cs(dw_cs), 
   .sdcard_mosi(dw_mosi), 
   .sdcard_miso(sdcard_miso), 
   .sdcard_sclk(dw_sclk),
   
   .sdclock(sdclock),
   .sdreq(dw_sdreq),
   .sdack(dw_sdack),
   .sdmode(`DW_sdmode),          

// Адрес массива дисков на карте
   .start_offset({1'b0,sw_diskbank,22'hc000}),
   
// отладочные сигналы
   .sdcard_debug(dwsddebug)
   ); 

`else 
assign dw_ack=1'b0;
assign dw_sdreq = 1'b0;
assign dw_irq=1'b0;
`endif


//**********************************
//*   Дисковый контроллер RX01
//**********************************
wire [3:0] rxsddebug;

`ifdef DX_module

rx01 dxdisk (
// шина wishbone
   .wb_clk_i(wb_clk),      // тактовая частота шины
   .wb_rst_i(sys_init),    // сброс
   .wb_adr_i(wb_adr[1:0]), // адрес 
   .wb_dat_i(wb_out),      // входные данные
   .wb_dat_o(rx_dat),      // выходные данные
   .wb_cyc_i(1'b1),      // начало цикла шины
   .wb_we_i(wb_we),        // разрешение записи (0 - чтение)
   .wb_stb_i(rx_stb),      // строб цикла шины
   .wb_sel_i(wb_sel),      // выбор конкретных байтов для записи - старший, младший или оба
   .wb_ack_o(rx_ack),      // подтверждение выбора устройства

// обработка прерывания   
   .irq(rx_irq),           // запрос
   .iack(rx_iack),         // подтверждение
   
   
// интерфейс SD-карты
   .sdcard_cs(dx_cs), 
   .sdcard_mosi(dx_mosi), 
   .sdcard_miso(sdcard_miso), 
   .sdcard_sclk(dx_sclk),

   .sdmode(`DX_sdmode),          
   .sdreq(dx_sdreq),
   .sdack(dx_sdack),
   .sdclock(sdclock),
   
// Адрес массива дисков на карте
   .start_offset({1'b0,sw_diskbank,22'h2c000}),
   
// отладочные сигналы
   .sdcard_debug(rxsddebug)
   ); 

`else 
assign rx_ack=1'b0;
assign dx_sdreq = 1'b0;
assign rx_irq=1'b0;
`endif
   
//****************************************************
//*  Дисковый контроллер MY
//****************************************************

// Сигналы запроса-подтверждения DMA
wire my_dma_req;
wire my_dma_gnt;

// выходная шина DMA
wire [15:0]  my_adr;                     
wire         my_dma_stb;
wire         my_dma_we;
wire [15:0]  my_dma_out;

wire [3:0]   mysddebug;


`ifdef MY_module

fdd_my mydisk (

// шина wishbone
   .wb_clk_i(wb_clk),       // тактовая частота шины
   .wb_rst_i(sys_init),     // сброс
   .wb_adr_i(wb_adr[3:0]),  // адрес 
   .wb_dat_i(wb_out),       // входные данные
   .wb_dat_o(my_dat),       // выходные данные
   .wb_cyc_i(1'b1),       // начало цикла шины
   .wb_we_i(wb_we),         // разрешение записи (0 - чтение)
   .wb_stb_i(my_stb),       // строб цикла шины
   .wb_sel_i(wb_sel),       // выбор конкретных байтов для записи - старший, младший или оба
   .wb_ack_o(my_ack),       // подтверждение выбора устройства

// обработка прерывания   
   .irq(my_irq),            // запрос
   .iack(my_iack),          // подтверждение
   
// DMA
   .dma_req(my_dma_req),    // запрос DMA
   .dma_gnt(my_dma_gnt),    // подтверждение DMA
   .dma_adr_o(my_adr),      // выходной адрес при DMA-обмене
   .dma_dat_i(wb_mux),      // входная шина данных DMA
   .dma_dat_o(my_dma_out),  // выходная шина данных DMA
   .dma_stb_o(my_dma_stb),  // строб цикла шины DMA
   .dma_we_o(my_dma_we),    // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   .dma_ack_i(my_dma_ack),  // Ответ от устройства, с которым идет DMA-обмен
   
// интерфейс SD-карты
   .sdcard_cs(my_cs), 
   .sdcard_mosi(my_mosi), 
   .sdcard_miso(sdcard_miso), 
   .sdcard_sclk(my_sclk),

   .sdclock(sdclock),
   .sdreq(my_sdreq),
   .sdack(my_sdack),
   .sdmode(`MY_sdmode),          
   
// Адрес массива дисков на карте
   .start_offset({1'b0,sw_diskbank,22'h2e000}),

// отладочные сигналы
   .sdcard_debug(mysddebug)
   ); 

`else 
assign my_ack=1'b0;
assign my_dma_req=1'b0;
assign my_sdreq = 1'b0;
assign my_irq=1'b0;
`endif

//**********************************
//*  Диспетчер доступа к SD-карте
//**********************************
//always @(posedge wb_clk) 
reg [1:0] my_sdreq_filter;
reg [1:0] rk_sdreq_filter;
reg [1:0] dw_sdreq_filter;
reg [1:0] dm_sdreq_filter;
reg [1:0] dx_sdreq_filter;

// фильтрация сигналов запроса
always @(posedge sdclock) begin
  my_sdreq_filter[0]=my_sdreq;
  my_sdreq_filter[1]=my_sdreq_filter[0];
  
  dx_sdreq_filter[0]=dx_sdreq;
  dx_sdreq_filter[1]=dx_sdreq_filter[0];
  
  dw_sdreq_filter[0]=dw_sdreq;
  dw_sdreq_filter[1]=dw_sdreq_filter[0];
  
  dm_sdreq_filter[0]=dm_sdreq;
  dm_sdreq_filter[1]=dm_sdreq_filter[0];
  
  rk_sdreq_filter[0]=rk_sdreq;
  rk_sdreq_filter[1]=rk_sdreq_filter[0];
end  
  
always @(posedge sdclock) begin
   // сброс
   if (sys_init == 1'b1) begin
      rk_sdack <= 1'b0;
      dw_sdack <= 1'b0;
      dm_sdack <= 1'b0;
      dx_sdack <= 1'b0;
      my_sdack <= 1'b0;
   end   
   else
   // поиск контроллера, желающего доступ к карте
    if ((rk_sdack == 1'b0) && (dm_sdack == 1'b0) && (dw_sdack == 1'b0) && (dx_sdack == 1'b0) && (my_sdack == 1'b0)) begin 
       // неактивное состояние - ищем источник запроса 
       if (rk_sdreq == 1'b1) rk_sdack <=1'b1;
       else if (dw_sdreq_filter[1] == 1'b1) dw_sdack <=1'b1;
       else if (dm_sdreq_filter[1] == 1'b1) dm_sdack <=1'b1;
       else if (dx_sdreq_filter[1] == 1'b1) dx_sdack <=1'b1;
       else if (my_sdreq_filter[1] == 1'b1) my_sdack <=1'b1;
    end    
    else 
    // активное состояние - ждем освобождения карты
       if ((rk_sdack == 1'b1) && rk_sdreq_filter[1] == 1'b0) rk_sdack <= 1'b0;
       else if ((dw_sdack == 1'b1) && (dw_sdreq_filter[1] == 1'b0)) dw_sdack <= 1'b0;
       else if ((dm_sdack == 1'b1) && (dm_sdreq_filter[1] == 1'b0)) dm_sdack <= 1'b0;
       else if ((dx_sdack == 1'b1) && (dx_sdreq_filter[1] == 1'b0)) dx_sdack <= 1'b0;
       else if ((my_sdack == 1'b1) && (my_sdreq_filter[1] == 1'b0)) my_sdack <= 1'b0;
end
   
//**********************************
//* Мультиплексор линий SD-карты
//**********************************
assign sdcard_mosi =
         dw_sdack? dw_mosi: // DW
         dm_sdack? dm_mosi: // DM
         dx_sdack? dx_mosi: // DX
         my_sdack? my_mosi: // MY
         rk_sdack? rk_mosi: // RK
                   `def_mosi; // по умолчанию - контроллер с ведущим SDSPI

assign sdcard_cs =
         dw_sdack? dw_cs:   // DW
         dm_sdack? dm_cs:   // DM
         dx_sdack? dx_cs:   // DX
         my_sdack? my_cs:   // MY
         rk_sdack? rk_cs:   // RK
                   `def_cs;   // по умолчанию - контроллер с ведущим SDSPI
                   
assign sdcard_sclk =                    
         dw_sdack? dw_sclk:   // DW
         dm_sdack? dm_sclk:   // DM
         dx_sdack? dx_sclk:   // DX
         my_sdack? my_sclk:   // MY
         rk_sdack? rk_sclk:   // RK
                   `def_sclk;   // по умолчанию - контроллер с ведущим SDSPI
            
//**********************************
//*  Контроллер прерываний
//**********************************
wbc_vic #(.N(10)) vic
(
   .wb_clk_i(wb_clk),
   .wb_rst_i(vm_dclo_in),
   .wb_irq_o(vm_virq),
   .wb_dat_o(vm_ivec),
   .wb_stb_i(vm_istb),
   .wb_ack_o(vm_iack),
//         UART1-Tx     UART1-Rx   UART2-Tx    UART2-Rx     RK-11D        IRPR           DW         RX-11         MY         DM
   .ivec({16'o000064, 16'o000060, 16'o000334,  16'o000330, 16'o000220,  16'o000330, 16'o000300, 16'o000264, 16'o000170, 16'o000210 }),   // векторы
   .ireq({irpstx_irq, irpsrx_irq, irpstx2_irq, irpsrx2_irq, rk11_irq,     lpt_irq,    dw_irq,     rx_irq,      my_irq,  rk611_irq  }),   // запрос прерывания
   .iack({irpstx_iack,irpsrx_iack,irpstx2_iack,irpsrx2_iack,rk11_iack,    lpt_iack,   dw_iack,    rx_iack,     my_iack, rk611_iack })    // подтверждение прерывания
);

//*****************************************************************************
//* Диспетчер доступа к общей шине по запросу от разных мастеров (арбитр DMA)
//*****************************************************************************
reg rk11_dma_state;
reg my_dma_state;
reg rk611_dma_state;
// линии подтверждения разрешения доступа к шине
assign rk11_dma_gnt = rk11_dma_state;
assign rk611_dma_gnt = rk611_dma_state;
assign my_dma_gnt = my_dma_state;
assign cpu_access_req = ~ (rk11_dma_state | rk611_dma_state | my_dma_state);

always @(posedge wb_clk) 
   if (sys_init == 1'b1) begin
      rk11_dma_state <= 1'b0;
      rk611_dma_state <= 1'b0;
      my_dma_state <= 1'b0;
   end   
  // переключение источника - только в отсутствии активного цикла шины
   else if (wb_stb == 1'b0) begin
     if (rk11_dma_req == 1'b1)  rk11_dma_state <= 1'b1;  // запрос от RK11
     else if (my_dma_req == 1'b1)  my_dma_state <= 1'b1; // запрос от MY
     else if (rk611_dma_req == 1'b1)  rk611_dma_state <= 1'b1; // запрос от DM
     else begin
        // нет активных DMA-запросов - шина подключается к процессору
        rk11_dma_state <= 1'b0;       
        rk611_dma_state <= 1'b0;       
        my_dma_state <= 1'b0;       
     end
  end

 
//*******************************************************************
//*  Коммутатор источника управления (мастера) шины wishbone
//*******************************************************************
assign wb_adr =   (rk11_dma_state) ? rk11_adr : 16'o0
                | (rk611_dma_state) ? rk611_adr[15:0] : 16'o0
                | (my_dma_state)   ? my_adr   : 16'o0
                | (cpu_access_req) ? cpu_adr  : 16'o0;
                                           
assign wb_out =   (rk11_dma_state) ? rk11_dma_out: 16'o0
                | (rk611_dma_state) ? rk611_dma_out: 16'o0 
                | (my_dma_state)   ? my_dma_out  : 16'o0
                | (cpu_access_req) ? cpu_data_out: 16'o0;
                                           
assign wb_we =  (rk11_dma_state == 1'b1) ? rk11_dma_we:
                (rk611_dma_state == 1'b1) ? rk611_dma_we:
                (my_dma_state == 1'b1)   ? my_dma_we:
                                           cpu_we;
                                           
assign wb_sel =   (rk11_dma_state|rk611_dma_state|my_dma_state) ? 2'b11: cpu_bsel;
                                          
assign wb_stb = (rk11_dma_state == 1'b1) ? rk11_dma_stb:
                (rk611_dma_state == 1'b1) ? rk611_dma_stb:
                (my_dma_state == 1'b1)   ? my_dma_stb:
                                           cpu_stb;
                                           
assign cpu_ack = ((
                  rk11_dma_state | 
                  rk611_dma_state | 
                  my_dma_state) == 1'b0) ? global_ack: 1'b0;
                  
assign rk11_dma_ack = (rk11_dma_state == 1'b1) ? global_ack: 1'b0;
assign rk611_dma_ack = (rk611_dma_state == 1'b1) ? global_ack: 1'b0;
assign my_dma_ack = (my_dma_state == 1'b1) ? global_ack: 1'b0;
  
//*******************************************************************
//*  Сигналы управления шины wishbone
//******************************************************************* 

// Страница ввода-выводв
assign uart1_stb  = wb_stb & (wb_adr[15:3] == (16'o177560 >> 3));   // ИРПС консольный (TT) - 177560-177566 
assign uart2_stb  = wb_stb & (wb_adr[15:3] == (16'o176500 >> 3));   // ИРПС дополнительный - 176500-177506
assign lpt_stb    = wb_stb & (wb_adr[15:2] == (16'o177514 >> 2));   // ИРПР (LP) - 177514-177516
assign rk11_stb   = wb_stb & (wb_adr[15:4] == (16'o177400 >> 4));   // RK - 177400-177416
assign rk611_stb  = wb_stb & (wb_adr[15:5] == (16'o177440 >> 5));   // DM - 177440-177476
assign dw_stb     = wb_stb & (wb_adr[15:5] == (16'o174000 >> 5));   // DW - 174000-174026
assign rx_stb     = wb_stb & (wb_adr[15:2] == (16'o177170 >> 2));   // DX - 177170-177172
assign my_stb     = wb_stb & (wb_adr[15:2] == (16'o172140 >> 2));   // MY - 172140-172142 
assign kgd_stb    = wb_stb & (wb_adr[15:3] == (16'o176640 >> 3));   // КГД - 176640-176646

// ПЗУ пользователя
`ifdef userrom
assign rom_stb = wb_stb & (wb_adr[15:13] == 3'b110);
`else
assign rom_stb=1'b0;
`endif

// Размещение основной памяти : 
// + если требуется, добавляется служебная область памяти по сигналу sysram_stb процессорной платы
`ifdef userrom
// вариант при наличии ПЗУ пользователя - RAM находится в пространстве 000000 - 137777 
assign sdram_stb =  (wb_stb & (wb_adr[15:14] != 2'b11)) | sysram_stb;
`else
// вариант без ПЗУ - RAM находится в пространстве 000000 - 157777 
assign sdram_stb =  (wb_stb & (wb_adr[15:13] != 3'b111)) | sysram_stb;
`endif

// Сигналы подтверждения - собираются через OR со всех устройств
assign global_ack  = sdram_ack | rom_ack | uart1_ack | uart2_ack | rk11_ack | rk611_ack | lpt_ack | dw_ack | rx_ack | my_ack | kgd_ack;

// Мультиплексор выходных шин данных всех устройств
assign wb_mux = 
       (sdram_stb ? sdram_dat : 16'o000000)
     | (rom_stb   ? rom_dat   : 16'o000000)
     | (uart1_stb ? uart1_dat : 16'o000000)
     | (uart2_stb ? uart2_dat : 16'o000000)
     | (rk11_stb  ? rk11_dat  : 16'o000000)
     | (rk611_stb ? rk611_dat : 16'o000000)
     | (lpt_stb   ? lpt_dat   : 16'o000000)
     | (dw_stb    ? dw_dat    : 16'o000000)
     | (rx_stb    ? rx_dat    : 16'o000000)
     | (my_stb    ? my_dat    : 16'o000000)
     | (kgd_stb   ? kgd_dat   : 16'o000000)
;

  
endmodule


