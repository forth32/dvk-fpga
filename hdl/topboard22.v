//
// FPGA-версия советских PDP-11-совместимых ЭВМ
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
//
//!  Этот вариант модуля предназначен для подключения процессорных плат с 22-битным адресом
//
//
module topboard22 (

   input          clk50,        // Тактовый сигнал 50 MHz
   input          clk_p,        // основной синхросигнал, прямая фаза
   input          clk_n,        // основной синхросигнал, инверсная фаза
   input          sdclock,      // синхросигнал Sd-карты
   input          clkrdy,       // готовность PLL
   
   // кнопки
   input          bt_reset,         // общий сброс
   input          bt_halt,          // выход из HALT-режима
   input          bt_terminal_rst,  // сброс терминальной подсистемы
   input          bt_timer,         // в этом варианте не используется
   
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
   output         db_led,               // запрос обмена диска DB
   output         timer_led,            // индикация включения таймера
   output         idle_led,             // признак ожидания прерывания по WAIT
   output         mmu_led,              // признак включения MMU 
   output         run_led,              // признак активности секвенсера команд
   
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
wire        wb_clk;                  // тактовая частота
wire [21:0] wb_adr;                  // полный 22-битный адрес  
wire [15:0] wb_out;                  // выход данных от процессора   
wire [15:0] wb_mux;                  // вход данных в процессор  
wire        wb_we;                   // разрешение записи  
wire [1:0]  wb_sel;                  // выбор байтов из слова  
wire        wb_stb;                  // строб обмена по шине  
wire        global_ack;              // подтверждение обмена от устройств на шине      
wire [17:0] dma_adr18;               // 18-битный unibus-адрес от устройств, использующих DMA в режиме UBM
wire        dma_stb_ubm;             // строб обмена от устройств, работающих с DMA через механизм UBM

// Основная шина процессора
wire        cpu_access_req;          // разрешение доступа к шине
wire [21:0] cpu_adr;                 // шина адреса
wire [15:0] cpu_data_out;            // выход шины данных
wire        cpu_we;                  // направление передачи (1 - от процессора)
wire [1:0]  cpu_bsel;                // выбор байтов из слова
wire        cpu_ram_stb;             // строб доступа к памяти со  стороны процессора

// сигналы выбора периферии
wire uart1_stb;
wire uart2_stb;
wire rk11_stb;
wire rk611_stb;
wire rh70_stb;
wire lpt_stb;
wire dw_stb;
wire rx_stb;
wire my_stb;
wire kgd_stb;

wire bus_stb;

// линии подтверждения обмена, исходяшие из устройства
wire uart1_ack;
wire uart2_ack;
wire rk11_ack;
wire rk611_ack;
wire rh70_ack;
wire lpt_ack;
wire dw_ack;
wire rx_ack;
wire my_ack;
wire kgd_ack;

//  Шины данных от периферии
wire [15:0] uart1_dat;
wire [15:0] uart2_dat;
wire [15:0] rk11_dat;
wire [15:0] rk611_dat;
wire [15:0] rh70_dat;
wire [15:0] lpt_dat;
wire [15:0] dw_dat;
wire [15:0] rx_dat;
wire [15:0] my_dat;
wire [15:0] kgd_dat;


// линии процессорных сбросов 
wire        vm_dclo_in;                // вход сброса
wire        vm_aclo_in;                // прерывание по аварии питания

// линии прерывания внешних устройств                                       
wire        uart1_tx_irq, uart1_tx_iack;            
wire        uart1_rx_irq, uart1_rx_iack;            
wire        uart2_tx_irq, uart2_tx_iack;            
wire        uart2_rx_irq, uart2_rx_iack;            
wire        rk11_irq, rk11_iack;
wire        rk611_irq, rk611_iack;
wire        lpt_irq, lpt_iack;
wire        dw_irq, dw_iack;
wire        rx_irq, rx_iack;
wire        my_irq, my_iack;
wire        rh70_irq, rh70_iack;

wire        global_reset;   // повторитель кнопки сброса

wire [8:0] vector;      // передаваемый процессору вектор прерывания
wire [15:0] irq4_ivec;  // выход вектора приоритета 4
wire [15:0] irq5_ivec;  // выход вектора приоритета 5
wire [5:4]  istb;       // стробы запроса вектора от CPU для каждого приоритета

// линии запроса прерывания к процессору
wire br4_irq;
wire br5_irq;
// линии подтверждения приема вектора от процессора
wire br4_iack;
wire br5_iack;


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
wire         db_mosi;       // mosi от DB
wire         db_cs;         // cs от DB
wire         db_sclk;

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
wire        db_sdreq;
reg         db_sdack; 

// пищалка
wire nbuzzer;
assign buzzer=~nbuzzer;

// основная тактовая частота шины
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
//* Светодиоды дисковой активности
//********************************************
assign rk_led = ~rk_sdreq;   // запрос обмена диска RK
assign dw_led = ~dw_sdreq;   // запрос обмена диска DW
assign dm_led = ~dm_sdreq;   // запрос обмена диска DM
assign my_led = ~my_sdreq;   // запрос обмена диска MY
assign dx_led = ~dx_sdreq;   // запрос обмена диска DX
assign db_led = ~db_sdreq;   // запрос обмена диска DB


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
   .global_reset(global_reset)  // выход повторителя кнопки сброса, с отфильтрованным дребезгом
);

//*********************************************
//*  Интерфейс к модулю SDRAM
//*********************************************

assign sdram_reset=global_reset;       // сигнал сброса модуля SDRAM
assign sdram_we=wb_we;                 // признак транзакции записи
assign sdram_sel=wb_sel;               // выбор байтов
assign sdram_adr=wb_adr[21:1];         // шина адреса
assign sdram_out=wb_out;               // выходная шина данных

//**********************************************************
//*       Процессорная плата
//**********************************************************
`BOARD cpu(
// Синхросигналы  
   .clk_p(clk_p),
   .clk_n(clk_n),
   .cpuslow(sw_cpuslow),          // Режим замедления процессора

   .wb_adr_o(cpu_adr),            // выход шины адреса
   .wb_dat_o(cpu_data_out),       // выход шины данных
   .wb_dat_i(wb_mux),             // вход шины данных
   .wb_we_o(cpu_we),              // разрешение записи
   .wb_sel_o(cpu_bsel),           // выбор байтов для передачи
   .global_ack(global_ack),       // подтверждение обмена от памяти и устройств страницы ввода-вывода
   .ram_stb(cpu_ram_stb),         // строб обращения к основной памяти
   .bus_stb(bus_stb),             // строб обращения к общей шине
   
// DMA   
   .dma_req(dma_req),             // запрс DMA 
   .dma_ack(dma_ack),             // подтверждение DMA - процессор освободил шину
   .dma_adr18(dma_adr18),         // ввод 18-битного адреса для устройств, работающих через Unibsus Mapping 
   .dma_stb(dma_stb_ubm),         // строб данных для устройств, работающих через Unibsus Mapping 

// Сбросы и прерывания
   .bus_reset(sys_init),           // Выход сброса для периферии
   .dclo(vm_dclo_in),              // Вход сброса процессора
   .aclo(vm_aclo_in),              // Сигнал аварии питания
   
// Ручное управление   
   .resume(bt_halt),               // Запуск после HALT
   .csw(16'o0),                    // регистр консольных переключателей
   
// Индикаторы   
   .led_idle(idle_led),            // индикация бездействия (WAIT)
   .led_run(run_led),              // индикация работы процессора (~HALT)
   .led_mmu(mmu_led),              // индикация включения MMU
   .led_timer(timer_led),          // индикация включения таймера
   
// Шины обработки прерываний                                       
   .irq_i({br5_irq, br4_irq}),     // Запрос на векторное прерывание 
   .istb_o(istb),                  // Строб от процессора, разрешающий выдачу вектора 
   .ivec(vector),                  // Шина приема вектора прерывания
   .iack_i(br4_iack|br5_iack)      // Подтверждение приема вектора прерывания
   
);

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
   .tx_irq_o(uart1_tx_irq),
   .tx_iack_i(uart1_tx_iack),
   .rx_irq_o(uart1_rx_irq),
   .rx_iack_i(uart1_rx_iack),

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

   .tx_irq_o(uart2_tx_irq),
   .tx_iack_i(uart2_tx_iack),
   .rx_irq_o(uart2_rx_irq),
   .rx_iack_i(uart2_rx_iack),

   .cfg_bdiv(uart2_speed),
   .cfg_nbit(2'b11),
   .cfg_nstp(1'b1),
   .cfg_pena(1'b0),
   .cfg_podd(1'b0)
);
`else 
assign uart2_txd=1'b1;
assign uart2_tx_irq=1'b0;
assign uart2_rx_irq=1'b0;
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

// выходная шина DMA
wire [17:0] rk11_adr;                     
wire        rk11_dma_stb;
wire        rk11_dma_we;
wire [15:0] rk11_dma_out;

`ifdef RK_module

rk11 rkdisk (

// шина wishbone
   .wb_clk_i(wb_clk),      // тактовая частота шины
   .wb_rst_i(sys_init),    // сброс
   .wb_adr_i(wb_adr[3:0]), // адрес 
   .wb_dat_i(wb_out),      // входные данные
   .wb_dat_o(rk11_dat),    // выходные данные
   .wb_cyc_i(1'b1),        // начало цикла шины
   .wb_we_i(wb_we),        // разрешение записи (0 - чтение)
   .wb_stb_i(rk11_stb),    // строб цикла шины
   .wb_sel_i(wb_sel),      // выбор конкретных байтов для записи - старший, младший или оба
   .wb_ack_o(rk11_ack),    // подтверждение выбора устройства

// обработка прерывания   
   .irq(rk11_irq),         // запрос
   .iack(rk11_iack),       // подтверждение
   
// DMA
   .dma_req(rk11_dma_req), // запрос DMA
   .dma_gnt(rk11_dma_state), // подтверждение DMA
   .dma_adr_o(rk11_adr),   // выходной адрес при DMA-обмене
   .dma_dat_i(wb_mux),     // входная шина данных DMA
   .dma_dat_o(rk11_dma_out), // выходная шина данных DMA
   .dma_stb_o(rk11_dma_stb), // строб цикла шины DMA
   .dma_we_o(rk11_dma_we),   // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   .dma_ack_i(global_ack), // Ответ от устройства, с которым идет DMA-обмен
   
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
   .start_offset({1'b0,sw_diskbank,22'h0})

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
   .dma_gnt(rk611_dma_state), // подтверждение DMA
   .dma_adr_o(rk611_adr),   // выходной адрес при DMA-обмене
   .dma_dat_i(wb_mux),     // входная шина данных DMA
   .dma_dat_o(rk611_dma_out), // выходная шина данных DMA
   .dma_stb_o(rk611_dma_stb), // строб цикла шины DMA
   .dma_we_o(rk611_dma_we),   // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   .dma_ack_i(global_ack), // Ответ от устройства, с которым идет DMA-обмен
   
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
   .start_offset({1'b0,sw_diskbank,22'hc000})
   ); 

`else 
assign dw_ack=1'b0;
assign dw_sdreq = 1'b0;
assign dw_irq=1'b0;
`endif


//**********************************
//*   Дисковый контроллер RX01
//**********************************
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
   .start_offset({1'b0,sw_diskbank,22'h2c000})
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

// выходная шина DMA
wire [21:0]  my_dma_adr;                     
wire         my_dma_stb;
wire         my_dma_we;
wire [15:0]  my_dma_out;

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
   .dma_gnt(my_dma_state),    // подтверждение DMA
   .dma_adr_o(my_dma_adr),      // выходной адрес при DMA-обмене
   .dma_dat_i(sdram_dat), //wb_mux),      // входная шина данных DMA
   .dma_dat_o(my_dma_out),  // выходная шина данных DMA
   .dma_stb_o(my_dma_stb),  // строб цикла шины DMA
   .dma_we_o(my_dma_we),    // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   .dma_ack_i(global_ack),  // Ответ от устройства, с которым идет DMA-обмен
   
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
   .start_offset({1'b0,sw_diskbank,22'h2e000})
   ); 

`else 
assign my_ack=1'b0;
assign my_dma_req=1'b0;
assign my_sdreq = 1'b0;
assign my_irq=1'b0;
`endif

//****************************************************
//*  Дисковый контроллер RH-70
//****************************************************

// Сигналы запроса-подтверждения DMA
wire rh70_dma_req;

// выходная шина DMA
wire [21:0] rh70_dma_adr;                     
wire        rh70_dma_stb;
wire        rh70_dma_we;
wire [15:0] rh70_dma_out;

`ifdef DB_module

rh70 db_disk (

// шина wishbone
   .wb_clk_i(wb_clk),      // тактовая частота шины
   .wb_rst_i(sys_init),    // сброс
   .wb_adr_i(wb_adr[5:0]), // адрес 
   .wb_dat_i(wb_out),      // входные данные
   .wb_dat_o(rh70_dat),    // выходные данные
   .wb_cyc_i(1'b1),        // начало цикла шины
   .wb_we_i(wb_we),        // разрешение записи (0 - чтение)
   .wb_stb_i(rh70_stb),    // строб цикла шины
   .wb_sel_i(wb_sel),      // выбор конкретных байтов для записи - старший, младший или оба
   .wb_ack_o(rh70_ack),    // подтверждение выбора устройства

// обработка прерывания   
   .irq(rh70_irq),         // запрос
   .iack(rh70_iack),       // подтверждение
   
// DMA
   .dma_req(rh70_dma_req),   // запрос DMA
   .dma_gnt(rh70_dma_state), // подтверждение DMA
   .dma_adr_o(rh70_dma_adr), // выходной адрес при DMA-обмене
   .dma_dat_i(wb_mux),       // входная шина данных DMA
   .dma_dat_o(rh70_dma_out), // выходная шина данных DMA
   .dma_stb_o(rh70_dma_stb), // строб цикла шины DMA
   .dma_we_o(rh70_dma_we),   // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   .dma_ack_i(global_ack), // Ответ от устройства, с которым идет DMA-обмен
   
// интерфейс SD-карты
   .sdcard_cs(db_cs), 
   .sdcard_mosi(db_mosi), 
   .sdcard_miso(sdcard_miso), 
   .sdcard_sclk(db_sclk),

   .sdclock(sdclock),
   .sdreq(db_sdreq),
   .sdack(db_sdack),
   .sdmode(`DB_sdmode),           // режим ведущего-ведомого
   
// Адрес массива дисков на карте
   .start_offset({1'b0,3'b000,22'h30000})
   ); 
`else 
assign db_ack=1'b0;
assign db_dma_req=1'b0;
assign db_sdreq = 1'b0;
assign db_irq=1'b0;
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
reg [1:0] db_sdreq_filter;

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
  
  db_sdreq_filter[0]=db_sdreq;
  db_sdreq_filter[1]=db_sdreq_filter[0];
  
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
      db_sdack <= 1'b0;
      my_sdack <= 1'b0;
   end   
   else
   // поиск контроллера, желающего доступ к карте
    if ((rk_sdack == 1'b0) && (dm_sdack == 1'b0) && (db_sdack == 1'b0) && (dw_sdack == 1'b0) && (dx_sdack == 1'b0) && (my_sdack == 1'b0)) begin 
       // неактивное состояние - ищем источник запроса 
       if (rk_sdreq == 1'b1) rk_sdack <=1'b1;
       else if (dw_sdreq_filter[1] == 1'b1) dw_sdack <=1'b1;
       else if (dm_sdreq_filter[1] == 1'b1) dm_sdack <=1'b1;
       else if (db_sdreq_filter[1] == 1'b1) db_sdack <=1'b1;
       else if (dx_sdreq_filter[1] == 1'b1) dx_sdack <=1'b1;
       else if (my_sdreq_filter[1] == 1'b1) my_sdack <=1'b1;
    end    
    else 
    // активное состояние - ждем освобождения карты
       if ((rk_sdack == 1'b1) && rk_sdreq_filter[1] == 1'b0) rk_sdack <= 1'b0;
       else if ((dw_sdack == 1'b1) && (dw_sdreq_filter[1] == 1'b0)) dw_sdack <= 1'b0;
       else if ((dm_sdack == 1'b1) && (dm_sdreq_filter[1] == 1'b0)) dm_sdack <= 1'b0;
       else if ((db_sdack == 1'b1) && (db_sdreq_filter[1] == 1'b0)) db_sdack <= 1'b0;
       else if ((dx_sdack == 1'b1) && (dx_sdreq_filter[1] == 1'b0)) dx_sdack <= 1'b0;
       else if ((my_sdack == 1'b1) && (my_sdreq_filter[1] == 1'b0)) my_sdack <= 1'b0;
end
   
//**********************************
//* Мультиплексор линий SD-карты
//**********************************
assign sdcard_mosi =
         dw_sdack? dw_mosi: // DW
         dm_sdack? dm_mosi: // DM
         db_sdack? db_mosi: // DB
         dx_sdack? dx_mosi: // DX
         my_sdack? my_mosi: // MY
         rk_sdack? rk_mosi: // RK
                   `def_mosi; // по умолчанию - контроллер с ведущим SDSPI

assign sdcard_cs =
         dw_sdack? dw_cs:   // DW
         dm_sdack? dm_cs:   // DM
         db_sdack? db_cs:   // DB
         dx_sdack? dx_cs:   // DX
         my_sdack? my_cs:   // MY
         rk_sdack? rk_cs:   // RK
                   `def_cs;   // по умолчанию - контроллер с ведущим SDSPI
                   
assign sdcard_sclk =                    
         dw_sdack? dw_sclk:   // DW
         dm_sdack? dm_sclk:   // DM
         db_sdack? db_sclk:   // DB
         dx_sdack? dx_sclk:   // DX
         my_sdack? my_sclk:   // MY
         rk_sdack? rk_sclk:   // RK
                   `def_sclk;   // по умолчанию - контроллер с ведущим SDSPI
            
//************************************************
//*  Контроллеры прерываний
//************************************************


// приоритет 4
wbc_vic #(.N(5)) vic4
(
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_irq_o(br4_irq),
   .wb_dat_o(irq4_ivec),
   .wb_stb_i(istb[4]),
   .wb_ack_o(br4_iack),
//         UART1-Tx       UART1-Rx        UART2-Tx      UART2-Rx        ИРПР -LP  
   .ivec({16'o000064,    16'o000060  ,   `irps2_ti,   `irps2_ri,    16'o0000200}),
   .ireq({uart1_tx_irq,  uart1_rx_irq,  uart2_tx_irq, uart2_rx_irq,  lpt_irq}),
   .iack({uart1_tx_iack, uart1_rx_iack, uart2_tx_iack,uart2_rx_iack, lpt_iack})
);

// приоритет 5
wbc_vic #(.N(6)) vic5
(
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_irq_o(br5_irq),
   .wb_dat_o(irq5_ivec),
   .wb_stb_i(istb[5]),
   .wb_ack_o(br5_iack),
//      RX-11            DW          RK11        RH70          RK611         MY
   .ivec({16'o000264, 16'o000300, 16'o000220, 16'o000254,   16'o000210, 16'o000170}),
   .ireq({rx_irq,      dw_irq,     rk11_irq,    rh70_irq,   rk611_irq,   my_irq}),
   .iack({rx_iack,     dw_iack,    rk11_iack,   rh70_iack,  rk611_iack,  my_iack})
);

// коммутатор источника вектора прерывания
assign vector= (istb[4]) ? irq4_ivec[8:0] : 9'o0
             | (istb[5]) ? irq5_ivec[8:0] : 9'o0;

//*****************************************************************************
//* Диспетчер доступа к общей шине по запросу от разных мастеров (арбитр DMA)
//*****************************************************************************
reg rk11_dma_state;
reg rh70_dma_state;
reg my_dma_state;
reg rk611_dma_state;

wire dma_req;  // запрос DMA
wire dma_ack;  // подтверждение DMA

// запрос DMA к процессору
assign dma_req = rk11_dma_req | rk611_dma_req | my_dma_req | rh70_dma_req;

// арбитр DMA
always @(posedge wb_clk) begin
   if (sys_init) begin
      // сброс арбитра
      rk11_dma_state <= 1'b0;
      rh70_dma_state <= 1'b0;
      rk611_dma_state <= 1'b0;
      my_dma_state <= 1'b0;
   end   
   // поиск активного запроса DMA
   else if (dma_ack) begin
         // Нет активного DMA-устройства - выбор устройства, которому предоставляется доступ к шине
         if (~(rk11_dma_state | my_dma_state | rk611_dma_state | rh70_dma_state)) begin
           if (rk11_dma_req == 1'b1)  rk11_dma_state <= 1'b1;  // запрос от RK11
           else if (my_dma_req == 1'b1)  my_dma_state <= 1'b1; // запрос от MY
           else if (rk611_dma_req == 1'b1)  rk611_dma_state <= 1'b1; // запрос от DM
           else if (rh70_dma_req == 1'b1)  rh70_dma_state <= 1'b1; // запрос от DB
         end  
         else begin
         // Имеется активное DMA-устройство - ожидание освобождения шины
           if (rk11_dma_req == 1'b0) rk11_dma_state <= 1'b0;       
           if (my_dma_req == 1'b0) my_dma_state <= 1'b0;       
           if (rk611_dma_req == 1'b0) rk611_dma_state <= 1'b0;       
           if (rh70_dma_req == 1'b0) rh70_dma_state <= 1'b0;       
         end  
   end
end

 
//*******************************************************************
//*  Коммутатор источника управления (мастера) шины wishbone
//*******************************************************************

// Основная адресная шина
// адрес переключается только для контроллеров DB/RH70 и MY
// адреса остальных контроллеров идут через  Unibus Mapping
assign wb_adr = 
`ifdef massbus
                  (rh70_dma_state) ? rh70_dma_adr : 
`endif
                  (my_dma_state)   ? my_dma_adr :
                  cpu_adr;

// Адресная шина UNIBUS - DMA-запрсы идут через MMU подсистему Unibus Mapping
assign dma_adr18 = (rk11_dma_state) ? rk11_adr : 18'o0 
`ifndef massbus
                |  (rh70_dma_state)? rh70_dma_adr[17:0]: 18'o0 
`endif                
                |  (rk611_dma_state)? rk611_adr: 18'o0 ;

// Выходная шина данных, от мастера DMA к ведомому устройству
assign wb_out =   (rk11_dma_state) ? rk11_dma_out: 16'o0
                | (rk611_dma_state)? rk611_dma_out: 16'o0 
                | (my_dma_state)   ? my_dma_out  : 16'o0
                | (rh70_dma_state) ? rh70_dma_out  : 16'o0
                | (~dma_ack) ? cpu_data_out: 16'o0;

// Сигнал направления передачи - 1 = от устройства в память, 0 = из памяти в устройство
assign wb_we =  rk11_dma_we | rk611_dma_we | my_dma_we | rh70_dma_we | (~dma_ack & cpu_we);

// Выбор байтов для записи                                           
assign wb_sel =   (dma_ack) ? 2'b11: cpu_bsel;
                          
// Строб SDRAM                          
assign sdram_stb = my_dma_stb 
`ifdef massbus
   |rh70_dma_stb   // в режиме massbus отключаем строб памяти при обмене с диском DB
`endif
`ifdef RAM256
   | (cpu_ram_stb && (wb_adr[21:18] == 4'b0000)); // обрезка памяти до 256К
`elsif RAM1M	
   | (cpu_ram_stb && (wb_adr[21:20] == 2'b00)); // обрезка памяти до 1M
`else
   | cpu_ram_stb;   // полные 4М памяти
`endif

// Строб данных от DMA-мастера, работающего через Unibus Mapping
`ifdef massbus
  assign dma_stb_ubm = rk11_dma_stb | rk611_dma_stb;
`else
  assign dma_stb_ubm = rk11_dma_stb | rk611_dma_stb | rh70_dma_stb; 
`endif
 
//*******************************************************************
//*  Сигналы управления шины wishbone
//******************************************************************* 
// Страница ввода-вывода
assign uart1_stb  = bus_stb & (wb_adr[15:3] == (16'o177560 >> 3));   // ИРПС консольный (TT) - 177560-177566 
assign uart2_stb  = bus_stb & (wb_adr[15:3] == (16'o176500 >> 3));   // ИРПС дополнительный - 176500-177506
assign lpt_stb    = bus_stb & (wb_adr[15:2] == (16'o177514 >> 2));   // ИРПР (LP) - 177514-177516
assign rk11_stb   = bus_stb & (wb_adr[15:4] == (16'o177400 >> 4));   // RK - 177400-177416
assign rk611_stb  = bus_stb & (wb_adr[15:5] == (16'o177440 >> 5));   // DM - 177440-177476
assign dw_stb     = bus_stb & (wb_adr[15:5] == (16'o174000 >> 5));   // DW - 174000-174026
assign rx_stb     = bus_stb & (wb_adr[15:2] == (16'o177170 >> 2));   // DX - 177170-177172
assign my_stb     = bus_stb & (wb_adr[15:2] == (16'o172140 >> 2));   // MY - 172140-172142 
assign kgd_stb    = bus_stb & (wb_adr[15:3] == (16'o176640 >> 3));   // КГД - 176640-176646
`ifdef massbus
 assign rh70_stb   = bus_stb & (wb_adr[15:6] == (16'o176700 >> 6));   // DB - 176700-176776 для massbus-конфигураций
`else 
assign rh70_stb   = bus_stb & (wb_adr[15:6] == 10'o1767) & ((wb_adr[5] == 1'b0) || (wb_adr[5:3] == 3'b100)); // DB - 176700-176746 для обычных qbus/unibus
`endif

// Сигналы подтверждения - собираются через OR со всех устройств
assign global_ack  = sdram_ack | uart1_ack | uart2_ack | rk11_ack | rk611_ack | lpt_ack | dw_ack | rx_ack | my_ack | kgd_ack | rh70_ack;

// Мультиплексор выходных шин данных всех устройств
assign wb_mux = 
       (sdram_stb ? sdram_dat : 16'o000000)
     | (uart1_stb ? uart1_dat : 16'o000000)
     | (uart2_stb ? uart2_dat : 16'o000000)
     | (rk11_stb  ? rk11_dat  : 16'o000000)
     | (rh70_stb  ? rh70_dat  : 16'o000000)
     | (rk611_stb ? rk611_dat : 16'o000000)
     | (lpt_stb   ? lpt_dat   : 16'o000000)
     | (dw_stb    ? dw_dat    : 16'o000000)
     | (rx_stb    ? rx_dat    : 16'o000000)
     | (my_stb    ? my_dat    : 16'o000000)
     | (kgd_stb   ? kgd_dat   : 16'o000000)
;

  
endmodule


