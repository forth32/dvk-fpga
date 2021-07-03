//
//  Терминал КСМ (контроллер символьного монитора) - текстовая часть видеоподсистемы ДВК
//==============================================================================================
//
// Начальная скорость последовательного порта initspeed:
//            000 - 1200
//            001 - 2400
//            010 - 4800
//            011 - 9600
//            100 - 19200
//            101 - 38400
//            110 - 57600
//            111 - 115200

module ksm (
   // VGA
   output vgahs,       // горизонтальная синхронизация
   output vgavs,       // вертикальная синхронизация  
   output vgared,      // красный
   output vgagreen,    // зеленый
   output vgablue,     // синий
   
   // последовательный порт
   output tx, 
   input  rx, 
   
   // клавиатура PS/2
   input  ps2_clk, 
   input  ps2_data, 
   
   // пищалка
   output buzzer,
   
// синхронизация с КГД
   output [10:0] col,        // колонка X, 0-1055
   output [9:0]  row,        // строка Y, 0-627
   
   output [2:0] vspeed,      // индекс скорости порта
   input  [2:0] initspeed,   // индекс начальной скорости
   input  clk50,             // 50 MHz
   input  reset              // сброс
); 
 
//----------------------------------------------------------------
wire         sys_init;   // шина сброса - от процессора к периферии

// шина wishbone
wire         wb_clk = clk50;           // синхронизация wishbone - от положительного синхросигнала
wire [15:0]   wb_adr;                  //   шина адреса
wire [15:0] wb_out;                    // вывод данных
wire [15:0] wb_mux;                    //   ввод данных
wire         wb_cyc;                   // начало цикла
wire         wb_we;                    // разрешение записи
wire [1:0]   wb_sel;                   // выбор байтов
wire         wb_stb;                   // строб транзакции
wire         wb_ack;                   // ответ от устройства

// Шины векторного прерывания                                       
wire         vm_istb;                  // строб векторного прерывания
wire         vm_iack;                  // подтверждение векторного прерывания
wire [15:0]   vm_ivec;                 // вектор внешнего прерывания
wire         vm_virq;                  // запрос векторного прерывания

// сбросы
wire         dclo;                     // сброс процессора
wire         aclo;                     // прерывание по сбою питания

// сигналы выбора периферии
wire cpu_dev_stb;                     
wire uart_stb;
wire vtram_stb;
wire vga_stb;
wire vregs_stb;
wire ps2_stb;

// линии подтверждения обмена
wire cpu_dev_ack;
wire uart_ack;
wire vtram_ack;
wire vga_ack;
wire vregs_ack;
wire ps2_ack;

wire vm_una;    

// шины данных от периферии к процессору
wire [15:0]   cpu_dev_dat;
wire [15:0] uart_dat;
wire [15:0] vtram_dat;
wire [15:0] vga_dat;
wire [15:0] vregs_dat;
wire [15:0] ps2_dat;

// линии прерывания                                     
wire   tx_irq, tx_iack;            
wire   rx_irq, rx_iack;            
wire   ps2_irq, ps2_iack;
wire  irq50;                

wire [10:0] cursor_adr;       // адрес курсора
wire [15:0] vtcsr;            // регистр управления видеоконтроллером

assign vspeed=vtcsr[10:8];    // индекс скорости - извлекаем из регистра управления

// переключатель локальной петли
//   online        offline
//  vtx -> tx      1  -> tx
//  rx  -> vrx     vtx -> vrx
wire vtx,vrx;
//                      online  offline
assign vrx = vtcsr[0]?  rx   :  vtx;
assign tx  = vtcsr[0]?  vtx  :  1'b1;

// пищалка
assign buzzer=vtcsr[4];

//**********************************
//* модуль формирования сбросов
//**********************************
vtreset sysreset (
   .clk(clk50),
   .rstin(reset),   // вход сброса
   .dclo(dclo),     // dclo - сброс от источника питания
   .aclo(aclo),     // aclo - прерывание по сбою питания
   .irq50(irq50)    // сигнал интервального таймера 50 Гц
);
 
//*************************************
//*  Процессор К1801ВМ2
//*************************************
defparam cpu.VM2_CORE_FIX_PREFETCH = 1;
 
vm2_wb cpu
(
// Синхросигналы  
   .vm_clk_p(clk50),               // Положительный синхросигнал
   .vm_clk_n(~clk50),              // Отрицательный синхросигнал
   .vm_clk_slow(1'b0),             // Режим задержки синхросигнала

// Шина Wishbone                                       
   .wbm_gnt_i(1'b1),                   // 1 - разрешение cpu работать с шиной
   .wbm_adr_o(wb_adr),                 // выход шины адреса
   .wbm_dat_o(wb_out),                 // выход шины данных
   .wbm_dat_i(wb_mux),                 // вход шины данных
   .wbm_cyc_o(wb_cyc),                 // Строб цила wishbone
   .wbm_we_o(wb_we),                   // разрешение записи
   .wbm_sel_o(wb_sel),                 // выбор байтов для передачи
   .wbm_stb_o(wb_stb),                 // строб данных
   .wbm_ack_i(wb_ack),                 // вход подтверждения данных

// Сбросы и прерывания
   .vm_init(sys_init),                 // Выход сброса для периферии
   .vm_dclo(dclo),                     // Вход сброса процессора
   .vm_aclo(aclo),                     // Сигнал аварии питания
   .vm_halt(1'b0),                     // Прерывание входа в пультовыый режим
   .vm_evnt(irq50),                    // Прерывание от таймера 50 гц
   .vm_virq(vm_virq),                  // Векторное прерывание

// Шины обработки прерываний                                       
   .wbi_dat_i(vm_ivec),                // Шина приема вектора прерывания
   .wbi_stb_o(vm_istb),                // Строб приема вектора прерывания
   .wbi_ack_i(vm_iack),                // Подтверждение приема вектора прерывания
   .wbi_una_o(vm_una)                  // Строб безадресного чтения
);


//******************************************************************
//* Модуль статической памяти с резидентным монитором
//******************************************************************
vtram ram(
   .wb_clk_i(wb_clk),
   .wb_adr_i(wb_adr),
   .wb_we_i(wb_we),
   .wb_dat_i(wb_out),
   .wb_dat_o(vtram_dat),
   .wb_cyc_i(wb_cyc),
   .wb_stb_i(vtram_stb),
   .wb_sel_i(wb_sel),
   .wb_ack_o(vtram_ack)
);

//**********************************
//*         UART
//**********************************

// константа скорости
wire [31:0]   baud;                        
assign  baud = 
  (vspeed == 3'd0)   ? 32'd767: // 1200
  (vspeed == 3'd1)   ? 32'd383: // 2400
  (vspeed == 3'd2)   ? 32'd191: // 4800
  (vspeed == 3'd3)   ? 32'd95:  // 9600
  (vspeed == 3'd4)  ?  32'd47:  // 19200
  (vspeed == 3'd5)  ?  32'd23:  // 38400
  (vspeed == 3'd6)  ?  32'd15:  // 57600
                        32'd7;  // 115200

wbc_uart #(.REFCLK(50000000)) uart
(
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_adr_i(wb_adr[2:0]),
   .wb_dat_i(wb_out),
   .wb_dat_o(uart_dat),
   .wb_cyc_i(wb_cyc),
   .wb_we_i(wb_we),
   .wb_stb_i(uart_stb),
   .wb_ack_o(uart_ack),

   .tx_cts_i(1'b0),
   .txd(vtx),
   .rxd(vrx),

   .tx_irq_o(tx_irq),
   .tx_iack_i(tx_iack),
   .rx_irq_o(rx_irq),
   .rx_iack_i(rx_iack),

   .cfg_bdiv(baud[15:0]),
   .cfg_nbit(2'b11),
   .cfg_nstp(1'b1),
   .cfg_pena(1'b0),
   .cfg_podd(1'b0)
);

//**********************************
//*  Видеоконтроллер
//**********************************
vga video (
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_adr_i(wb_adr),
   .wb_dat_i(wb_out),
   .wb_dat_o(vga_dat),
   .wb_cyc_i(wb_cyc),
   .wb_we_i(wb_we),
   .wb_stb_i(vga_stb),
   .wb_ack_o(vga_ack),
   .wb_sel_i(wb_sel),
   .cursor(cursor_adr),     // текущий адрес курсора
   .hsync(vgahs),           // строчный синхросигнал
   .vsync(vgavs),           // кадровый синхросигнал
   .vgag(vgagreen),         // зеленый цвет
   .vgar(vgared),           // красный цвет
   .vgab(vgablue),          // синий цвет
   .cursor_on(vtcsr[2]),    // видимость курсора
   .cursor_type(vtcsr[3]),  // форма курсора
   .col(col),               // текущая колонка экрана
   .row(row),               // текущая строка экрана
   .flash(vtcsr[5]),        // импульсы мерцания символов
   .clk50(clk50)
);
   
//*******************************************************
//*  Регистры курсора и управления видеоконтроллером
//*******************************************************
vregs videoreg (
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_adr_i(wb_adr),
   .wb_dat_i(wb_out),
   .wb_dat_o(vregs_dat),
   .wb_cyc_i(wb_cyc),
   .wb_we_i(wb_we),
   .wb_stb_i(vregs_stb),
   .wb_ack_o(vregs_ack),
   .wb_sel_i(wb_sel),
   .cursor(cursor_adr),   // регистр адреса курсора
   .vtcsr(vtcsr),         // регистр управления
   .initspeed(initspeed)  // константа начальной скорости
); 

//**********************************
//*   контроллер клавиатуры PS/2
//**********************************
ps2 ps20 (
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_adr_i(wb_adr),
   .wb_dat_i(wb_out),
   .wb_dat_o(ps2_dat),
   .wb_cyc_i(wb_cyc),
   .wb_we_i(wb_we),
   .wb_stb_i(ps2_stb),
   .wb_ack_o(ps2_ack),
   .wb_sel_i(wb_sel),
   .irq(ps2_irq),
   .iack(ps2_iack),
   .ps2_clk(ps2_clk), 
   .ps2_data(ps2_data)
); 


//**********************************
//*  Контроллер прерываний
//**********************************
ksm_vic #(.N(3)) svic   (
   .wb_clk_i(wb_clk),
   .wb_rst_i(dclo),
   .wb_irq_o(vm_virq),   
   .wb_dat_o(vm_ivec),
   .wb_stb_i(vm_istb),
   .wb_ack_o(vm_iack),
   .rsel(16'o000000),    // содержимое слова безадресного чтения
   .wb_una_i(vm_una),
//         UART-Tx     UART-Rx         PS2
   .ivec({16'o000064, 16'o000060, 16'o000054}),   // векторы
   .ireq({tx_irq,       rx_irq,     ps2_irq }),   // запрос прерывания
   .iack({tx_iack,      rx_iack,    ps2_iack})    // подтверждение прерывания
);

//*******************************************************************
//*  Сигналы управления шины wishbone
//******************************************************************* 

// Формирователь сигналов выборки устройств
assign uart_stb   = wb_stb & wb_cyc & (wb_adr[15:3] == (16'o177560 >> 3)); // UART DL11 - 177560-177567
assign vtram_stb = wb_stb & wb_cyc & (wb_adr[15:12] == 4'b0000);           // RAM 000000-017777
assign vga_stb = wb_stb & wb_cyc & (wb_adr[15:13] == 3'b110);              // видеопамять 140000 - 156200/157777
assign vregs_stb = wb_stb & wb_cyc & (wb_adr[15:2] == (16'o170000>>2));    // регистры курсора и управления видеоконтроллером 170000-170002
assign ps2_stb = wb_stb & wb_cyc & (wb_adr[15:2] == (16'o171000>>2));      // контроллер клавиатуры PS/2 171000-171002

// Сигналы подтверждения - собираются через OR со всех устройств
assign wb_ack   = vtram_ack | uart_ack | vga_ack | vregs_ack | ps2_ack;

// Мультиплексор выходных шин данных всех устройств
assign wb_mux      =  (vtram_stb ? vtram_dat : 16'o000000)
                   | (vga_stb ? vga_dat : 16'o000000)
                   | (vregs_stb ? vregs_dat : 16'o000000)
                   | (ps2_stb ? ps2_dat : 16'o000000)
                   | (uart_stb ? uart_dat : 16'o000000);
endmodule
