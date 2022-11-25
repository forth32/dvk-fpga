//
// Процессорная плата KDF11, основана на чипсете DEC  F-11.
// Использовалась в ЭВМ PDP-11/23
// 
// ======================================================================================

module kdf11 (
// Синхросигналы  
   input          clk_p,         // тактовый сигнал, прямая фаза
   input          clk_n,         // тактовый сигнал, инверсная фаза
   input          cpuslow,       // Режим замедления процессора
                                       
// Шина Wishbone                                       
   output [21:0] wb_adr_o,       // выход шины адреса
   output [15:0] wb_dat_o,       // выход шины данных
   input  [15:0] wb_dat_i,       // вход шины данных
   output wb_we_o,               // разрешение записи
   output [1:0] wb_sel_o,        // выбор байтов для передачи

   output ram_stb,               // строб обращения к системной памяти
   output bus_stb,               // строб обращения к общей шине
   input  global_ack,            // подтверждение обмена от памяти и устройств страницы ввода-вывода

// DMA   
   input  dma_req,               // запрос DMA
   output reg dma_ack,           // подтверждение DMA
   input  [17:0] dma_adr18,      // 18-битный UNIBUS-адрес для устройств, работающих через UBM
   input  dma_stb,               // строб данных для устройств, работающих через UBM
   
// Сбросы и прерывания
   output bus_reset,             // Выход сброса для периферии
   input dclo,                   // Вход сброса процессора
   input aclo,                   // Сигнал аварии питания
   
// Ручное управление
   input [15:0] csw,
   input resume,	
	
// Информационные индикаторы   
   output led_idle,              // индикация бездействия (WAIT)
   output led_run,               // индикация работы процессора (~HALT)
   output led_mmu,               // индикация включения MMU
   output led_timer,             // индикация включения таймера
   output reg [15:0] swr_out,    // вывод регистра консольной индикации процессора
   
// Шины обработки прерываний                                       
   input [5:4] irq_i,            // Запрос прерывания
   output [5:4] istb_o,          // Строб приема вектора
   input [8:0] ivec,             // Шина приема вектора прерывания
   input iack_i                  // Подтверждение приема вектора прерывания
);   
   
//==========================================================================================


assign led_idle=1'b1;
assign led_mmu=1'b1;
assign led_run=1'b1;

wire sys_init;              
                                      
wire [15:0] wb_mux;    // сборная шина данных от периферии к процессору                
wire        cpu_stb;   // строб данных от процессор на шину            
wire ioaccess;   // признак доступа процессора к периферийной шине
wire fdin_stb;
wire [15:0] kw11l_dat; // шина данных таймера

// Слово конфигурации начального пуска - помещается в регистр быстрого ввода
wire [15:0] fdin_data= 16'o165000;

// сигналы подтверждения обмена
wire wb_ack;    // подтверждения обмена от шины 
reg  kw11l_ack;       
wire cpu_ack;          // подтверждение обмена для транзакций шины, генерируемых процессором

// стробы выбора периферии
wire kw11l_stb;             

// сброс системы
assign      sys_init = bus_reset;

// таймер
reg timer_ie;    // разрешение прерывания
reg timer_rdy;   // готовность таймера
reg timer_irq;   // запрос прерывания
wire timer_istb; // строб запроса вектора
wire timer_iack=timer_istb;

// шина адреса
wire [21:0] cpu_adr;
assign wb_adr_o= dma_ack? {4'b0000, dma_adr18} : cpu_adr; // для операций DMA-18 на шину выставляется входной DMA-адрес

// Прерывания 
wire [7:4] vstb;          // строб приема вектора
wire [7:4] virq;          // запрос прерывания
wire cpu_istb;
wire [15:0]cpu_int_vector;

assign cpu_int_vector=fdin_ack? fdin_data: {8'h00, vector};

// линии запроса прерывания
assign virq[7]=1'b0;      // уровень 7 - нет
assign virq[6]=timer_irq; // уровень 6 - таймер
assign virq[5]=irq_i[5];  // уровень 5 - быстрая (блочная) периферия
assign virq[4]=irq_i[4];  // уровень 4 - медленная (байтовая) периферия

// запрос на прием вектора
assign timer_istb=vstb[6];
assign istb_o[5]=vstb[5];
assign istb_o[4]=vstb[4];
// шина ввода вектора прерывания в процессор
wire [8:0] vector = (vstb[6])? 8'o100:             // таймер
                    ivec;                          // входной вектор от контроллеров прерывания  

                    
//*************************************
// счетчик замедления процессора
//*************************************
reg [4:0] cpudelay;
reg cpu_clk_enable;

always @ (posedge clk_p) begin
    if (cpudelay != 5'd21) begin
        cpudelay <= cpudelay + 1'b1;  // считаем от 0 до 22
        cpu_clk_enable <= 1'b0;
    end     
    else begin
        cpudelay <= 5'd0;
        cpu_clk_enable <= 1'b1;
    end     
end                     
                    
//*************************************
//*  Процессор F-11
//*************************************
f11_wb cpu (
//
// F11_CORE_MMU enables code MMU generation in the dc304 module
// F11_CORE_FPP enables code of FPP MiCROM in the dc303 module
//
//    F11_CORE_MMU = 1,
//    F11_CORE_FPP = 1

   .vm_clk_p(clk_p),     
   .vm_clk_n(clk_n),     
   .vm_clk_ena(cpu_clk_enable), 
   .vm_clk_slow(1'b0), 
                           
   .vm_init(bus_reset),          // peripheral reset output
   .vm_dclo(dclo),       // processor reset
   .vm_aclo(aclo),       // power fail notificaton
   .vm_halt(resume),       // halt mode interrupt
   .vm_evnt(1'b0),       // timer interrupt requests
   .vm_virq(irq_i),   // vectored interrupt request
   
   .wbm_gnt_i(~dma_ack),       // master wishbone granted
   .wbm_ios_o(ioaccess),         // master wishbone bank I/O select
   .wbm_adr_o(cpu_adr),         // master wishbone address
   .wbm_dat_o(wb_dat_o),         // master wishbone data output
   .wbm_dat_i(wb_mux),        // master wishbone data input
//   .wbm_cyc_o(cpu_cyc),     // master wishbone cycle
   .wbm_we_o(wb_we_o),       // master wishbone direction
   .wbm_sel_o(wb_sel_o),     // master wishbone byte select
   .wbm_stb_o(cpu_stb),     // master wishbone strobe
   .wbm_ack_i(cpu_ack),     // master wishbone acknowledgement

   .wbi_dat_i(cpu_int_vector),     // interrupt vector input
   .wbi_ack_i(iack_i|fdin_ack),     // interrupt vector acknowledgement
   .wbi_stb_o(cpu_istb),     // interrupt vector strobe
   .wbi_una_o(fdin_stb),     // unaddressed fast input read
                  
   .vm_bsel(2'b01)        // boot mode selector
);


//*****************************************************
//* Преобразования управляющих сигналов процессора
//*****************************************************

assign ram_stb = dma_ack? dma_stb : /*cpu_cyc &*/ cpu_stb & ~ioaccess;
assign bus_stb = /*cpu_cyc &*/ cpu_stb & ioaccess & ~dma_ack; 
assign cpu_ack = wb_ack & ~dma_ack;

// приоритетный выбор линии подтверждения прерывания						
assign vstb[6] = cpu_istb & virq[6];
assign vstb[5] = cpu_istb & virq[5] & ~vstb[6];
assign vstb[4] = cpu_istb & virq[4] & ~vstb[5] & ~vstb[6];
						
// формирователь сигналов DMA
always @(posedge clk_p)
   if (dclo) dma_ack <= 1'b0;
   else if (dma_req & ~cpu_stb) dma_ack <= 1'b1;
   else if (~dma_req) dma_ack <= 1'b0;

// Формирователь сигнала подтверждения безадресного чтения
reg fdin_ack;
always @(posedge clk_p or posedge dclo)
  if (dclo) fdin_ack <=1'b0;
  else fdin_ack <= fdin_stb;

	
//*******************************************
//* ПЗУ монитора-загрузчика
//*******************************************

// эмулятор пульта и набор загрузчиков m9312, состоит из 2 частей:
// - консоль, 165000-165777
// - загрузчики, 173000-173777
// Обе части лежат в одном блоке ПЗУ, консоль в младшей части, загрузчики в старшей.

wire bootrom0_sel;
wire bootrom1_sel;
wire bootrom_stb;
wire bootrom_ack;
wire [15:0] bootrom_dat;
reg [1:0]bootrom_ack_reg;

boot_rom bootrom(
   .address({bootrom1_sel, wb_adr_o[8:1]}),
   .clock(clk_p),
   .q(bootrom_dat));

// сигнал ответа
always @ (posedge clk_p) begin
   bootrom_ack_reg[0] <= bootrom_stb & ~wb_we_o;
   bootrom_ack_reg[1] <= bootrom_stb & bootrom_ack_reg[0] & ~wb_we_o;
end
assign bootrom_ack = bus_stb & bootrom_ack_reg[1];

// сигналы выбора частей ПЗУ
assign bootrom1_sel =  (wb_adr_o[15:9] == 7'o173);                // загрузчики, 173000-173776
assign bootrom0_sel =  (wb_adr_o[15:9] == 7'o165);                // консоль, 165000-165776
assign bootrom_stb  = bus_stb & (bootrom0_sel | bootrom1_sel);    // строб выбора ПЗУ


//*************************************************************************
//* Генератор прерываний от таймера
//* Сигнал имеет частоту 50 Гц и ширину импульса в 1 такт
//*************************************************************************
reg timer_50;
reg [20:0] timercnt;

wire [20:0] timer_limit=31'd`clkref/6'd50-1'b1;

always @ (posedge clk_p) begin
  if (timercnt == timer_limit) begin
     // заворот счетчика
     timercnt <= 21'd0;  // перезагрузка
     timer_50 <= 1'b1;   // формируем импульс таймера
  end  
  else begin
     // счет тактов
     timercnt <= timercnt + 1'b1; // счетчик++
     timer_50 <= 1'b0;            // завершаем импульс таймера
  end     
end

//************************************************
//* Сетевой таймер KW11-L 177546
//************************************************
reg tirq_prev_state;  // состояние таймера в предыдущем такте

// чтение регистра таймера
//                           7           6
assign kw11l_dat = {8'o0, timer_rdy, timer_ie, 6'o0};
assign led_timer=~timer_ie;

always @ (posedge clk_p) 
  // сброс системы
  if (sys_init == 1'b1) begin 
     timer_ie <= 1'b0;
     timer_rdy <= 1'b1;
     timer_irq <= 1'b0;
  end     
  else begin     
    // обмен с общей шиной
    if ((kw11l_stb == 1'b1) && (wb_we_o == 1'b1)) begin
        // запись
        timer_ie  <= wb_dat_o[6];
        timer_rdy <= wb_dat_o[7];
    end
    tirq_prev_state <= timer_50;   // сохранение предыдущего состояния сигнала
    // детектор перепадов сигнала таймера - только  0->1
    if ((tirq_prev_state != timer_50) && (timer_50 == 1'b1)) begin
        timer_rdy <= 1'b1;  // взводим сигнал готовности таймера
        // формирователь сигнала прерывания
        if (timer_ie) timer_irq <= 1'b1;
    end
     // формирователь вектора
     if (timer_irq == 1'b1) begin 
       if (~timer_ie | timer_istb) timer_irq <= 1'b0;
     end
  end     
  
// формирователь ответа       
wire kw11l_reply= kw11l_stb & ~kw11l_ack;
always @(posedge clk_p)
    if (sys_init == 1'b1) kw11l_ack <= 1'b0;
    else kw11l_ack <= kw11l_reply;


    
//*******************************************************************
//*  Формирователь сигналов выбора устройств на шине
//*******************************************************************

// стробы выбора периферии
assign kw11l_stb  = bus_stb & (wb_adr_o[15:1] == (16'o177546 >> 1));   // KW11-L - 177546

// сигнал ответа
assign wb_ack     = global_ack | kw11l_ack | bootrom_ack;

// сборная шина входных данных к процессору
assign wb_mux     = wb_dat_i
                  | (kw11l_stb ? kw11l_dat : 16'o000000)
                  | (bootrom_stb ? bootrom_dat : 16'o000000);

endmodule
