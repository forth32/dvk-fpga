//
// 
// 
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
   input   dma_req,              // запрос DMA
   output  dma_ack,              // подтверждение DMA
   input   [17:0] dma_adr18,     // 18-битный UNIBUS-адрес для устройств, работающих через UBM
   input   dma_stb,              // строб данных для устройств, работающих через UBM
   
// Сбросы и прерывания
   output bus_reset,             // Выход сброса для периферии
   input dclo,                   // Вход сброса процессора
   input aclo,                   // Сигнал аварии питания
   
// сигналы ручного управления   
   input resume,                 // Сигнал запуска процессора после HALT
   input [15:0] csw,             // регистр консольных переключателей процессора     

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


wire sys_init;              
                                      
wire [15:0] wb_mux;    // сборная шина данных от периферии к процессору                
wire        cpu_stb;   // строб данных от процессор на шину            
wire        wb_ack;    // подтверждения обмена от шины к процессору            

// шины данных внутренней периферии
wire [15:0] ccr_dat;   // блок управляющих регистров          
wire [15:0] kw11l_dat; // таймер
wire [15:0] mmu_dat;   // MMU

// сигналы подтверждения обмена
wire ccr_ack;             
reg swr_ack;             
reg kw11l_ack;       
wire cpu_ack;      

// стробы выбора периферии
wire swr_stb;             
wire kw11l_stb;             
wire ccr_stb;

// PSW
wire [15:0] cpu_psw_in;   // Ввод PSW , записываемого под адресу 177776
wire cpu_psw_we_even;     // Разрешение записи младшего байта PSW
wire cpu_psw_we_odd;      // Разрешение записи старшего байта PSW
wire [15:0] cpu_psw_out;  // Вывод PSW , читаемого под адресу 177776

// флаги ошибок
wire oddabort;
wire cpu_ysv;            // желтая граница стека
wire cpu_rsv;            // красная граница стека
wire cpu_illegal_halt;   // HALT не в режиме kernel
wire bus_timeout;        // таймаут обмена по шине

// регистр границы стека
wire [15:0] cpu_stack_limit;

// индикация состояния
wire iwait;
wire run;
assign led_mmu= ~(cons_map18 | cons_map22);
assign led_run=~run;
assign led_idle=~iwait;

// сброс системы
assign      sys_init = bus_reset;

// интерфейс cpu<->mmu
wire cpu_cp;
wire cpu_id;
wire mmutrap;
wire ack_mmutrap;
wire mmuabort;
wire sr0_ic;
wire [15:0] sr1;
wire [15:0] sr2;
wire cons_map16;
wire cons_map18;
wire cons_map22;
wire cons_id;
wire cons_ubm;
wire ifetch;
wire dstfreference;
wire dw8;
wire [15:0] cpu_adr;
wire ack_mmuabort;
wire mmuoddabort;

// таймер
reg timer_ie;    // разрешение прерывания
reg timer_rdy;   // готовность таймера
reg timer_irq;   // запрос прерывания
wire timer_istb; // строб запроса вектора
wire timer_iack=timer_istb;

// Прерывания 
wire [7:4] vstb;          // строб приема вектора
wire [7:4] virq;          // запрос прерывания
wire [15:0] cpu_pir_in;   // регистр программных прерываний
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
wire [8:0] vector = (vstb[6])? 9'o100:             // таймер
                    ivec;                          // входной вектор от контроллеров прерывания  

                    

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
assign swr_stb    = bus_stb & (wb_adr_o[15:1] == (16'o177570 >> 1));   // SWR - 177570
assign kw11l_stb  = bus_stb & (wb_adr_o[15:1] == (16'o177546 >> 1));   // KW11-L - 177546
assign ccr_stb = bus_stb & (wb_adr_o[15:5] == 11'b11111111111);        // 177740 - 177777 (на полной шине - 777740-777777) - внутренние регистры процессора

// сигнал ответа
assign wb_ack     = global_ack | ccr_ack | swr_ack | kw11l_ack | bootrom_ack;

// сборная шина входных данных к процессору
assign wb_mux     = mmu_dat | wb_dat_i
                  | (ccr_stb ? ccr_dat : 16'o000000)
                  | (kw11l_stb ? kw11l_dat : 16'o000000)
                  | (swr_stb ? swr_dat : 16'o000000)
                  | (bootrom_stb ? bootrom_dat : 16'o000000);

endmodule
