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


//assign led_idle=1'b1;
//assign led_mmu=1'b1;
//assign led_run=1'b1;

assign led_run=cdr_o[0];
assign led_idle=cdr_o[1];
assign led_mmu=cdr_o[2];
assign led_timer=cdr_o[3];

wire [15:0] wb_mux;    // сборная шина данных от периферии к процессору                
wire        cpu_stb;   // строб данных от процессор на шину            
wire        cpu_cyc;   // не уверен что он нужен
wire 			ioaccess;   // признак доступа процессора к периферийной шине
wire 			fdin_stb;
wire [15:0] lks_dat; // шина данных таймера

// Слово конфигурации начального пуска - помещается в регистр безадресного ввода
wire [15:0] fdin_data= 16'o173002;  // стартовый адрес - 173000, 
                                    // режим пуска - старт с адреса
												// HALT вываливает в ODT

// сигналы подтверждения обмена
wire wb_ack;    // подтверждения обмена от шины 
reg  lks_ack;   // таймер       
wire  rom_ack;  // оконо доступа к ROM
reg  bdr_ack;   // Boot/diagnostic regs
wire cpu_ack;   // подтверждение обмена для транзакций шины, генерируемых процессором
reg fdin_ack;   // подтверждение доступа к регистру безадресного чтения


// стробы выбора периферии
wire lks_stb;             
wire rom_stb;             
wire bdr_stb;             

// таймер
reg timer_ie;    // разрешение прерывания
reg timer_rdy;   // готовность таймера
wire bevent;     // сигнал запроса прерывания

// шина адреса
wire [21:0] cpu_adr;
// коммутатор шины адреса для режимов CPU/DMA
assign wb_adr_o= dma_ack? {4'b0000, dma_adr18} : cpu_adr; // для операций DMA-18 на шину выставляется входной DMA-адрес

// Прерывания 
wire [7:4] vstb;          // строб приема вектора
wire [7:4] virq;          // запрос прерывания
wire cpu_istb;
wire [15:0]cpu_int_vector;

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

   // синхронизация
   .vm_clk_p(clk_p),        // прямой тактовый сигнал
   .vm_clk_n(clk_n),        // инверсный тактовый сигнал
   .vm_clk_ena(cpu_clk_enable),  // сигнал разрешения работы в данном такте
   .vm_clk_slow(1'b0),      // включение режима замедления  

   // сбросы и прерывания	
   .vm_init(bus_reset),     // выход сигнала сброса устройств на шине
   .vm_dclo(dclo),          // вход сброса системы
   .vm_aclo(aclo),          // прерывание по аварии источника питания
   .vm_evnt(bevent),        // запрос прерывания от таймера
   .vm_virq(irq_i),         // входы запросов векторного прерывания от периферии

   // основная шина Wishbone
   .wbm_gnt_i(~dma_ack),    // запрос на приостановку работы процессора
   .wbm_ios_o(ioaccess),    // флаг доступа к странице ввода-вывода
   .wbm_adr_o(cpu_adr),     // полный физический адрес, сформированный процессором
   .wbm_dat_o(wb_dat_o),    // выход шины данных
   .wbm_dat_i(wb_mux),      // вход шины данных
   .wbm_cyc_o(cpu_cyc),     
   .wbm_we_o(wb_we_o),      // флаг вывода данных на шину (0-ввод, 1-вывод)
   .wbm_sel_o(wb_sel_o),    // выбор байтов для записи
   .wbm_stb_o(cpu_stb),     // строб операции на шине
   .wbm_ack_i(cpu_ack),     // вход подтверждения ввода-вывода (REPLY)

   // шина обработки прерываний
   .wbi_dat_i(cpu_int_vector),    // ввод вектора прерывания от устройства
   .wbi_ack_i(iack_i|fdin_ack),   // подтверждение передачи вектора или данных безадресного ввода
   .wbi_stb_o(cpu_istb),          // строб запроса вектора прцессором
   .wbi_una_o(fdin_stb),          // строб доступа к регистру безадресного ввода

   // управление/индикация	
   .vm_halt(resume),       // запрос перехода в пультовый ODT
   .vm_bsel(2'b01)         // режим пуска процессора
);


//*****************************************************
//* Преобразования управляющих сигналов процессора
//*****************************************************

assign ram_stb = dma_ack? dma_stb : cpu_cyc & cpu_stb & ~ioaccess; // строб доступа к памяти
assign bus_stb = cpu_cyc & cpu_stb & ioaccess & ~dma_ack;          // строб доступа к странице ввода-вывода
assign cpu_ack = wb_ack & ~dma_ack;  // сигнал подтверждения обмена, в режиме DMA неактивен

// приоритетный выбор линии подтверждения прерывания	
assign vstb[6] = cpu_istb & virq[6];
assign vstb[5] = cpu_istb & virq[5] & ~vstb[6];
assign vstb[4] = cpu_istb & virq[4] & ~vstb[5] & ~vstb[6];

// линии запроса прерывания
assign virq[7]=1'b0;      // уровень 7 - нет
assign virq[6]=1'b0;      // уровень 6 - нет (таймер работает на уровне 6, но через вход event)
assign virq[5]=irq_i[5];  // уровень 5 - быстрая (блочная) периферия
assign virq[4]=irq_i[4];  // уровень 4 - медленная (байтовая) периферия

// запрос на прием вектора
assign istb_o[5]=vstb[5];
assign istb_o[4]=vstb[4];

// шина ввода вектора прерывания в процессор
wire [8:0] vector = ivec; // входной вектор от контроллеров прерывания  
// выбор ввода вектора или содержимого регистра безадресного чтения
assign cpu_int_vector=fdin_ack? fdin_data: {8'h00, vector};
						
// формирователь сигналов DMA
always @(posedge clk_p)
   if (dclo) dma_ack <= 1'b0;
   else if (dma_req & ~cpu_stb) dma_ack <= 1'b1; // переход в режим DMA только если процессор не работает с шиной
   else if (~dma_req) dma_ack <= 1'b0;           // снятие запроса DMA

// Формирователь сигнала подтверждения безадресного чтения
always @(posedge clk_p or posedge dclo)
  if (dclo) fdin_ack <=1'b0;
  else fdin_ack <= fdin_stb;

	
//*******************************************
//* Подсистема Bootrom/Diagnostic
//*******************************************

wire[3:0] current_pcr;  // выбранная часть регистра PCR

// ПЗУ загрузки и диагностики, отображается в окно 173000-173777
//   ----- протокол выбора адреса ПЗУ ------------
// ПЗУ имеет размер 16К, адрес 13:0
// старшие 6 бит адреса ПЗУ, 13:8, оперделяются регистром PCR. Биты 7:0 адреса ПЗУ берутся с системной шины адреса.
//  Бит 8 шинного адреса adr[8] определяет, кикая часть регистра PCR будет использована
//  adr[8]==0 - используются pcr[5:0]
//  adr[8]==1 - используется pcr[13:8]
wire [15:0] rom_dat;

kdf11b_rom bdrom(
   .address({current_pcr, wb_adr_o[7:1]}),
   .clock(clk_p),
   .q(rom_dat));

// сигнал ответа
reg [1:0] rom_ack_reg;
always @ (posedge clk_p) begin
   rom_ack_reg[0] <= rom_stb & ~wb_we_o;
   rom_ack_reg[1] <= rom_stb & rom_ack_reg[0] & ~wb_we_o;
end
assign rom_ack = bus_stb & rom_ack_reg[1];
	
// выбор старшей части адреса ПЗУ
assign current_pcr=cpu_adr[8] ? pcr_h : pcr_l;	

//------ регистры подсистемы BDR ------------
// 177520 (W/O) - PCR, выбор старшей части адреса ПЗУ
// 177522 (R/W) - реигстр обслуживания, пока непонятно для чего он нужен
// 177524 (R)   - CDR[7:0] чтение переключателей конфигурации
// 177524 (W)   - CDR[3:0] управление индикаторными светодиодами
reg [3:0] pcr_h;
reg [3:0] pcr_l;
reg [15:0] bd_maint;
reg [3:0] cdr_o;  // вывод в регистр CDR

// доступ к регистра с шины
reg [15:0] bdr_dat; 

always @ (posedge clk_p) 
  // сброс системы
  if (dclo) begin 
     pcr_h <= 4'o0;
     pcr_l <= 4'o0;
	  bd_maint <= 16'o0;
	  cdr_o <= 3'o0;
  end     
  else begin     
    //  обмен с общей шиной
    if (bdr_stb && (~wb_we_o)) begin
        // чтение
		  case (cpu_adr[2:1])
         2'b00:  bdr_dat <= {4'b0000, pcr_h, 4'b0000, pcr_l};
         2'b01:  bdr_dat <= bd_maint;
         2'b10:  bdr_dat <= 8'b00001000; //csw[7:0];  // набор из 8 переключателей на плате
			default: bdr_dat <= 15'o0;
		  endcase	
    end
	 else if (bdr_stb && wb_we_o) begin
	     // запись
		  case (cpu_adr[2:1])
         2'b00:  begin
			          if (wb_sel_o[0]) pcr_l <= wb_dat_o[3:0];
						 if (wb_sel_o[1]) pcr_h <= wb_dat_o[11:8];
					  end	 
         2'b01:  begin
			          if (wb_sel_o[0]) bd_maint[7:0] <= wb_dat_o[7:0];
			          if (wb_sel_o[1]) bd_maint[15:8] <= wb_dat_o[15:8];
					  end	 
         2'b10:  if (wb_sel_o[0]) cdr_o <= wb_dat_o[3:0];
		  endcase	
		  
    end	 
  end

// сигнал ответа
// формирователь ответа       
wire bdr_reply= bdr_stb & ~bdr_ack;
always @(posedge clk_p)
    if (bus_reset == 1'b1) bdr_ack <= 1'b0;
    else bdr_ack <= bdr_reply;
  

//*************************************************************************
//*  Формирователь прерываний таймера
//*************************************************************************

// Генератор прерываний от таймера
// Сигнал имеет частоту 50 Гц и ширину импульса в 1 такт
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

// Регистр управления прерываниями таймера - LKS

// образ для чтения регистра таймера
//                         D6
assign lks_dat = {9'o0, timer_ie, 6'o0};
// Индикатор включения таймера
//assign led_timer=~timer_ie;
// обработка шинных транзакций
always @ (posedge clk_p) 
  // сброс системы
  if (bus_reset == 1'b1) begin 
     timer_ie <= 1'b0;
  end     
  else begin     
    // обмен с общей шиной
    if ((lks_stb == 1'b1) && (wb_we_o == 1'b1)) begin
        // запись
        timer_ie  <= wb_dat_o[6];
    end
  end	 
	 
// формирователь ответа       
wire lks_reply= lks_stb & ~lks_ack;
always @(posedge clk_p)
    if (bus_reset == 1'b1) lks_ack <= 1'b0;
    else lks_ack <= lks_reply;

// сигнал прерывания от таймера   
assign bevent = timer_50 & timer_ie;

    
//*******************************************************************
//*  Формирователь сигналов выбора устройств на шине
//*******************************************************************

// стробы выбора периферии
assign lks_stb  = bus_stb & (wb_adr_o[15:1] == (16'o177546 >> 1));   // LKS (таймер)- 177546
assign rom_stb  = bus_stb & (wb_adr_o[15:9] == (16'o173000 >> 9));   // ROM - 173000-173777
assign bdr_stb  = bus_stb & (wb_adr_o[15:3] == (16'o177520 >> 3));   // Boot/Diagnostic reg- 177520-177524

// сигнал ответа
assign wb_ack     = global_ack | lks_ack | rom_ack | bdr_ack;

// сборная шина входных данных к процессору
assign wb_mux     = wb_dat_i
                  | (lks_stb ? lks_dat : 16'o000000)
                  | (bdr_stb ? bdr_dat : 16'o000000)
                  | (rom_stb ? rom_dat : 16'o000000);

endmodule
