//
//  Процессорный модуль - плата PDP2011 (Электроника-85)
//  Центральный процессор - PDP11/70
// 
// ======================================================================================

module pdp2011 (
// Синхросигналы  
   input          clk_p,      
   input          clk_n,      
   input          cpuslow,              // Режим замедления процессора
                                       
// Шина Wishbone                                       
   output [21:0] wb_adr_o,        // выход шины адреса
   output [15:0] wb_dat_o,       // выход шины данных
   input  [15:0] wb_dat_i,         // вход шины данных
   output wb_we_o,              // разрешение записи
   output [1:0] wb_sel_o,           // выбор байтов для передачи

   output ram_stb,        // строб обращения к системной памяти
   output bus_stb,        // строб обращения к общей шине
   input  global_ack,           // подтверждение обмена от памяти и устройств страницы ввода-вывода

// DMA	
   input   dma_req,
	output  dma_ack,
   input   [17:0] dma_adr18,
   input   dma_stb,
   
// Сбросы и прерывания
   output bus_reset,      // Выход сброса для периферии
   input dclo,              // Вход сброса процессора
   input aclo,              // Сигнал аварии питания
   input resume,            // Сигнал запуска процессора после HALT

	output led_idle,         // индикация бездействия (WAIT)
	output led_run,          // индикация работы процессора (~HALT)
	
// Шины обработки прерываний                                       
   input [5:4] irq_i,      // Векторное прерывание уровня 5
   output [5:4] istb_o,      // Векторное прерывание уровня 5
   input [8:0] ivec,                 // Шина приема вектора прерывания
   input iack_i                 // Подтверждение приема вектора прерывания
);   
   
//==========================================================================================
wire sw_cont=1'b0;
wire        sys_init;              
                                      
wire [21:0] mmu_adr; 
wire [15:0] wb_out;                    
wire [15:0] wb_mux;                    
wire        cpu_stb;               
wire        wb_ack;                

wire [15:0] cpu_out;       // выходная шина данных процессора   
assign wb_dat_o=cpu_out;

wire [15:0] ccr_dat;             
wire [15:0] kw11l_dat;
wire [15:0] mmu_dat;

wire ccr_ack;             
reg swr_ack;             
reg kw11l_ack;       
wire cpu_ack;      

wire swr_stb;             
wire kw11l_stb;             
wire ccr_stb;
wire global_reset;

reg [15:0] swr_out;

wire [15:0] cpu_psw_in;       // Ввод PSW , записываемого под адресу 177776
wire cpu_psw_we_even;   // Разрешение записи младшего байта PSW
wire cpu_psw_we_odd;    // Разрешение записи старшего байта PSW
wire [15:0] cpu_psw_out;   // Вывод PSW , читаемого под адресу 177776

wire oddabort;
wire [15:0] cpu_stack_limit;
wire [15:0] cpu_pir_in;
wire cpu_ysv;
wire cpu_rsv;
wire cpu_illegal_halt;
wire [7:4] vstb;
wire [7:4] virq;

reg timer_ie;
reg timer_rdy;
reg timer_irq;
wire timer_istb;
wire timer_iack=timer_istb;

wire bus_timeout;
wire run;

assign      sys_init = bus_reset;

assign timer_istb=vstb[6];
assign istb_o[5]=vstb[5];
assign istb_o[4]=vstb[4];

assign virq[7]=1'b0;
assign virq[6]=timer_irq;
assign virq[5]=irq_i[5];
assign virq[4]=irq_i[4];

// шина ввода вектора прерывания в процессор
wire [8:0] vector = (vstb[6])? 9'o100:             // таймер
                    ivec;                          // входной вектор от контроллеров прерывания  

wire [15:0] csw=16'o0;						  
						  
//******************************************
//*  Процессор PDP2011
//******************************************

cpu2011 cpu (
   .wbm_adr_o(cpu_adr), 
   .wbm_dat_o(cpu_out),   
   .wbm_dat_i(wb_mux),   
   .wbm_we_o(wb_we_o),  
   .wbm_sel_o(wb_sel_o),             
   .wbm_stb_o(cpu_stb),  
   .wbm_ack_i(cpu_ack),    
   .cp(cpu_cp),           // 1 - обмен по шине происходит в предыдущем режиме процессора
   .ifetch(ifetch),   // признак выборки инструкции
   .id(cpu_id),           // 1 - обращение к данным
	.oddabort(oddabort),

   // прерывания
   .virq(virq), 
   .vector(vector),      // ввод адреса вектора
   .vstb(vstb),
   .vack(iack_i | timer_iack),     // подтверждение приема вектора
   .pirq(cpu_pir_in),     // Регистр PIRQ, формирователь программных прерываний
    
    // линии прерываний от MMU
   .mmutrap(mmutrap),          // запрос прерывания от MMU по окончании обработки текущей инструкции
   .ack_mmutrap(ack_mmutrap), // подтверждение прерывания от MMU
   .mmuabort(mmuabort),         // запрос от MMU на отмену обработки текущей инструкции
   .ack_mmuabort(ack_mmuabort),// подтверждение отмены обработки инструкции
   
   // DMA
   .npr(dma_req),              // запрос DMA
   .npg(dma_ack),         // подтверждение DMA

   // Флаги ошибок и состояния процессора
   .illhalt(cpu_illegal_halt),     // признак недопустимости инструкции HALT в текущем режиме процессора
	.bus_timeout(bus_timeout),
   .ysv(cpu_ysv),             // желтое состояние стека
   .rsv(cpu_rsv),             // красное состояние стека
   .cpu_stack_limit(cpu_stack_limit), // значение нижней границы стека 
   .sr0_ic(sr0_ic), // sr0/mmr0 IC - флаг завершения декодирования инструкции
   .sr1(sr1),       // sr1/mmr1 - адрес текущей инструкции
   .sr2(sr2),       // sr2/mmr2 - информация об автоинкременте/декременте регистров в текущей инструкции
   .dstfreference(dstfreference), 
	.dw8(dw8),

	// PSW
   .psw_in(cpu_psw_in),     // Ввод PSW , записываемого под адресу 177776
   .psw_in_we_even(cpu_psw_we_even),   // Разрешение записи младшего байта PSW
   .psw_in_we_odd(cpu_psw_we_odd),    // Разрешение записи старшего байта PSW
   .psw_out(cpu_psw_out),   // Вывод PSW , читаемого под адресу 177776

	.sw_cont(sw_cont),
   .iwait(led_idle),       // 1 - процессор стоит на команде wait и ждет прерывания
   .run(led_run),
	
   .fpu_enable(`fpu_present),         // Признак наличия FPU в схеме процессора

   .init_r7(16'o164020),    // Адрес старта по сбросу
   .init_psw(16'o340),   // Стартовое PSW

   .clk(clk_p),              // тактовый синхросигнал
   .reset(dclo),             // сброс процессора
   .init(bus_reset)     // выход сброса для всей периферии на шине
);

//******************************************
//*  Управляющие регистры процессора
//******************************************
cpu_control_regs ccr(

   .wb_clk_i(clk_p),
   .wb_rst_i(sys_init),
   .wb_adr_i(wb_adr_o[4:0]),
   .wb_dat_i(cpu_out),
   .wb_dat_o(ccr_dat),
   .wb_cyc_i(1'b1),
   .wb_we_i(wb_we_o),
   .wb_stb_i(ccr_stb),
   .wb_ack_o(ccr_ack),
   .wb_sel_i(wb_sel_o),

   .psw_in(cpu_psw_in),                // Ввод PSW , записываемого под адресу 177776
   .psw_in_we_even(cpu_psw_we_even),   // Разрешение записи младшего байта PSW
   .psw_in_we_odd(cpu_psw_we_odd),     // Разрешение записи старшего байта PSW
   .psw_out(cpu_psw_out),              // Вывод PSW , читаемого под адресу 177776
	
   .cpu_stack_limit(cpu_stack_limit),  // нижняя граница стека, для схем контроля стека
   .pir_in(cpu_pir_in),                // сигналы программных прерываний
	
	// сигналы регистра ошибок
   .cpu_illegal_halt(cpu_illegal_halt), 
   .cpu_address_error(oddabort), 
   .cpu_nxm(bus_timeout & ram_stb), 
   .cpu_iobus_timeout(bus_timeout & ~ram_stb), 
   .cpu_ysv(cpu_ysv), 
   .cpu_rsv(cpu_rsv) 
);

//*******************************************
//*  Дспетчер памяти
//*******************************************
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

mmu mmu1(

   // Интерфейс к CPU
   .cpu_addr_v(cpu_adr),    // виртуальный 16-битный адрес
   .mmu_dat_o(mmu_dat),   // выход локальной шины данных
   .mmu_dat_i(cpu_out),   // вход локальной шины данных
   .cpu_we(wb_we_o),    // признак цикла записи на шину
   .cpu_sel(wb_sel_o),   // выбор байтов для записи
   .cpu_stb(cpu_stb),   // строб обмена по шине
   .cpu_ack(cpu_ack),   // подтверждение транзакции (REPLY)
   .cpu_cp(cpu_cp),              // 1 - обмен по шине происходит в предыдущем режиме процессора
   .psw(cpu_psw_out),           // текущий PSW
   .id(cpu_id),                  // 0-обращение к инструкции, 1 - к данным
	
	// Линии прерываний
   .mmutrap(mmutrap),        // запрос прерывания от MMU по окончании обработки текущей инструкции
   .ack_mmutrap(ack_mmutrap),         // подтверждение
	
   .mmuabort(mmuabort),           // запрос от MMU на отмену обработки текущей инструкции
   .ack_mmuabort(ack_mmuabort),        // подтверждение
	
   .mmuoddabort(mmuoddabort),        // прерывание из-за словного обращения по нечетному адресу
	
	// регистры MMR
   .sr0_ic(sr0_ic),              // регистр MMR0, бит 7 (IC) - флаг завершения декодирования инструкции
   .sr1_in(sr1),        // регистр MMR1 - информация об автоинкременте/автодекременте текущей инструкции
   .sr2_in(sr2),        // регистр MMR2 - виртуальный адрес текущей инструкции
	
   .dstfreference(dstfreference),       // 1 - адрес приемника данных поностью сформирован
   .ifetch(ifetch),              // 1 - идет выборка инструкции
	.cpu_dw8(dw8),                // 1- байтовый обмен
	
	// Интерфейс к внешней шине
   .mmu_adr_o(wb_adr_o),     // выходная шина адреса к блоку RAM
   .RAM_stb_o(ram_stb),
	.BUS_stb_o(bus_stb),
	.wb_ack_i(wb_ack),
		
	// Шины DMA-режима
   .DMA_gnt(dma_ack),     // подтверждение DMA
   .DMA_addr_i(dma_adr18),    // ввод адреса в режиме DMA
	.DMA_stb_i(dma_stb),       // строб транзакции DMA
	
	// индикаторы режима работы
//   .cons_map16(cons_map16),                      // режим 16-битного адреса
//   .cons_map18(cons_map18),                      // режим 18-битного адреса 
//   .cons_map22(cons_map22),                      // режим 22-битного адреса
//   .cons_id(cons_id),                         // режим разделения I/D
//	.cons_ubm(cons_ubm),                        // режим Unibus Mapping
	
   .reset(sys_init),                          // общий сброс
   .clk(clk_p)                               // синхросигнал
);

//*******************************************
//* ПЗУ монитора-загрузчика
//*******************************************
wire bootrom_stb;
wire bootrom_ack;
wire [15:0] bootrom_dat;
reg [1:0]bootrom_ack_reg;

// эмулятор пульта и набор загрузчиков - ПЗУ  164000-165777
boot_rom bootrom(
   .address(wb_adr_o[9:1]),
   .clock(clk_p),
   .q(bootrom_dat));

always @ (posedge clk_p) begin
   bootrom_ack_reg[0] <= bootrom_stb & ~wb_we_o;
   bootrom_ack_reg[1] <= bootrom_stb & ~wb_we_o & bootrom_ack_reg[0];
end
assign bootrom_ack = bus_stb & bootrom_ack_reg[1];



//************************************************
//* Регистр переключателей/индикации 177570
//************************************************

wire [15:0] swr_dat=csw; 

always @ (posedge clk_p) 
  if ((swr_stb == 1'b1) && (wb_we_o == 1'b1)) swr_out <= cpu_out;

wire swr_reply= swr_stb & ~swr_ack;
always @(posedge clk_p)
    if (sys_init == 1'b1) swr_ack <= 1'b0;
    else swr_ack <= swr_reply;

//*************************************************************************
//* Генератор прерываний от таймера
//* Сигнал имеет частоту 50 Гц и ширину импульса в 1 такт
//*************************************************************************
reg timer_50;
reg [20:0] timercnt;

wire [20:0] timer_limit=31'd`clkref/6'd50-1'b1;

always @ (posedge clk_p) begin
  if (timercnt == timer_limit) begin
     timercnt <= 21'd0;
     timer_50 <= 1'b1;
  end  
  else begin
     timercnt <= timercnt + 1'b1;
     timer_50 <= 1'b0;
  end     
end

//************************************************
//* Сетевой таймер KW11-L 177546
//************************************************
reg tirq_prev_state;  // состояние таймера в предыдущем такте
assign kw11l_dat = {8'o0, timer_rdy, timer_ie, 6'o0};

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
        timer_ie  <= cpu_out[6];
	     timer_rdy <= cpu_out[7];
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
assign swr_stb    = bus_stb & (wb_adr_o[15:1] == (16'o177570 >> 1));   // SWR - 177570
assign kw11l_stb  = bus_stb & (wb_adr_o[15:1] == (16'o177546 >> 1));   // KW11-L - 177546
assign ccr_stb = bus_stb & (wb_adr_o[15:5] == 11'b11111111111);        // 177740 - 177777 (на полной шине - 777740-777777) - внутренние регистры процессора
assign bootrom_stb   = bus_stb & (wb_adr_o[15:10] == 6'o72);           // ROM 164000-165776  

assign wb_ack     = global_ack | ccr_ack | swr_ack | kw11l_ack | bootrom_ack;
assign wb_mux     = mmu_dat | wb_dat_i
                  | (ccr_stb ? ccr_dat : 16'o000000)
                  | (kw11l_stb ? kw11l_dat : 16'o000000)
                  | (swr_stb ? swr_dat : 16'o000000)
                  | (bootrom_stb ? bootrom_dat : 16'o000000);

endmodule
