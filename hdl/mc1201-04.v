//
// Процессорная плата МС1201-04(03)
// Оcнована на процессоре 1801ВМ3
// Использовалась в советских ЭВМ ДВК-4
//   Платы 03 и 04 отличаются только объемом ОЗУ - 256К или 1М. В данном проекте объем памяти 
//   может быть 256К, 1М или 4М.
// В отличие от оригинальной платы здесь задействован механизм UMR, позволяющий подключить
// устройства с 18-битным DMA
// 
// ======================================================================================

module mc1201_04 ( 
// Синхросигналы  
   input          clk_p,         // тактовый сигнал, прямая фаза
   input          clk_n,         // тактовый сигнал, инверсная фаза
   input          cpuslow,       // Режим замедления процессора
                                       
// Шина Wishbone                                       
   output [21:0] wb_adr_o,       // выход шины адреса
   output [15:0] wb_dat_o,       // выход шины данных
   input  [15:0] wb_dat_i,       // вход шины данных
   output        wb_we_o,        // разрешение записи
   output [1:0]  wb_sel_o,       // выбор байтов для передачи

   output ram_stb,               // строб обращения к системной памяти
   output bus_stb,               // строб обращения к общей шине
   input  global_ack,            // подтверждение обмена от памяти и устройств страницы ввода-вывода
   
// DMA   
   input  dma_req,               // запрос DMA
   output reg dma_ack,           // подтверждение DMA
   input  [17:0] dma_adr18,      // 18-битный UNIBUS-адрес для устройств, работающих через UMR
   input  dma_stb,               // строб данных для устройств, работающих через UMR
   
// Сбросы и прерывания
   output bus_reset,             // Выход сброса для периферии
   input dclo,                   // Вход сброса процессора
   input aclo,                   // Сигнал аварии питания
   
// Ручное управление
   input [15:0] csw,             // регистр консольных переключателей              
   input resume,                  // кнопка перехода в пультовый режим
   
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




wire [15:0] wb_mux;    // сборная шина данных от периферии к процессору                
wire [15:0] lks_dat;   // шина данных таймера
wire        local_stb; // строб данных от процессора
wire        cpu_stb;   // строб данных на общую шину            
wire        cpu_cyc;   // не уверен что он нужен
wire        ioaccess;  // признак доступа процессора к периферийной шине
wire        um_enable; // признак включения режима UNIBUS MAPPING
wire        mmu_en;    // признак включения режима трансляции адресов
wire        cpu_sel;   // признак включения процессором теневой адресации пультового режима - сигнал SEL

// индикация
wire halt_flag;
assign led_run=halt_flag;
assign led_idle=1'b0;
assign led_mmu=~1'b0;
assign led_timer=~timer_ie;

// сигналы подтверждения обмена
wire wb_ack;    // подтверждения обмена от шины 
reg  lks_ack;   // таймер       
wire rom_ack;   // окно доступа к теневому ПзУ
wire hram_ack;  // окно доступа к теневому ОЗУ
wire cpu_ack;   // подтверждение обмена для транзакций шины, генерируемых процессором
reg um_ack;     // подтверждение доступа к массиву регистров UMR

// стробы выбора периферии
wire lks_stb;  // таймер           
wire um_stb;   // регистры подсистемы unibus mapping
wire rom_stb;  // строб обращения к теневому ПЗУ
wire hram_stb; // строб обращения к теневому OЗУ

// таймер
reg timer_ie;    // разрешение прерывания
reg timer_rdy;   // готовность таймера
wire bevent;     // сигнал запроса прерывания

// шина адреса
wire [21:0] cpu_adr;
// коммутатор шины адреса для режимов CPU/DMA
assign wb_adr_o= dma_ack? um_adr : cpu_adr; // для операций DMA-18 на шину выставляется входной DMA-адрес

// Выбор строба теневого режима
assign cpu_stb = ~cpu_sel & local_stb;
assign halt_stb = cpu_sel & local_stb;

// Прерывания 
wire [7:4] vstb;          // строб приема вектора
wire [7:4] virq;          // запрос прерывания
wire cpu_istb;
wire [15:0]cpu_int_vector;

// подсистема отображения пространства 18-битного DMA-адреса на физический адрес (unibus mapping)
wire[4:0] um_reg;  // текущий используемый UMR-регистр в режиме DMA
wire[21:0] um_offset;            
wire[21:0] um_adr; 
reg[15:0] um_dat;  // шина данных

// адресные регистры UMR, 32 регистра
reg[7:1] um_l[31:0];   // младший байт
reg[7:0] um_m[31:0];   // средний байт
reg[5:0] um_h[31:0];   // старший байт


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
//*  Процессор 1801ВМ3
//*************************************
vm3_wb  #(.VM3_CORE_FIX_SR3_RESERVED(1)) cpu (
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
   .wbm_adr_o({cpu_sel, cpu_adr}),     // полный физический адрес, сформированный процессором
   .wbm_dat_o(wb_dat_o),    // выход шины данных
   .wbm_dat_i(wb_mux),      // вход шины данных
   .wbm_cyc_o(cpu_cyc),     
   .wbm_we_o(wb_we_o),      // флаг вывода данных на шину (0-ввод, 1-вывод)
   .wbm_sel_o(wb_sel_o),    // выбор байтов для записи
   .wbm_stb_o(local_stb),     // строб операции на шине
   .wbm_ack_i(cpu_ack),     // вход подтверждения ввода-вывода (REPLY)
   .vm_umap(um_enable),     // признак включения режима Unibus Mapping

   // шина обработки прерываний
   .wbi_dat_i(cpu_int_vector),    // ввод вектора прерывания от устройства
   .wbi_ack_i(iack_i),   // подтверждение передачи вектора или данных безадресного ввода
   .wbi_stb_o(cpu_istb),          // строб запроса вектора прцессором

   // управление/индикация   
   .vm_halt(resume),       // запрос перехода в пультовый режим
   .vm_bsel(1'b1),         // выбор способа пуска процессора
   .vm_hltm(halt_flag),    // признак включения пультового режима
//   .mmu_en(mmu_en)         // признак включения MMU

);


//*****************************************************
//* Преобразования управляющих сигналов процессора
//*****************************************************

assign ram_stb = dma_ack? dma_stb : cpu_stb & ~ioaccess; // строб доступа к памяти
assign bus_stb = ~dma_ack &         cpu_stb & ioaccess;   // строб доступа к странице ввода-вывода
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
assign cpu_int_vector= {8'h00, ivec};
                  
// формирователь сигналов DMA
always @(posedge clk_p)
   if (dclo) dma_ack <= 1'b0;
   else if (dma_req & ~cpu_stb) dma_ack <= 1'b1; // переход в режим DMA только если процессор не работает с шиной
   else if (~dma_req) dma_ack <= 1'b0;           // снятие запроса DMA
   
//*****************************************************************
//*  Подсистема отображения адресов Unibus Mapping в режиме DMA
//*****************************************************************

// Выбор номера регистра отображения UNIBUS из 18-битного адреса
assign um_reg = dma_adr18[17:13];                                                                            
    
// Смещение адреса unibus, получаемое из текущего выбранного адресного регистра UMR      
assign um_offset={um_h[um_reg], um_m[um_reg], um_l[um_reg], 1'b0};

// Признак доступа к странице ввода-вывода  -760000 - 777777
assign dma_iopage_mapped = 1'b0; //(dma_adr18[17:13] == 5'b11111);

// Полный физический адрес после отображения UNIBUS 
assign um_adr = (dma_iopage_mapped)?  {9'b111111111, dma_adr18[12:0]} :    // доступ к странице ввода-вывода
                 (um_enable)?         um_offset+dma_adr18[12:0] :    // доступ к пространству ОЗУ в режиме Unibus mapping
                                         {4'b0000, dma_adr18[17:0]};          // доступ к пространству ОЗУ без mapping
                                         
// Обработка шинных транзакций
//    17770200-17770366 - регистры отображения Unibus DMA адресов                                         

// Строб выбора регистров отображения Unibus Mapping 1770200-1770377                        
assign um_stb = bus_stb & (wb_adr_o[12:7] == 6'b100001);     

always @ (posedge clk_p) 
    // чтение
    if (um_stb && (~wb_we_o))
        if (wb_adr_o[1] == 1'b0) begin
              // чтение младшего слова
              um_dat[0] <= 1'b0 ; 
              um_dat[7:1] <= um_l[wb_adr_o[6:2]] ; 
              um_dat[15:8] <= um_m[wb_adr_o[6:2]] ; 
        end   
        // чтение старшего слова 
        else   um_dat <= { 10'o0, um_h[wb_adr_o[6:2]] }; 

     // запись
     else if (um_stb && wb_we_o)
        if (wb_adr_o[1] == 1'b0)  begin
              // Младший байт
              if (wb_sel_o[0]) um_l[wb_adr_o[6:2]] <= wb_dat_o[7:1] ; 
              // Средний байт
              if (wb_sel_o[1]) um_m[wb_adr_o[6:2]] <= wb_dat_o[15:8] ; 
         end 
            // старший байт
         else  if (wb_sel_o[0]) um_h[wb_adr_o[6:2]] <= wb_dat_o[5:0] ; 

// сигнал ответа
wire um_reply= um_stb & ~um_ack;
// формирователь ответа       
always @(posedge clk_p)
    if (bus_reset == 1'b1) um_ack <= 1'b0;
    else um_ack <= um_reply;
    
//*******************************************
//* Теневое ПЗУ 134
//*******************************************
wire [15:0] rom_dat;
reg rom_ack0, rom_ack1;
// Строб выбора ПЗУ
assign rom_stb  = halt_stb & (wb_adr_o[12:11] != 2'b11);   // теневой ROM

rom134 hrom(
   .address(cpu_adr[12:1]),
   .clock(clk_p),
   .q(rom_dat)
);

// формирователь cигнала подверждения транзакции с задержкой на 1 такт
always @ (posedge clk_p) begin
   rom_ack0 <= rom_stb;         
   rom_ack1  <= rom_stb & rom_ack0;
end
assign rom_ack=rom_stb & rom_ack1;

//*******************************************
//*  Теневое ОЗУ
//*******************************************
reg[15:0] hram_dat;
reg hram_ack0, hram_ack1;
wire [7:0] hram_adr;
// Строб выбора ОЗУ
assign hram_stb = halt_stb & (wb_adr_o[12:11] == 2'b11);   
// Адресные линии ОЗУ
assign hram_adr=cpu_adr[8:1];

// старший и младший байты теневого ОЗУ
reg [7:0] hram_l[0:255];
reg [7:0] hram_h[0:255];

always @ (posedge clk_p) begin
   if (hram_stb)
     if (wb_we_o) begin
			if (wb_sel_o[0]) hram_l[hram_adr] <= wb_dat_o[7:0];
			if (wb_sel_o[1]) hram_h[hram_adr] <= wb_dat_o[15:8];
	  end	
     else hram_dat <= {hram_h[hram_adr],hram_l[hram_adr]};

end

// формирователь cигнала подверждения транзакции с задержкой на 1 такт
always @ (posedge clk_p) begin
   hram_ack0 <= hram_stb;         
   hram_ack1  <= hram_stb & hram_ack0;
end

assign hram_ack=hram_ack1 & hram_stb;

//*************************************************************************
//*  Подсистема сетевых часов - LTC
//*  Формирует прерывания уровня 6 с частотой 50 Гц
//*************************************************************************

// Генератор прерываний от таймера
// Сигнал имеет частоту 50 Гц и ширину импульса в 1 такт
reg timer_50;    // регистр импульса таймера
reg [20:0] timercnt;  // счетчик делителя частоты таймера
wire [20:0] timer_limit=31'd`clkref/6'd50-1'b1;  // предел счетчика таймера
reg ltcmon;   // бит отслеживания импульсов таймера - LTC monitor

// ДЕлитель частоты и формирователь импульса таймера
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

// Регистр управления таймера (LTC)- LKS

// образ для чтения регистра таймера
//                         D7    D6 
assign lks_dat = {8'o0, ltcmon, timer_ie, 6'o0};
// обработка шинных транзакций
always @ (posedge clk_p) 
  // сброс системы
  if (bus_reset == 1'b1) begin 
     timer_ie <= 1'b0;
     ltcmon <= 1'b1;
  end     
  else begin     
    // обмен с общей шиной
    if (lks_stb == 1'b1) begin
        // запись
        if (wb_we_o == 1'b1)  begin
           timer_ie <= wb_dat_o[6];
           // запись 0 сбрасывает бит программного отслеживания таймера ltcmon
           if (wb_dat_o[7] == 1'b0) ltcmon <= 1'b0; 
        end     
    end
    // взводим бит ltcmon при каждом импульсе таймера
    else if (timer_50 & (~timer_ie)) ltcmon <= 1'b1;  
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

// сигнал ответа
assign wb_ack     = global_ack | lks_ack | rom_ack | um_ack | rom_ack | hram_ack;

// сборная шина входных данных к процессору
assign wb_mux     = wb_dat_i
                  | (lks_stb ? lks_dat : 16'o000000)
                  | (hram_stb ? hram_dat : 16'o000000)
                  | (um_stb ? um_dat : 16'o000000)
                  | (rom_stb ? rom_dat : 16'o000000);

endmodule
