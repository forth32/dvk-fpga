//====================================================
//  Модуль диспетчера памяти процессора PDP2011
//====================================================
module mmu (

   // Интерфейс к CPU
   input[15:0] cpu_addr_v,      // виртуальный 16-битный адрес
   output[15:0] mmu_dat_o,      // выход локальной шины данных
   input[15:0] mmu_dat_i,       // вход локальной шины данных
   input        cpu_we,         // признак цикла записи на шину
   input [1:0]  cpu_sel,        // выбор байтов для записи
   input        cpu_stb,        // строб обмена по шине
   output reg   cpu_ack,        // подтверждение транзакции (REPLY)
   output reg [21:0] mmu_adr_o, // выходная 21-битная шина адреса 

   input cpu_cp,                // 1 - обмен по шине происходит в предыдущем режиме процессора
   input[15:0] psw,             // текущий PSW
   input id,                    // 0-обращение к инструкции, 1 - к данным
   
   // Линии прерываний
   output reg mmutrap,        // запрос прерывания от MMU по окончании обработки текущей инструкции
   input ack_mmutrap,         // подтверждение
   
   output mmuabort,           // запрос от MMU на отмену обработки текущей инструкции
   input ack_mmuabort,        // подтверждение
   
   output mmuoddabort,        // прерывание из-за словного обращения по нечетному адресу
   
   // регистры MMR
   input sr0_ic,              // регистр MMR0, бит 7 (IC) - флаг завершения декодирования инструкции
   input[15:0] sr1_in,        // регистр MMR1 - информация об автоинкременте/автодекременте текущей инструкции
   input[15:0] sr2_in,        // регистр MMR2 - виртуальный адрес текущей инструкции
   
   input dstfreference,       // 1 - адрес приемника данных поностью сформирован
   input ifetch,              // 1 - идет выборка инструкции
   input cpu_dw8,
   output dma_iopage_mapped,  // Признак доступа к странице ввода-вывода
   
   // Cигналы управления обменом
   output reg RAM_stb_o,
   output reg BUS_stb_o,
   input wb_ack_i,
      
   // Шины DMA-режима
   input DMA_gnt,     // подтверждение DMA
   input[17:0] DMA_addr_i,      // ввод 18-битного адреса в режиме DMA
   input DMA_stb_i,
   
   // индикаторы режима работы
   output cons_map18,                      // режим 18-битного адреса 
   output cons_map22,                      // режим 22-битного адреса
   output cons_id,                         // режим разделения I/D
   output cons_ubm,                        // режим Unibus Mapping
   
   input reset,                            // общий сброс
   input clk                               // синхросигнал
);
//--------------------------------------------------------------------------------------------------------------------------------

// Регистры MMR/SR
reg[15:0] sr0; 
reg[15:0] sr1; 
reg[15:0] sr2; 
reg[5:0] sr3; 

wire id_current;         // тип обращения к данным - I или D для текущего шинного цикла 
wire[1:0] psw_mmumode;   // текущий режим процессора - kernel, user, supervisor
wire[15:0] PAR;          // содержимое текущего PAR
wire[15:0] PDR;          // содержимое текущего PDR
reg [15:0] mmu_dataout;  // локальная выходная шина данных
wire[21:0] addr_p;       // полный сформированный физический адрес
wire[21:0] addr_full;    // полный сформированный физический адрес с учетом unibus
wire[17:6] addr_p18;     // старшая часть 18-битного адреса
wire[21:6] addr_p22;     // старшая часть 22-битного адреса 
reg mmu_stb;             // строб  доступа к локальным регистра MMU
wire mmu_ack;            

// биты W всех регистров PDR
reg kpdr_w[15:0]; 
reg spdr_w[15:0]; 
reg updr_w[15:0]; 
// биты A всех регистров PDR
reg kpdr_a[15:0]; 
reg spdr_a[15:0]; 
reg updr_a[15:0]; 

// признаки страничных ошибок
wire abort_nonresident; 
wire abort_pagelength; 
wire abort_readonly; 
wire trap_flag; 
wire oddaddress; 
reg abort_acknowledged; 
reg block_abort;

// флаги режимов работы
wire mmu_enabled=sr0[0];             // признак включения MMU
wire a22_enabled=sr3[4] & mmu_enabled;    // признак работы в 22-битном режиме
wire ubm_enabled=sr3[5] & a22_enabled;    // признак включения Unibus Mapping
wire maint_mode=sr0[8];             // диагностический режим работы MMU

// Состояние машины управления внешней шиной
parameter[1:0] io_idle = 0 ;
parameter[1:0] io_ack = 1 ;
parameter[1:0] io_reply = 2 ;

reg [1:0] io_state;      

// подсистема отображения пространства 18-битного UNIBUS на физический адрес (unibus mapping)
wire[4:0] UBM_dma_reg;  // текущий используемый UBM-регистр в режиме DMA
wire[4:0] UBM_mmu_reg;  // текущий используемый UBM-регистр при доступе со стороны процессора
wire[21:0] ubm_offset;            
wire[21:0] ubm_dma_offset;        
wire[21:0] ubm_paddr; 
wire unibus_area;
wire unibus_mapped;
wire iopage_access;
wire ram_access;

// адресные регистры UBM, 32 регистра
reg[7:1] UBM_l[31:0];   // младший байт
reg[7:0] UBM_m[31:0];   // средний байт
reg[5:0] UBM_h[31:0];   // старший байт

// Набор регистров PAR, 8 регистров I и 8 регистров D
//   старшие байты
reg[7:0] par_h_kernel[15:0]; 
reg[7:0] par_h_supervisor[15:0]; 
reg[7:0] par_h_user[15:0]; 
//   младшие байты
reg[7:0] par_l_kernel[15:0]; 
reg[7:0] par_l_supervisor[15:0]; 
reg[7:0] par_l_user[15:0]; 

// Набор регистров PDR, 8 регистров I и 8 регистров D
//   старшие байты
reg[6:0] pdr_h_kernel[15:0]; 
reg[6:0] pdr_h_supervisor[15:0]; 
reg[6:0] pdr_h_user[15:0]; 
//   младшие байты
reg[3:0] pdr_l_kernel[15:0]; 
reg[3:0] pdr_l_supervisor[15:0]; 
reg[3:0] pdr_l_user[15:0]; 


// Сигналы управления обменом с процессором

wire MMU_select;
wire cpu_rd = cpu_stb & (~cpu_we);  // строб чтения
wire cpu_wr = cpu_stb & cpu_we;     // строб записи
wire MMU_reg_select;

// выходной сигнал подтверждения обмена
wire bus_ack = mmu_ack | wb_ack_i;

// Индикаторы текущего режима адресации
assign cons_map18 = mmu_enabled & (~a22_enabled);  // 18-битный адрес
assign cons_map22 = mmu_enabled & a22_enabled;     // 22-битный адрес
assign cons_id = id_current;                       // разделение I/D
assign cons_ubm=ubm_enabled;                       // Unibus Mapping, преобразование адресов 18-битной шины 

// Определение используемого режима процессора
//                                        текущий     предыдущий
assign psw_mmumode = (cpu_cp == 1'b0) ? psw[15:14] : psw[13:12] ;

// Определение, работает ли процессор с данными (1) или инструкцией (0)
assign id_current = (psw_mmumode == 2'b00 & (sr3[2]) == 1'b1) ? id :   // режим KERNEL
                    (psw_mmumode == 2'b01 & (sr3[1]) == 1'b1) ? id :   // режим SUPERVISOR
                    (psw_mmumode == 2'b11 & (sr3[0]) == 1'b1) ? id :   // режим USER
                                                              1'b0 ;   // I/D отключен

// Определение номера текущего PAR и PDR
//                      I/D       APF - номер PR из виртуального адреса
wire[3:0] apr_adr = {id_current, cpu_addr_v[15:13]} ;

// Содержимое текущего PAR
assign PAR=(psw_mmumode==2'b00)? {par_h_kernel[apr_adr],     par_l_kernel[apr_adr]} :  {16{1'b0}}    // KERNEL
          |(psw_mmumode==2'b01)? {par_h_supervisor[apr_adr], par_l_supervisor[apr_adr]} : {16{1'b0}} // SUPERVISOR    
          |(psw_mmumode==2'b11)? {par_h_user[apr_adr],       par_l_user[apr_adr]} :    {16{1'b0}} ;   // USER
          
// Содержимое текущего PDR
assign PDR =  (psw_mmumode==2'b00)? {1'b0, pdr_h_kernel[apr_adr], 4'b0000, pdr_l_kernel[apr_adr]} :        {16{1'b0}} // KERNEL
            | (psw_mmumode==2'b01)? {1'b0, pdr_h_supervisor[apr_adr], 4'b0000, pdr_l_supervisor[apr_adr]}: {16{1'b0}} // SUPERVISOR    
            | (psw_mmumode==2'b11)? {1'b0, pdr_h_user[apr_adr], 4'b0000, pdr_l_user[apr_adr]}:             {16{1'b0}} // USER
            | (psw_mmumode==2'b10)?                                                   16'o177777 :           {16{1'b0}};// в неправильном режиме из PDR читаются все 1

//***************************************************************************
//*  Подсистема отображения адресов Unibus Mappingв режиме DMA
//***************************************************************************

// Выбор номера регистра отображения UNIBUS из 18-битного unibus-адреса
assign UBM_dma_reg = DMA_addr_i[17:13];    // идет DMA - адрес беретсяс шины
                                                                               // нет DMA - адрес берется из MMU
    
// Смещение адреса unibus, получаемое из текущего выбранного адресного регистра UBM         
assign ubm_dma_offset={UBM_h[UBM_dma_reg], UBM_m[UBM_dma_reg], UBM_l[UBM_dma_reg], 1'b0};

// Признак доступа к странице ввода-вывода  -760000 - 777777
assign dma_iopage_mapped = (DMA_addr_i[17:13] == 5'b11111);

// Полный физический адрес после отображения UNIBUS 
assign ubm_paddr = (dma_iopage_mapped)?  {9'b111111111, DMA_addr_i[12:0]} :    // доступ к странице ввода-вывода
                   (ubm_enabled)?         ubm_dma_offset+DMA_addr_i[12:0] :    // доступ к пространству ОЗУ в режиме UBM
                                         {4'b0000, DMA_addr_i[17:0]};          // доступ к пространству ОЗУ без UBM

                       
//***************************************************************************
//*  Преобразование виртуального 16-битного адреса в физический 22-битный
//***************************************************************************

// младшие 5 бит физического адреса - просто копируем из виртуального, это смещение внутри блока
assign addr_p[5:0] = cpu_addr_v[5:0] ;  

// старшая часть адреса для разных  разрядностей адреса - номер первого блока страницы
assign addr_p18 = PAR[11:0] + {5'b00000, cpu_addr_v[12:6]} ;    // 18-битный адрес блока
assign addr_p22 = PAR   + {9'b000000000, cpu_addr_v[12:6]} ;    // 22-битный адрес блока без учета ubnibus-отображения

// Старшая часть физического адреса
assign addr_p[21:6] = 
   // *** MMU включен, рабочий режим ***
                      // 18-битный режим, доступ к пространству ОЗУ
                      (mmu_enabled & (~maint_mode) & (~a22_enabled) & (addr_p18[17:13] != 5'b11111)) ? {4'b0000, addr_p18} : {16{1'b0}}
                      //  18-битный режим, доступ к странице ввода-вывода
                   |  (mmu_enabled & (~maint_mode) & (~a22_enabled)  & (addr_p18[17:13] == 5'b11111)) ? {4'b1111, addr_p18} : {16{1'b0}} 
                       // 22-битный режим - адрес уже сформирован 
                   |    (mmu_enabled & (~maint_mode) & a22_enabled) ? addr_p22 : {16{1'b0}}
   // *** Cлужебный режим отладки ***
                       // 18-битный режим, доступ к пространству ОЗУ
                   |    (maint_mode & dstfreference & (~a22_enabled) & (addr_p18[17:13] != 5'b11111)) ? {4'b0000, addr_p18} : {16{1'b0}}
                       // 18-битный режим, доступ к странице ввода-вывода 
                   |    (maint_mode & dstfreference & (~a22_enabled) & (addr_p18[17:13] == 5'b11111)) ? {4'b1111, addr_p18} : {16{1'b0}}
                      // 22-битный режим
                   |    (maint_mode & dstfreference & a22_enabled) ? addr_p22 : {16{1'b0}}
   // *** MMU отключен, служебный режим отключен ***            
                      // адресация памяти   
                   |    (~mmu_enabled & ~(maint_mode & dstfreference) & (cpu_addr_v[15:13] != 3'b111)) ? {6'b000000, cpu_addr_v[15:6]} :  {16{1'b0}}
                      // адресация страницы ввода-вывода
                   |    (~mmu_enabled & ~(maint_mode & dstfreference) & (cpu_addr_v[15:13] == 3'b111)) ? {6'b111111, cpu_addr_v[15:6]} : {16{1'b0}} ;                                  

// Признак доступа к странице ввода-вывода
assign iopage_access = (addr_p[21:13] == 9'b111111111);

// Подсистема UNIBUS Mapping
//----------------------------
// Признак доступа к памяти через область отображения UBM
assign unibus_area = (addr_p22[21:18] == 4'b1111) & ~iopage_access;
// Признак доступа к памяти через область отображения UBM при включенном механизме UBM
assign unibus_mapped = unibus_area & ubm_enabled;
// Выбор номера регистра отображения UNIBUS из 18-битного unibus-адреса
assign UBM_mmu_reg = addr_p22[17:13];    // номер регистра UBM из сформированного в mmu адреса
// Смещение адреса unibus, получаемое из текущего выбранного адресного регистра UBM         
assign ubm_offset={UBM_h[UBM_mmu_reg], UBM_m[UBM_mmu_reg], UBM_l[UBM_mmu_reg], 1'b0};

// полный выходной адрес, выставляемый на шину - с учетом UBM
assign addr_full= (unibus_mapped)?  
   {9'b000000000, addr_p[12:0]} + ubm_offset: // при обращении через UBM
   addr_p;                                      // без использования UBM

// Признак доступа к RAM   
assign ram_access = (addr_full[21:18] != 4'b1111);
                   
//***************************************************************************
//*  Флаги ошибочных ситуаций
//***************************************************************************

// Признак полной недоступности страницы
assign abort_nonresident =  mmu_enabled  &  // MMU включен
                            cpu_stb &  // идет обмен по шине
                            (PDR[2:0] == 3'b000 | PDR[2:0] == 3'b011 | PDR[2:0] == 3'b111); // страница имеет флаг доступа not resident или ошибочный флаг

// Ошибка выхода за пределы страницы
assign abort_pagelength =     cpu_stb &  // идет обмен по шине
                              mmu_enabled &  // MMU включен
                              ((PDR[3] == 1'b0 & (cpu_addr_v[12:6] > PDR[14:8])) |     // ED=0 - расширение вверх. Номер блока больше размера страницы
                               (PDR[3] == 1'b1 & (cpu_addr_v[12:6] < PDR[14:8])));     // ED=1 - расширение вниз. Номер блока меньше размера страницы

// Признак записи в защищенную от записи страницу
assign abort_readonly = cpu_wr  &    // идет запись
                        mmu_enabled  &   // MMU включен
                        (PDR[2:0] == 3'b001 | PDR[2:0] == 3'b010);  // нет доступа или только чтение

// Признак нечетного адреса при словном обмене                    
assign oddaddress = ~cpu_dw8 & cpu_addr_v[0];
// Строб ошибки обращения по нечетному адресу
assign mmuoddabort = cpu_stb & oddaddress;
// Флаг наличия ошибочной ситуации
wire abort_state=(abort_nonresident|abort_pagelength|abort_readonly) & ~block_abort;

// Формирователь сигнала отмены обработки текущей инструкции (MMU Abort)
assign mmuabort = ~mmuoddabort &          // Запрет, если возникла ситуация словного доступа по нечетному адресу - эта ситуация более приоритетна
                  abort_state             // по одному из трех ошибочных событий
                  & ~abort_acknowledged;  // пока процессор не подтвердил прием прерывания

// Запрос прерывания от MMU, не отменяющего текущую инструкцию (MMU Trap)
assign trap_flag = (mmu_enabled       // MMU включен
                 & ~MMU_reg_select    // идет обращение не к регистрам MMU
                  &( 
                //---------------------------------------
                  (cpu_rd   // идет чтение
                     & ((PDR[2:0] == 3'b001) | (PDR[2:0] == 3'b100)))  // доступ R/O или R/W c прерыванием
                 //---------------------------------------      
               | (cpu_wr    // идет запись
                      & ((PDR[2:0] == 3'b100) | (PDR[2:0] == 3'b101)))  // R/W с прерыванием на запись или на любой доступ
                 //---------------------------------------      
                 ));


//***************************************************************************
//*  Интерфейс CPU <-> MMU
//***************************************************************************

// Сигнал выбора регистров PAR/PDR
wire APR_select = iopage_access &
                 ((addr_p[12:6] == 7'o176)      // User, 777600-777677
                | (addr_p[12:7] == 6'b101001)); // Kernel+Supervisor, 772200-772360
                
// Сигнал выбора регистров отображения Unibus Mapping 1770200-1770377                        
wire UBM_select=  iopage_access & (addr_p[12:7] == 6'b100001);     

// Регистры MMR/SR
wire MMR0_select = iopage_access & (addr_p[12:1] == 13'o17572>>1);
wire MMR1_select = iopage_access & (addr_p[12:1] == 13'o17574>>1);
wire MMR2_select = iopage_access & (addr_p[12:1] == 13'o17576>>1);
wire MMR3_select = iopage_access & (addr_p[12:1] == 13'o12516>>1);

// Строб доступа к регистрам PAR, PDR, MMR
assign MMU_reg_select = APR_select | MMR0_select | MMR1_select| MMR2_select| MMR3_select;
// Строб доступа ко всем внутренним регистрам модуля MMU
assign MMU_select = MMU_reg_select | UBM_select;

// Выходная шина данных, bus -> CPU
assign mmu_dat_o = (mmu_stb) ? mmu_dataout : 16'o0; 

// формирователь ответа на цикл шины   
reg reply;
always @(posedge clk)
    if (reset == 1'b1) reply <=1'b0;
    else if (mmu_stb) reply <= 1'b1;
    else reply <=1'b0;
    
assign mmu_ack = mmu_stb & reply;
                        
//***********************************
//*  Обработка шинных транзакций
//***********************************
always @(posedge clk) begin
   if (reset)    begin
      // сброс
      abort_acknowledged <= 1'b0 ; 
      mmutrap <= 1'b0 ; 
      sr0[15:0] <= 16'b0000000000000000 ; 
      sr1 <= sr1_in ; 
      sr2 <= sr2_in ; 
      sr3 <= 6'b000000 ; 
      block_abort <= 1'b0;
      RAM_stb_o <= 1'b0;
      BUS_stb_o <= 1'b0;
      mmu_stb <= 1'b0;
      cpu_ack <= 1'b0;
      io_state <= io_idle;
   end
   else begin
      //**********************************************
      //* подтверждение прерывания и его блокировка
      //**********************************************
      if (ifetch == 1'b1) begin
        abort_acknowledged <= 1'b0 ;  // подтверждение аборта снимается при выборе новой инструкции
        block_abort <= 1'b0;
      end  

      //**************************
      //*  Прерывания
      //**************************

      // подтверждение прерывания MMU Abort
      if (ack_mmuabort == 1'b1)  begin
         sr1 <= sr1_in ;    // обновляем регистр MMR1
         abort_acknowledged <= 1'b1 ; // снимаем запрос прерывания
      end 
      // запрос прерывания MMU Trap
      if (mmuabort) mmutrap <= 1'b0;   // снимаем при наступлении аборта - трап и аборт в одной инструкции несовместимы
      else  if (trap_flag & sr0[9] & ~sr0[12])  mmutrap <= 1'b1 ; 
      // Снятие запроса после подтверждения прерывания
      if (ack_mmutrap == 1'b1)  mmutrap <= 1'b0 ; 
      
      //************************************
      //*  Обновления регистра состояния
      //***********************************
      if ((trap_flag | mmuabort) & sr0[15:13] == 3'b000) begin
         // если флаги ошибок еще не выставлены
         abort_acknowledged <= 1'b0 ; 
         sr2 <= sr2_in ;          
         if (mmu_enabled) begin
            // если MMU включен
            // устанавливаем флаги состояния в регистре MMR0
            sr0[15] <= abort_nonresident ; 
            sr0[14] <= abort_pagelength ; 
            sr0[13] <= abort_readonly ; 
            sr0[12] <= trap_flag ;    
            sr0[7] <= sr0_ic & (~mmuabort);       // завершена обработка инструкции
            if (cpu_stb) begin
               sr0[6:5] <= psw_mmumode ;    // режим процессора в момент ошибки
               sr0[4] <= id_current ;       // I/D в момент ошибки
               sr0[3:1] <= cpu_addr_v[15:13] ;  // номер страницы в момент ошибки
            end 
         end 
      end

      //**************************
      // обработка бита W      
      //**************************
      if (mmu_enabled== 1'b1 &     // MMU включен 
          cpu_wr == 1'b1 &    // идет запись
          APR_select == 1'b0 &   // выбрано что-то кроме местных регистров
          // и ошибок никаких нет 
          abort_nonresident == 1'b0 & 
          abort_pagelength == 1'b0 & 
          abort_readonly == 1'b0)  begin
         case (psw[15:14])
            // поднимаем бит W
            2'b00 : kpdr_w[{id_current, cpu_addr_v[15:13]}] <= 1'b1 ; 
            2'b01 : spdr_w[{id_current, cpu_addr_v[15:13]}] <= 1'b1 ; 
            2'b11 : updr_w[{id_current, cpu_addr_v[15:13]}] <= 1'b1 ; 
         endcase 
      end 
      
      //**************************
      // обработка бита А
      //**************************
      if (mmu_enabled == 1'b1 &           // MMU включен 
          APR_select == 1'b0 &   // выбрано что-то кроме местных регистров
          // ошибок не обнаружено
          abort_nonresident == 1'b0 & 
          abort_pagelength == 1'b0 & 
          abort_readonly == 1'b0 & 
            ((PDR[2:0] == 3'b001 & cpu_rd ) |   // чтение с трапом 
             (PDR[2:0] == 3'b100 & cpu_stb) |  // чтение/запись с трапом
             (PDR[2:0] == 3'b101 & cpu_wr ))   // запись с трапом
             ) begin
         case (psw[15:14])
            // поднимаем бит А
            2'b00 :  kpdr_a[{id_current, cpu_addr_v[15:13]}] <= 1'b1 ; 
            2'b01 :  spdr_a[{id_current, cpu_addr_v[15:13]}] <= 1'b1 ; 
            2'b11 :  updr_a[{id_current, cpu_addr_v[15:13]}] <= 1'b1 ; 
         endcase 
      end 

      //*****************************************
      //* Формирование сигналов выходной шины
      //*****************************************
      
       // обработка шинных транзакций процессора
      if (~DMA_gnt) begin

        case(io_state)
        
         io_idle: begin
            if (cpu_stb) begin
              mmu_adr_o <= addr_full;
              if (~abort_state  & ~oddaddress ) begin // ошибок нет
                // Cтроб доступа к внутренним регистрам
                mmu_stb <= MMU_select;
                // Строб доступа к странице ввода-вывода                       
                BUS_stb_o <= iopage_access & ~MMU_select;
                // Строб доступа к RAM под адресам 00000000 - 16777777 
                RAM_stb_o <= ram_access;
                io_state <= io_ack;
              end   
            end
          end 
         
           io_ack:
            if (bus_ack | ~cpu_stb)  begin
              if (bus_ack) cpu_ack <= 1'b1;
              io_state <= io_reply;
            end
   
         io_reply: begin
              cpu_ack <= 1'b0;
              RAM_stb_o <= 1'b0;
              BUS_stb_o <= 1'b0;
              mmu_stb <= 1'b0;
              cpu_ack <= 1'b0;
              io_state <= io_idle;
           end    
           
         default: io_state <= io_idle;
         endcase          
      end  
      
      // Режим DMA
      else begin
        case(io_state)
         io_idle: begin
            if (DMA_stb_i) begin
               mmu_adr_o <= ubm_paddr;
                // Cтроб доступа к памяти
                if (~dma_iopage_mapped) RAM_stb_o <= 1'b1;
                else BUS_stb_o <= 1'b1;
                io_state <= io_ack;
            end
          end 
         
           io_ack:
            if (bus_ack | ~DMA_stb_i)  begin
              RAM_stb_o <= 1'b0;
              BUS_stb_o <= 1'b0;
              io_state <= io_idle;
            end
   
         default: io_state <= io_idle;
         endcase          
      end  
      
      //======================== Доступ к внутренним регистра MMU =====================
      
      //----------------------- Ч Т Е Н И Е--------------------------------------------------------

      //**************************
      // Чтение регистров PAR/PDR
      //**************************
      if (MMU_select & cpu_rd) begin
       if (APR_select) 
         case (addr_p[11:5])
            12'o2340>>5:
                     begin
                        // 17772340-17772376: PAR, Kernel 
                        mmu_dataout <= {par_h_kernel[addr_p[4:1]], par_l_kernel[addr_p[4:1]]} ; 
                     end
            12'o2240>>5:
                     begin
                        // 17772240-17772276: PAR, Supervisor
                        mmu_dataout <= {par_h_supervisor[addr_p[4:1]], par_l_supervisor[addr_p[4:1]]} ; 
                     end
            12'o7640>>5:
                     begin
                        // 17777640-17777676: PAR,User
                        mmu_dataout <= {par_h_user[addr_p[4:1]], par_l_user[addr_p[4:1]]} ; 
                     end
            12'o2300>>5:   
                     begin
                        // 17772300-17772336: PDR, Kernel
                        mmu_dataout <= {pdr_h_kernel[addr_p[4:1]], kpdr_a[addr_p[4:1]], kpdr_w[addr_p[4:1]], 2'b00, pdr_l_kernel[addr_p[4:1]]} ; 
                     end
            12'o2200>>5:
                     begin
                        // 17772200-17772236: PDR, Supervisor
                        mmu_dataout <= {pdr_h_supervisor[addr_p[4:1]], spdr_a[addr_p[4:1]], spdr_w[addr_p[4:1]], 2'b00, pdr_l_supervisor[addr_p[4:1]]} ; 
                     end
            12'o7600>>5:
                     begin
                        // 17777600-17777636: PDR, User
                        mmu_dataout <= {pdr_h_user[addr_p[4:1]], updr_a[addr_p[4:1]], updr_w[addr_p[4:1]], 2'b00, pdr_l_user[addr_p[4:1]]} ; 
                     end
         endcase        
         
         //**************************************************************************
         //* Чтение 17770200-17770366 - регистры отображения Unibus DMA адресов
         //**************************************************************************
         else if (UBM_select) begin
           if (addr_p[1] == 1'b0) begin
              // чтение младшего слова
              mmu_dataout[0] <= 1'b0 ; 
              mmu_dataout[7:1] <= UBM_l[addr_p[6:2]] ; 
              mmu_dataout[15:8] <= UBM_m[addr_p[6:2]] ; 
           end   
           // чтение старшего слова 
           else   mmu_dataout <= { 10'o0, UBM_h[addr_p[6:2]] }; 
         end 

         //**************************************************************************
         //* Чтение регистров SR/MMR
         //**************************************************************************
         else if (MMR0_select) mmu_dataout <= sr0;   // регистр MMR0/SR0
         else if (MMR1_select) mmu_dataout <= sr1;   // регистр MMR1/SR1
         // регистр MMR2/SR2
         else if (MMR2_select) begin
            if (sr0[15:13] == 3'o0)   mmu_dataout <= sr2_in;   
            else                        mmu_dataout <= sr2;   
         end   
         else if (MMR3_select) mmu_dataout <= {10'b0000000000, sr3};  // регистр MMR3/SR3

      end

      
      //======================= З А П И С Ь =======================================================
      else if (MMU_select & cpu_wr)  begin
        //**************************************************************************
        //* Запись 17770200-17770366 - регистры отображения Unibus DMA адресов
        //**************************************************************************
        if (UBM_select) begin
           if (addr_p[1] == 1'b0)  begin
              // Младший байт
              if (cpu_sel[0]) UBM_l[cpu_addr_v[6:2]] <= mmu_dat_i[7:1] ; 
              // Средний байт
              if (cpu_sel[1]) UBM_m[cpu_addr_v[6:2]] <= mmu_dat_i[15:8] ; 
         end 
            // старший байт
         else  if (cpu_sel[0]) UBM_h[cpu_addr_v[6:2]] <= mmu_dat_i[5:0] ; 
        end 
         
         //**************************************************************************
         //* Запись регистров MMR/SR
         //**************************************************************************
         else if (MMR0_select)   begin
            // SR0/MMR0 - младший байт
            if (cpu_sel[0]) begin
               sr0[0] <= mmu_dat_i[0] ; 
               block_abort <= mmu_dat_i[0];  // блокируем MMU Abort до выборки новой инструкции
            end   
            
            // SR0/MMR0 - старший байт
            if (cpu_sel[1])   begin
               sr0[15:12] <= mmu_dat_i[15:12] ; 
               sr0[9] <= mmu_dat_i[9] ; // разрешение прерывания
               sr0[8] <= mmu_dat_i[8] ; // режим отладки
               // Обновление регистров состояния
               if (mmu_dat_i[15:13] != 3'b000)  begin
                if (mmu_enabled) begin          // если MMU работает
                 sr0[6:5] <= psw_mmumode ;    // режим процессора в момент ошибки
                 sr0[4] <= id_current ;       // I/D в момент ошибки
                 sr0[3:1] <= cpu_addr_v[15:13] ;  // номер страницы в момент ошибки
                end 
                else sr0[6:1] <= 6'o0;  // если MMU отключен
                sr1 <= sr1_in ;
                sr2 <= sr2_in ;          
               end  
               else sr0[6:1] <= mmu_dat_i[6:1];
            end 
         end
         
         else if (MMR3_select)  begin
               // SR3/MMR3
               if (cpu_sel[0])  begin
                  sr3[5:4] <= mmu_dat_i[5:4]; 
                  sr3[2:0] <= mmu_dat_i[2:0]; 
               end   
            end
         
        //**************************************************************************
        //* Запись регистров PAR и PDR
        //**************************************************************************
        else if (APR_select)   
         case (addr_p[11:5])
            12'o2340>>5:
                     begin
                        // 17772340-17772376: PAR0-7, Kernel
                        kpdr_a[addr_p[4:1]] <= 1'b0 ; 
                        kpdr_w[addr_p[4:1]] <= 1'b0 ; 
                        if (cpu_sel[0])  par_l_kernel[addr_p[4:1]] <= mmu_dat_i[7:0] ; 
                        if (cpu_sel[1])  par_h_kernel[addr_p[4:1]] <= mmu_dat_i[15:8] ; 
                     end
            12'o2240>>5:
                     begin
                        // 17772240-17772276: PAR0-7, Supervisor
                        spdr_a[addr_p[4:1]] <= 1'b0 ; 
                        spdr_w[addr_p[4:1]] <= 1'b0 ; 
                        if (cpu_sel[0])  par_l_supervisor[addr_p[4:1]] <= mmu_dat_i[7:0] ; 
                        if (cpu_sel[1])  par_h_supervisor[addr_p[4:1]] <= mmu_dat_i[15:8] ; 
                     end
            12'o7640>>5:
                     begin
                        // 17777640-17777676: PAR0-7,User
                        updr_a[addr_p[4:1]] <= 1'b0 ; 
                        updr_w[addr_p[4:1]] <= 1'b0 ; 
                        if (cpu_sel[0])  par_l_user[addr_p[4:1]] <= mmu_dat_i[7:0] ; 
                        if (cpu_sel[1])  par_h_user[addr_p[4:1]] <= mmu_dat_i[15:8] ; 
                     end
            12'o2300>>5:
                     begin
                        // 17772300-17772336: PDR0-7, Kernel
                        kpdr_a[addr_p[4:1]] <= 1'b0 ; 
                        kpdr_w[addr_p[4:1]] <= 1'b0 ; 
                        if (cpu_sel[0])  pdr_l_kernel[addr_p[4:1]] <= mmu_dat_i[3:0];
                        if (cpu_sel[1])  pdr_h_kernel[addr_p[4:1]] <= mmu_dat_i[14:8] ; 
                     end
            12'o2200>>5:
                     begin
                        // 17772200-17772236: PDR0-7, Supervisor
                        spdr_a[addr_p[4:1]] <= 1'b0 ; 
                        spdr_w[addr_p[4:1]] <= 1'b0 ; 
                        if (cpu_sel[0])  pdr_l_supervisor[addr_p[4:1]] <= mmu_dat_i[3:0];
                        if (cpu_sel[1])  pdr_h_supervisor[addr_p[4:1]] <= mmu_dat_i[14:8] ; 
                     end
            12'o7600>>5:
                     begin
                        // 17777600-17777636: PDR0-7, User
                        updr_a[addr_p[4:1]] <= 1'b0 ; 
                        updr_w[addr_p[4:1]] <= 1'b0 ; 
                        if (cpu_sel[0])  pdr_l_user[addr_p[4:1]] <= mmu_dat_i[3:0]; 
                        if (cpu_sel[1])  pdr_h_user[addr_p[4:1]] <= mmu_dat_i[14:8] ; 
                     end
         endcase 
      end 
   end  
end 

endmodule
