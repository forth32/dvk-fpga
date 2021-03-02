module rk11 (

// шина wishbone
   input                  wb_clk_i,   // тактовая частота шины
   input                  wb_rst_i,   // сброс
   input    [3:0]         wb_adr_i,   // адрес 
   input    [15:0]        wb_dat_i,   // входные данные
   output reg [15:0]      wb_dat_o,   // выходные данные
   input                  wb_cyc_i,   // начало цикла шины
   input                  wb_we_i,    // разрешение записи (0 - чтение)
   input                  wb_stb_i,   // строб цикла шины
   input    [1:0]         wb_sel_i,   // выбор конкретных байтов для записи - старший, младший или оба
   output reg             wb_ack_o,   // подтверждение выбора устройства

// обработка прерывания   
   output reg             irq,         // запрос
   input                  iack,        // подтверждение
   
// DMA
   output reg             dma_req,    // запрос DMA
   input                  dma_gnt,    // подтверждение DMA
   output reg[15:0]       dma_adr_o,  // выходной адрес при DMA-обмене
   input[15:0]            dma_dat_i,  // входная шина данных DMA
   output reg[15:0]       dma_dat_o,  // выходная шина данных DMA
   output reg             dma_stb_o,  // строб цикла шины DMA
   output reg             dma_we_o,   // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   input                  dma_ack_i,  // Ответ от устройства, с которым идет DMA-обмен
   
// интерфейс SD-карты
   output                 sdcard_cs, 
   output                 sdcard_mosi, 
   output                 sdcard_sclk, 
   input                  sdcard_miso, 
   output reg             sdreq,      // запрос доступа к карте
   input                  sdack,      // подтверждение доступа к карте
   input                  sdmode,     // режим SDSPI
   
// тактирование SD-карты
   input                  sdclock,   

// Адрес начала банка на карте
   input [26:0]           start_offset,
   
// отладочные сигналы
   output [3:0]           sdcard_debug
   ); 

   // Сигналы упраления обменом с шиной
   
   wire bus_strobe = wb_cyc_i & wb_stb_i;         // строб цикла шины
   wire bus_read_req = bus_strobe & ~wb_we_i;     // запрос чтения
   wire bus_write_req = bus_strobe & wb_we_i;     // запрос записи
   wire reset=wb_rst_i;
 
   reg interrupt_trigger;     // триггер запроса прерывания
   // состояние машины обработки прерывания
   parameter[1:0] i_idle = 0; 
   parameter[1:0] i_req = 1; 
   parameter[1:0] i_wait = 2; 
   reg[1:0] interrupt_state; 
   
   // регистр данных - rkds - 177400
   reg[2:0] rkds_dri;        // Номер устройства, вызвавшего прерывание
   wire rkds_dpl=1'b0;       // потеря электропитания
   wire rkds_rk05=1'b1;      // признак устройства типа RK05
   wire rkds_dru=1'b0;       // нестабильное состояние
   wire rkds_sin=1'b0;       // позиционирование не закончено - у нас так не бывает
   wire rkds_sok=1'b1;       // sector counter ok
   wire rkds_dry;            // drive ready
   wire rkds_rwsrdy;         // read/write/seek ready
   reg rkds_wps;             // write protected if 1
   reg rkds_scsa;            // disk address = sector counter
   reg[3:0] rkds_sc;         // sector counter
   wire[15:0] rkds; 
   
   // регистр ошибок - rker - 177402
   reg rker_wce;             // write check error
   reg rker_cse;             // checksum error
   reg rker_nxs;             // nx sector
   reg rker_nxc;             // nx cylinder
   reg rker_nxd;             // nx disk
   reg rker_te;              // timing error
   reg rker_dlt;             // data late
   reg rker_nxm;             // nxm
   reg rker_pge;             // programming error
   reg rker_ske;             // seek error
   reg rker_wlo;             // write lockout
   reg rker_ovr;             // overrun
   reg rker_dre;             // drive error
   wire[15:0] rker; 
   
   // регистр управления/состояния - rkcs - 177404
   wire rkcs_err;           // error
   wire rkcs_he;            // hard error
   reg rkcs_scp;            // search complete
   reg rkcs_iba;            // inhibit increment rkba
   reg rkcs_fmt;            // format
   reg rkcs_exb;            // extra bit, unused?
   reg rkcs_sse;            // stop on soft error
   reg rkcs_rdy;            // ready
   reg rkcs_ide;            // interrupt on done enable
   reg[1:0] rkcs_mex;       // bit 18, 17 of address
   reg[2:0] rkcs_fu;        // function code
   reg rkcs_go;             // go
   wire[15:0] rkcs; 
   
   // счетчик пересылаемых слов - rkwc - 177406
   reg[15:0] rkwc; 
   
   // логический адрес буфера в памяти - rkba - 177410
   reg[15:0] rkba; 
   
   // адрес CHS- rkda - 177412
   reg[2:0] rkda_dr; 
   reg[7:0] rkda_cy; 
   reg rkda_hd; 
   reg[3:0] rkda_sc; 
   wire[15:0] rkda;
   
   // регистр данных - rkdb - 177416
   reg[15:0] rkdb; 

   reg start;            // флаг запуска команды
   reg update_rkwc;      // признак обновления счетчика слов
   reg[15:0] wcp;        // счетчик читаемых слов, положительный (не инверсия)
   reg[15:1] ram_phys_addr;  // адрес для DMA-обмена
   reg[11:0] rkclock; 
   reg[1:0] rksi[7:0]; 
   reg scpset; 
   reg[17:0] rkdelay; 
   reg[13:0] rkcs_godelay; 
   reg write_start; 
   reg[15:0] wrkdb; 
   reg [5:0] reply_count;   // таймер ожидания ответа при DMA-обмене
      
   // регистры контроллера DMA
   reg nxm;                    // признак таймаута шины
   reg[8:0] sector_data_index; // указатель текущего слова в секторном буфере
   // машина состояний контроллера
   parameter[3:0] busmaster_idle = 0; 
   parameter[3:0] busmaster_read = 1; 
   parameter[3:0] busmaster_readh = 2; 
   parameter[3:0] busmaster_readh2 = 3; 
   parameter[3:0] busmaster_read1 = 4; 
   parameter[3:0] busmaster_read_done = 5; 
   parameter[3:0] busmaster_write1 = 6; 
   parameter[3:0] busmaster_write = 7; 
   parameter[3:0] busmaster_write_fill = 8; 
   parameter[3:0] busmaster_write_wait = 9; 
   parameter[3:0] busmaster_write_done = 10; 
   parameter[3:0] busmaster_wait = 11; 
   parameter[3:0] busmaster_write_delay = 12; 
   reg[3:0] busmaster_state; 


   // сборка регистров RK11

   assign rkcs_err = (rker_wce == 1'b1 | rker_cse == 1'b1 | rker_nxs == 1'b1 | rker_nxc == 1'b1 | rker_nxd == 1'b1 | rker_te == 1'b1 | rker_dlt == 1'b1 | rker_nxm == 1'b1 | rker_pge == 1'b1 | rker_ske == 1'b1 | rker_wlo == 1'b1 | rker_ovr == 1'b1 | rker_dre == 1'b1) ? 1'b1 : 1'b0 ;
   assign rkcs_he = (rker_nxs == 1'b1 | rker_nxc == 1'b1 | rker_nxd == 1'b1 | rker_te == 1'b1 | rker_dlt == 1'b1 | rker_nxm == 1'b1 | rker_pge == 1'b1 | rker_ske == 1'b1 | rker_wlo == 1'b1 | rker_ovr == 1'b1 | rker_dre == 1'b1) ? 1'b1 : 1'b0 ;
   assign rkcs = {rkcs_err, rkcs_he, rkcs_scp, 1'b0, rkcs_iba, rkcs_fmt, rkcs_exb, rkcs_sse, rkcs_rdy, rkcs_ide, rkcs_mex, rkcs_fu, rkcs_go} ;

   assign rkds_dry = 1'b1;
   assign rkds_rwsrdy = (rksi[rkda_dr] == 0 & sdcard_idle == 1'b1) ? 1'b1 : 1'b0 ;
   assign rkds = {rkds_dri, rkds_dpl, rkds_rk05, rkds_dru, rkds_sin, rkds_sok, rkds_dry, rkds_rwsrdy, rkds_wps, rkds_scsa, rkds_sc} ;

   assign rkda = {rkda_dr, rkda_cy, rkda_hd, rkda_sc} ;

   assign rker = {rker_dre, rker_ovr, rker_wlo, rker_ske, rker_pge, rker_nxm, rker_dlt, rker_te, rker_nxd, rker_nxc, rker_nxs, 3'b000, rker_cse, rker_wce} ;

   // интерфейс к SDSPI
   wire [26:0] sdcard_addr;       // адрес сектора карты
   wire sdcard_read_done;         // флаг окончагия чтения
   wire sdcard_write_done;        // флаг окончания записи
   wire sdcard_error;             // флаг ошибки
   wire [15:0] sdcard_xfer_out;   // слово; читаемое из буфера чтения
   wire sdcard_idle;              // признак готовности контроллера
   reg sdcard_read_start;         // строб начала чтения
   reg sdcard_read_ack;           // флаг подтверждения окончания чтения
   reg sdcard_write_start;        // строб начала записи
   reg sdcard_write_ack;          // флаг подтверждения команды записи
   reg [7:0] sdcard_xfer_addr;    // адрес в буфере чтния/записи
   reg sdcard_xfer_write;         // строб записи буфера
   reg [15:0] sdcard_xfer_in;     // слово; записываемое в буфер записи

//***********************************************
//*  Контроллер SD-карты
//***********************************************
   sdspi sd1 (
      // интерфейс к карте
      .sdcard_cs(sdcard_cs), 
      .sdcard_mosi(sdcard_mosi), 
      .sdcard_miso(sdcard_miso),
      .sdcard_sclk(sdcard_sclk),
      
      .sdcard_debug(sdcard_debug),                // информационные индикаторы   
   
      .sdcard_addr(sdcard_addr),                  // адрес блока на карте
      .sdcard_idle(sdcard_idle),                  // сигнал готовности модуля к обмену
      
      // сигналы управления чтением 
      .sdcard_read_start(sdcard_read_start),       // строб начала чтения
      .sdcard_read_ack(sdcard_read_ack),           // флаг подтверждения команды чтения
      .sdcard_read_done(sdcard_read_done),         // флаг окончагия чтения
      
      // сигналы управления записью
      .sdcard_write_start(sdcard_write_start),     // строб начала записи
      .sdcard_write_ack(sdcard_write_ack),         // флаг подтверждения команды записи
      .sdcard_write_done(sdcard_write_done),       // флаг окончания записи
      .sdcard_error(sdcard_error),                 // флаг ошибки

      // интерфейс к буферной памяти контроллера
      .sdcard_xfer_addr(sdcard_xfer_addr),         // текущий адрес в буферах чтения и записи
      .sdcard_xfer_out(sdcard_xfer_out),           // слово, читаемое из буфера чтения
      .sdcard_xfer_in(sdcard_xfer_in),             // слово, записываемое в буфер записи
      .sdcard_xfer_write(sdcard_xfer_write),       // строб записи буфера
      
      .mode(sdmode),                               // режим ведущего-ведомого контроллера
      .controller_clk(wb_clk_i),                   // синхросигнал общей шины
      .reset(reset),                               // сброс
      .sdclk(sdclock)                              // синхросигнал SD-карты
   ); 
   
   // формирователь ответа на цикл шины   
   wire reply=wb_cyc_i & wb_stb_i & ~wb_ack_o;

   //**************************************
   //*  Сигнал ответа 
   //**************************************
   always @(posedge wb_clk_i or posedge wb_rst_i)
     if (wb_rst_i == 1) wb_ack_o <= 0;
     else wb_ack_o <= reply;

   //**************************************************
   // Логика обработки прерываний и общего сброса
   //**************************************************
   always @(posedge wb_clk_i)  begin
      if (reset == 1'b1) begin
         // сброс системы
         interrupt_trigger <= 1'b0 ; 
         interrupt_state <= i_idle ; 
         scpset <= 1'b0 ; 
         rksi[7] <= 0 ; 
         rksi[6] <= 0 ; 
         rksi[5] <= 0 ; 
         rksi[4] <= 0 ; 
         rksi[3] <= 0 ; 
         rksi[2] <= 0 ; 
         rksi[1] <= 0 ; 
         rksi[0] <= 0 ; 
         rkds_dri <= 3'b000 ;   
         rkds_wps <= 1'b0 ;     
         rkds_scsa <= 1'b1 ;    
         rkds_sc <= 4'b0000 ;
         rker_wce <= 1'b0 ; 
         rker_cse <= 1'b0 ; 
         rker_nxs <= 1'b0 ; 
         rker_nxc <= 1'b0 ; 
         rker_nxd <= 1'b0 ; 
         rker_te <= 1'b0 ; 
         rker_dlt <= 1'b0 ; 
         rker_nxm <= 1'b0 ; 
         rker_pge <= 1'b0 ; 
         rker_ske <= 1'b0 ; 
         rker_wlo <= 1'b0 ; 
         rker_ovr <= 1'b0 ; 
         rker_dre <= 1'b0 ; 
         rkba <= {16{1'b0}} ; 
         rkcs_scp <= 1'b0 ; 
         rkcs_iba <= 1'b0 ; 
         rkcs_fmt <= 1'b0 ; 
         rkcs_exb <= 1'b0 ; 
         rkcs_sse <= 1'b0 ; 
         rkcs_rdy <= 1'b1 ; 
         rkcs_ide <= 1'b0 ; 
         rkcs_mex <= 2'b00 ; 
         rkcs_fu <= 3'b000 ; 
         rkcs_go <= 1'b0 ; 
         rkda_dr <= {3{1'b0}} ; 
         rkda_cy <= {8{1'b0}} ; 
         rkda_hd <= 1'b0 ; 
         rkda_sc <= {4{1'b0}} ; 
         rkdb <= {16{1'b0}} ; 
         start <= 1'b0 ; 
         rkwc <= {16{1'b0}} ; 
         update_rkwc <= 1'b1 ; 
         rkclock <= 0 ; 
         rkdelay <= 0 ; 
         rkcs_godelay <= 30 ; 
         irq <= 1'b0 ;    // снимаем запрос на прерывания
         sdreq <= 1'b0;
      end
      
      // рабочие состояния
      else   begin
        //******************************
        //* обработка прерывания
        //******************************
            case (interrupt_state)
                // нет активного прерывания
              i_idle :
                        begin
                           irq <= 1'b0 ;    // снимаем запрос на прерывания                           
                           //  Если поднят флаг завершения позиционирования - поднимаем триггер прерывания
                           if (rkcs_ide == 1'b1 & scpset == 1'b1)  begin
                              interrupt_state <= i_req ; 
                              irq <= 1'b1 ;    // запрос на прерывание
                              scpset <= 1'b0 ;  // снимаем флаг завершения позиционирования
                           end 
                           // Прерывание по готовности устройства
                           if (rkcs_ide == 1'b1 & rkcs_rdy == 1'b1) begin                           
                              if (interrupt_trigger == 1'b0) begin
                                 interrupt_state <= i_req ; 
                                 irq <= 1'b1 ; 
                                 interrupt_trigger <= 1'b1 ; 
                              end                               
                           end                           
                           else interrupt_trigger <= 1'b0 ;                            
                        end
               // Формирование запроса на прерывание         
               i_req :
                        begin
                           if (rkcs_ide == 1'b1) begin                           
                              // если прерывания вообще разрешены
                              if (iack == 1'b1) begin
                                 // если получено подтверждение прерывания от процессора
                                 irq <= 1'b0 ;               // снимаем запрос
                                 interrupt_state <= i_wait ; // переходим к ожиданию окончания обработки
                              end 
                           end
                           
                           else begin                           
                             // если прерывания запрещены
                              interrupt_trigger <= 1'b0 ; 
                              interrupt_state <= i_idle ; 
                           end 
                        end
                        
                        
               // Ожидание окончания обработки прерывания         
               i_wait :   if (iack == 1'b0)  interrupt_state <= i_idle ; 
             endcase
                                                   
   //************************************************
   //*  Эмуляция последовательной смены секторов
   //************************************************
            rkclock <= rkclock + 1'b1 ;  // таймер-делитель частоты смены секторов
            // переход на новый сектор
            if (rkclock == 0)  begin
               // продвигаем счетчик секторов
               if (rkds_sc[3:0] == 4'b1011) rkds_sc <= 4'b0000 ; // 12 секторов на дорожку
               else                         rkds_sc <= rkds_sc + 1'b1 ;
               
               // счетчики задержки позиционирования для каждого диска
               if (rksi[7] > 1)  rksi[7] <= rksi[7] - 1'b1 ; 
               if (rksi[6] > 1)  rksi[6] <= rksi[6] - 1'b1 ; 
               if (rksi[5] > 1)  rksi[5] <= rksi[5] - 1'b1 ; 
               if (rksi[4] > 1)  rksi[4] <= rksi[4] - 1'b1 ; 
               if (rksi[3] > 1)  rksi[3] <= rksi[3] - 1'b1 ; 
               if (rksi[2] > 1)  rksi[2] <= rksi[2] - 1'b1 ; 
               if (rksi[1] > 1)  rksi[1] <= rksi[1] - 1'b1 ; 
               if (rksi[0] > 1)  rksi[0] <= rksi[0] - 1'b1 ; 
            end 
            // Признак совпадения номера текущего и выбранного сектора
            if (rkds_sc == rkda_sc)   rkds_scsa <= 1'b1 ; 
            else                      rkds_scsa <= 1'b0 ; 
            
          //*********************************************
          //* Обработка unibus-транзакций 
          //*********************************************            
            // чтение регистров
            if (bus_read_req == 1'b1)   begin
               case (wb_adr_i[3:1])
                  3'b000 :   wb_dat_o <= rkds ; 
                  3'b001 :   wb_dat_o <= rker ; 
                  3'b010 :   wb_dat_o <= rkcs ; 
                  3'b011 :   wb_dat_o <= (~wcp) + 1'b1 ; 
                  3'b100 :   wb_dat_o <= rkba ; 
                  3'b101 :   wb_dat_o <= rkda ; 
                  3'b111 :   wb_dat_o <= rkdb ; 
                  default :  wb_dat_o <= {16{1'b0}} ; 
               endcase 
            end
         
            // запись регистров   
            if (bus_write_req == 1'b1)  begin
               // запись четных байтов
               if (wb_sel_i[0] == 1'b1)  begin
                  case (wb_adr_i[3:1])
                     3'b010 :  begin  // RKCS
                                 rkcs_go <= wb_dat_i[0] ; 
                                 if ((wb_dat_i[0]) == 1'b1) rkcs_rdy <= 1'b0 ; // поднят флаг запуска - снимаем флаг готовности 
                                 rkcs_fu <= wb_dat_i[3:1] ;      // код команды
                                 rkcs_mex <= wb_dat_i[5:4] ; 
                                 rkcs_ide <= wb_dat_i[6] ; 
                                 if (wb_dat_i[6] == 1'b1 &       // установка IDE, разрешение прерывания 
                                     wb_dat_i[0] == 1'b0 &       // бит GO не установлен
                                     rkcs_rdy == 1'b1)           // контроллер готов к приему команды  
                                     interrupt_trigger <= 1'b1 ; // это сразу приводит к прерыванию
                              end
                              
                     3'b011 : begin   // RKWC
                                 rkwc[7:0] <= wb_dat_i[7:0] ; 
                                 update_rkwc <= 1'b1 ;  // поднимаем признак изменения RKWC 
                              end
                              
                     3'b100 : rkba[7:0] <= wb_dat_i[7:0] ;
                     
                     3'b101 : begin  // RKDA
                                 rkda_cy[2:0] <= wb_dat_i[7:5] ; 
                                 rkda_hd <= wb_dat_i[4] ; 
                                 rkda_sc <= wb_dat_i[3:0] ; 
                              end
                  endcase 
               end 
               
               // запись нечетных байтов
               if (wb_sel_i[1] == 1'b1) begin
                  case (wb_adr_i[3:1])
                     3'b010 : begin   // RKCS
                                 rkcs_sse <= wb_dat_i[8] ; 
                                 rkcs_exb <= wb_dat_i[9] ; 
                                 rkcs_fmt <= wb_dat_i[10] ; 
                                 rkcs_iba <= wb_dat_i[11] ; 
                              end
                              
                     3'b011 : begin  // RKWC
                                 rkwc[15:8] <= wb_dat_i[15:8] ; 
                                 update_rkwc <= 1'b1 ; 
                              end
                              
                     3'b100 : rkba[15:8] <= wb_dat_i[15:8] ; 
                     
                     3'b101 : begin  // RKDA
                                 rkda_dr <= wb_dat_i[15:13] ; 
                                 rkda_cy[7:3] <= wb_dat_i[12:8] ; 
                              end
                  endcase 
               end 
            end
            
            // обновление RKWC - вычисляем значение счетчика, обратное от  записанного в RKWC
            if (update_rkwc == 1'b1)  begin
               wcp <= (~rkwc) + 1'b1 ; 
               update_rkwc <= 1'b0 ; 
            end
            
            // Принят бит GO, обмен е запущен - логика  снятия бита GO
            if (rkcs_go == 1'b1 & start == 1'b0) begin
               rksi[rkda_dr] <= 3 ; 
               if (rkcs_godelay == 0)  begin
                  // задержка перед снятием бита GO окончена - снимаем бит
                  rkcs_go <= 1'b0 ; 
                  rkcs_godelay <= 180 ; // перезагружаем счетчик задержки
               end
               else  rkcs_godelay <= rkcs_godelay - 1'b1 ; // счетчик задержки перед снятием бита GO
            end
            
            // Подготовка запуска обмена после снятия бита GO
            if (rkcs_rdy == 1'b0 & start == 1'b0 & rkcs_go == 1'b0)  begin
                  start <= 1'b1 ; 
                  rkdelay <= 120 ; // задержка запуска команды
                  rkcs_scp <= 1'b0 ; 
                  rkds_dri <= 3'b000 ; // номер устройства, запросившего прерывание
            end 
         
            // запуск команды
              if (start == 1'b1)  begin
               case (rkcs_fu)  // выбор действия по коду функции 
               
                  // сброс контроллера
                  3'b000 :  begin 
                              rkcs_rdy <= 1'b1 ; 
                              rkcs_ide <= 1'b0 ; // после сброса прерывания запрещены
                              rkcs_iba <= 1'b0 ; 
                              rkcs_exb <= 1'b0 ; 
                              rkcs_fmt <= 1'b0 ; 
                              rkcs_sse <= 1'b0 ; 
                              rkcs_mex <= 2'b00 ; 
                              rkwc <= {16{1'b0}} ; 
                              update_rkwc <= 1'b1 ; 
                              rkba <= {16{1'b0}} ; 
                              rkda_dr <= {3{1'b0}} ; 
                              rkda_cy <= {8{1'b0}} ; 
                              rkda_hd <= 1'b0 ; 
                              rkda_sc <= {4{1'b0}} ; 
                              rkdb <= {16{1'b0}} ; 
                              start <= 1'b0 ; 
                              rker_wce <= 1'b0 ; 
                              rker_cse <= 1'b0 ; 
                              rker_nxs <= 1'b0 ; 
                              rker_nxc <= 1'b0 ; 
                              rker_nxd <= 1'b0 ; 
                              rker_te <= 1'b0 ; 
                              rker_dlt <= 1'b0 ; 
                              rker_nxm <= 1'b0 ; 
                              rker_pge <= 1'b0 ; 
                              rker_ske <= 1'b0 ; 
                              rker_wlo <= 1'b0 ; 
                              rker_ovr <= 1'b0 ; 
                              rker_dre <= 1'b0 ; 
                           end
                           
                   // запись         
                  3'b001 :    begin   
                              sdreq <= 1'b1;
                              if (rkdelay != 0) rkdelay <= rkdelay - 1'b1 ; // задержка запуска команды
                              else if (sdack) begin
                                 rksi[rkda_dr] <= 0 ; 
                                 // запись еще не запущена, SD-карта готова к  работе
                                 if (start == 1'b1 & sdcard_idle == 1'b1 & write_start == 1'b0) begin
                                    
                                    // проверка на выход цилиндра за границу диска
                                    if (rkda_cy > 8'o312)  begin
                                       rker_nxc <= 1'b1 ; 
                                       rkcs_rdy <= 1'b1 ; 
                                       rkdb <= wrkdb ; 
                                       start <= 1'b0 ; 
                                    end

                                    // проверка номера сектора - от 0 до 11
                                    else if (rkda_sc[3:2] == 2'b11)  begin
                                       rker_nxs <= 1'b1 ; 
                                       rkcs_rdy <= 1'b1 ; 
                                       rkdb <= wrkdb ; 
                                       start <= 1'b0 ; 
                                    end
                                    // проверки окончены - запускаем запись
                                    else  write_start <= 1'b1 ; 
                                 end
                                 
                                 // запись сектора завершена
                                 else if (sdcard_write_ack == 1'b1 &   // мы подтвердили завершение 
                                         sdcard_write_done == 1'b0 &   // sdspi снял строб окончания 
                                         write_start == 1'b1           // идет запись
                                       ) begin
                                    write_start <= 1'b0 ;              // снимаем строб записи
                                    if (nxm == 1'b0 & sdcard_error == 1'b0)  begin
                                       // запись окончилась без ошибок
                                       rkcs_mex <= 2'b00; //ram_phys_addr[17:16] ;  // адрес окончания записи - старшая часть, пока, увы, не нужна
                                       rkba <= {ram_phys_addr[15:1], 1'b0} ;  // младшая часть
                                       // сектор меньше 11 - дорожку не меняем.
                                       if ((rkda_sc[3:0]) < (4'b1011))  rkda_sc[3:0] <= rkda_sc[3:0] + 1'b1 ; // прсто увеличиваем # сектора 
                                       else  begin
                                          // переход на новую дорожку
                                          rkda_sc[3:0] <= 4'b0000 ; // обнуляем номер сектора
                                          if (rkda_hd == 1'b0) rkda_hd <= 1'b1 ; // с головки 0 переходим на головку 1
                                          else  begin
                                             // переход на новый цилиндр
                                             if ((rkda_cy) == (8'b11001010) & rkcs_fmt != 1'b1 & (wcp) > (16'b0000000100000000))  begin
                                                // вышли за пределы диска 312 цилиндров
                                                rker_ovr <= 1'b1 ;   // ошибка OVR
                                                rkcs_rdy <= 1'b1 ; 
                                                rkdb <= wrkdb ; 
                                                start <= 1'b0 ; 
                                             end
                                             else  begin
                                                // до границы диска не доехали
                                                rkda_cy <= rkda_cy + 1'b1 ; // цилиндр++
                                                rkda_hd <= 1'b0 ; // переходим на головку 0
                                             end 
                                          end 
                                       end 
                                        
                                       // переход к записи следующего сектора
                                       if ((wcp) > (16'b0000000100000000))  begin
                                          // осталось записать больше одного полного сектора
                                          wcp <= (wcp) - (16'b0000000100000000) ; // уменьшаем счетчик на размер полного сектора
                                       end
                                       else begin
                                          // запись завершена
                                          wcp <= {16{1'b0}} ; 
                                          start <= 1'b0 ; 
                                          rkcs_rdy <= 1'b1 ; 
                                          rkdb <= wrkdb ; 
                                       end 
                                    end
                                    
                                    // обработка ошибок записи
                                    else begin
                                       rkcs_mex <= 2'b00; //ram_phys_addr[17:16] ; 
                                       rkba <= {ram_phys_addr[15:1], 1'b0} ; 
                                       rkdb <= wrkdb ; 
                                       rkcs_rdy <= 1'b1 ; 
                                       if (nxm == 1'b1)  rker_nxm <= 1'b1 ; // ошибка NXM - запись в несуществующую память
                                       if (sdcard_error == 1'b1) rker_dre <= 1'b1 ;   // ошибка SD-карты
                                    end 
                                 end 
                              end 
                           end
                           
                  // чтение, верификация         
                  3'b010, 3'b011 :
                           begin
                              sdreq <= 1'b1;
                              if (rkdelay != 0) rkdelay <= rkdelay - 1'b1 ; // задержка запуска команды
                              else if (sdack) begin
                                 rksi[rkda_dr] <= 0 ; 
                                    // если SD-модуль свободен, чтение еще не запущено и не завершено
                                    if (start == 1'b1 & sdcard_idle == 1'b1 & sdcard_read_start == 1'b0 & sdcard_read_done == 1'b0) begin
                                    // проверка номера цилиндра
                                    if (rkda_cy > 8'o312)  begin
                                       // больше 312 - выходит за пределы диска
                                       rker_nxc <= 1'b1 ; // ошибка nxc
                                       rkcs_rdy <= 1'b1 ; // готов к приему следующей команды
                                       rkdb <= wrkdb ; 
                                       start <= 1'b0 ; 
                                    end
                                    // проверка номера сектора
                                    else if (rkda_sc[3:2] == 2'b11) begin
                                       rker_nxs <= 1'b1 ; 
                                       rkcs_rdy <= 1'b1 ; 
                                       rkdb <= wrkdb ; 
                                       start <= 1'b0 ; 
                                    end
                                    // чтение и форматирование несовместимы
                                    else if (rkcs_fu != 3'b010 & (rkcs_fmt == 1'b1 | rkcs_exb == 1'b1))  begin
                                       rker_pge <= 1'b1 ; 
                                       rkcs_rdy <= 1'b1 ; 
                                       rkdb <= wrkdb ; 
                                       start <= 1'b0 ; 
                                    end
                                    // проверка окончена - запускаем чтение SD
                                    else  sdcard_read_start <= 1'b1 ; 
                                 end
                                 
                                 
                                 else if (sdcard_read_ack == 1'b1 & 
                                          sdcard_read_done == 1'b0 & 
                                          sdcard_read_start == 1'b1) begin
                                    sdcard_read_start <= 1'b0 ; 
                                    if (nxm == 1'b0 & sdcard_error == 1'b0)   begin
                                       rkcs_mex <= 2'b00; //ram_phys_addr[17:16] ; 
                                       rkba <= {ram_phys_addr[15:1], 1'b0} ; 
                                          //
                                       if ((rkda_sc[3:0]) < (4'b1011)) rkda_sc[3:0] <= rkda_sc[3:0] + 1'b1 ; 
                                       else  begin
                                          rkda_sc[3:0] <= 4'b0000 ; 
                                          if (rkda_hd == 1'b0) rkda_hd <= 1'b1 ; 
                                          else begin
                                             if ((rkda_cy) == (8'b11001010) & rkcs_fmt != 1'b1 & (wcp) > (16'b0000000100000000)) begin
                                                // 
                                                rker_ovr <= 1'b1 ; 
                                                rkcs_rdy <= 1'b1 ; 
                                                rkdb <= wrkdb ; 
                                                start <= 1'b0 ; 
                                             end
                                             else begin
                                                rkda_cy <= rkda_cy + 1'b1 ; 
                                                rkda_hd <= 1'b0 ; 
                                             end 
                                          end 
                                       end 
                                       if (rkcs_fmt == 1'b1)  begin

                                          if ((wcp) > (16'b0000000000000001))  wcp <= (wcp) - (16'b0000000000000001) ; 
                                          else  begin
                                             wcp <= {16{1'b0}} ; 
                                             start <= 1'b0 ; 
                                             rkcs_rdy <= 1'b1 ; 
                                             rkdb <= wrkdb ; 
                                          end 
                                       end
                                       else  begin

                                          if ((wcp) > (16'b0000000100000000))   wcp <= (wcp) - (16'b0000000100000000) ; 
                                          else  begin
                                             // чтение завершено
                                             wcp <= {16{1'b0}} ; 
                                             start <= 1'b0 ; 
                                             rkcs_rdy <= 1'b1 ; 
                                             rkdb <= wrkdb ; 
                                          end 
                                       end 
                                    end
                                    else  begin
                                       rkcs_mex <= 2'b00; //ram_phys_addr[17:16] ; 
                                       rkba <= {ram_phys_addr[15:1], 1'b0} ; 
                                       rkdb <= wrkdb ; 
                                       rkcs_rdy <= 1'b1 ; 
                                       if (nxm == 1'b1)  rker_nxm <= 1'b1 ; 
                                       if (sdcard_error == 1'b1)  rker_dre <= 1'b1 ; 
                                    end 
                                 end 
                              end 
                           end
                           
                  // позиционирование         
                  3'b100 :
                           begin
                              if (rkdelay == 0)  begin
                                 // seek
                                 if (rkcs_fmt == 1'b1 | rkcs_exb == 1'b1) begin
                                    // режим форматирования и позиционирование  несовместимы
                                    rker_pge <= 1'b1 ; 
                                    rkcs_rdy <= 1'b1 ; 
                                    start <= 1'b0 ; 
                                 end
                                 else if ((rkda_cy) > (8'o312))  begin
                                    // цилиндр выходит за пределы диска
                                    rker_nxc <= 1'b1 ; 
                                    rkcs_rdy <= 1'b1 ; 
                                    start <= 1'b0 ; 
                                 end
                                 else  begin
                                    // отработка команды позиционирования
                                    rksi[rkda_dr] <= 3 ;  // взводим таймер позиционирования
                                    rkcs_rdy <= 1'b1 ; 
                                    start <= 1'b0 ; 
                                 end 
                              end
                              else  rkdelay <= rkdelay - 1'b1 ; // задержка запуска команды
                           end
                           
                  // read check         
                  3'b101 :
                           begin
                              if (rkdelay == 0)  begin
                                 if ((rkda_sc[3:0]) < (4'b1011))  begin
                                    rkda_sc[3:0] <= rkda_sc[3:0] + 1'b1 ; 
                                 end
                                 else  begin
                                    rkda_sc[3:0] <= 4'b0000 ; 
                                    if (rkda_hd == 1'b0)   rkda_hd <= 1'b1 ; 
                                    else  begin
                                       // read check 
                                       if ((rkda_cy) == (8'b11001010) & rkcs_fmt != 1'b1 & (wcp) > (16'b0000000100000000)) begin
                                          rker_ovr <= 1'b1 ; 
                                          rkcs_rdy <= 1'b1 ; 
                                          start <= 1'b0 ; 
                                       end
                                       else begin
                                          rkda_cy <= rkda_cy + 1'b1 ; 
                                          rkda_hd <= 1'b0 ; 
                                       end 
                                    end 
                                 end 

                                 if ((wcp) > (16'b0000000100000000))   wcp <= (wcp) - (16'b0000000100000000) ; 
                                 else   begin
                                    wcp <= {16{1'b0}} ; 
                                    start <= 1'b0 ; 
                                    rkcs_rdy <= 1'b1 ; 
                                 end 
                              end
                              else  rkdelay <= rkdelay - 1'b1 ; 
                           end
                           
                  // сброс устройства         
                  3'b110 :
                           begin
                              if (rkdelay == 0) begin
                                 // drive reset
                                 if (rkcs_fmt == 1'b1 | rkcs_exb == 1'b1)  begin
                                    rker_pge <= 1'b1 ; 
                                    rkcs_rdy <= 1'b1 ; 
                                    start <= 1'b0 ; 
                                 end
                                 else  begin
                                    rksi[rkda_dr] <= 3 ; 
                                    rkcs_rdy <= 1'b1 ; 
                                    start <= 1'b0 ; 
                                 end 
                              end
                              else rkdelay <= rkdelay - 1'b1 ; 
                           end
                  default :
                           begin
                              // ошибочные состояния машины
                              rkcs_rdy <= 1'b1 ; 
                              start <= 1'b0 ; 
                           end
               endcase 
            end

            else  begin
               sdreq <= 1'b0;            // снимаем запрос доступа к SD-карте
               if (rkcs_rdy == 1'b1)  begin
                  if (rksi[7] == 1)  begin
                     rksi[7] <= 0 ; 
                     if (rkcs_ide == 1'b1)  begin
                        rkds_dri <= 3'b111 ; 
                        rkcs_scp <= 1'b1 ; 
                        scpset <= 1'b1 ; 
                     end 
                  end

                  else if (rksi[6] == 1) begin
                     rksi[6] <= 0 ; 
                     if (rkcs_ide == 1'b1)  begin
                        rkds_dri <= 3'b110 ; 
                        rkcs_scp <= 1'b1 ; 
                        scpset <= 1'b1 ; 
                     end 
                  end

                  else if (rksi[5] == 1)  begin
                     rksi[5] <= 0 ; 
                     if (rkcs_ide == 1'b1) begin
                        rkds_dri <= 3'b101 ; 
                        rkcs_scp <= 1'b1 ; 
                        scpset <= 1'b1 ; 
                     end 
                     end

                  else if (rksi[4] == 1) begin
                     rksi[4] <= 0 ; 
                     if (rkcs_ide == 1'b1) begin
                        rkds_dri <= 3'b100 ; 
                        rkcs_scp <= 1'b1 ; 
                        scpset <= 1'b1 ; 
                     end 
                  end

                  else if (rksi[3] == 1) begin
                     rksi[3] <= 0 ; 
                     if (rkcs_ide == 1'b1) begin
                        rkds_dri <= 3'b011 ; 
                        rkcs_scp <= 1'b1 ; 
                        scpset <= 1'b1 ; 
                     end 
                  end

                  else if (rksi[2] == 1) begin
                     rksi[2] <= 0 ; 
                     if (rkcs_ide == 1'b1) begin
                        rkds_dri <= 3'b010 ; 
                        rkcs_scp <= 1'b1 ; 
                        scpset <= 1'b1 ; 
                     end 
                  end

                  else if (rksi[1] == 1) begin
                     rksi[1] <= 0 ; 
                     if (rkcs_ide == 1'b1) begin
                        rkds_dri <= 3'b001 ; 
                        rkcs_scp <= 1'b1 ; 
                        scpset <= 1'b1 ; 
                     end 
                  end

                  else if (rksi[0] == 1) begin
                     rksi[0] <= 0 ; 
                     if (rkcs_ide == 1'b1) begin
                        rkds_dri <= 3'b000 ; 
                        rkcs_scp <= 1'b1 ; 
                        scpset <= 1'b1 ; 
                     end 
                  end 

               end 
            end 
      end  
   end 

   //**********************************************
   // Вычисление адреса блока на SD-карте
   //**********************************************
   wire[16:0] hs_offset; 
   wire[16:0] ca_offset; 
   wire[16:0] dn_offset;
   // 
   // Головка
   assign hs_offset = (rkda_hd == 1'b1) ? 17'b00000000000001100 : 17'b00000000000000000 ;
   // Цилиндр
   assign ca_offset = {5'b00000, rkda_cy, 4'b0000} + {6'b000000, (rkda_cy), 3'b000} ;
   // Начало образа диска на карте
   //
   //  0 0ddd 0000 0000 0000   1000 + 0800 = 1800
   //  0 00dd d000 0000 0000
   //
   assign dn_offset = {2'b00, rkda_dr, 12'b000000000000} + {3'b000, rkda_dr, 11'b00000000000} ;
   // полный абсолютный адрес 
   assign sdcard_addr = {6'b000000, dn_offset} + hs_offset + ca_offset + rkda_sc + start_offset ;

   // DMA и работа с картой памяти
   //---------------------------------------
   always @(posedge wb_clk_i)  begin
      if (reset == 1'b1)  begin
         // сброс
         busmaster_state <= busmaster_idle ; 
         dma_req <= 1'b0 ; 
         sdcard_read_ack <= 1'b0 ; 
         sdcard_write_start <= 1'b0 ; 
         nxm <= 1'b0 ; 
      end
      
      // рабочие состояния
      else  begin
            case (busmaster_state)
               // ожидание запроса
               busmaster_idle :
                        begin
                           nxm <= 1'b0 ; //  снимаем флаг ошибки nxm
                           
                           // старт процедуры записи
                           if (write_start == 1'b1) begin
                              dma_req <= 1'b1 ;    // поднимаем запрос DMA
                              if (dma_gnt == 1'b1) begin  // ждем подтверждения DMA
                                 busmaster_state <= busmaster_write1 ; // переходим к этапу 1 записи
                                 ram_phys_addr <= rkba[15:1];  // полный физический адрес памяти
                                 // вычисление количества байтов в текущем секторе (передача может быть неполной)
                                 if (wcp >= 16'b0000000100000000) sector_data_index <= 9'b100000000 ; 
                                 else if (wcp == 16'b0000000000000000) sector_data_index <= 9'b000000000 ; 
                                 else                sector_data_index <= {1'b0, wcp[7:0]} ; 
                                 sdcard_xfer_addr <= 8'b11111111 ;        // адрес в буфере sd-контроллера
                              end 
                           end
                           // старт процедуры чтения
                           else if (sdcard_read_done == 1'b1) begin
                              dma_req <= 1'b1 ;  // поднимаем запрос DMA
                              if (dma_gnt == 1'b1)  begin // ждем подтверждения DMA
                                 ram_phys_addr <= rkba[15:1]; // полный физический адрес памяти
                                    // проверка на режим чтения заголовков
                                 if (rkcs_fmt == 1'b1) busmaster_state <= busmaster_readh ; // переходим к чтению заголовков
                                 else  busmaster_state <= busmaster_read1 ;  // переходим к чтению данных
                                 // коррекция счетчика читаемых слов
                                 if (wcp >= 16'b0000000100000000)  sector_data_index <= 9'b100000000 ; 
                                 else                  sector_data_index <= {1'b0, wcp[7:0]} ; 
                                 sdcard_xfer_addr <= 0 ;   // начальный адрес в буфере SD-контроллера
                              end 
                           end 
                        end
                        
                        // чтение заголовоков секторов
               busmaster_readh : 
                        begin
                           dma_adr_o <= {ram_phys_addr[15:1], 1'b0} ; 
                           dma_dat_o <= {3'b000, rkda_cy, 5'b00000} ; 
                           dma_stb_o <= 1'b1 ; 
                           dma_we_o <= 1'b1;
                           if (rkcs_iba == 1'b0)       ram_phys_addr <= ram_phys_addr + 1'b1 ; 
                           if (dma_ack_i == 1'b1) busmaster_state <= busmaster_read_done ; 
                        end
               busmaster_readh2 :
                        begin
                           dma_adr_o <= {ram_phys_addr[15:1], 1'b0} ; 
                           dma_dat_o <= 16'h1111 ; 
                           dma_stb_o <= 1'b1 ; 
                           dma_we_o <= 1'b1;
                           if (rkcs_iba == 1'b0)     ram_phys_addr <= ram_phys_addr + 1'b1 ; 
                           if (dma_ack_i == 1'b1) busmaster_state <= busmaster_read_done ; 
                        end
                        
                        // чтение данных - подготовка
               busmaster_read1 :
                        begin
                           busmaster_state <= busmaster_read ; 
                           dma_adr_o <= {ram_phys_addr[15:1], 1'b0} ; // выставляем адрес на шину
                           dma_stb_o <= 1'b0 ;                // снимаем строб данных 
                           dma_we_o <= 1'b0 ; 
                           reply_count <= 6'b111111;  // взводим таймер обменв
                        end
                        // чтение данных - обмен по шине
               busmaster_read :
                        begin
                           if (sector_data_index != 9'b000000000)  begin
                              // передача данных сектора
                              dma_dat_o <= sdcard_xfer_out ;             // выставляем данные
                                dma_we_o <= 1'b1;
                              dma_stb_o <= 1'b1 ;             // строб записи на шину
                              wrkdb <= sdcard_xfer_out ;                   // регистр данных RKDB
                              reply_count <= reply_count - 1'b1;
                              if (|reply_count == 1'b0) begin
                                nxm <= 1'b1;
                                busmaster_state <= busmaster_read_done ; 
                              end  
                              if (dma_ack_i == 1'b1) begin   // устройство подтвердило обмен
                                    busmaster_state <= busmaster_read1; 
                                  if (rkcs_iba == 1'b0) ram_phys_addr <= ram_phys_addr + 1'b1 ; // если разрешено, увелисиваем физический адрес
                                  sector_data_index <= sector_data_index - 1'b1 ;       // уменьшаем счетчик данных сектора
                                  sdcard_xfer_addr <= sdcard_xfer_addr + 1'b1 ; // увеличиваем адрес буфера SD
                              end    
                           end
                           else begin
                              // все сектора прочитаны 
                              busmaster_state <= busmaster_read_done ; 
                              dma_stb_o <= 1'b0 ; 
                              dma_we_o <= 1'b0 ; 
                           end 
                        end
               busmaster_read_done :
                        begin
                           dma_req <= 1'b0 ; 
                           sdcard_read_ack <= 1'b1 ; 
                           dma_stb_o <= 1'b0 ; 
                           dma_we_o <= 1'b0 ; 
                           if (sdcard_read_ack == 1'b1 & sdcard_read_done == 1'b0)  begin
                              busmaster_state <= busmaster_idle ; 
                              sdcard_read_ack <= 1'b0 ; 
                           end 
                        end
                        
               // этап 1 записи - подготовка шины к DMA
               busmaster_write1 :
                        begin
                              sector_data_index <= sector_data_index - 1'b1 ; // уменьшаем счетчик записанных данных
                              sdcard_xfer_write <= 1'b1 ;         // поднимаем флаг режима записи sdspi
                              dma_we_o <= 1'b0 ; 
                              sdcard_xfer_addr <= sdcard_xfer_addr + 1'b1 ; // адрес буфера sdspi++
                              dma_stb_o <= 1'b1 ;  // поднимаем строб чтения
                              if (rkcs_iba == 1'b0)  ram_phys_addr <= ram_phys_addr + 1'b1 ; // если разрешено, увеличиваем адрес
                              dma_adr_o <= {ram_phys_addr[15:1], 1'b0} ; // выставляем на шину адрес
                              busmaster_state <= busmaster_write ;  // 
                              reply_count <= 6'b111111;  // взводим таймер обменв
                        end
                        
               // перепись данных сектора из памяти в буфер контроллера через DMA         
               busmaster_write :
                        begin
                              // еще есть данные для записи
                           wrkdb <= {16{1'b0}} ;               // очистка буфера данных
                           reply_count <= reply_count - 1'b1;
                           if (|reply_count == 1'b0) begin
                                nxm <= 1'b1;
                                busmaster_state <= busmaster_write_done ; 
                           end  
                             if (dma_ack_i == 1'b1) begin   // устройство подтвердило обмен
                                 sdcard_xfer_in <= dma_dat_i ; // передаем байт данные с шины на вход sdspi
                                 dma_adr_o <= {ram_phys_addr[15:1], 1'b0} ; // выставляем на шину адрес
                                 dma_stb_o <= 1'b0 ; 
                              if (sector_data_index == 9'b000000000) begin
                                if (sdcard_xfer_addr == 255) busmaster_state <= busmaster_write_wait ; 
                                else                         busmaster_state <= busmaster_write_fill; 
                                dma_req <= 1'b0 ; 
                              end 
                              else  busmaster_state <= busmaster_write_delay ;  
                           end   
                        end
               // задержка 1 такт между операциями DMA-чтения         
               busmaster_write_delay: busmaster_state <= busmaster_write1;         
               // дописывание нулей в конец неполного сектора         
               busmaster_write_fill :
                        begin
                           dma_req <= 1'b0 ; 
                           if (sdcard_xfer_addr == 255)  busmaster_state <= busmaster_write_wait ; 
                           else   begin
                              sdcard_xfer_in <= {16{1'b0}} ; 
                              sdcard_xfer_addr <= sdcard_xfer_addr + 1'b1 ; 
                              sdcard_xfer_write <= 1'b1 ; 
                           end 
                        end
                        
               busmaster_write_wait :
                        begin
                           sdcard_write_start <= 1'b1 ; 
                           sdcard_xfer_write <= 1'b0 ; 
                           if (sdcard_write_done == 1'b1)   begin
                              busmaster_state <= busmaster_write_done ; 
                              sdcard_write_start <= 1'b0 ; 
                           end 
                        end
               busmaster_write_done :
                        begin
                           sdcard_write_ack <= 1'b1 ; 
                           if (sdcard_write_ack == 1'b1 & sdcard_write_done == 1'b0)  begin
                              busmaster_state <= busmaster_idle ; 
                              sdcard_write_ack <= 1'b0 ; 
                           end 
                        end
            endcase 
      end  
   end 
endmodule
