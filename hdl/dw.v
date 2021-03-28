//
//  Контроллер жесткого диска RD50C в варианте ДВК
//
module dw (

// шина wishbone
   input                wb_clk_i,   // тактовая частота шины
   input                wb_rst_i,   // сброс
   input    [4:0]       wb_adr_i,   // адрес 
   input    [15:0]      wb_dat_i,   // входные данные
   output reg [15:0]    wb_dat_o,   // выходные данные
   input                wb_cyc_i,   // начало цикла шины
   input                wb_we_i,    // разрешение записи (0 - чтение)
   input                wb_stb_i,   // строб цикла шины
   input    [1:0]       wb_sel_i,   // выбор конкретных байтов для записи - старший, младший или оба
   output reg           wb_ack_o,   // подтверждение выбора устройства

// обработка прерывания   
   output reg           irq,        // запрос
   input                iack,       // подтверждение
   
// интерфейс SD-карты
   output               sdcard_cs, 
   output               sdcard_mosi, 
   output               sdcard_sclk, 
   input                sdcard_miso, 
   output reg           sdreq,      // запрос доступа к карте
   input                sdack,      // подтверждение доступа к карте
   input                sdmode,     // режим SDSPI
   
// тактирование SD-карты
   input                sdclock,   

// Адрес начала банка на карте
   input [26:0]         start_offset,
   
// отладочные сигналы
   output [3:0]         sdcard_debug
   ); 
//-----------------------------------------------
//  Регистры контроллера
//
// 174000  DWID   - регистр идентификации РИ
//                    D0-D15  R/O  идентификация 401
// 174002  DWERR  - регистр ошибок/предкомпенсации РОШПК
//                    D0-D7   W/O   номер цилиндра предкомпенсации
//                    D8      R/O   ошибка поиска маркера данных
//                    D9      R/O   ошибка позиционирования на дорожку 0
//                    D10     R/O   неисправность или неверная команда
//                    D11     R/O   0
//                    D12     R/O   ошбика поиска адреса 
//                    D13     R/O   ошибка CRC адреса
//                    D14     R/O   ошибка CRC данных
//                    D15     R/O   0
// 174006  DWSEC  - регистр адреса сектора/служебной зоны РАССЗ
//                    D0-D4   R/W   номер сектора текущей операции
//                    D8-D15  R/W   содержимое первого байта служебной зоны 
// 174010  DWBUF  - регистр (буфер) данных РД
// 174012  DWCYL  - регистр адреса цилиндра РАЦ
//                    D0-D9   R/W   текущий цилиндр  
// 174014  DWHD   - регистр адреса поверхности (головка) РАП
//                    D0-D2   R/w   текущая головка
// 174016  DWCS2  - регистр команд/состояния 2 РКС2
//                    D0-D7   W/O   команда контроллеру (20 - возврат на цилиндр 0, 40 - чтение, 60 - запись, 120 - форматирование
//                    D8      R/O   ошибка
//                    D9      R/O   0
//                    D10     R/O   0
//                    D11     R/O   DRQ, запрос данных 2  - буфер готов к чтению-записи, сбрасывается после считывания-записи всего буфера
//                    D12     R/O   установка завершена
//                    D13     R/O   ошибка записи
//                    D14     R/O   накопитель готов
//                    D15     R/O   0
// 174020  DWSTRS - регистр состояния/сброса РСНУ
//                    D0      R/O   DONE, операция завершена (ЗОА)
//                    D1      R/O   0
//                    D2      R/O   0 
//                    D3      R/W   начальная установка
//                    D4      R/O   0
//                    D5      R/O   0
//                    D6      R/W   запрет прерывания
//                    D7      R/O   запрос данных 1 (ЗОВ) - готово очередное слово буфера
//                    D8      R/O   тип накопителя (0-быстрый, 1-медленный)
//                    D9      R/O   0
//                    D10     R/O   0
//                    D11     R/O   0
//                    D12     R/O   0
//                    D13     R/O   0
//                    D14     R/O   0
//                    D15     R/O   контроллер занят
//
//-----------------------------------------------

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
reg rqa;    // ЗОА
reg ide;    // разрешение прерываний
reg drq;    // запрос данных (ЗОВ)

// CHS
reg [9:0] cyl;
reg [2:0] hd;
reg [4:0] sec;

reg start;   // запуск обмена
reg busy;    // флаг занятости контроллера
reg [7:0] cmd; // код команды

reg cmderr;  // флаг ошибки выполнения команды
reg rstreq;  // триггер запроса на программный сброс контроллера
      
// интерфейс к SDSPI
wire [26:0] sdcard_addr;  // адрес сектора карты
wire sdspi_io_done;       // флаг окончагия чтения
wire sdcard_error;        // флаг ошибки
wire [15:0] sdbuf_dataout;// слово; читаемое из буфера чтения
wire sdcard_idle;         // признак готовности контроллера
reg sdspi_start;          // строб начала чтения
reg sdspi_write_mode;     // строб начала записи
reg [7:0] sdbuf_addr;     // адрес в буфере чтния/записи
reg [15:0] sdbuf_datain;  // слово; записываемое в буфер записи
reg write_error;
reg sdbuf_write;

// состояния процесса записи
reg [2:0] wstate;
parameter[2:0] w_prepare=0;
parameter[2:0] w_waitdata=1;
parameter[2:0] w_start=2;
parameter[2:0] w_wait=3;
parameter[2:0] w_ack=4;
parameter[2:0] w_done=5;
parameter[2:0] w_skip=6;

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
      
      // сигналы управления чтением - записью
      .sdspi_start(sdspi_start),                  // строб запуска ввода вывода
      .sdspi_io_done(sdspi_io_done),              // флаг окончания обмена данными
      .sdspi_write_mode(sdspi_write_mode),        // режим: 0 - чтение, 1 - запись
      .sdcard_error(sdcard_error),                // флаг ошибки

      // интерфейс к буферной памяти контроллера
      .sdbuf_addr(sdbuf_addr),                    // текущий адрес в буферах чтения и записи
      .sdbuf_dataout(sdbuf_dataout),              // слово, читаемое из буфера чтения
      .sdbuf_datain(sdbuf_datain),                // слово, записываемое в буфер записи
      .sdbuf_we(sdbuf_write),                     // разрешение записи буфера
      
      .mode(sdmode),                              // режим ведущего-ведомого контроллера
      .controller_clk(wb_clk_i),                  // синхросигнал общей шины
      .reset(reset),                              // сброс
      .sdclk(sdclock)                             // синхросигнал SD-карты
); 
   
//**************************************
// формирователь ответа на цикл шины   
//**************************************
wire reply=wb_cyc_i & wb_stb_i & ~wb_ack_o;

//**************************************
//*  Сигнал ответа 
//**************************************
always @(posedge wb_clk_i or posedge wb_rst_i)
    if (wb_rst_i == 1'b1) wb_ack_o <= 1'b0;
    else wb_ack_o <= reply;

//**************************************************
// Логика обработки прерываний 
//**************************************************
always @(posedge wb_clk_i)  
      if (reset == 1'b1 || rstreq == 1'b1) begin
       // сброс системы
         interrupt_state <= i_idle ; 
         irq <= 1'b0 ;    // снимаем запрос на прерывания
      end
      
      else   begin
        //******************************
        //* обработка прерывания
        //******************************
        case (interrupt_state)
                // нет активного прерывания
              i_idle :
                        begin
                     //  Если поднят флаг A или B - поднимаем триггер прерывания
                           if ((ide ==1'b1) & (rqa == 1'b1 | drq== 1'b1))  begin
                              interrupt_state <= i_req ; 
                              irq <= 1'b1 ;    // запрос на прерывание
                           end 
                           else   irq <= 1'b0 ;    // снимаем запрос на прерывания
                        end
               // Формирование запроса на прерывание         
               i_req :     if (ide == 1'b0) interrupt_state <= i_idle ;    
                           else if (iack == 1'b1) begin
                              // если получено подтверждение прерывания от процессора
                              irq <= 1'b0 ;               // снимаем запрос
                              interrupt_state <= i_wait ; // переходим к ожиданию окончания обработки
                           end 
               // Ожидание окончания обработки прерывания         
               i_wait :    if (iack == 1'b0)  interrupt_state <= i_idle ; 
       endcase
end

//**************************************************
// Работа с шиной и SDSPI
//**************************************************
always @(posedge wb_clk_i)  
      if (reset == 1'b1 || rstreq == 1'b1) begin
       // сброс системы
         start <= 1'b0 ; 
         rqa <= 1'b1;
         ide <= 1'b0;
         cmderr <= 1'b0;
         drq <= 1'b0;
         rstreq <= 1'b0;
         busy <= 1'b0;
         cyl <= 10'o0;
         hd <= 3'o0;
         sec <= 5'o0;
         sdspi_start <= 1'b0;
         sdspi_write_mode <= 1'b0;
         wstate <= w_prepare;
         write_error <= 1'b0;
         sdreq <= 1'b0;
         sdbuf_write <= 1'b0;
      end
      
      // рабочие состояния
      else   begin
            
         //*********************************************
         //* Обработка unibus-транзакций 
         //*********************************************
            // чтение регистров
            if (bus_read_req == 1'b1)   begin
               case (wb_adr_i[4:1])
                  4'b0000 :   wb_dat_o <= 16'o401;        // 174000 - DWID
                  4'b0001 :   wb_dat_o <= {5'b00000, cmderr,10'o0};    // 174002 - DWERR
                  4'b0011 :   begin                        // 174006 - DWSEC
                                 wb_dat_o <= {11'o0, sec};   
                                 rqa <= 1'b0;
                              end   
                  4'b0100 :   begin                       // 174010 - DWBUF
                                 wb_dat_o <= sdbuf_dataout;      
                                 if (wb_ack_o) begin
                                       if (&sdbuf_addr == 1'b1) begin
                                          rqa <= 1'b1;
                                          drq <= 1'b0;
                                       end   
                                       else sdbuf_addr <= sdbuf_addr + 1'b1;
                                 end  
                              end 
                  4'b0101 :   wb_dat_o <= {6'o0,cyl} ;  // 174012 - DWCYL
                  4'b0110 :   wb_dat_o <= {13'o0,hd} ;  // 174014 - DWHD
                  4'b0111 :   begin                       // 174016 - DWCS2
                                 wb_dat_o <= {1'b0, sdcard_idle,write_error, sdcard_idle, drq, 1'b0, 1'b0, cmderr, 8'b0};  
                                 rqa <= 1'b0;
                              end
                  4'b1000 :   begin
                                 wb_dat_o <= {busy|(~sdcard_idle), 7'b0000000, 1'b1/*готовность буфера к обмену*/, ide, 5'b00000, rqa}; // 174020 -  DWSTRS
                              end   
                  4'b1001 :   begin
                                 wb_dat_o <= sdbuf_addr; // 174022 - текущий адрес в буфере SDSPI, для отладки, этого регистра в настоящем контроллере нет
                              end   
                  default :   wb_dat_o <= {16{1'b0}} ; 
               endcase 
            end
         
            // запись регистров   
            else if (bus_write_req == 1'b1)  begin
                if (wb_sel_i == 2'b11)  begin
                    case (wb_adr_i[4:1])
                    // 174006 - DWSEC
                     4'b0011 :  begin
                                    sec <= wb_dat_i[4:0] - 1'b1;    // номера секторов начинаются с 1
                                    rqa <= 1'b0;
                                end
                    // 174010 - DWBUF
                     4'b0100 :  begin   
                                 sdbuf_datain<= wb_dat_i; 
                                 if (reply)  sdbuf_addr <= sdbuf_addr + 1'b1;                              
                                end
                    // 174012 - DWCYL
                     4'b0101 :  begin
                                    cyl <= wb_dat_i[9:0] ;
                                end
                    // 174014 - DWHD
                     4'b0110 :  begin  
                                    hd <= wb_dat_i[2:0] ; 
                                end
                    // 174016 - DWCS2
                     4'b0111 :  begin  
                                    cmd <= wb_dat_i[7:0] ; 
                                    cmderr <= 1'b0;
                                    start <= 1'b1;
                                    rqa <= 1'b0;
                                end
                    // 174020 - DWSTRS
                     4'b1000 :  begin  
                                    ide <= wb_dat_i[6] ; 
                                    rstreq <= wb_dat_i[3];
                                end
                  endcase 
               end 
            end
            
            //*********************************************
            // запуск команды
            //*********************************************
              if (start == 1'b1)  begin
               case (cmd)  // выбор действия по коду функции 
               
                  // Возврат на цилиндр 0 - у нас просто NOP
                  8'o20:    begin
                              start <= 1'b0;
                              rqa <= 1'b1;
                           end   
                        
                
                  // чтение
                  8'o40: begin
                           // запрос доступа к SD-карте
                           sdreq <= 1'b1;
                           if (sdack) // подтверждение получено
                              // если SD-модуль свободен, чтение еще не запущено и не завершено
                              if (sdcard_idle == 1'b1 & sdspi_start == 1'b0 & sdspi_io_done == 1'b0) begin
                                       sdspi_write_mode <= 1'b0; 
                                       sdspi_start <= 1'b1 ;        // режим чтения
                                       busy <= 1'b1;                // запускаем sdspi
                              end
                              // чтение окончено
                              else if (sdspi_io_done == 1'b1 & sdspi_start == 1'b1) begin  
                                      sdspi_start <= 1'b0 ;        // снимаем запрос чтения
                                      busy <= 1'b0;                // снимаем занятость
                                      start <= 1'b0;               // завершаем обработку команды
                                      if (sdcard_error == 1'b0)   begin        
                                      // чтение окончилось без ошибок
                                         sdbuf_addr <= 8'b00000000;   // инициализируем адрес секторного буфера
                                         drq <= 1'b1;
                                      end     
                                      // ошибка чтения
                                      else  cmderr <= 1'b1;
                                 end 
                           end
                           
                   // запись         
                  8'o60 :    begin   
                               case (wstate) 
                                 // подготовка к приему секторного буфера
                                 w_prepare: begin
                                          drq <= 1'b1;                 // запрос данных
                                          sdbuf_write <= 1'b1;         // буфер sdspi - в режим записи
                                          sdbuf_addr <= 8'b11111111;   // инициализируем адрес секторного буфера
                                          wstate <= w_skip;
                                       end   
                                       
                                 w_skip:   
                                       if (|sdbuf_addr == 1'b0) wstate <=w_waitdata;
                                 // ожидание заполнения секторного буфера 
                                 w_waitdata:
                                       if (&sdbuf_addr == 1'b1) begin
                                          // буфер заполнен
                                          sdbuf_write <= 1'b0;
                                          drq <= 1'b0;
                                          wstate <= w_start;
                                       end
                                          
                                 // запуск записи
                                 w_start: begin
                                       sdreq <= 1'b1;   // запрос доступа к карте
                                       if (sdack & (sdcard_idle == 1)) begin
                                          sdspi_write_mode <= 1'b1 ;  // ражим записи
                                          sdspi_start <= 1'b1;        // запускаем sdspi
                                          busy <= 1'b1;               // снимаем бит готовности контроллера
                                          wstate <= w_wait;
                                       end   
                                    end   
                                    
                                 // ожидание окончания записи сектора на карту   
                                 w_wait:
                                       if (sdspi_io_done == 1'b1) begin
                                          wstate <= w_done;
                                       end   
                                       
                                 // запись подтверждена - освобождаем sdspi
                                 w_done: begin
                                    sdspi_start <= 1'b0 ;              // снимаем строб записи
                                    rqa <= 1'b1;                       // флаг завершения команды
                                    start <= 1'b0;                     // заканчиваем обработку команды
                                    busy <= 1'b0;
                                    wstate <= w_prepare;
                                    if (sdcard_error) write_error <= 1'b1; // была ошибка записи
                                 end   
                              endcase   
                           end
                                       
                  default:   begin
                              start <= 1'b0;
                              cmderr <= 1'b1;
                           end   
               endcase 
            end
            
            // нет активной команды - снимаем запрос доступа к карте
            else sdreq <= 1'b0;
   end 

//********************************************
// Вычисление адреса блока на SD-карте
//********************************************
//
// Формат диска:
//  16 секторов (512 байт) на дорожку
//  8 дорожек в цилиндре
//  1024 цилиндра
//
//reg [9:0] cyl;
//reg [2:0] hd;
//reg [4:0] sec;  // реально используются 4 бита
//
//  Общая ширина адреса одного диска - 17 бит
//
// полный абсолютный адрес 
assign sdcard_addr = start_offset+{6'b0,cyl,hd,sec[3:0]};

endmodule
