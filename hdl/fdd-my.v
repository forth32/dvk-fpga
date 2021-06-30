//
//  Контроллер дисковода MY
//==============================================================
module fdd_my (

// шина wishbone
   input                  wb_clk_i,   // тактовая частота шины
   input                  wb_rst_i,   // сброс
   input      [1:0]       wb_adr_i,   // адрес 
   input      [15:0]      wb_dat_i,   // входные данные
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
   output reg           dma_req,    // запрос DMA
   input                dma_gnt,    // подтверждение DMA
   output reg[15:0]     dma_adr_o,  // выходной адрес при DMA-обмене
   input[15:0]          dma_dat_i,  // входная шина данных DMA
   output reg[15:0]     dma_dat_o,  // выходная шина данных DMA
   output reg           dma_stb_o,  // строб цикла шины DMA
   output reg           dma_we_o,   // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   input                dma_ack_i,  // Ответ от устройства, с которым идет DMA-обмен
   
// интерфейс SD-карты
   output     sdcard_cs, 
   output     sdcard_mosi, 
   input      sdcard_miso,
   output     sdcard_sclk,

   output reg sdreq,      // запрос доступа к карте
   input      sdack,      // подтверждение доступа к карте
   input      sdmode,     // режим SDSPI
   
// тактирование SD-карты
   input      sdclock,   
   
// Адрес начала банка на карте
   input [26:0] start_offset,
   
// отладочные сигналы
   output [3:0] sdcard_debug
   ); 
//-----------------------------------------------
//  Регистры контроллера
//
// 177170  MYCSR  - регистр управления/состояния
//                    D0      W    go - запуск команды
//                    D1-D4   W    код команды:
//                                   0000 - RD - чтение данных
//                                   0001 - WR - запись данных
//                                   0010 - RMD - чтение с меткой
//                                   0011 - WRM - запись с меткой
//                                   0100 - RDTR - чтение дорожки
//                                   0101 - RDID - чтение заголовка
//                                   0110 - FORMAT - форматирование дорожки
//                                   0111 - SEEK - переход на дорожку
//                                   1000 - SET - установка параметров
//                                   1001 - RDER - чтение регистра состояния и ошибок
//                                   1010 -----------------------
//                                   1011 -
//                                   1100 -    Р Е З Е Р В
//                                   1101 -
//                                   1110 -----------------------
//                                   1111 - LOAD -чтение загрузочного блока
//                    D5    R     DONE, признак завершения операции
//                    D6    R/W   IE, разрешение прерывания
//                    D7    R     DRQ  запрос на запись регистра данных
//                    D8-D13 W    расширение адреса, здесь не используется
//                    D14   W     сброс
//                    D15   R     ошибка
//
// 177172 MYDR   - регистр данных - адрес блока параметров или код ошибки
//
//        Регистр ошибок и состояния
//   D0    ошибка CRC данных или диск защищен от записи
//   D1    ошибка CRC заголовка 
//   D2    начальная установка завершена   
//   D3    ошибка возврата на дорожку 0 
//   D4    ошибка поиска дорожки 
//   D5    не найден сектор
//   D6    прочитан сектор с меткой
//   D7    нет сигнала от индексного датчика вращения диска
//   D8,9  номер дисковода, с которым работала последняя команда
//   D10   головка, с которой работала последняя команда
//   D11   ошибка DMA - обращение по несуществующему адресу
//   D12   не найден адресный маркер
//   D13   не найден маркер данных
//   D14   неправильный формат разметки дискеты
//   D15   внутренняя ошибка контроллера
//-----------------------------------------------

// Сигналы упраления обменом с шиной
   
wire bus_strobe = wb_cyc_i & wb_stb_i;         // строб цикла шины
wire bus_read_req = bus_strobe & ~wb_we_i;     // запрос чтения
wire bus_write_req = bus_strobe & wb_we_i;     // запрос записи
wire reset=wb_rst_i;
 
reg interrupt_trigger;     // триггер запроса прерывания

// Регистр ошибок/состояния
//                         15    14   13   12  11  10  9-8  7    6       5      4      3    2    1    0
wire [15:0] errstatus = {1'b0, 1'b0,1'b0,1'b0,nxp,hd, drv,1'b0,1'b0,err_sec,err_cyl,1'b0,1'b1,1'b0,1'b0};

// состояние машины обработки прерывания
parameter[1:0] i_idle = 0; 
parameter[1:0] i_req = 1; 
parameter[1:0] i_wait = 2; 
reg[1:0] interrupt_state; 
reg done;     // операция завершена
reg ie;       // разрешение прерывания
reg drq;      // запрос на чтение-запись регистра данных

// блок параметров
reg [14:0] parm_addr; // адрес блока параметров в ОЗУ системы
reg [6:0] cyl;        // цилиндр 0 - 79
reg hd;               // головка 
reg [3:0] sec;        // сектор 0 - 10
reg [1:0] drv;        // номер привода
reg [15:0] wordcount; // число читаемых слов
reg [15:0] ioadr;     // адрес для чтения-записи данных
reg alltrk_mode;      // признак режима чтения полной дорожки
reg start;            // признак запуска команды на выполнение (go)
reg [3:0] cmd;        // код команды
reg rstreq;           // запрос на программный сброс

// Флаги ошибок
reg nxp;         // таймаут шины при DMA-обмене
reg err_sec;     // неправильный номер сектора
reg err_cyl0;    // неправильный номер цилиндра, обнаруженный в обработчике команд
reg err_cyl1;    // неправильный номер цилиндра, обнаруженный в DMA-контроллере
wire err_cyl = err_cyl0 | err_cyl1; // объединенный флаг ошибки

// интерфейс к SDSPI
wire [26:0] sdaddr;  // адрес сектора карты
reg  [26:0] sdcard_addr;  // адрес сектора карты
wire sdspi_io_done;       // флаг заверщение операции обмена с картой
wire sdcard_error;        // флаг ошибки
wire [15:0] sdbuf_dataout;// слово; читаемое из буфера чтения
wire sdcard_idle;         // признак готовности контроллера
reg [7:0] sdbuf_addr;     // адрес в буфере чтния/записи
reg [15:0] sdbuf_datain;  // слово; записываемое в буфер записи
reg sdbuf_we;             // разрешение записи в буфер
reg sdspi_start;          // строб запуска sdspi
reg sdspi_write_mode;     // 0-чтение, 1-запись

//  Интерфейс к DMA-контроллеру 
reg io_complete;          // окончание процедуры передачи данных
reg [15:0] pdata;         // очередное слово, загружаемое из списка параметров
//    Запросы на выполнение команд:
reg start_loadparm;       // начало загрузки параметров
reg start_rd;             // чтение данных
reg start_wr;             // запись данных
reg start_bootparm;       // установка параметров загрузочного сектора

// Состояния процесса обработки команд контроллера
reg [3:0] cmdstate;
parameter [3:0] CMD_START = 0;
parameter [3:0] CMD_WAITDATA = 1;
parameter [3:0] CMD_LOADPARM = 2;
parameter [3:0] CMD_WAITDMA = 3;
parameter [3:0] CMD_DCOMPLETE = 4;
parameter [3:0] CMD_STARTSECTOR = 5;

// Состояния DMA-контроллера
reg [4:0] dma_state;
reg [4:0] dmanextstate;
parameter [4:0] DMA_IDLE = 0;
parameter [4:0] DMA_LOADPARM1 = 1;
parameter [4:0] DMA_LOADPARM2 = 2;
parameter [4:0] DMA_LOADPARM3 = 3;
parameter [4:0] DMA_LOADPARM4 = 4;
parameter [4:0] DMA_LOADPARM5 = 5;
parameter [4:0] DMA_LOADWORD = 6;
parameter [4:0] DMA_LW_WAITREPLY = 7;
parameter [4:0] DMA_STARTREAD = 8;
parameter [4:0] DMA_BUF2HOST_PREPARE = 9;
parameter [4:0] DMA_BUF2HOST = 10;
parameter [4:0] DMA_BUF2HOST_NEXT = 11;
parameter [4:0] DMA_SD2HOST_COMPLETE = 12;
parameter [4:0] DMA_READ_WAITSDSPI = 13;
parameter [4:0] DMA_STARTWRITE = 14;
parameter [4:0] DMA_HOST2BUF = 15;
parameter [4:0] DMA_HOST2BUF_NEXT = 16;
parameter [4:0] DMA_BUF2SD = 17;
parameter [4:0] DMA_BUF2SD_NEXT = 18;
parameter [4:0] DMA_BUF2SD_COMPLETE = 19;
parameter [4:0] DMA_ALLTRK = 20;
parameter [4:0] DMA_BOOTPARM = 21;
parameter [4:0] DMA_ZEROBUF = 22;

// Таймер ожидания ответа шины
reg [7:0] dma_timer;

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
      .sdbuf_addr(sdbuf_addr),                   // текущий адрес в буферах чтения и записи
      .sdbuf_dataout(sdbuf_dataout),             // слово, читаемое из буфера чтения
      .sdbuf_datain(sdbuf_datain),               // слово, записываемое в буфер записи
      .sdbuf_we(sdbuf_we),                       // разрешение записи буфера
      
      .mode(sdmode),                             // режим ведущего-ведомого контроллера
      .controller_clk(wb_clk_i),                 // синхросигнал общей шины
      .reset(reset),                             // сброс
      .sdclk(sdclock)                            // синхросигнал SD-карты
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
// Логика обработки прерываний 
//**************************************************
always @(posedge wb_clk_i)   begin
   case (interrupt_state)
                // нет активного прерывания
              i_idle :
                        begin
                     //  Если поднят флаг - переходим в состояние активного прерывания
                           if ((ie == 1'b1) & (interrupt_trigger == 1'b1))  begin
                              interrupt_state <= i_req ; 
                              irq <= 1'b1 ;    // запрос на прерывание
                           end 
                           else   irq <= 1'b0 ;    // снимаем запрос на прерывания
                        end
               // Формирование запроса на прерывание         
               i_req :    if (ie == 1'b0)    interrupt_state <= i_idle ;    
                          else if (iack == 1'b1) begin
                              // если получено подтверждение прерывания от процессора
                              irq <= 1'b0 ;               // снимаем запрос
                              interrupt_trigger <= 1'b0;
                              interrupt_state <= i_wait ; // переходим к ожиданию окончания обработки
                          end 
               // Ожидание окончания обработки прерывания         
               i_wait :   if (iack == 1'b0)  interrupt_state <= i_idle ; 
    endcase

//**************************************************
// Работа с шиной
//**************************************************
    if ((reset == 1'b1) || (rstreq == 1'b1)) begin
       // сброс системы
        interrupt_state <= i_idle ; 
        irq <= 1'b0 ;    // снимаем запрос на прерывания
        start <= 1'b0 ; 
        done <= 1'b1;
        ie <= 1'b0;
        drq <= 1'b0;
        rstreq <= 1'b0;
        cmd <= 4'b0000;
        alltrk_mode <= 1'b0;
        start_loadparm <= 1'b0;
        start_rd <= 1'b0;
        start_wr <= 1'b0;
        start_bootparm <= 1'b0;
        err_cyl0 <= 1'b0;
        err_sec <= 1'b0;
        interrupt_trigger <= 1'b0;
        cmdstate <= CMD_START;
    end
      
   // рабочие состояния
    else   begin
            
         //*********************************************
         //* Обработка транзакций общей шины
         //*********************************************
            // чтение регистров
            if (bus_read_req == 1'b1)   begin
               case (wb_adr_i[1])
                  1'b0 : begin  // 177170 - MYCSR
                                //       15    14   13-8    7   6    5
                           wb_dat_o <= {1'b0, 1'b0, 6'b0, drq, ie, done, 5'b0};   
                   end      
                  1'b1 :   // 177172 - MYDR
                                if (!drq) wb_dat_o <= errstatus;  // если нет активной команды - читается реистр ошибок
                                else wb_dat_o <= 16'o0;            // иначе пока нули (что там должно быть на самом деле я не понял)
               endcase 
            end
         
            // запись регистров   
            else if (bus_write_req == 1'b1)  begin
                if (wb_sel_i[0] == 1'b1)  
                    // запись младших байтов
                    case (wb_adr_i[1])
                     // 177170 - MYCSR
                     1'b0:  
                            if (reply) begin
                                // принят бит GO при незапущенной операции
                                if ((start == 1'b0) && (wb_dat_i[0] == 1'b1)) begin 
                                    // Ввод новой команды
                                    start <= 1'b1;               // признак активной команды
                                    done <= 1'b0;                // сбрасываем признак завершения команды
                                    drq <= 1'b0;                 // сбрасываем запрос данных
                                    cmd <= wb_dat_i[4:1];        // код команды
                                    interrupt_trigger <= 1'b0;   // снимаем ранее запрошенное прерывание
                                    cmdstate <= CMD_START;       // первый этап обработки команды
                                    err_cyl0 <= 1'b0;            // сброс флагов ошибок
                                    err_sec <= 1'b0;
                                    alltrk_mode <= 1'b0;         // сброс режима полной дорожки
                                end         
                                ie <= wb_dat_i[6];               // флаг разрешения прерывания - доступен для записи всегда
                            end
                    // 177172 - MYDR
                     1'b1 :    
                              // DRQ поднят - идет чтение адреса блока параметров
                              if (drq) begin
                                 parm_addr[6:0] <= wb_dat_i[7:1];  // загрузка адреса параметров
                                 if (reply) drq <= 1'b0;           // снимаем DRQ
                              end   
                    endcase
                    
               if (wb_sel_i[1] == 1'b1)  begin
                    // запись старших байтов
                    case (wb_adr_i[1])
                     // 177170 - MYCSR
                     1'b0:  rstreq <= wb_dat_i[14];   // запрос на программный сброс
                     // 177172 - MYDR
                     1'b1:  if (drq) parm_addr[14:7] <= wb_dat_i[15:8];   // остальные биты адреса блока параметров
                    endcase   
               end 
            end
            
         //*********************************************
         // запуск команды
         //*********************************************
           if (start == 1'b1)  begin
           case (cmd)  // выбор действия по коду функции 
            4'b0000, // чтение
            4'b0001, // запись
            4'b0010, // чтение с меткой
            4'b0011: // запись с меткой
              case (cmdstate)
                // этап 1 - взводим DRQ
                CMD_START: begin
                  drq <= 1'b1;
                  cmdstate <= CMD_WAITDATA;
                  end
                  
                // этап 2 - ждем загрузки регистра данных
                CMD_WAITDATA: 
                  if (drq == 1'b0) begin   // drq опустился - регистр записан, можно продолжать
                     cmdstate <= CMD_LOADPARM; 
                     start_loadparm <= 1'b1;   // запускаем процедуру загрузки параметров через DMA, адрес у нас теперь есть
                  end
               
                // этап 3 - проверяем параметры
                CMD_LOADPARM:
                  if (io_complete == 1'b1) begin  // загрузка параметров окончена
                     start_loadparm <= 1'b0;      // снимаем запрос на загрузку параметров
                     // проверяем допустимость номера цилиндра 0 - 79
                     if (cyl > 7'd79) begin 
                       err_cyl0 <= 1'b1;
                       start <= 1'b0;    // при ошибке завершаем работу команды
                       cmdstate <= CMD_START;
                     end
                     // проверяем допустимость номера сектора
                     else if (sec > 4'd9) begin
                       err_sec <= 1'b1;
                       start <= 1'b0;
                       cmdstate <= CMD_START;
                     end 
                     else cmdstate <= CMD_STARTSECTOR; // ошибок нет - переходим к запуску обмена с картой
                   end
            
               CMD_STARTSECTOR: begin
                        // запускаем DMA-контроллер
                        if (cmd[0] == 0) start_rd <= 1'b1;  // чтение
                        else              start_wr <= 1'b1;    // запись
                        cmdstate <= CMD_WAITDMA;
                   end
                   
               // ожидание завершения работы DMA-контролера      
               CMD_WAITDMA:
                  if ((io_complete == 1'b1) || (nxp == 1'b1)) begin
                     start_rd <= 1'b0;               // снимаем команду чтения
                     start_wr <= 1'b0;               // снимаем команду записи
                     if ((|wordcount == 1'b0)  || (nxp == 1'b1)) begin      
                        // передано заказанное количество слов или уперлись в ошибку шины
                        start <= 1'b0;              // завершаем обработку команды
                        done <= 1'b1;               // признак завершения обработки
                        interrupt_trigger <= 1'b1;   // взводим триггер прерывания
                        cmdstate <= CMD_START;
                     end
                     // счетчик слов не исчерпан - читсем следующий сектор
                     else begin
                        cmdstate <= CMD_STARTSECTOR;
                     end   
                  end   
                endcase   // cmdstate
                
           // чтение дорожки
           4'b0100: begin
               alltrk_mode <= 1'b1;  // флаг полной дорожки
               cmd <= 4'b0000;       // далее обычная команда чтения
               end
           
           // Чтение загрузочного сектора
           4'b1111: 
              case (cmdstate)
               // этап 1 - поднимаем DRQ
               CMD_START: begin
                  drq <= 1'b1;
                  cmdstate <= CMD_WAITDATA;
                  end
                  
                // этап 2 - ждем загрузки регистра данных
                CMD_WAITDATA: 
                  if (drq == 1'b0) begin
                     start_bootparm <= 1'b1;   // запускаем установку загрузочных параметров
                     if (io_complete == 1'b1) begin  // установка окончена
                        start_bootparm <= 1'b0;  // снимаем команду
                        cmdstate <= CMD_STARTSECTOR; // переходим к чтению сектора
                        cmd <= 4'b0000;  // меняем код команды на команду чтения
                     end  
                  end
              endcase
               
           4'b1000, // установка парметров
           4'b0110, // форматирование дорожки
           4'b0111, // переход на дорожку
           4'b0101: // чтение заголовка
              case (cmdstate)
                CMD_START: begin
                  drq <= 1'b1;   // поднимаем DRQ
                  cmdstate <= CMD_DCOMPLETE; // уходим ждать записи в регистр данных
                  end
                  
                // drq опустился, данные в регистр загружены, но нам они не нужны
                CMD_DCOMPLETE: 
                  if (drq == 1'b0) begin 
                     start <= 1'b0;
                     done <= 1'b1;
                     interrupt_trigger <= 1'b1;
                     cmdstate <= CMD_START;
                  end
               endcase   
               
           // Остальные команды ничего не делают   
           // чтение регистра ошибок 1001 отрабатывается как nop, потому как из MYDR и так всегда читается слово ошибок при опущенном DRQ
           default: begin   
                     start <= 1'b0;
                     done <= 1'b1;
                     interrupt_trigger <= 1'b1;
                     cmdstate <= CMD_START;
                    end   
           endcase   
           
           
         end  // конец блок обработки команд
   end  // конец блок обработки рабочего состояния      
end   // конец всего always-блока

//**********************************************************************************************************************
//*   Контроллер DMA
//*
//* Выполняет команды:
//*   start_loadparm - загрузка блока параметров из памяти хоста 
//*   start_bootparm - загрузка блока параметров сектором 0 дорожки 0 стороны 0 - подготовка к загрузке ОС
//*   start_rd       - запуск операции чтения
//*   start_wr       - запуск операции записи
//**********************************************************************************************************************
always @(posedge wb_clk_i) 
  if ((reset == 1'b1) || (rstreq == 1'b1)) begin
   // Сброс контроллера
   dma_state <= DMA_IDLE;
   dma_req <= 1'b0;
   io_complete <= 1'b0;
   sdbuf_we <= 1'b0;
   sdspi_start <= 1'b0;
   sdreq <= 1'b0;
   sdspi_start <= 1'b0;
   sdspi_write_mode <= 1'b0;
   err_cyl1 <= 1'b0;
  end
  
  else case (dma_state)
  // машина состояний контроллера
    // ожидание команды
    DMA_IDLE: begin
      io_complete <= 1'b0;
      dma_stb_o <= 1'b0;
      dma_req <= 1'b0;
      // команда загрузки параметров 
      if (start_loadparm == 1'b1) begin
        nxp <= 1'b0;
        err_cyl1 <= 1'b0;
        dma_req <= 1'b1;   // запрос DMA
        if (dma_gnt == 1'b1) dma_state <= DMA_LOADPARM1; // получили доступ к шине
      end
      
      // команда чтения сектора
      else if (start_rd == 1'b1) begin   
        nxp <= 1'b0;
        err_cyl1 <= 1'b0;
        sdreq <= 1'b1;  // запрос доступа к SD-карте
        dma_state <= DMA_STARTREAD;
      end
      
      // команда записи сектора
      else if (start_wr == 1'b1) begin   
        nxp <= 1'b0;
        sdreq <= 1'b1;
        err_cyl1 <= 1'b0;
        dma_state <= DMA_STARTWRITE;
      end
      
      else if (start_bootparm == 1'b1) dma_state <= DMA_BOOTPARM;
    end
    
    // загрузка блока параметров - подготовка
    DMA_LOADPARM1: begin
      dmanextstate <= DMA_LOADPARM2;
      dma_state <= DMA_LOADWORD;
      dma_adr_o <= {parm_addr, 1'b0};  // выставляем адрес блока параметров на шину адреса
      end
    // загрузка блока параметров - слово 1   
    DMA_LOADPARM2: begin
      drv <= pdata[1:0];      // номер привода
      hd <= pdata[2];         // головка
      dmanextstate <= DMA_LOADPARM3;  
      dma_adr_o <= {parm_addr+1'b1, 1'b0}; // адрес следующего слова парамтеров
      dma_state <= DMA_LOADWORD;  // переходим к загрузке данных
      end
    // загрузка блока параметров - слово 2   
    DMA_LOADPARM3: begin
      ioadr <= pdata;     // адрес буфера в памяти хоста
      dmanextstate <= DMA_LOADPARM4;
      dma_adr_o <= {parm_addr+2'd2, 1'b0};
      dma_state <= DMA_LOADWORD;
      end
    // загрузка блока параметров - слово 3   
    DMA_LOADPARM4: begin
      cyl <= pdata[14:8];   // цилиндр
      sec <= pdata[3:0]-1'b1; // сектор, сразу уменьшаем его на 1 (сектора 0 нет, допустимы номера 1-10)
      dmanextstate <= DMA_LOADPARM5;
      dma_adr_o <= {parm_addr+2'd3, 1'b0};
      dma_state <= DMA_LOADWORD;
      end
   
    // загрузка блока параметров - слово4 и завершение процесса   
    DMA_LOADPARM5: begin
      dma_req <= 1'b0;      // снимаем запрос DMA
      wordcount <= pdata;   // последний параметр - счетчик слов для обмена
      if (alltrk_mode == 1'b1) dma_state <= DMA_ALLTRK;  // режим полной дорожки
      else begin
         io_complete <= 1'b1;  // подтверждаем окончание выполнения команды
         if (start_loadparm == 0) dma_state <= DMA_IDLE; // ждем снятия запроса
      end   
     end
    
    // обработка запроса на полную дорожку
    DMA_ALLTRK:   begin
      // подмена части параметров
      sec <= 4'b0;     // сектор 0
      wordcount <= 16'd2560; // размер одной дорожки
      io_complete <= 1'b1;
      if (start_loadparm == 0) dma_state <= DMA_IDLE;
     end   
      
    // загрузка одного слова из памяти - старт
    DMA_LOADWORD: begin
      dma_stb_o <= 1'b1;    // строб начала обмена
      dma_we_o <= 1'b0;     // чтение
      dma_timer <= 8'd200;  // взводим таймер ожидания ответа
      dma_state <= DMA_LW_WAITREPLY;
      end
      
    // загрузка одного слова из памяти - ожидание ответа шины   
    DMA_LW_WAITREPLY: begin
      dma_timer <= dma_timer-1'b1;   // таймер--
      if (|dma_timer == 0) begin
         // таймаут шины
         nxp <= 1'b1;    // флаг таймаута
         dma_state <= DMA_IDLE;  // завершаем процесс
      end
      else if (dma_ack_i == 1'b1) begin // получили подтверждение обмена
         pdata <= dma_dat_i;   // вынимаем данные с шины
         dma_adr_o <= dma_adr_o + 2'd2; // адрес++
         dma_stb_o <= 1'b0;    // снимаем строб транзакции
         dma_state <= dmanextstate;  // возвращаемся в вызывающий узел
      end
     end   
     
      // старт чтения блока данных
    DMA_STARTREAD: 
         if (sdack == 1'b1) begin  // получили доступ к SD-карте
            // чтение блока окончено 
            if (sdspi_io_done == 1'b1) begin  
               sdspi_start <= 1'b0;       // снимаем запрос на чтение
               dma_state <= DMA_BUF2HOST_PREPARE;
               dma_req <= 1'b1;  // поднимаем запрос на доступ к шине
            end
            // чтение еще не запущено
            else begin 
               sdcard_addr <= sdaddr;
               sdspi_start <= 1'b1;         // запускаем SDSPI на чтение
               sdspi_write_mode <= 1'b0;
            end   
         end
         
      // подготовка к передаче блока данных из буфера к хосту через DMA
      DMA_BUF2HOST_PREPARE:
         if (dma_gnt == 1'b1) begin  // получили доступ к шине
            sdbuf_addr <= 8'o0;   // начальный адрес в буфере sdspi
            dma_we_o <= 1'b1;     // режим записи данных в буфер
            dma_adr_o <= {ioadr[15:1],1'b0};  // начальный адрес на шине для обмена
            dma_state <= DMA_BUF2HOST;
            dma_timer <= 8'd200;  // взводим таймер ожидания ответа
         end
      // передача одного слова через DMA из буфера в хост-память
      DMA_BUF2HOST: begin
         dma_dat_o <= sdbuf_dataout;  // данные загружаем из буфера  sdspi
         dma_stb_o <= 1'b1;           // поднимаем строб транзакции
         dma_timer <= dma_timer-1'b1;   // таймер--
         if (|dma_timer == 0) begin
            // таймаут шины
            nxp <= 1'b1;    // флаг таймаута
            dma_state <= DMA_IDLE;  // завершаем процесс
         end
         else if (dma_ack_i == 1'b1) dma_state <= DMA_BUF2HOST_NEXT ; // получили ответ с шины - продолжаем
        end   
      
      // продолжение переноса данных в ОЗУ хоста
      DMA_BUF2HOST_NEXT:    begin
         dma_stb_o <= 1'b0;                // снимаем строб транзакции
         sdbuf_addr <= sdbuf_addr + 1'b1;  // переходим к следующему слову в буфере
         dma_adr_o <= dma_adr_o + 2'd2;    // сдвигаем адрес на шине
         wordcount <= wordcount - 1'b1;    // счетчик слов --
         if (wordcount == 16'o1) dma_state <= DMA_SD2HOST_COMPLETE; // переданы все слова - завершаем процесс
         else if (&sdbuf_addr == 1'b1) begin  // доехали до конца блочного буфера
            // переход к следующему сектору
            if (sec != 4'd9) sec <= sec + 1'b1;
            else begin
               sec <= 4'd0;
               if (hd == 1'b0) hd <= 1'b1;
               else begin
                  hd<= 1'b0;
                  if (cyl != 7'd79) cyl <= cyl + 1'b1;
                  else err_cyl1 <= 1'b1;  // выход за границы дискеты
               end   
            end
            ioadr <= ioadr + 16'o1000;  // сдвигаем адрес хост-буфера
            dma_req <= 1'b0;  // освобождаем шину
            dma_state <= DMA_STARTREAD; // и продолжаем чтение секторов
         end   
         else begin 
            // до конца блочного буфера не доехали
            dma_state <= DMA_BUF2HOST;  // продолжаем передавать данные из буфера
            dma_timer <= 8'd200;  // взводим таймер ожидания ответа
         end   
       end
      
      // завершение процедуры чтения данных
      DMA_SD2HOST_COMPLETE: begin
         dma_req <= 1'b0;  // освобождаем шину
         sdreq <= 1'b0;  // освобождаем SD-карту
         io_complete <= 1'b1;  // поднимае флаг завершения команды
         if (start_rd == 1'b0) dma_state <= DMA_IDLE;
         end      
         
      // старт процедуры записи
      DMA_STARTWRITE: begin
         dma_req <= 1'b1;    // запрос на доступ к шине
         if (dma_gnt == 1'b1) begin
            sdcard_addr <= sdaddr;
            sdbuf_addr <= 8'o0; // адрес в буфере sdspi начинается с 0
            dma_we_o <= 1'b0;   // снимаем флаг записи на шину
            sdbuf_we <= 1'b1;   // включаем режим записи буфера
            dma_adr_o <= {ioadr[15:1],1'b0};  // начальный адрес хост-буфера
            dma_timer <= 8'd200;  // взводим таймер ожидания ответа
            dma_state <= DMA_ZEROBUF;
         end
       end   
       
       // очистка буфер sdspi
       DMA_ZEROBUF: begin
            sdbuf_datain <= 16'O0;
            if (&sdbuf_addr != 1'b1) sdbuf_addr <= sdbuf_addr + 1'b1;
            else begin
               dma_state <= DMA_HOST2BUF;
               sdbuf_addr <= 8'o0;
            end
         end   

      // передача одного слова через DMA из хост-памяти в буфер
      DMA_HOST2BUF: begin
         dma_stb_o <= 1'b1;   // поднимаем строб транзакции
         dma_timer <= dma_timer-1'b1;   // таймер--
         if (|dma_timer == 0) begin
            // таймаут шины
            nxp <= 1'b1;    // флаг таймаута
            dma_state <= DMA_IDLE;  // завершаем процесс
         end
         // получен ответ от шины
         else if (dma_ack_i == 1'b1) begin
            dma_state <= DMA_HOST2BUF_NEXT;
            sdbuf_datain <= dma_dat_i;  // вводим в буфер полученное слово
         end   
        end   
        
      // продолжение передачи слова в буфер sdspi   
      DMA_HOST2BUF_NEXT: begin
         dma_stb_o <= 1'b0;  // снимаем строб транзакции
         if (dma_ack_i == 1'b0) begin  // ждем снятия сигнала подтверждения
            sdbuf_addr <= sdbuf_addr + 1'b1;  // сдвигаем адрес блочного буфера sdspi
            dma_adr_o <= dma_adr_o+2'o2;      // сдвигаем адрес на шине хоста
            wordcount <= wordcount - 1'b1;    // счетчик слов --
            if ((&sdbuf_addr == 1'b1) || (wordcount == 16'o1)) begin
               // буфер заполнен до конца
               sdbuf_we <= 1'b0;         // снимаем разрешение записи в буфер sdspi
               dma_req <= 1'b0;            // освобождаем общую шину
               sdspi_start <= 1'b1;      // запускаем SDSPI на запись
               sdspi_write_mode <= 1'b1;
               dma_state <= DMA_BUF2SD;
            end   
            // буфер не заполнен - продолжаем передачу данных
            else begin
               dma_state <= DMA_HOST2BUF;
               dma_timer <= 8'd200;  // взводим таймер ожидания ответа
            end
         end
        end   
        
       // ожидание окончания записи данных на карту 
       DMA_BUF2SD: 
         if (sdspi_io_done == 1'b1) begin 
            // SDSPI закончил запись
            sdspi_start <= 1'b0;         // снимаем запрос на запись
            sdspi_write_mode <= 1'b0;
            if (wordcount == 16'o0) dma_state <= DMA_BUF2SD_COMPLETE; // пепреданы все слова - заканчиваем выполнение команды
            else begin
               // переход к следующему сектору
               if (sec != 4'd9) sec <= sec + 1'b1;
               else begin
                  sec <= 4'd0;
                  if (hd == 1'b0) hd <= 1'b1;
                  else begin
                     hd<= 1'b0;
                     if (cyl != 7'd79) cyl <= cyl + 1'b1;
                     else err_cyl1 <= 1'b1;  // выход за границы дискеты
                  end   
               end
               ioadr <= ioadr + 16'o1000;  // сдвигаем адрес в памяти хоста
               dma_state <= DMA_STARTWRITE; // продолжаем запись данных
            end   
         end   
      // Завершение выполнения команды записи данных   
      DMA_BUF2SD_COMPLETE: begin
         sdreq <= 1'b0;     // освобождаем SD-карту
         io_complete <= 1'b1;  // признак завершения команды
         if (start_wr == 1'b0) dma_state <= DMA_IDLE;
        end      
         
      // установка параметров загрузчоного сектора   
      DMA_BOOTPARM: begin   
         cyl <= 7'o0;   // цилиндр 0 
         hd <= 1'b0;    // головка 0
         sec <= 4'o0;   // сектор 0
         drv <= parm_addr[1:0]; // номер привода
         wordcount <= 16'd256;  // число читаемых слов (1 сектор)
         ioadr <= 16'o0;  // адрес в памяти
         io_complete <= 1'b1;
         if (start_bootparm == 1'b0) dma_state <= DMA_IDLE;
       end   
   endcase   
    
//********************************************
// Вычисление адреса блока на SD-карте
//********************************************
//
// Формат образа диска:
//  10 секторов (512 байт) на дорожк
//  2 головы
//  80 цилиндров
//
// полный абсолютный адрес    
// cyl*20+hd*10+sec   cyl*16+cyl*4 + hd*10 + sec
//
// Смещение головки
wire [3:0] hd_offset = (hd == 1'b0)? 4'd0:4'd10; 
// Смещение цилиндра cyl*20
wire [10:0] cyl_offset = {cyl[6:0], 4'b0} + {2'b0, cyl[6:0], 2'b0};
// Полное смещение от начала образа диска
wire [10:0] fulloffset = cyl_offset + hd_offset + sec;
// Абсолютный адрес блока на карте                        
assign sdaddr = {start_offset[26:13], drv[1:0], fulloffset[10:0]};

endmodule

