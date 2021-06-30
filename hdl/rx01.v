module rx01 (

// шина wishbone
   input                  wb_clk_i,   // тактовая частота шины
   input                  wb_rst_i,   // сброс
   input    [1:0]         wb_adr_i,   // адрес 
   input    [15:0]        wb_dat_i,   // входные данные
   output reg [15:0]      wb_dat_o,   // выходные данные
   input                  wb_cyc_i,   // начало цикла шины
   input                  wb_we_i,    // разрешение записи (0 - чтение)
   input                  wb_stb_i,   // строб цикла шины
   input    [1:0]         wb_sel_i,   // выбор конкретных байтов для записи - старший, младший или оба
   output reg             wb_ack_o,   // подтверждение выбора устройства

// обработка прерывания   
   output reg             irq,        // запрос
   input                  iack,       // подтверждение
   
// интерфейс SD-карты
   output                 sdcard_cs, 
   output                 sdcard_mosi, 
   output                 sdcard_sclk, 
   input                  sdcard_miso, 
   
   output reg             sdreq,      // запрос доступа к карте
   input                  sdack,      // подтверждение доступа к карте
   input                  sdmode,     // режим SDSPI
   
// тактирование SD-карты
   input sdclock,   
   
// Адрес начала банка на карте
   input [26:0] start_offset,
   
// отладочные сигналы
   output [3:0] sdcard_debug
   ); 
//-----------------------------------------------
//  Регистры контроллера
//
// 177170  DXCSR  - регистр управления/состояния
//                    D0      W    go - запуск команды
//                    D1-D3   W    код команды:
//                         000 - запись буфера 
//                         001 - чтение буфера 
//                         010 - запись сектора на диск
//                         001 - чтение сектора с диска
//                         100 -  
//                         101 - чтение регистра состояния
//                         110 - запись сектора с признаком удаления
//                         111 - чтение регистра ошибок
//                    D4    R/W   выбор дисковода
//                    D5    R     DONE, признак завершения операции
//                    D6    R/W   IDE, разрешение прерывания
//                    D7    R     DRQ  готовоность данных для чтения-записи
//                    D8-D13 ----------
//                    D14   W     сброс
//                    D15   R     ошибка
//
// 177172  DXBUF  - буферный регистр данных
//
//  номер дорожки - D0-D6 (0-114/8)
//  номер сектора - D0-D4 (1-32/8)
//
//         DXES - регистр ошибок и состояния:
//              D0 - CRC, ошибка CRC в данных
//              D1 - PAR, ошибка четности в данных
//              D2 - ID, инициализация закончена
//              D6 - DD, признак маркера удаленных данных
//              D7 - RDY, готовность привода
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
reg done;     // операция завершена
reg ide;
reg drq;

// CHS
reg [6:0] cyl;
reg [4:0] sec;
reg drv;

reg start; 
reg [2:0] cmd;

wire[21:0] bank_offset;   
reg cmderr;
reg rstreq;
reg sec_phase;
reg trk_phase;
reg io_phase;

reg delflag;     // признак удаленного сектора
      
// интерфейс к SDSPI
wire [26:0] sdaddr;   // адрес сектора карты
reg  [26:0] sdcard_addr;   // адрес сектора карты
wire sdspi_io_done;        // флаг окончагия чтения
reg  sdspi_write_mode;     //  выбо режима чтения-записи
wire sdcard_error;         // флаг ошибки
wire [15:0] sdbuf_dataout; // слово; читаемое из буфера чтения
wire sdcard_idle;          // признак готовности контроллера
reg  sdspi_start;          // строб запуска sdspi
reg [7:0] sdbuf_addr;      // адрес в буфере чтния/записи
reg [15:0] sdbuf_datain;   // слово; записываемое в буфер записи
reg donetrigger;             
reg sdbuf_write;

// состояния процесса обмена с sdspi
reg [1:0] iostate;
parameter[1:0] io_start=0;
parameter[1:0] io_wait=1;
parameter[1:0] io_ack=2;
parameter[1:0] io_done=3;

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
      .sdspi_start(sdspi_start),                  // строб начала чтения
      .sdspi_io_done(sdspi_io_done),              // флаг окончания чтения
      .sdspi_write_mode(sdspi_write_mode),        // строб начала записи
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
   //******************************
   //* обработка прерывания
   //******************************
   case (interrupt_state)
                // нет активного прерывания
              i_idle :
                        begin
                     //  Если поднят флаг - переходим в состояние активного прерывания
                           if ((ide == 1'b1) & (interrupt_trigger == 1'b1))  begin
                              interrupt_state <= i_req ; 
                              irq <= 1'b1 ;    // запрос на прерывание
                           end 
                           else   irq <= 1'b0 ;    // снимаем запрос на прерывания
                        end
               // Формирование запроса на прерывание         
               i_req :     if (ide == 1'b0)    interrupt_state <= i_idle ;    
                           else if (iack == 1'b1) begin
                                 // если получено подтверждение прерывания от процессора
                                 irq <= 1'b0 ;               // снимаем запрос
                                 interrupt_trigger <= 1'b0;
                                 interrupt_state <= i_wait ; // переходим к ожиданию окончания обработки
                           end 
               // Ожидание окончания обработки прерывания         
               i_wait :
                           if (iack == 1'b0)  interrupt_state <= i_idle ; 
    endcase

//**************************************************
// Работа с шиной и SDSPI
//**************************************************
    if (reset == 1'b1 || rstreq == 1'b1) begin
       // сброс системы
        interrupt_state <= i_idle ; 
        irq <= 1'b0 ;    // снимаем запрос на прерывания
        start <= 1'b0 ; 
        done <= 1'b0;
        donetrigger <= 1'b0;
        ide <= 1'b0;
        drv <= 1'b0;
        cmderr <= 1'b0;
        drq <= 1'b0;
        rstreq <= 1'b0;
      cyl <= 7'o0;
      sec <= 5'o0;
      sdspi_start <= 1'b0;
      sdspi_write_mode <= 1'b0;
      iostate <= io_start;
      cmd <= 3'b100;
      sec_phase <= 1'b0;
      trk_phase <= 1'b0;
      io_phase <= 1'b0;
      interrupt_trigger <= 1'b1;
      delflag <= 1'b0;
      sdreq <= 1'b0;
      sdbuf_write <= 1'b0;
    end
      
   // рабочие состояния
    else   begin
            
         //*********************************************
         //* Обработка шинных транзакций 
         //*********************************************
            // чтение регистров
            if (bus_read_req == 1'b1)   begin
               case (wb_adr_i[1])
                  1'b0 : begin  // 177170 - DXCSR
                                      //  15                 7     6    5
                           wb_dat_o <= {cmderr, 1'b0, 6'b0, drq, ide, done, 5'b0};   
                           // эмуляция сброса-установки бита done после сброса
                           if (reply & !donetrigger) begin
                              donetrigger <= 1'b1;
                              done <= 1'b1;
                           end   
                         end      
                  1'b1 :   // 177172 - DXBUF
                             case(cmd) 
                              // чтение буфера                                
                              3'b001: begin  
                                wb_dat_o <= {8'b00000000, sdbuf_dataout[7:0]};
                                if (drq & reply) begin 
                                  sdbuf_addr <= sdbuf_addr + 1'b1;
                                  if (sdbuf_addr == 8'b01111111) begin
                                    drq <= 1'b0;
                                    done <= 1'b1;
                                    interrupt_trigger <= 1'b1;
                                    start <= 1'b0;
                                  end  
                                end
                              end   
                              // чтение регистра состояния RXES
                              3'b101: begin 
                                       wb_dat_o <= {8'o0, sdcard_idle, delflag,  3'o0, 1'b1, 1'b0, 1'b0};
                                      end      
                              // чтение регистра ошибок 
                              3'b111: begin
                                       wb_dat_o<=16'o0;
                                      end       
                              default: wb_dat_o<=16'o0;
                             endcase
               endcase 

               end
         
            // запись регистров   
            else if (bus_write_req == 1'b1)  begin
                if (wb_sel_i[0] == 1'b1)  
                    // запись младших байтов
                    case (wb_adr_i[1])
                     // 177170 - DXCSR
                     1'b0:  
                            if (reply) begin
                                // принят бит GO при незапущенной операции
                                if ((start == 1'b0) && (wb_dat_i[0] == 1'b1)) begin 
                                    // Ввод новой команды
                                    sdbuf_write <= 1'b0;
                                    start <= 1'b1;              // признак активной команды
                                    done <= 1'b0;               // сбрасываем признак завершения команды
                                    drq <= 1'b0;
                                    cmd <= wb_dat_i[3:1];       // код команды
                                    cmderr <= 1'b0;             // сбрасываем ошибки
                                    iostate <= io_start;        // начальная фаза взаимодействия с SDSPI
                                    drv <= wb_dat_i[4];         // номер устройства
                                    interrupt_trigger <= 1'b0;  // снимаем ранее запрошенное прерывание
                                    // установка флагов файзы ввода команды согласно протокола (сектор - цилиндр - начало выполнения)
                                    io_phase <= 1'b0;            
                                    trk_phase <= 1'b0;
                                    sec_phase <= 1'b1;
                                end         
                                ide <= wb_dat_i[6];            // флаг разрешения прерывания - доступен для записи всегда
                                if ((wb_dat_i[6] == 1'b1) && (done == 1'b1)) interrupt_trigger <= 1'b1;
                            end
                    // 177172 - DXBUF
                     1'b1 :   
                             case(cmd)
                              // запись буфера
                              3'b000:  begin
                                 if (drq) begin
                                  sdbuf_datain<= {8'b00000000, wb_dat_i[7:0]}; 
                                   sdbuf_write <= 1'b1;
                                  if (!reply) sdbuf_addr <= sdbuf_addr + 1'b1;
                                  else  begin  
                                    if (sdbuf_addr == 8'b01111111) begin
                                       drq <= 1'b0;
                                       done <= 1'b1;
                                       interrupt_trigger <= 1'b1;
                                       start <= 1'b0;
                                    end  
                                  end
                                 end 
                              end
                              
                              // чтение и запись секторов
                              3'b010,3'b011:
                               // фаза чтения номера сектора
                               if (sec_phase) begin
                                sec <= wb_dat_i[4:0];
                                if (reply) begin
                                  sec_phase <= 1'b0;
                                  trk_phase <= 1'b1;
                                end
                               end  
                              
                               // фаза чтения номера дорожки
                               else if (trk_phase) begin
                                cyl <= wb_dat_i[6:0];
                                if (reply) begin
                                  trk_phase <= 1'b0;
                                  io_phase <= 1'b1;
                                  drq <= 1'b0;
                                end
                               end  
                           endcase
                  endcase 
            
               if ((wb_sel_i[1] == 1'b1) && (wb_adr_i[1] == 1'b0))  begin
                    // запись старших байтов
                    rstreq <= wb_dat_i[14];
               end 
            end
            
         //*********************************************
         // запуск команды
         //*********************************************
           if (start == 1'b1)  begin
           case (cmd)  // выбор действия по коду функции 
            // запись буфера, чтение буфера      
            3'b000,3'b001: begin
                           if (drq == 0) begin
                                drq <= 1'b1;
                                sdbuf_addr <= 8'o0;
                           end
                        end
                        
            // запись сектора, запись удаленного сектора
            3'b010,3'b110:
                         if (sec_phase == 1'b1) drq <= 1'b1;
                         else if (io_phase == 1'b1) begin
                          case (iostate) 
                     // запуск записи
                     //  номер дорожки - D0-D6 (0-114/8)
                     //  номер сектора - D0-D4 (1-32/8)
                     io_start: begin
                           // проверка допустимости значений цилиндра и сектора
                           if ((cyl > 7'o114) || (sec>5'o32) || (sec == 0)) begin
                              cmderr<=1'b1;
                              done <= 1'b1;
                              start <= 1'b0;
                              interrupt_trigger <= 1'b1;
                           end   
                           else begin
                              sdreq <= 1'b1;   // запрос доступа к карте
                              // запись маркера удаленного сектора
                              sdbuf_addr <= 8'd255;
                              sdbuf_datain <= {15'o0,cmd[2]};  // cmd[2]=0 для обычных секторов, 1 для удаленных
                              // получен ответ sdack
                              if (sdack) begin
                                 sdcard_addr <= sdaddr;
                                 sdspi_write_mode=1'b1;
                                 sdspi_start <= 1'b1 ;  // запускаем SDSPI
                                 iostate <= io_wait;
                              end   
                           end   
                        end   
                        
                     // ожидание окончание заиси сектора на карту   
                     io_wait:
                           if (sdspi_io_done == 1'b1) iostate <= io_done;
                           
                     // запись подтверждена - освобождаем sdspi
                     io_done: begin
                        sdspi_start <= 1'b0 ;              // снимаем строб записи
                        sdspi_write_mode=1'b0;
                        done <= 1'b1;                       // флаг завершения команды
                        interrupt_trigger <= 1'b1;
                        start <= 1'b0;                     // заканчиваем обработку команды
                        cmderr <= sdcard_error;
                        iostate <= io_start;
                        sdreq <= 1'b0;
                     end   
                  endcase   
                 end
            
                
            // чтение сектора
                3'b011:   
                         if (sec_phase == 1'b1) drq <= 1'b1;
                         else if (io_phase == 1'b1) begin
                          case (iostate) 
                     // запуск чтения
                     //  номер дорожки - D0-D6 (0-114/8)
                     //  номер сектора - D0-D4 (1-32/8)
                     io_start: begin
                           // проверка допустимости значений цилиндра и сектора
                           if ((cyl > 7'o114) || (sec>5'o32) || (sec == 0)) begin
                              cmderr<=1'b1;
                              done <= 1'b1;
                              start <= 1'b0;
                              interrupt_trigger <= 1'b1;
                           end   
                           else begin
                              sdreq <= 1'b1;   // запрос доступа к карте
                              if (sdack) begin
                                 sdcard_addr <= sdaddr;
                                 sdspi_start <= 1'b1 ; 
                                 sdspi_write_mode=1'b0;
                                 iostate <= io_wait;
                              end   
                           end   
                        end   
                        
                     // ожидание окончания чтения сектора
                     io_wait:
                           if (sdspi_io_done == 1'b1) iostate <= io_done;
                           
                     // освобождаем sdspi
                     io_done: begin
                        sdspi_start <= 1'b0 ;               // снимаем строб записи
                        done <= 1'b1;                      // флаг завершения команды
                        interrupt_trigger <= 1'b1;
                        start <= 1'b0;                     // заканчиваем обработку команды
                        iostate <= io_start;
                        sdreq <= 1'b0;
                        // чтение флага удаленного сектора
                        sdbuf_addr <= 8'd255;
                        delflag <= sdbuf_datain[0];        // 0 для обычеых секторов, 1 для удаленных
                        
                     end   
                  endcase   
                 end
                
            // чтение регистров состояния и ошибок
            3'b101, 3'b111: begin
                      start <= 1'b0;
                      done <= 1'b1;
                      interrupt_trigger <= 1'b1;
                     end 
            // ошибочная команда               
            3'b100: begin                           
                     start <= 1'b0;
                     cmderr <= 1'b1;
                     done <= 1'b1;
                     interrupt_trigger <= 1'b1;
                   end   
                    
           endcase 
          end
   end 
end 

//********************************************
// Вычисление адреса блока на SD-карте
//********************************************
//
// Формат образа диска:
//  25 секторов (128 байт) на дорожку(1-26)
//  76 цилиндров
//
//reg [6:0] cyl;
//reg [4:0] sec;
//
// полный абсолютный адрес 
assign sdaddr = start_offset + {10'b0,drv,cyl,sec[4:0]};

endmodule
