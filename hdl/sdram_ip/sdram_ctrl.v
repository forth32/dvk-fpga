`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Description   : SDRAM state control module
//                     SDRAM initialization and timing refresh, read and write control
// Revision      : V1.0
// Additional Comments   :  
// 
////////////////////////////////////////////////////////////////////////////////
module sdram_ctrl(
            clk,rst_n,
            sdram_wr_req,sdram_rd_req,
            sdwr_byte,sdrd_byte,
            sdram_wr_ack,sdram_rd_ack,
            //sdram_busy,
            sdram_init_done,
            init_state,work_state,cnt_clk,sys_r_wn
         );
   //System signal interface
input clk;            //System clock, 50MHz
input rst_n;         //Reset signal, active low


   // SDRAM package interface
input sdram_wr_req;         //System write SDRAM request signal
input sdram_rd_req;         //System read SDRAM request signal
input[8:0] sdwr_byte;      //Burst write SDRAM bytes（1-25个）
input[8:0] sdrd_byte;      //Burst read SDRAM bytes（1-256）
output sdram_wr_ack;      //The system writes the SDRAM response signal as the output valid signal of wrFIFO
output sdram_rd_ack;      //System read SDRAM response signal   

output   sdram_init_done;      //System initialization signal
//output sdram_busy;      // SDRAM busy flag, high indicates busy

   // SDRAM internal interface
output[3:0] init_state;   // SDRAM initialization register
output[3:0] work_state;   // SDRAM working status register
output[8:0] cnt_clk;   //Clock count
output sys_r_wn;      // режим чтения-записи - 0 - запись, 1 - остальное
wire done_200us;      //200us input stable period end flag after power-on
//wire sdram_init_done;   // SDRAM initialization completion flag, high indicates completion
wire sdram_busy;      // SDRAM busy flag, high indicates that SDRAM is in operation
reg sdram_ref_req;      // SDRAM self-refresh request signal
wire sdram_ref_ack;      // SDRAM self-refresh request response signal

`include "sdram_para.v"      // Contains SDRAM parameter definition module


reg[8:0] cnt_clk_r;   // счетчик тактов от начала рабочего цикла
reg cnt_rst_n;         // запрос на сброс вышеописанного счетчика - начало нового цикла

// Временные параметры SDRAM 
parameter   TRP_CLK      = 9'd4,//1,   //TRP=18ns Период запуска саморегенерации
            TRFC_CLK   = 9'd6,//3,   //TRC=60ns Automatic pre-refresh cycle
            TMRD_CLK   = 9'd6,//2,   // Задержка для записи регистра управления
            TRCD_CLK   = 9'd2,//1,   //TRCD=18ns Период строба строк RAS
//            TCL_CLK      = {7'd0, `cas_latency},      //Latency TCL_CLK = 3 CLK, can be set in the initialization mode register
            //TREAD_CLK   = 9'd256,//8,      //Burst read data cycle 8CLK
            //TWRITE_CLK   = 9'd256,//8,     //Burst write data 8CLK
            TDAL_CLK   = 9'd3;      // Задержка ожидания записи

//************************************************************************************
// Счетчик стартовой задержки 200мкс
//   done_200us - флаг, устанавливаемый через 200мкс после сброса
//************************************************************************************
reg[14:0] cnt_200us; 
always @ (posedge clk or negedge rst_n) 
   if(!rst_n) cnt_200us <= 15'd0;
   else if(cnt_200us < 15'd20_000) cnt_200us <= cnt_200us+1'b1;   //count

assign done_200us = (cnt_200us == 15'd20_000);

//************************************************************************************
//* Цепочка инициализации SDRAM
//************************************************************************************
reg[3:0] init_state_r;   // SDRAM initialization status

always @ (posedge clk or negedge rst_n)
   if(!rst_n) init_state_r <= `I_NOP; // сброс - переход в состояние NOP
   else 
      case (init_state_r)
            `I_NOP:    init_state_r <= done_200us ? `I_PRE:`I_NOP;      //прошло 200мкс - переходим в I_PRE, 
            `I_PRE:    init_state_r <= `I_TRP;                           // 1 такт i_PRE, затем I_TRP
            `I_TRP:    init_state_r <= (`end_trp) ? `I_AR1:`I_TRP;      // ждем TRP_CLK тактов
            `I_AR1:    init_state_r <= `I_TRF1;                        // 1 такт запуска саморегенерации
            `I_TRF1:   init_state_r <= (`end_trfc) ? `I_AR2:`I_TRF1;   // ждем окончания саморегенерации TRFC_CLK тактов
            `I_AR2:    init_state_r <= `I_TRF2;                         // 1 такт запуска второй саморегенерации
            `I_TRF2:   init_state_r <= (`end_trfc) ? `I_MRS:`I_TRF2;    // ждем окончания второй саморегенерации TRFC_CLK тактов
            `I_MRS:   init_state_r <= `I_TMRD;                        // настроиваем регистр режимов SDRAM
            `I_TMRD:   init_state_r <= (`end_tmrd) ? `I_DONE:`I_TMRD;   // ждем окончания записи регистра TMRD_CLK тактов
            `I_DONE:   init_state_r <= `I_DONE;                        // DRAM инициализирована - устанавливаем флаг I_DONE
            default: init_state_r <= `I_NOP;                         // выход по ошибкам и сбооям последовательности
            endcase


assign init_state = init_state_r;
assign sdram_init_done = (init_state_r == `I_DONE);      // признак окончания инициализации


//**************************************************************************************************
//* Запуск автоматической саморегенерации каждые 7.5мкс
//**************************************************************************************************
reg[10:0] cnt_7_5us;   //счетчик периода запуска

always @ (posedge clk or negedge rst_n)
   if(!rst_n) cnt_7_5us <= 11'd0;
   else if(cnt_7_5us < 11'd749) cnt_7_5us <= cnt_7_5us+1'b1;   // каждые 7.5мкс поднимаем флаг
   else cnt_7_5us <= 11'd0;   

// триггер запроса саморегенерации   
always @ (posedge clk or negedge rst_n)
   if(!rst_n) sdram_ref_req <= 1'b0;
   else if(cnt_7_5us == 11'd749) sdram_ref_req <= 1'b1;   // установка запроса по флагу
   else if(sdram_ref_ack) sdram_ref_req <= 1'b0;         // сброс запроса по подтверждению

//**************************************************************************************************
// Обработчик рабочих состояний DRAM
//**************************************************************************************************
reg[3:0] work_state_r;   // текущее состояние
reg sys_r_wn;            // флаг активного чтения/записи
// Варианты рабочих состояний 
parameter[3:0]      W_ACTIVE=    4'd1   ;   //Valid, read and write
parameter[3:0]      W_TRCD   =    4'd2   ;   //Waiting effectively
parameter[3:0]      W_IDLE   =    4'd0   ;   //Idle state
//***********************************************************
parameter[3:0]      W_READ=       4'd3   ;   //Read data status
parameter[3:0]      W_CL   =    4'd4;      //Waiting for latency
parameter[3:0]      W_RD   =    4'd5   ;   //Reading data
parameter[3:0]      W_RWAIT=       4'd6   ;   //Precharge wait state after reading is completed
//************************************************************
parameter[3:0]      W_WRITE   =    4'd7   ;   //Write data status
parameter[3:0]      W_WD      = 4'd8   ;   //Write data
parameter[3:0]      W_TDAL   =    4'd9   ;   //Waiting to write data and end self-refresh
//*************************************************************
parameter[3:0]      W_AR      = 4'd10   ;   //Self refresh
parameter[3:0]      W_TRFC   =    4'd11;      //Self-refresh waiting

always @ (posedge clk or negedge rst_n) begin
   if(!rst_n) 
      work_state_r <= W_IDLE; // начальное состояние - IDLE
   else begin
      // автомат переключения рабочих состояний   
      case (work_state_r)  
      //-----------------------------------------------------------------
      // Начальное состояние - нет выполняемых операций
      //-----------------------------------------------------------------
      W_IDLE:   if(sdram_ref_req & sdram_init_done) 
                  // запрос саморегенерации
                  begin
                  work_state_r <= W_AR;       //переходим в режим W_AR, саморегенерация
                  sys_r_wn <= 1'b1;
                  end       
               else if(sdram_wr_req & sdram_init_done)
                  // запрос на запись   
                  begin
                  work_state_r <= W_ACTIVE;   // активный режим чтения-записи
                  sys_r_wn <= 1'b0;    // флаг записи
                  end                                 
               else if(sdram_rd_req && sdram_init_done) 
                  // запрос на чтение
                  begin
                  work_state_r <= W_ACTIVE;   // активный режим чтения-записи
                  sys_r_wn <= 1'b1;   // флаг не-записи
                  end
               else 
                  begin 
                  work_state_r <= W_IDLE;
                  sys_r_wn <= 1'b1; // для остальных режимов устанавливаем флаг на не-запись.
                  end
      //-----------------------------------------------------------------
      // Активный режим чтения-записи 
      //-----------------------------------------------------------------
      W_ACTIVE:    if(TRCD_CLK == 0)  // время ожидания строба адреса  строки
                   if(sys_r_wn) work_state_r <= W_READ; // режим чтения
                   else work_state_r <= W_WRITE;        // режим записи
               else work_state_r <= W_TRCD;             // ражим ожидания строба строки
      //-----------------------------------------------------------------
      // ожидание строба строки RAS
      //-----------------------------------------------------------------
      W_TRCD:    if(`end_trcd)
                   if(sys_r_wn) work_state_r <= W_READ; // jожидание окончено - выставляем режим чтения
                   else work_state_r <= W_WRITE;        // или режим записи
               else work_state_r <= W_TRCD;             // ждем окончания RAS
               
      //-----------------------------------------------------------------
      // Режим чтения
      //-----------------------------------------------------------------
      W_READ:   work_state_r <= W_CL;   
      // Ждем CL - задержка между RAS и CAS
      W_CL:      work_state_r <= (`end_tcl) ? W_RD:W_CL;   
      // Входим в режим чтения
      W_RD:      work_state_r <= (`end_tread) ? W_IDLE:W_RD;
      // Ожидание закрытия строки   
      W_RWAIT:   work_state_r <= (`end_trwait) ? W_IDLE:W_RWAIT;
      
      //-----------------------------------------------------------------
      // Режим записи
      //-----------------------------------------------------------------
      W_WRITE:   work_state_r <= W_WD;
      // запись данных
      W_WD:      work_state_r <= (`end_twrite) ? W_TDAL:W_WD;
      // Ожидание окончания записи
      W_TDAL:   work_state_r <= (`end_tdal) ? W_IDLE:W_TDAL;
      
      //-----------------------------------------------------------------
      // Режим саморегенерации
      //-----------------------------------------------------------------
      W_AR:      work_state_r <= W_TRFC; 
      // ожидание окончания саморегенерации
      W_TRFC:   work_state_r <= (`end_trfc) ? W_IDLE:W_TRFC;
      /*************************************************************/
      default:    work_state_r <= W_IDLE;
      endcase
      end
end

assign work_state = work_state_r;      // Текущее состояние системы
assign sdram_ref_ack = (work_state_r == W_AR);      // Сигнал подтверждения входа в режим саморегенерации



//------------------------------------------------------------------------------
// Формирователи сигналов окончания чтения-записи
//------------------------------------------------------------------------------

// запись
assign sdram_wr_ack =    ((work_state == W_TRCD) & ~sys_r_wn) | 
                  (work_state == W_WRITE)|
                  ((work_state == W_WD) & (cnt_clk_r < sdwr_byte -2'd2));      

// чтение
assign sdram_rd_ack =    (work_state_r == W_RD) & 
                  (cnt_clk_r >= 9'd1) & (cnt_clk_r < sdrd_byte + 2'd1);      

//assign sdram_busy = (sdram_init_done && work_state_r == W_IDLE) ? 1'b0:1'b1;   // SDRAM busy flag

//*************************************************************************
//* Счетчик тактов рабочего цикла
//*************************************************************************
always @ (posedge clk or negedge rst_n) 
   if(!rst_n) cnt_clk_r <= 9'd0;   
   else if(!cnt_rst_n) cnt_clk_r <= 9'd0;   // По флагу cnt_rst_n сбрасываем счетчик и начинаем новый цикл
   else cnt_clk_r <= cnt_clk_r+1'b1;      // продвигаем счетчик каждый такт
   
assign cnt_clk = cnt_clk_r;         //Count register is taken out and used in internal `define

//*************************************************************************
//*  Формирователь сигнала начала нового цикла (сброс счетчика тактов)
//*************************************************************************
always @ (init_state_r or work_state_r or cnt_clk_r or sdwr_byte or sdrd_byte) begin
   case (init_state_r)
          `I_NOP:   cnt_rst_n <= 1'b0;
         `I_PRE:   cnt_rst_n <= 1'b1;                              //начало цикла закрытия строки
         `I_TRP:   cnt_rst_n <= (`end_trp) ? 1'b0:1'b1;            //конец цикла закрытия строки
          `I_AR1,`I_AR2:  cnt_rst_n <= 1'b1;                        // Начало саморегенерации
          `I_TRF1,`I_TRF2: cnt_rst_n <= (`end_trfc) ? 1'b0:1'b1;   //конец саморегенерации
         `I_MRS:   cnt_rst_n <= 1'b1;                              //Начало настройки управляющего регистра
         `I_TMRD:   cnt_rst_n <= (`end_tmrd) ? 1'b0:1'b1;   //Clear the counter after waiting for the self-refresh delay count to end
         `I_DONE:
            // конец цикла инициализации - управляем командами рабочего цикла
            begin
            case (work_state_r)
            W_IDLE:   cnt_rst_n <= 1'b0;
             // перезапускаем счетчик по любой активной команде
            W_ACTIVE:    cnt_rst_n <= 1'b1;
            W_TRCD:   cnt_rst_n <= (`end_trcd) ? 1'b0:1'b1;
            W_CL:      cnt_rst_n <= (`end_tcl) ? 1'b0:1'b1;
            W_RD:      cnt_rst_n <= (`end_tread) ? 1'b0:1'b1;
            W_RWAIT:   cnt_rst_n <= (`end_trwait) ? 1'b0:1'b1;
            W_WD:      cnt_rst_n <= (`end_twrite) ? 1'b0:1'b1;
            W_TDAL:   cnt_rst_n <= (`end_tdal) ? 1'b0:1'b1;
            W_TRFC:   cnt_rst_n <= (`end_trfc) ? 1'b0:1'b1;
            default: cnt_rst_n <= 1'b0;
            endcase
            end
      default: cnt_rst_n <= 1'b0;
      endcase
end

endmodule
