//  Контроллер ИРПР для подключения принтера
//
module irpr
(
   input                wb_clk_i,   
   input                wb_rst_i,   
                                    
   input  [1:0]         wb_adr_i,   
   input  [15:0]        wb_dat_i,   
   output reg [15:0]    wb_dat_o,   
   input                wb_cyc_i,   
   input                wb_we_i,    
   input                wb_stb_i,   
   output reg           wb_ack_o,   
   
   output reg           irq,        // Запрос прерывания
   input                iack,       // Подтверждение прерывания
   
   // интерфейс принтера
   output reg [7:0]     lp_data,    // данные для передачи к принтеру
   output reg           lp_stb_n,   // строб записи в принтер
   output               lp_init_n,  // строб сброса
   input                lp_busy,    // сигнал занятости принтера
   input                lp_err_n    // сигнал ошибки
                                    
);
//
// Регистры ИРПР
// 
//  CSR=177514
//  DAT=177516
//  вектор 200
//
// Формат CSR:
//
// 15 R   ERROR - флаг ошибка
// 14 W   RESET - сброс внешнего устройства
// 13
// 12
// 11
// 10
// 09
// 08
// 07 R   DRQ   - требование передачи
// 06 RW  IE    - разрешение прерывания
// 05 R   DONE  - флаг завершения передачи
// 04
// 03
// 02
// 01
// 00


reg [15:0] dat;
wire [15:0] csr;
reg drq;
reg done;
reg [7:0] reset_delay;
wire csr_wstb;
wire dat_wstb;

reg busy;
reg err_n;

reg interrupt_trigger;     // триггер запроса прерывания
reg ie;                    // флаг разрешения прерывания

// регистры фильтрации входных сигналов
reg[3:0] busy_filter; 
reg[3:0] err_filter; 

// состояние машины обработки прерывания
parameter[1:0] i_idle = 0; 
parameter[1:0] i_req = 1; 
parameter[1:0] i_wait = 2; 
reg[1:0] interrupt_state; 

// сигнал сброса принтера -INIT
assign lp_init_n=(|reset_delay)? 1'b0:1'b1;

// Формирователь данных при чтении CSR
assign csr = {~err_n, 7'o0, drq, ie, done, 5'o0};

// Стробы записи в регистры
assign csr_wstb = wb_cyc_i & wb_stb_i &  wb_we_i &  wb_ack_o & (wb_adr_i[1] == 1'b0);
assign dat_wstb = wb_cyc_i & wb_stb_i &  wb_we_i &  wb_ack_o & (wb_adr_i[1] == 1'b1);


// Сигнал REPLY
always @(posedge wb_clk_i)
   wb_ack_o <= wb_cyc_i & wb_stb_i & ~wb_ack_o;

// Фильтрация входных сигналов
always @(posedge wb_clk_i or posedge wb_rst_i) 
   if (wb_rst_i)    begin
     busy <= 1'b0;
     err_n <= 1'b1;
   end
   else begin   
      busy_filter <= {busy_filter[2:0], lp_busy} ;  // фильтр сигнала занятости
      err_filter <= {err_filter[2:0], lp_err_n} ;   // фильтр сигнала ошибки

      if (busy_filter == {4{1'b0}})     busy <= 1'b0 ; 
      else if (busy_filter == {4{1'b1}}) busy <= 1'b1 ;
      if (err_filter == {4{1'b0}})     err_n <= 1'b0 ; 
      else if (err_filter == {4{1'b1}}) err_n <= 1'b1 ;
   end

   
   
always @(posedge wb_clk_i or posedge wb_rst_i) 
   if (wb_rst_i)    begin
   // сброс 
      ie    <= 1'b0;
      irq <= 1'b0;
      reset_delay <= 8'hff;
      drq <= 1'b0;             // начальное значение сигнала DRQ - 0
      done <= 1'b0;          
      lp_stb_n <= 1'b1;        // строб данных
      wb_dat_o <= 16'h0000;
      interrupt_trigger <= 1'b0;
   end
//**************************************************
// Логика обработки прерываний 
//**************************************************
   else begin
     case (interrupt_state)
                // нет активного прерывания
              i_idle :
                        begin
                     //  Если поднят флаг A или B - поднимаем триггер прерывания
                           if ((ie == 1'b1) & (interrupt_trigger == 1'b1))  begin
                              interrupt_state <= i_req ; 
                              irq <= 1'b1 ;    // запрос на прерывание
                           end 
                           else   irq <= 1'b0 ;    // снимаем запрос на прерывания
                        end
               // Формирование запроса на прерывание         
               i_req :     if (ie == 1'b0)    interrupt_state <= i_idle ;    
                           else if (iack == 1'b1) begin
                              // если получено подтверждение прерывания от процессора
                              irq <= 1'b0 ;               // снимаем запрос
                              interrupt_trigger <= 1'b0;
                              interrupt_state <= i_wait ; // переходим к ожиданию окончания обработки
                           end 
               // Ожидание окончания обработки прерывания         
               i_wait :    if (iack == 1'b0)  interrupt_state <= i_idle ; 
      endcase
   
//**************************************************
// логика работы с шиной и принтером
//**************************************************

       // таймер сброса принтера
     if (|reset_delay) reset_delay <= reset_delay - 1'b1;

     // Чтение регистров
     if (wb_cyc_i & wb_stb_i & ~wb_ack_o & ~wb_adr_i[1]) begin 
        // 177514 - CSR
        wb_dat_o <= csr;  
        done <= 1'b0; // снимаем признак done
     end     
     else wb_dat_o <= 16'o000000; // нет чтения или чтение регистра данных
   
     // запись регистра CSR   
     if (csr_wstb)  begin
         ie  <= wb_dat_i[6];  // разрешение прерывания
         reset_delay <= (wb_dat_i[14]? 8'hff:8'h00);  // сброс принтера
     end  

     // Запись регистра данных
     if ((drq == 1'b1) & (dat_wstb == 1'b1) & (busy == 1'b0) & (err_n == 1'b1)) begin // если контроллер и принтер готовы к приему данных
         drq <= 1'b0;        // снимаем DRQ
         lp_data <= wb_dat_i[7:0];  // выставляем данные 
         done <= 1'b0;       // снимаем признак done   
         lp_stb_n <= 1'b0;          // строб к принтеру
     end
     
     if ((drq == 1'b0) & (lp_stb_n == 1'b0) & (busy == 1'b1))  lp_stb_n <= 1'b1;  // принтер ушел в busy - снимаем строб
     if ((drq == 1'b0) & (lp_stb_n == 1'b1) & (busy == 1'b0))  begin
       // принтер снял busy
       drq  <= 1'b1;   // поднимаем DRQ
       done <= 1'b1;   // поднимаем флаг done
       interrupt_trigger <= 1'b1;   // вызываем прерывание
      end    
   end 

endmodule
