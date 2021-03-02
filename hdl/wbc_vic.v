//******************************************************  
//*  Контроллер прерываний
//******************************************************  

module wbc_vic 
#(parameter N=1)      // количество векторов
(
   input                wb_clk_i,   // system clock
   input                wb_rst_i,   // peripheral reset
   output reg           wb_irq_o,   // запрос прерывания к процессору
   output reg  [15:0]   wb_dat_o,   // шина для передачи вектора прерывания к процессору
   input                wb_stb_i,   // строб от прецессора для приема вектора
   output reg           wb_ack_o,   // подтверждение передачи вектора процессору
                                    //
   input    [N*16-1:0]  ivec,       // массив значений векторов прерывания
   input       [N-1:0]  ireq,       // линии запроса прерывания
   output reg  [N-1:0]  iack        // линии подтверждения прерывания
);
localparam W = log2(N); // число бит для записи номера вектора

reg   [W-1:0] nvec; // номер прерывания с наивысшим приоритетом, или 0 если активных прерываний нет.
integer i;

always @(posedge wb_clk_i or posedge wb_rst_i)
begin
   if (wb_rst_i)
   begin
      // сброс контроллера
      wb_ack_o <= 1'b0;
      wb_dat_o <= 16'O000000;
      iack     <= 0;
      nvec     <= {(W){1'b1}};  // nvec заполнен единицами при отсутствии прерывания
   end
   else
   // обработка запросов на прерывания
   begin
      // сигнал подтверждения запроса вектора 
      wb_ack_o <= wb_stb_i &  // в ответ на строб от процессора
                  wb_irq_o &  // если мы выставляем вектор 
                 ~wb_ack_o;   // если строб уже был выставлен в предыдущем такте - то снимаем его

      // сигнал запроса векторного прерывания         
      wb_irq_o <= ~(&nvec) &  // если есть активный ветор
                  ~wb_ack_o;  // и если вектор еще не передан

      // сигнал подтверждения прерывания к периферийному устройству    
      for (i=N-1; i>=0; i=i-1)
         iack[i] <= 
            (nvec == i)    // это текущая линия обработки прерывания
             & ireq[i]     // был запрос на прерывание 
             & wb_stb_i    // был строб от процессора - он готов принять вектор
             & wb_irq_o    // уже выставлен запрос векторного прерывания к процессору
             & ~iack[i];   // если сигнал в предыдущем такте выставлен - снимаем

      // выставляем вектор на шину
      if (wb_stb_i & ~wb_ack_o & wb_irq_o) // пришел строб от прецессора и еще не выставлено подтверждение
            // передача вектора прерывания
            wb_dat_o <= trunc_w16(ivec >> (nvec*16));

      if (~wb_stb_i)
      begin
         nvec <= {(W){1'b1}};
         for (i=N-1; i>=0; i=i-1)
            if (ireq[i]) nvec <= trunc_int(i);
      end
      else
         if (wb_ack_o) nvec <= {(W){1'b1}};
   end
end

function integer log2(input integer value);
   for (log2=0; value>0; log2=log2+1)
      value = value >> 1;
endfunction

function [15:0] trunc_w16(input [N*16-1:0] value);
   trunc_w16 = value[15:0];
endfunction

function [W-1:0] trunc_int(input integer value);
   trunc_int = value[W-1:0];
endfunction
endmodule
