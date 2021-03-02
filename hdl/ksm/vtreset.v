`define    DCLO_WIDTH_CLK         5
`define   ACLO_DELAY_CLK         3

//*****************************************************
//* Модуль генерации сбросов
//*****************************************************
module vtreset (
   input    clk,     // тактовая частота 50 МГц
   input    rstin,   // кнопка сброса, 0 - сброс, 1 - работа
   output   dclo,    // авария DC
   output   aclo,    // авария AC
   output reg irq50  // прерывания с частотой 50 Гц
);
localparam DCLO_COUNTER_WIDTH = log2(`DCLO_WIDTH_CLK);
localparam ACLO_COUNTER_WIDTH = log2(`ACLO_DELAY_CLK);

reg [DCLO_COUNTER_WIDTH-1:0] dclo_cnt;
reg [ACLO_COUNTER_WIDTH-1:0] aclo_cnt;
reg [1:0] reset;
reg [18:0] intcount;      // счетчик для генерации прерываний
reg aclo_out, dclo_out;

assign dclo = dclo_out;
assign aclo = aclo_out;

always @(posedge clk) begin
   //
   // Синхронизация сигнала сброса для предотвращения метастабильности
   //
   reset[0] <= rstin; 
   reset[1] <= reset[0];
   
   if (reset[1])   begin
      dclo_cnt     <= 0;
      aclo_cnt     <= 0;
      aclo_out      <= 1'b1;
      dclo_out      <= 1'b1;
      intcount    <= 19'd000000;
      irq50       <= 1'b0;
   end
   else  begin
      //
      // Счетчик задержки DCLO
      //
      if (dclo_cnt != `DCLO_WIDTH_CLK)   dclo_cnt <= dclo_cnt + 1'b1;
      else  dclo_out <= 1'b0;  // снимаем DCLO
         
      //
      // Счетчик задержки ACLO
      //
      if (~dclo_out)
         if (aclo_cnt != `ACLO_DELAY_CLK) aclo_cnt <= aclo_cnt + 1'b1;
         else   aclo_out <= 1'b0;
         
      // генерация импульсов прерывания
      if (|intcount == 1'b0) begin
         intcount <= 19'd500000;      
         irq50 <= ~irq50;
      end
      else intcount <= intcount-1'b1;   
  
   end
end

// получение количества двоичных разрядов в  числе
function integer log2(input integer value);
   begin
      for (log2=0; value>0; log2=log2+1) 
         value = value >> 1;
   end
endfunction
endmodule

