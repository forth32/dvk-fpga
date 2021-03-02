//**************************************************************
//*   Моудль генерации сбросов
//**************************************************************
module wbc_rst #(parameter

   OSCCLK=50000000,           // входяшая тактовая частота платы
   PWR_WIDTH=10,              // minimal Power Reset width in system ticks
   DCLO_WIDTH=15,             // длительность импльса DCLO
   ACLO_DELAY=7,              // задержка снятия ACLO после DCLO
   DEBOUNCE=5                 // задержка для подавления дребезга кнопки сброса в миллисекундах
)
(
   input       osc_clk,       // входяшая тактовая частота платы
   input       sys_clk,       // тактовая частота процессора
   input       pll_lock,      // готовность PLL
   input       button,        // сигнал аппаратного сброса (кнопка)
   input       sys_ready,     // вход готовности внешних системных компонентов
                              //
   output reg  sys_dclo,      // DCLO output
   output reg  sys_aclo,      // ACLO output
   output      global_reset   // Выход сигнала сброса без учета sys_read, 1 - сброс, 0 - работа
);
localparam DB_COUNTER_WIDTH = log2(DEBOUNCE);
localparam OS_COUNTER_WIDTH = log2(OSCCLK/1000);

localparam PW_COUNTER_WIDTH = log2(PWR_WIDTH);
localparam DC_COUNTER_WIDTH = log2(DCLO_WIDTH);
localparam AC_COUNTER_WIDTH = log2(ACLO_DELAY);

reg   [DB_COUNTER_WIDTH-1:0] count_db;
reg   [OS_COUNTER_WIDTH-1:0] count_os;

reg   [PW_COUNTER_WIDTH-1:0] count_pw;
reg   [DC_COUNTER_WIDTH-1:0] count_dc;
reg   [AC_COUNTER_WIDTH-1:0] count_ac;

reg   osc_ms;
reg   pll_reg;
reg   but_reg;
reg   key_down;

reg   pwr_rst;       // power reset - сброс с задержкой PWR_WIDTH
reg   sys_rst;       // system reset - сброс с ожиданием готовности 


reg   [1:0] key_syn;
reg   pwr_event;
wire  key_event;

assign global_reset=key_event;
//______________________________________________________________________________
//
// Oscillator clock domain - button and PLL failure detectors
//
always @(posedge osc_clk)
begin
   if (count_os < ((OSCCLK/1000)-1))
   begin
      count_os <= count_os + 1'b1;
      osc_ms <= 1'b0;
   end
   else
   begin
      count_os <= 0;
      osc_ms <= 1'b1;
   end
end
//
// External button debouncer
//
always @(posedge osc_clk)
begin
   if (~but_reg | ~pll_reg)
   begin
      count_db <= 0;
      key_down <= 1'b1;
   end
   else
      if (osc_ms)
      begin
         if (count_db < (DEBOUNCE-1))
            count_db <= count_db + 1'b1;
         else
            key_down <= 1'b0;
      end
   pll_reg <= pll_lock;
   but_reg <= button;
   pwr_event <= ~pll_reg;
end
//______________________________________________________________________________
//
// System clock domain metastability synchronizers
//
always @(posedge sys_clk)
begin
   key_syn[0] <= key_down;
   key_syn[1] <= key_syn[0];
end
assign key_event = key_syn[1];

always @(posedge sys_clk or posedge pwr_event)
begin
   if (pwr_event)
   begin
      count_pw <= 0;
      count_dc <= 0;
      count_ac <= 0;

      pwr_rst  <= 1'b1;
      sys_rst  <= 1'b1;
      sys_dclo <= 1'b1;
      sys_aclo <= 1'b1;
   end
   else
   begin
      //
      // Power Reset deassertion delay after Power Event
      //
      if (count_pw < (PWR_WIDTH-1))
         count_pw <= count_pw + 1'b1;
      else
         pwr_rst <= 1'b0;
      //
      // Assert System Reset if button is pressed
      //
      if (key_event)
      begin
         count_dc <= 0;
         count_ac <= 0;
         sys_rst  <= 1'b1;
         sys_dclo <= 1'b1;
         sys_aclo <= 1'b1;
      end
      //
      // System Reset waits for System Ready
      // Some system components may require time to complete
      // its internal initialization, for example, memory
      // controller may initialize the SDRAM chips.
      //
      if (~pwr_rst & sys_ready & ~key_event)
         sys_rst <= 1'b0;
      //
      // DCLO and ACLO synchronous deassertions
      //
      if (~pwr_rst & ~sys_rst & ~key_event)
      begin
         //
         // Count the DCLO pulse
         //
         if (count_dc < (DCLO_WIDTH-1))
            count_dc <= count_dc + 1'b1;
         else
            sys_dclo <= 1'b0;
         //
         // After DCLO deassertion start ACLO counter
         //
         if (~sys_dclo)
            if (count_ac < (ACLO_DELAY-1))
               count_ac <= count_ac + 1'b1;
            else
               sys_aclo <= 1'b0;
      end
   end
end

function integer log2(input integer value);
   begin
      for (log2=0; value>0; log2=log2+1)
         value = value >> 1;
   end
endfunction

endmodule
