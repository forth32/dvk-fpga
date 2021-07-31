//
// Copyright (c) 2014-2019 by 1801BM1@gmail.com
//
// Universal Serial Asynchronous Receiver-Transmitter (065-style)
//
// - Wishbone complatible
// - Configurable with external signals
// - Phase accumulator generates arbitrary baud rate without extra PLL
// - Supports vectored interruts in cooperation with wbc_vic
//______________________________________________________________________________
//
module wbc_uart #(parameter REFCLK=100000000)
(
   input                wb_clk_i,   // system clock
   input                wb_rst_i,   // peripheral reset
                                    //
   input  [2:0]         wb_adr_i,   //
   input  [15:0]        wb_dat_i,   //
   output reg [15:0]    wb_dat_o,   //
   input                wb_cyc_i,   //
   input                wb_we_i,    //
   input                wb_stb_i,   //
   output               wb_ack_o,   //
                                    //
   output               txd,        // output serial data (txd))
   input                tx_cts_i,   // enable transmitter (rts)
   input                rxd,        // input serial data (rxd)
   output               rx_dtr_o,   // receiver ready (cts)
                                    //
   output reg           tx_irq_o,   // tx interrupt request
   input                tx_iack_i,   //
   output reg           rx_irq_o,   // rx interrupt request
   input                rx_iack_i,   //
                                    //
   input [15:0]         cfg_bdiv,   // baudrate divisor: (921600/baud)-1
   input [1:0]          cfg_nbit,   // word length: 00-5, 01-6, 10-7, 11-8 bit
   input                cfg_nstp,   // tx stop bits: 0 - 1 stop, 1 - 2 stop
   input                cfg_pena,   // parity control enable
   input                cfg_podd    // odd parity (complete to odd ones)
);

reg            reply;
wire  [63:0]   add_arg;
reg   [16:0]   add_reg;
reg   [15:0]   baud_div;
reg            baud_x16;
wire           baud_ref;
reg   [1:0]    tx_cts_reg;
reg   [1:0]    rx_rdata_reg;

reg   [1:0]    tx_irq_edge, rx_irq_edge;
wire           rx_csr_wstb, rx_dat_rstb;
wire           tx_csr_wstb, tx_dat_wstb;

wire           tx_par;
wire  [15:0]   tx_csr;
reg   [7:0]    tx_dat;
reg   [9:0]    tx_shr;
reg   [7:0]    tx_bcnt;
reg            tx_busy;
reg            tx_csr_ie, tx_rdy, tx_csr_tst, tx_csr_brk;

wire           rx_rdata;
wire  [15:0]   rx_csr;
reg   [7:0]    rx_dat;
reg   [8:0]    rx_shr;
reg   [7:0]    rx_bcnt;
reg            rx_csr_ie, rx_empty, rx_csr_perr, rx_csr_ovf, rx_csr_brk;
wire           rx_load, rx_stb;
reg            rx_frame, rx_start, rx_par;

//______________________________________________________________________________
//
assign   add_arg = (64'h0000000000010000 * 64'd921600 * 64'd16)/REFCLK;
assign   baud_ref = add_reg[16];
//
// Phase accumulator to generate 921600 * 16 Hz reference clock strobe
//
always @(posedge wb_clk_i or posedge wb_rst_i) begin
   if (wb_rst_i) add_reg <= 17'h00000;
   else          add_reg <= {1'b0, add_reg[15:0]} + add_arg[16:0];
end
//
// Baud rate x16 generator
//
always @(posedge wb_clk_i or posedge wb_rst_i) begin
   if (wb_rst_i)  begin
      baud_div <= 16'h0000;
      baud_x16 <= 1'b0;
   end
   else begin
      if (baud_ref)
         if (baud_div == cfg_bdiv)  baud_div <= 16'h0000;
         else                       baud_div <= baud_div + 16'h0001;
      baud_x16 <= baud_ref & (baud_div == 16'h0000);
   end

end

//______________________________________________________________________________
//
// сборка регистров CSR для чтения
assign tx_csr = {8'b00000000, tx_rdy, tx_csr_ie, 3'b000, tx_csr_tst, 1'b0, tx_csr_brk};
assign rx_csr = {rx_csr_perr, 2'b00, rx_csr_ovf, 4'b0000, rx_empty, rx_csr_ie, 5'b00000, rx_csr_brk}; 

// стробы чтения-записи регистров
assign rx_csr_wstb = wb_cyc_i & wb_stb_i &  wb_we_i & ~wb_ack_o & (wb_adr_i[2:1] == 2'b00);
assign rx_dat_rstb = wb_cyc_i & wb_stb_i & ~wb_we_i & ~wb_ack_o & (wb_adr_i[2:1] == 2'b01);
assign tx_csr_wstb = wb_cyc_i & wb_stb_i &  wb_we_i & ~wb_ack_o & (wb_adr_i[2:1] == 2'b10);
assign tx_dat_wstb = wb_cyc_i & wb_stb_i &  wb_we_i & ~wb_ack_o & (wb_adr_i[2:1] == 2'b11);

//**************************************
//*  Сигнал ответа 
//**************************************
always @(posedge wb_clk_i or posedge wb_rst_i)
    if (wb_rst_i) reply <= 1'b0;
    else if (wb_stb_i) reply <= 1'b1;
    else reply <= 1'b0;

assign wb_ack_o = reply & wb_stb_i;
//**************************************
// Чтение регистров
//**************************************
always @(posedge wb_clk_i or posedge wb_rst_i) begin
   if (wb_rst_i)   wb_dat_o <= 16'h0000;
   else if (wb_cyc_i & wb_stb_i & ~wb_ack_o)
         case(wb_adr_i[2:1])
            2'b00:   wb_dat_o <= rx_csr;  // 177560
            2'b01:   wb_dat_o <= rx_dat;  // 177562
            2'b10:   wb_dat_o <= tx_csr;  // 177564
            default: wb_dat_o <= 16'o000000;
         endcase
end

//**************************************
// Запись регистров
//**************************************
always @(posedge wb_clk_i or posedge wb_rst_i)
begin
   // сброс
   if (wb_rst_i)  begin
      rx_csr_ie  <= 1'b0;
      tx_csr_ie  <= 1'b0;
      tx_csr_tst <= 1'b0;
      tx_csr_brk <= 1'b0;
   end
   // запись
   else  begin
      // CSR приемника
      if (rx_csr_wstb)  rx_csr_ie <= wb_dat_i[6]; // бит IE приемника
      // CSR передатчика
      if (tx_csr_wstb)    begin
         tx_csr_ie  <= wb_dat_i[6];
         tx_csr_tst <= wb_dat_i[2];
         tx_csr_brk <= wb_dat_i[0];
      end
   end
end

//**************************************
// Обработка прерываний
//**************************************
always @(posedge wb_clk_i or posedge wb_rst_i) begin
   if (wb_rst_i)   begin
      // сброс
      rx_irq_edge <= 2'b00;
      tx_irq_edge <= 2'b00;
      tx_irq_o    <= 1'b0;
      rx_irq_o    <= 1'b0;
   end
   else  begin
      // детектор события прерывания от приемника
      rx_irq_edge[0] <= rx_empty & rx_csr_ie;      // буфер пуст, прерывания разрешены
      if (rx_empty & rx_csr_ie & ~rx_irq_edge[0])  rx_irq_edge[1] <= 1'b1;  // взводим триггер прерывания, только один раз
      else  if (rx_iack_i | rx_dat_rstb)            rx_irq_edge[1] <= 1'b0; // снимаем его после подтверждения прерывания или чтения регистра данных

      // детектор события прерывания от передатчика
      tx_irq_edge[0] <= tx_rdy & tx_csr_ie;
      if (tx_rdy & tx_csr_ie & ~tx_irq_edge[0])  tx_irq_edge[1] <= 1'b1;
      else  if (tx_iack_i)                       tx_irq_edge[1] <= 1'b0;
      

      // формирователь запроса прерывания приемника
      if (rx_iack_i) rx_irq_o <= 1'b0;
      else          rx_irq_o <= rx_irq_edge[1] & rx_empty & rx_csr_ie;

      // формирователь запроса прерывания передатчика
      if (tx_iack_i) tx_irq_o <= 1'b0;
      else          tx_irq_o <= tx_irq_edge[1] & tx_rdy & tx_csr_ie;
   end
end

//**************************************
// Межклоковая фильтрация сигналов
//**************************************
always @(posedge wb_clk_i) begin
   tx_cts_reg[0] <= ~tx_cts_i;
   tx_cts_reg[1] <= tx_cts_reg[0];

   rx_rdata_reg[0] <= rxd;
   rx_rdata_reg[1] <= rx_rdata_reg[0];
end

//**************************************
// передатчик
//**************************************

// генератор бита четности
assign tx_par = tx_dat[0] ^ tx_dat[1] ^ tx_dat[2] ^ tx_dat[3] ^ tx_dat[4]
              ^ (tx_dat[5] & (cfg_nbit >= 2'b01)) // 6-битный кадр
              ^ (tx_dat[6] & (cfg_nbit >= 2'b10)) // 7-битный кадр
              ^ (tx_dat[7] & (cfg_nbit == 2'b11)) // 8-битный кадр
              ^ cfg_podd;                         // инверсия для нечетной четности
// выход передатчика - бит 0 сдвигового регистра.              
assign txd = tx_shr[0] 
                  & ~tx_csr_brk;  // сигнал break притягивает выход к 0

// блок вывода данных                  
always @(posedge wb_clk_i or posedge wb_rst_i) begin
   if (wb_rst_i)   begin
      // сброс
      tx_rdy <= 1'b1;
      tx_shr  <= 10'o1777;    // сдвиговый регистр заполняем 1
      tx_busy <= 1'b0;
      tx_bcnt <= 8'b00000000;
      tx_dat  <= 8'b00000000;
   end
   else  begin
      // запись буферного регистра передатчика
      if (tx_dat_wstb)  begin
         tx_rdy <= 1'b0;            // снимаем готовность передатчика
         tx_dat <= wb_dat_i[7:0];   // получаем байт в буфер
      end
      // обработка фронтов кадрового синхросигнала UART
      if (baud_x16) begin
         // если производится вывод кадра
         if (tx_busy)  begin
            // счетчик перепадов синхросигнала
            if (tx_bcnt != 8'b00000000) tx_bcnt <= tx_bcnt - 8'b00000001;
            // счетчик бит досчитал до 1 - снимаем занятость передатчика
            if (tx_bcnt == 8'b00000001) tx_busy <= 1'b0;
            // каждые 16 перепадов счетчика - сдвиг регистра вывода
            if (tx_bcnt[3:0] == 4'b0000) tx_shr  <= {1'b1, tx_shr[9:1]};
         end

         // Старт очередного кадра
         if (~tx_rdy &   // бит готовности снят
            ~tx_busy &   // процесс вывода еще не начат
            tx_cts_reg[1]) begin // CTS=1
            
            tx_busy    <= 1'b1;  // поднимаем признак обработки кадра
            tx_rdy     <= ~tx_dat_wstb;
            // начальное значение счетчика перепадов синхросигнала кадра
            tx_bcnt    <= {4'b0110  // минимально 6 бит
                        + {2'b00, cfg_nbit}  // дополнительно 1 или 2 бита
                        + {3'b000, cfg_pena} // бит четности
                        + {3'b000, cfg_nstp}, // стопбит
                          4'b1111};       // x16

            if (cfg_pena)
               // контроль четности включен
               case(cfg_nbit)
                  2'b00:   tx_shr <= {3'b111, tx_par, tx_dat[4:0], 1'b0}; 
                  2'b01:   tx_shr <= {2'b11,  tx_par, tx_dat[5:0], 1'b0};
                  2'b10:   tx_shr <= {1'b1,   tx_par, tx_dat[6:0], 1'b0};
                  default: tx_shr <= {        tx_par, tx_dat[7:0], 1'b0};
               endcase
            else
               // контроль четности отключен
               case(cfg_nbit)
                  2'b00:   tx_shr <= {4'b1111, tx_dat[4:0], 1'b0};
                  2'b01:   tx_shr <= {3'b111,  tx_dat[5:0], 1'b0};
                  2'b10:   tx_shr <= {2'b11,   tx_dat[6:0], 1'b0};
                  default: tx_shr <= {1'b1,    tx_dat[7:0], 1'b0};
               endcase
         end
      end
   end
end

//**************************************
//* Приемник
//**************************************
assign rx_dtr_o = rx_empty;
assign rx_rdata =  tx_csr_tst & txd
              | ~tx_csr_tst & rx_rdata_reg[1];

assign rx_load = rx_stb & (rx_bcnt[7:4] == 4'b0000);
assign rx_stb  = (rx_bcnt[3:0] == 4'b0001) & baud_x16;

always @(posedge wb_clk_i or posedge wb_rst_i)
begin
   if (wb_rst_i)
   begin
      rx_empty  <= 1'b0;
      rx_csr_perr <= 1'b0;
      rx_csr_ovf  <= 1'b0;
      rx_csr_brk  <= 1'b0;
      rx_frame    <= 1'b0;
      rx_start    <= 1'b0;
      rx_par      <= 1'b0;
      rx_dat  <= 8'b00000000;
      rx_shr  <= 9'b00000000;
      rx_bcnt <= 8'b00000000;
   end
   else
   begin
      if (rx_load)
      begin
         rx_empty  <= 1'b1;
         case(cfg_nbit)
            2'b00:   rx_dat <= {3'b000, rx_shr[4:0]};
            2'b01:   rx_dat <= {2'b00, rx_shr[5:0]};
            2'b10:   rx_dat <= {1'b0, rx_shr[6:0]};
            default: rx_dat <= rx_shr[7:0];
         endcase
         rx_csr_perr <= rx_par;
         rx_csr_ovf  <= rx_empty;
         rx_csr_brk  <= ~rx_rdata;
      end
      else
         if (rx_dat_rstb)
         begin
            rx_empty  <= 1'b0;
            rx_csr_perr <= 1'b0;
            rx_csr_ovf  <= 1'b0;
         end

      if (baud_x16)
      begin
         if (~rx_frame)
            //
            // Waiting for start bit
            //
            if (~rx_rdata)
            begin
               rx_par   <= cfg_pena & cfg_podd;
               rx_start <= 1'b1;
               rx_frame <= 1'b1;
               rx_bcnt  <= {4'b0110 + {2'b00, cfg_nbit} + {3'b000, cfg_pena}, 4'b0111};
            end
            else
            begin
               rx_start <= 1'b0;
               rx_bcnt  <= 8'b00000000;
            end
         else
         begin
            //
            // Receiving frame
            //
            if (rx_bcnt != 8'b00000000)
               rx_bcnt <= rx_bcnt - 8'b00000001;
            //
            // Start bit monitoring
            //
            if (rx_start)
            begin
               if (rx_rdata)
               begin
                  //
                  // Spurrious start bit
                  //
                  rx_start <= 1'b0;
                  rx_frame <= 1'b0;
                  rx_bcnt  <= 8'b00000000;
               end
               else
                  if (rx_bcnt[3:0] == 4'b0010)
                     rx_start <= 1'b0;
            end
            else
            begin
               //
               // Receiving data
               //
               if (rx_stb)
               begin
                  rx_par <= (rx_par ^ rx_rdata) & cfg_pena;
                  if (cfg_pena)
                     case(cfg_nbit)
                        2'b00:   rx_shr <= {3'b000, rx_rdata, rx_shr[5:1]};
                        2'b01:   rx_shr <= {2'b00, rx_rdata, rx_shr[6:1]};
                        2'b10:   rx_shr <= {1'b0, rx_rdata, rx_shr[7:1]};
                        default: rx_shr <= {rx_rdata, rx_shr[8:1]};
                     endcase
                  else
                     case(cfg_nbit)
                        2'b00:   rx_shr <= {4'b0000, rx_rdata, rx_shr[4:1]};
                        2'b01:   rx_shr <= {3'b000, rx_rdata, rx_shr[5:1]};
                        2'b10:   rx_shr <= {2'b00, rx_rdata, rx_shr[6:1]};
                        default: rx_shr <= {1'b0, rx_rdata, rx_shr[7:1]};
                     endcase
                  if (rx_load & rx_dat)
                  begin
                     //
                     // Stop bit detected
                     //
                     rx_frame <= 1'b0;
                     rx_bcnt  <= 8'b00000000;
                  end
               end
               if ((rx_bcnt == 8'b00000000) & rx_rdata)
                  rx_frame <= 1'b0;
            end
         end
      end
   end
end
endmodule
