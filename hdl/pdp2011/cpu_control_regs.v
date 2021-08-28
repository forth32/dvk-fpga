//====================================================
// Набор внутренних регистров процессора PDP2011
//====================================================
module cpu_control_regs (

// шина wishbone
   input                  wb_clk_i,   // тактовая частота шины
   input                  wb_rst_i,   // сброс
   input    [4:0]         wb_adr_i,   // адрес 
   input    [15:0]        wb_dat_i,   // входные данные
   output reg [15:0]      wb_dat_o,   // выходные данные
   input                  wb_cyc_i,   // начало цикла шины
   input                  wb_we_i,    // разрешение записи (0 - чтение)
   input                  wb_stb_i,   // строб цикла шины
   input    [1:0]         wb_sel_i,   // выбор конкретных байтов для записи - старший, младший или оба
   output reg             wb_ack_o,   // подтверждение выбора устройства
// PSW
   output reg [15:0]      psw_in,     // вывод PSW для чтения
   output reg         psw_in_we_even, // разрешение записи младшего байта PSW
   output reg         psw_in_we_odd,  // разрешение записи старшего байта PSW
   input      [15:0]  psw_out,        // ввод PSW для записи
   output reg [15:0]  cpu_stack_limit,// регистр границы стека 
   output     [15:0]  pir_in,         // регистр программных прерываний
   
// флаги ошибок
   input          cpu_illegal_halt,   // команда HALT не в режиме kernel
   input          cpu_address_error,  // словное обращение по нечетному адресу
   input          cpu_nxm,            // таймаут шины при обращении к RAM
   input          cpu_iobus_timeout,  // таймаут шины при обращении к странице ввода-вывода
   input          cpu_ysv,            // желтая граница стека
   input          cpu_rsv             // красная граница стека
);

wire we; 
wire wo; 
reg[15:0] pir; 
wire[2:0] pia; 
reg cer_illhlt; 
reg cer_addrerr; 
reg cer_nxm; 
reg cer_iobto; 
reg cer_ysv; 
reg cer_rsv; 
reg[5:0] ccr; 
reg [7:0] microbreak;
reg [15:0]dummyreg;

// Сигналы упраления обменом с шиной
   
wire bus_strobe = wb_cyc_i & wb_stb_i;         // строб цикла шины
wire bus_read_req = bus_strobe & ~wb_we_i;     // запрос чтения
wire bus_write_req = bus_strobe & wb_we_i;     // запрос записи
assign we = bus_write_req & wb_sel_i[0];       // строб записи четных байтов
assign wo = bus_write_req & wb_sel_i[1];       // строб записи нечетных байтов

// формирователь ответа на цикл шины   
wire reply=wb_cyc_i & wb_stb_i & ~wb_ack_o;
always @(posedge wb_clk_i or posedge wb_rst_i)
    if (wb_rst_i == 1) wb_ack_o <= 0;
    else wb_ack_o <= reply;

// формирователь номера программного прерывания
assign pia = ((pir[15]) == 1'b1) ? 3'b111 : 
             ((pir[14]) == 1'b1) ? 3'b110 : 
             ((pir[13]) == 1'b1) ? 3'b101 : 
             ((pir[12]) == 1'b1) ? 3'b100 : 
             ((pir[11]) == 1'b1) ? 3'b011 : 
             ((pir[10]) == 1'b1) ? 3'b010 : 
             ((pir[9]) == 1'b1) ? 3'b001 : 
                                  3'b000 ;
assign pir_in = pir ;

always @(posedge wb_clk_i) begin
   if (wb_rst_i == 1'b1)  begin
      // сброс
      pir <= 16'o0;
      psw_in <= 16'b0000000000000000 ; 
      cpu_stack_limit <= 16'h0000; 
      cer_illhlt <= 1'b0 ; 
      cer_addrerr <= 1'b0 ; 
      cer_nxm <= 1'b0 ; 
      cer_iobto <= 1'b0 ; 
      cer_ysv <= 1'b0 ; 
      cer_rsv <= 1'b0 ; 
      ccr <= 6'o77; //{6{1'b0}} ; // регистр управления кешем. 77 - кеш отключен.
      dummyreg <= 16'o0;
   end
   
   else  begin
      // ввод запроса программного прерывания
      pir [7:5] <= pia;
      pir [3:1] <= pia;
      cpu_stack_limit[7:0] <= 8'b00000000 ; 
      
      // установка флагов ошибок в регистре CER
      if (cpu_illegal_halt == 1'b1)  cer_illhlt <= 1'b1 ; 
      if (cpu_address_error == 1'b1) cer_addrerr <= 1'b1 ; 
      if (cpu_nxm == 1'b1)           cer_nxm <= 1'b1 ; 
      if (cpu_iobus_timeout == 1'b1) cer_iobto <= 1'b1 ; 
      if (cpu_ysv == 1'b1)           cer_ysv <= 1'b1 ; 
      if (cpu_rsv == 1'b1)           cer_rsv <= 1'b1 ; 
      
      // снимаем запрос передачи PSW в процессор
      psw_in_we_even <= 1'b0 ; 
      psw_in_we_odd <= 1'b0 ; 
      if (bus_strobe) begin
         // обработка шинных транзакций
         case (wb_adr_i[4:1])
            // PSW - 17 777 776
            4'b1111 :
                     begin
                           // чтение PSW
                           if (bus_read_req == 1'b1)  wb_dat_o <= psw_out ; 
                           
                           // запись старшего байта PSW
                           if (wo == 1'b1)  begin
                              psw_in[15:8] <= wb_dat_i[15:8] ; 
                              psw_in_we_odd <= 1'b1 ; 
                           end 
                           
                           // запись младшего байта PSW
                           if (we == 1'b1)  begin
                              psw_in[7:0] <= wb_dat_i[7:0] ; 
                              psw_in_we_even <= 1'b1 ; 
                           end 
                     end
            // 17 777 774 - граница стека (stack limit)
            4'b1110 :
                     begin
                           if (bus_read_req == 1'b1) wb_dat_o <= cpu_stack_limit ; 
                           if (wo == 1'b1)           cpu_stack_limit[15:8] <= wb_dat_i[15:8] ; 
                     end
            // 17 777 772  PIRQ - программные прерывания
            4'b1101 :
                     begin
                           if (bus_read_req == 1'b1)  wb_dat_o <= pir ; 
                           if (wo == 1'b1)            pir[15:9] <= wb_dat_i[15:9] ; 
                     end
            // 17 777 770  microbreak, EK-KB11C-TM-001_1170procMan.pdf
            4'b1100 :
                     begin
                           if (bus_read_req == 1'b1)   wb_dat_o <= {8'o0, microbreak}; 
                           if (we == 1'b1)             microbreak <= wb_dat_i[7:0] ; 
                     end
            // 17 777 766  CER - регистр кодов ошибок
            4'b1011 :
                     begin
                           if (bus_read_req == 1'b1)  begin
                              // cer
                                 // D7 - Illegal Halt
                                 // D6 - Odd Address Error
                                 // D5 - Memory Time-Out
                                 // D4 - Unibus Time-Out
                                 // D3 - Yellow stack trap
                                 // D2 - Red stack trap
                                 wb_dat_o <= {8'b00000000, cer_illhlt, cer_addrerr, cer_nxm, cer_iobto, cer_ysv, cer_rsv, 2'b00} ; 
                           end 
                           // любая запись - сброс всего регистра
                           if (we == 1'b1)  begin
                              cer_illhlt <= 1'b0 ; 
                              cer_addrerr <= 1'b0 ; 
                              cer_nxm <= 1'b0 ; 
                              cer_iobto <= 1'b0 ; 
                              cer_ysv <= 1'b0 ; 
                              cer_rsv <= 1'b0 ; 
                           end 
                     end
            // 17 777 764 - идентификатор процессора
            4'b1010 :
                     begin
                           if (bus_read_req == 1'b1)  wb_dat_o <= {1'b0, 15'd2011} ; 
                     end
            // 17 777 762
            4'b1001 :
                     begin
                           if (bus_read_req == 1'b1)  wb_dat_o <= {16{1'b0}} ; 
                     end
            // 17 777 760
            4'b1000 :
                     begin
                           // Размер установленной памяти
                           //    77777 соответствует 1024KW
                           if (bus_read_req == 1'b1)   wb_dat_o <= 16'o167777 ; //  1920 KW
//                           if (bus_read_req == 1'b1)   wb_dat_o <= 16'b0000001111111111 ; //  32kw
                     end
            // 17 777 756
            4'b0111 :
                           if (bus_read_req == 1'b1)  wb_dat_o <= {16{1'b0}} ; 
            // 17 777 754
            4'b0110 :
                           if (bus_read_req == 1'b1)  wb_dat_o <= {16{1'b0}} ; 
            // 17 777 752
            4'b0101 :
                     begin
                           if (bus_read_req == 1'b1)   wb_dat_o <= {16{1'b0}} ; 
                     end
            // 17 777 750
            4'b0100 :
                     begin
                           if (bus_read_req == 1'b1)  wb_dat_o <= dummyreg; 
                            if (we) dummyreg[7:0] <= wb_dat_i[7:0];
                            if (wo) dummyreg[15:8] <= wb_dat_i[15:8];
                     end
            // 17 777 746
            4'b0011 :
                     begin
                           // регистр управления кешем
                           if (bus_read_req == 1'b1)  begin
                              wb_dat_o <= {{10{1'b0}}, ccr} ; 
                           end 
                           if (we == 1'b1)  ccr <= wb_dat_i[5:0] ; 
                     end
            // 17 777 744
            4'b0010 :
                     begin
                           if (bus_read_req == 1'b1)  wb_dat_o <= {16{1'b0}} ; 
                     end
            // 17 777 742
            4'b0001 :
                     begin
                           if (bus_read_req == 1'b1) wb_dat_o <= {16{1'b0}} ; 
                     end
            // 17 777 740
            4'b0000 :
                     begin
                           if (bus_read_req == 1'b1)  wb_dat_o <= {16{1'b0}} ; 
                     end
         endcase 
      end 
      end 
   end  
endmodule
