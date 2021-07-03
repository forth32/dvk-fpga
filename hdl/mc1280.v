//
//  Процессорный модуль - плата МС1280
//
//  Центральный процессор - М4 (LSI-11M)
// 
// ======================================================================================

module mc1280 (
// Синхросигналы  
   input  clk_p,               // синхросигнал прмой фазы
   input  clk_n,               // синхросигнал инверсной фазы
   input  cpuslow,             // Режим замедления процессора

// Шина Wishbone                                       
   input  cpu_gnt_i,           // 1 - разрешение cpu работать с шиной
                               // 0 - DMA с внешними устройствами, cpu отключен от шины и бесконечно ждет ответа wb_ack
   output [15:0] cpu_adr_o,    // выход шины адреса
   output [15:0] cpu_dat_o,    // выход шины данных
   input  [15:0] cpu_dat_i,    // вход шины данных
   output cpu_cyc_o,           // Строб цила wishbone
   output cpu_we_o,            // разрешение записи
   output [1:0] cpu_sel_o,     // выбор байтов для передачи
   output cpu_stb_o,           // строб данных

   output sysram_stb,          // строб обращения к системной памяти
   input  global_ack,          // подтверждение обмена от памяти и устройств страницы ввода-вывода
   
// Сбросы и прерывания
   output vm_init,             // Выход сброса для периферии
   input  dclo,                // Вход сброса процессора
   input  aclo,                // Сигнал аварии питания
   input  halt,                // Прерывание входа в пультовоый режим
   input  virq,                // Векторное прерывание

// Шины обработки прерываний                                       
   input  [15:0] ivec,         // Шина приема вектора прерывания
   output istb,                // Строб приема вектора прерывания
   input  iack,                // Подтверждение приема вектора прерывания

// Таймер
   input  timer_button,        // кнопка включения-отключения таймера
   output reg timer_status     // линия индикатора состояния таймера
);

// Системной памяти здесь нет
assign sysram_stb=1'b0;

wire [15:0] local_dat_i;    // локальная входная шина данных
wire local_stb;             // локальный сигнал начала транзацкии
wire cpu_ack;               // вход REPLY процессора

// cyc - формальный сигнал, не имеющий смысла в нашей схеме
assign cpu_cyc_o=cpu_stb_o;

// Сигнал подтвреждения обмена - от общей шины и модуля ROM
assign cpu_ack = global_ack | bootrom_ack;

// мультиплексор входной шины данных
assign local_dat_i = (bootrom_stb) ?   bootrom_dat: 16'o0       // boot rom
                     | cpu_dat_i;                               // остальные устройства на шине
                   
// Разрешение транзакций на общей шине - только при отсутствии доступа к локальным устройствам
assign cpu_stb_o=local_stb & (~bootrom_stb);


//*************************************
//*  Процессор LSI-11M (M4)
//*************************************
am4_wb cpu
(
// Синхросигналы  
   .vm_clk_p(clk_p),                // Положительный синхросигнал
   .vm_clk_n(clk_n),                // Отрицательный синхросигнал
   .vm_clk_slow(1'b0),              // Режим замедления процессора - определяется переключателем 3
//   .vm_clk_ena(cpu_clk_enable),     // счетчик замедления - этот процессор виснет в таком режиме

// Шина Wishbone                                       
   .wbm_gnt_i(cpu_gnt_i),           // 1 - разрешение cpu работать с шиной
                                    // 0 - DMA с внешними устройствами, cpu отключен от шины и бесконечно ждет ответа wb_ack
   .wbm_adr_o(cpu_adr_o),           // выход шины адреса
   .wbm_dat_o(cpu_dat_o),           // выход шины данных
   .wbm_dat_i(local_dat_i),           // вход шины данных
//   .wbm_cyc_o(cpu_cyc_o),           // Строб цила wishbone
   .wbm_we_o(cpu_we_o),             // разрешение записи
   .wbm_sel_o(cpu_sel_o),           // выбор байтов для передачи
   .wbm_stb_o(local_stb),           // строб данных
   .wbm_ack_i(cpu_ack),          // вход подтверждения данных

// Сбросы и прерывания
   .vm_init(vm_init),               // Выход сброса для периферии
   .vm_dclo(dclo),                  // Вход сброса процессора
   .vm_aclo(aclo),                  // Сигнал аварии питания
   .vm_halt(halt),                  // Прерывание входа в пультовоый режим
   .vm_evnt(timer_50&timer_status), // Прерывание от таймера 
   .vm_virq(virq),                  // Векторное прерывание

// Шины обработки прерываний                                       
   .wbi_dat_i(ivec),                // Шина приема вектора прерывания
   .wbi_stb_o(istb),                // Строб приема вектора прерывания
   .wbi_ack_i(iack),                // Подтверждение приема вектора прерывания

// Режим начального пуска
//     00 - start reserved MicROM
//     01 - start from 173000
//     10 - break into ODT
//     11 - load vector 24
   .vm_bsel(2'b10)
);

//*******************************************
//* ПЗУ монитора-загрузчика
//*******************************************
wire bootrom_stb;
wire bootrom_ack;
wire [15:0] bootrom_dat;
reg [1:0]bootrom_ack_reg;

`ifdef bootrom_module
// эмулятор пульта и набор загрузчиков - ПЗУ  165000-166777
boot_rom bootrom(
   .address(cpu_adr_o[9:1]),
   .clock(clk_p),
   .q(bootrom_dat));

always @ (posedge clk_p) begin
   bootrom_ack_reg[0] <= bootrom_stb & ~cpu_we_o;
   bootrom_ack_reg[1] <= bootrom_stb & ~cpu_we_o & bootrom_ack_reg[0];
end
assign bootrom_ack = local_stb & bootrom_ack_reg[1];

// сигнал выбора
assign bootrom_stb   = local_stb & (cpu_adr_o[15:10] == 6'o72); // 164000-165776  

`else 
assign bootrom_ack=1'b0;
assign bootrom_stb=1'b0;
`endif


//*************************************************************************
//* Генератор прерываний от таймера
//* Сигнал имеет частоту 50 Гц и ширину импульса в 1 такт
//*************************************************************************
reg timer_50;
reg [20:0] timercnt;

wire [20:0] timer_limit=31'd`clkref/6'd50-1'b1;

always @ (posedge clk_p) begin
  if (timercnt == timer_limit) begin
     timercnt <= 21'd0;
     timer_50 <= 1'b1;
  end  
  else begin
     timercnt <= timercnt + 1'b1;
     timer_50 <= 1'b0;
  end     
end


//**********************************
//* Сигнал разрешения таймера
//**********************************
reg [1:0] tbshift;
reg tbevent;

// подавление дребезга кнопки
always @ (posedge timer_50) begin
  // отключение таймера по сбросу
  if (dclo) timer_status <= 1'b0;
  else begin
    // вводим кнопку в сдвиговый регистр
    tbshift[0] <= timer_button;
    tbshift[1] <= tbshift[0];
    // регистр заполнен - кнопка стабильно нажата
    if (&tbshift == 1'b1) begin
      if (tbevent == 1'b0) begin
        timer_status <= ~timer_status;  // переключаем состояние таймера
        tbevent <= 1'b1;                              // запрещаем дальнейшие изменения состояния таймиера
      end
    end
    // регистр очищен - кнопка стабильно отпущена
    else if (|tbshift == 1'b0) tbevent <= 1'b0;     // разрешаем изменения состояния таймера
  end 
end  

endmodule
