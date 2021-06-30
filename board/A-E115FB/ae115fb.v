//  Проект DVK-FPGA
//
//  Интерфейсный модуль для платы A-E115FB
//=================================================================
//

`include "config.v"

module ae115fb(
   input          clk25,        // тактовая частота 25 MHz
   input  [3:0]   button,       // кнопки 
   input  [3:0]   sw,           // переключатели конфигурации
   output [3:0]   led,          // индикаторные светодиоды   
   
   // интерфейс SD-карты
   output         sdcard_cs, 
   output         sdcard_mosi, 
   output         sdcard_sclk, 
   input          sdcard_miso, 
   
   // VGA
   output         vgah,         // горизонтальная синхронизация
   output         vgav,         // вертикакльная синхронизация
   output         [7:0]vgar,    // красный видеосигнал
   output         [7:0]vgag,    // зеленый видеосигнал
   output         [7:0]vgab,    // синий видеосигнал
   output         vgasync,      // дополнительный синхросигнал зеленого канала
   output         vgaclk,       // пиксельный синхросигнал внешнего DAC
   output         vgablank,     // сигнал отключения видеопотока

   // PS/2
   input          ps2_clk, 
   input          ps2_data,
   
   // пищалка    
   output         buzzer, 
    
   // дополнительный UART 
   output         irps_txd,
   input          irps_rxd,
   
   // принтер
   output [7:0]   lp_data,    // данные для передачи к принтеру
   output         lp_stb_n,   // строб записи в принтер
   output         lp_init_n,  // строб сброса
   input          lp_busy,    // сигнал занятости принтера
   input          lp_err_n    // сигнал ошибки
);

//********************************************
//* Светодиоды
//********************************************
wire dm_led, rk_led, dw_led, my_led, dx_led, timer_led;

assign led[0]=rk_led&dm_led; // запрос обмена диска RK и DM
assign led[1]=dw_led;        // запрос обмена диска DW
assign led[2]=my_led|dx_led; // запрос обмена диска MY или DX
assign led[3]=timer_led;     // индикация включения таймера

//************************************************
//* тактовый генератор 
//************************************************
wire clk_p;
wire clk_n;
wire sdclock;
wire clkrdy;
wire clk50;

pll pll1 (
   .inclk0(clk25), // вход 25 МГц
   .c0(clk_p),     // clk_p прямая фаза, основная тактовая частота
   .c1(clk_n),     // clk_n инверсная фаза
   .c2(sdclock),   // 12.5 МГц тактовый сигнал SD-карты
   .c3(clk50),     // 50 МГц, тактовый сигнал терминальной подсистемы
   .locked(clkrdy) // флаг готовности PLL
);

//******************************************
//* Модуль статической внутренней памяти
//******************************************

wire sdram_reset;       // не используется
wire sdram_we;     
wire sdram_stb;
wire [1:0] sdram_sel;
wire sdram_ack;
wire [21:1] sdram_adr;
wire [15:0] sdram_out;
wire [15:0] sdram_dat;
wire sdram_ready;
wire sdram_wr;
wire sdram_rd;

assign sdram_ready=1'b1;  // модуль всегда готов

// стробы чтения и записи 
assign sdram_wr=sdram_we & sdram_stb;      // строб записи
assign sdram_rd=(~sdram_we) & sdram_stb;   // строб чтения

// Модуль altsyncram размером 64К
baseram ram (
   .address(sdram_adr[15:1]),
   .byteena(sdram_sel),
   .clock(clk_p),
   .data(sdram_out),
   .rden(sdram_rd),
   .wren(sdram_wr),
   .q(sdram_dat)
   );
         
// формирователь сигнала подверждения транзакции
reg [1:0]dack;

assign sdram_ack = sdram_stb & (dack[1]);

// задержка сигнала подтверждения на 1 такт clk
always @ (posedge clk_p)  begin
   dack[0] <= sdram_stb;
   dack[1] <= sdram_stb & dack[0];
end

//************************************
//*  Управление VGA DAC (ADV7123)
//************************************
wire vgagreen,vgared,vgablue;
// выбор яркости каждого цвета  - сигнал, подаваемый на видео-ЦАП для светящейся и темной точки.   
assign vgag = (vgagreen == 1'b1) ? 8'b11111111 : 8'b00000000 ;
assign vgab = (vgablue == 1'b1)  ? 8'b11111111 : 8'b00000000 ;
assign vgar = (vgared == 1'b1)   ? 8'b11111111 : 8'b00000000 ;

assign vgaclk=clk50;    // пиксельная частота
assign vgablank=1'b1;   // затемнение не используется
assign vgasync=1'b0;    // дополнительный синхросигнал не используется

//************************************
//* Соединительная плата
//************************************
topboard kernel(

   .clk50(clk50),                   // 50 МГц
   .clk_p(clk_p),                   // тактовая частота процессора, прямая фаза
   .clk_n(clk_n),                   // тактовая частота процессора, инверсная фаза
   .sdclock(sdclock),               // тактовая частота SD-карты
   .clkrdy(clkrdy),                 // готовность PLL
   
   .bt_reset(~button[0]),            // общий сброс
   .bt_halt(~button[1]),             // режим программа-пульт
   .bt_terminal_rst(~button[2]),     // сброс терминальной подсистемы
   .bt_timer(~button[3]),            // выключатель таймера
   
   .sw_diskbank({2'b00,sw[1:0]}),   // выбор дискового банка
   .sw_console(sw[2]),              // выбор консольного порта: 0 - терминальный модуль, 1 - ИРПС 2
   .sw_cpuslow(sw[3]),              // режим замедления процессора
   
   // индикаторные светодиоды      
   .rk_led(rk_led),               // запрос обмена диска RK
   .dm_led(dm_led),               // запрос обмена диска RK
   .dw_led(dw_led),               // запрос обмена диска DW
   .my_led(my_led),               // запрос обмена диска MY
   .dx_led(dx_led),               // запрос обмена диска DX
   .timer_led(timer_led),         // индикация включения таймера
   
   // Интерфейс SDRAM
   .sdram_reset(sdram_reset),     // сброс
   .sdram_stb(sdram_stb),         // строб начала транзакции
   .sdram_we(sdram_we),           // разрешение записи
   .sdram_sel(sdram_sel),         // выбор байтов
   .sdram_ack(sdram_ack),         // подтверждение транзакции
   .sdram_adr(sdram_adr),         // шина адреса
   .sdram_out(sdram_out),         // выход шины данных
   .sdram_dat(sdram_dat),         // вход шины данных
   .sdram_ready(sdram_ready),     // флаг готовности SDRAM
   
   // интерфейс SD-карты
   .sdcard_cs(sdcard_cs), 
   .sdcard_mosi(sdcard_mosi), 
   .sdcard_sclk(sdcard_sclk), 
   .sdcard_miso(sdcard_miso), 

   // VGA
   .vgah(vgah),         // горизонтальная синхронизация
   .vgav(vgav),         // вертикакльная синхронизация
   .vgared(vgared),     // красный видеосигнал
   .vgagreen(vgagreen), // зеленый видеосигнал
   .vgablue(vgablue),   // синий видеосигнал

   // PS/2
   .ps2_clk(ps2_clk), 
   .ps2_data(ps2_data),
   
   // пищалка    
   .buzzer(buzzer), 
    
   // дополнительный UART 
   .irps_txd(irps_txd),
   .irps_rxd(irps_rxd),
   
   // LPT
   .lp_data(lp_data),    // данные для передачи к принтеру
   .lp_stb_n(lp_stb_n),  // строб записи в принтер
   .lp_init_n(lp_init_n),// строб сброса
   .lp_busy(lp_busy),    // сигнал занятости принтера
   .lp_err_n(lp_err_n)   // сигнал ошибки
);


endmodule
