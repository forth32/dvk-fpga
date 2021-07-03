//  Проект DVK-FPGA
//
//  Интерфейсный модуль для платы A-ESTF V2
//=================================================================
//

`include "config.v"

module aestf(
   input          clk50,        // clock input 50 MHz
   input  [3:0]   button,       // кнопки 
   input  [3:0]   sw,           // переключатели конфигурации
   output [3:0]   led,          // индикаторные светодиоды   
   
   // Интерфейс SDRAM
   inout  [15:0]  DRAM_DQ,      //   SDRAM Data bus 16 Bits
   output [12:0]  DRAM_ADDR,    //   SDRAM Address bus 12 Bits
   output         DRAM_LDQM,    //   SDRAM Low-byte Data Mask 
   output         DRAM_UDQM,    //   SDRAM High-byte Data Mask
   output         DRAM_WE_N,    //   SDRAM Write Enable
   output         DRAM_CAS_N,   //   SDRAM Column Address Strobe
   output         DRAM_RAS_N,   //   SDRAM Row Address Strobe
   output         DRAM_CS_N,    //   SDRAM Chip Select
   output         DRAM_BA_0,    //   SDRAM Bank Address 0
   output         DRAM_BA_1,    //   SDRAM Bank Address 0
   output         DRAM_CLK,     //   SDRAM Clock
   output         DRAM_CKE,     //   SDRAM Clock Enable

   // интерфейс SD-карты
   output         sdcard_cs, 
   output         sdcard_mosi, 
   output         sdcard_sclk, 
   input          sdcard_miso, 

   
   
   // VGA
   output         vgah,         // горизонтальная синхронизация
   output         vgav,         // вертикакльная синхронизация
   output         [4:0]vgar,    // красный видеосигнал
   output         [5:0]vgag,    // зеленый видеосигнал
   output         [4:0]vgab,    // синий видеосигнал

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
assign led[2]=my_led&dx_led; // запрос обмена диска MY или DX
assign led[3]=timer_led;     // индикация включения таймера

//************************************************
//* тактовый генератор 
//************************************************
wire clk_p;
wire clk_n;
wire sdclock;
wire clkrdy;

pll pll1 (
   .inclk0(clk50),
   .c0(clk_p),     // 100МГц прямая фаза, основная тактовая частота
   .c1(clk_n),     // 100МГц инверсная фаза
   .c2(sdclock),   // 12.5 МГц тактовый сигнал SD-карты
   .locked(clkrdy) // флаг готовности PLL
);

//**********************************
//* Модуль динамической памяти
//**********************************

wire sdram_reset;
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

reg [1:0] dreset;
reg [1:0] dr_cnt;
reg drs;

// формирователь сброса
always @(posedge clk_p)
begin
   dreset[0] <= sdram_reset; // 1 - сброс
   dreset[1] <= dreset[0];
   if (dreset[1] == 1) begin
     // системный сброс активен
     drs<=0;         // активируем сброс DRAM
     dr_cnt<=2'b0;   // запускаем счетчик задержки
   end  
   else 
     // системный сброс снят
     if (dr_cnt != 2'd3) dr_cnt<=dr_cnt+1'b1; // счетчик задержки ++
     else drs<=1'b1;                          // задержка окончена - снимаем сигнал сброса DRAM
end


// стробы подтверждения
wire sdr_wr_ack,sdr_rd_ack;
// тактовый сигнал на память - инверсия синхросигнала шины
assign DRAM_CLK=clk_n;

// стробы чтения и записи в sdram
assign sdram_wr=sdram_we & sdram_stb;
assign sdram_rd=(~sdram_we) & sdram_stb;

// Сигналы выбора старших-младших байтов
reg dram_h,dram_l;

always @ (posedge sdram_stb) begin
  if (sdram_we == 1'b0) begin
   // чтение - всегда словное
   dram_h<=1'b0;
   dram_l<=1'b0;
  end
  else begin
   // определение записываемых байтов
   dram_h<=~sdram_sel[1];  // старший
   dram_l<=~sdram_sel[0];  // младший
  end
end  

assign DRAM_UDQM=dram_h; 
assign DRAM_LDQM=dram_l; 

// контроллер SDRAM

sdram_top sdram(
    .clk(clk_p),
    .rst_n(drs), // запускаем модуль, как только pll выйдет в рабочий режим, запуска процессора не ждем
    .sdram_wr_req(sdram_wr),
    .sdram_rd_req(sdram_rd),
    .sdram_wr_ack(sdr_wr_ack),
    .sdram_rd_ack(sdr_rd_ack),
    .sdram_byteenable(sdram_sel),
    .sys_wraddr({1'b0,sdram_adr[21:1]}),
    .sys_rdaddr({1'b0,sdram_adr[21:1]}),
    .sys_data_in(sdram_out),
    .sys_data_out(sdram_dat),
    .sdwr_byte(1),
    .sdrd_byte(4),
    .sdram_cke(DRAM_CKE),
    .sdram_cs_n(DRAM_CS_N),
    .sdram_ras_n(DRAM_RAS_N),
    .sdram_cas_n(DRAM_CAS_N),
    .sdram_we_n(DRAM_WE_N),
    .sdram_ba({DRAM_BA_1,DRAM_BA_0}),
    .sdram_addr(DRAM_ADDR[12:0]),
    .sdram_data(DRAM_DQ),
    .sdram_init_done(sdram_ready)     // выход готовности SDRAM
);
         
// формирователь сигнала подверждения транзакции
reg [1:0]dack;

assign sdram_ack = sdram_stb & (dack[1]);

// задержка сигнала подтверждения на 1 такт clk
always @ (posedge clk_p)  begin
   dack[0] <= sdram_stb & (sdr_rd_ack | sdr_wr_ack);
   dack[1] <= sdram_stb & dack[0];
end

//************************************
//*  Управление VGA DAC
//************************************
wire vgagreen,vgared,vgablue;
// выбор яркости каждого цвета  - сигнал, подаваемый на видео-ЦАП для светящейся и темной точки.   
assign vgag = (vgagreen == 1'b1) ? 6'b111111 : 6'b000000 ;
assign vgab = (vgablue == 1'b1)  ? 5'b11111  : 5'b00000 ;
assign vgar = (vgared == 1'b1)   ? 5'b11110  : 5'b00000 ;

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
   .dm_led(rk_led),               // запрос обмена диска DM
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
