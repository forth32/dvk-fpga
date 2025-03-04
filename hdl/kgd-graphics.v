
//
// Графический контроллер КГД
//
module kgd (

// шина wishbone
   input                wb_clk_i,   // тактовая частота шины
   input                wb_rst_i,   // сброс интерфейсного блока
   input      [2:0]     wb_adr_i,   // адрес 
   input      [15:0]    wb_dat_i,   // входные данные
   output reg [15:0]    wb_dat_o,   // выходные данные
   input                wb_cyc_i,   // начало цикла шины
   input                wb_we_i,    // разрешение записи (0 - чтение)
   input                wb_stb_i,   // строб цикла шины
   input    [1:0]       wb_sel_i,   // выбор конкретных байтов для записи - старший, младший или оба
   output               wb_ack_o,   // подтверждение выбоора устройства

   input                clk50,      // тактовый сигнал 50 Мгц
   
// VGA      
   output reg           vgavideo,   // видеовыход 
   
// синхронизация с КСМ
   input [10:0]         col,        // колонка X, 0-1055
   input [9:0]          row,        // строка Y, 0-627
   
// Управление
   input                vreset,     // сброс графического блока
   output reg           genable,    // подключение графического контроллера к дисплею
   output reg           tdisable    // отключение текстового контроллера от дисплея
);


// регистр адреса
reg [13:0] areg;
reg vbuf_write;

// регистр текущего пикселя
wire [7:0] vbufdata;

wire videobit;      // бит данных из видеопамяти
reg [16:0] lineadr; // адрес первого бита текущей строки в видеопамяти


//********************************************
//*   Модуль двухпортовой видеопамяти
//********************************************
kgdvram vbuf(
    // порт А, 8 бит - доступ с общей шины
   .address_a(areg),
   .data_a(wb_dat_i[7:0]),
   .q_a(vbufdata),
   .wren_a(vbuf_write),
   
   // порт В, 1 бит - доступ от видеоконтроллера
   .address_b(lineadr+col[10:1]-11'd20),  // сумма адреса начала строки и номера пикселя, начиная с пикселя 11
   .wren_b(1'b0),                         // отсюда записи не бывает
   .q_b(videobit),                        // выход видеоданных
   
   .clock_a(wb_clk_i),
   .clock_b(clk50)
);   

//**************************************
//*  Сигнал ответа 
//**************************************
// формирователь ответа на цикл шины   
reg reply;
reg reply0;

always @(posedge wb_clk_i or posedge wb_rst_i) 
    if (wb_rst_i == 1) begin 
      reply <= 1'b0;
      reply0 <= 1'b0;
    end   
    else begin 
      if (wb_stb_i) reply <= 1'b1;
      else reply <= 1'b0;
      reply0 <= reply;
    end   

assign wb_ack_o = reply0 & wb_stb_i;    

//**************************************
// Сигналы упраления обменом с шиной
//**************************************
wire bus_strobe = wb_cyc_i & wb_stb_i & ~(reply|reply0);         // строб цикла шины
wire bus_read_req = bus_strobe & ~wb_we_i;     // запрос чтения
wire bus_write_req = bus_strobe & wb_we_i;     // запрос записи


//*******************************************
//* Обработка шинных транзакций
//******************************************   
always @(posedge wb_clk_i) 
   if (wb_rst_i == 1'b1) begin
      genable <= 1'b0;      // после сброса графика отключена
      tdisable <= 1'b0;     // текст включен
      areg <= 14'o0;
      vbuf_write <= 1'b0;
   end
   else begin
   // обработка запросов с шины
      // чтение регистров
      if (bus_read_req == 1'b1)   
         case (wb_adr_i[2:1])
            // 176640 - регистр управления   
            2'b00:   wb_dat_o <= {genable, tdisable, 14'b0};   
                        
            // 176640 - регистр данных
            2'b01:     wb_dat_o <= {8'o0,vbufdata};
               
            // 176644 - регистр адреса            
            2'b10:   wb_dat_o <= {2'b0, areg};
                        
            // 176646 - регистр счетчика
            2'b11:   wb_dat_o <= {row[4:0], col[10:0]};
         endcase         
   
      // запись регистров   
      else if (bus_write_req == 1'b1)  
         case (wb_adr_i[2:1])
            // 176640 - регистр управления   
            2'b00:  if (wb_sel_i[1] == 1'b1) begin
                        genable <= wb_dat_i[15];
                        tdisable <= wb_dat_i[14];
                    end   
                    
            // 176640 - регистр данных
            2'b01:  if (wb_sel_i[0] == 1'b1) vbuf_write <=1'b1;
                     
            // 176644 - регистр адреса            
            2'b10:     begin
                    if (wb_sel_i[0] == 1'b1) areg[7:0] <= wb_dat_i[7:0];
                    if (wb_sel_i[1] == 1'b1) areg[13:8] <= wb_dat_i[13:8];
             end     
         endcase
        else vbuf_write <= 1'b0;   
   end
   
//******************************************************
//* Видеоконтроллер   
//******************************************************

// Размер графического экрана - 400*286, в удвоенном режиме - 800*572. Первые 28 строк пусты.

//**********************************  
//* Процесс попиксельной обработки
//**********************************  
always @(posedge clk50) 
 if (vreset == 1'b1) begin
    // сброс контроллера
    vgavideo <= 1'b0;
    lineadr <= 17'd0;
 end
 else begin
  
  //***********************************************************
  //*  Формирователь начального адреса строки в видеобуфере
  //***********************************************************
  if (col == 11'd1055) begin // конец полной видеостроки 
    // конец полного кадра
    if (row == 10'd627) lineadr <= 17'd0;  // переход на новый кадр - сбрасываем счетчик адреса начала строки
    else if ((row > 10'd50) && (row[0] == 0)) lineadr <= lineadr+14'd400;   // смена стартового адреса строки через строку, начиная со строки 51
  end    
  
  //********************************
  //*  Формирователь видеосигнала
  //********************************
  // Формат строки: 
  //   0            40          840          928    1055 
  //   <back porch> <videoline> <front proch> <hsync>
  
  // Формат кадра
  // 0            23           623           624    627
  // <back porch> <videoframe> <front proch> <vsync>
  
  if (
      (col < 11'd40) || (col > 11'd839) ||   // левое и правое черное поле - horizontal porch
      (row < 10'd51) || (row > 10'd622)      // верхнее и нижнее черное поле - vertical porch
                                             //  51 - это 23 (само черное поле) + 28 (неиспользуемые графические строки)
  ) vgavideo <= 1'b0;
  // видимая часть строки
  else vgavideo <= videobit;  // выводим теущий бит из видеопамяти на экран
 end
endmodule   
      
