//======================================================================
//  Контроллер DEC RH-70 / RH-11 (DB:) с дисками RP06
//======================================================================
//
// В режиме massbus контроллер поддерживает 22-битный адрес DMA
// В режиме unibus/qbus - только 18-битный, и должен работать через Unibus Mapping.
//

module rh70 (
   // шина wishbone
   input                  wb_clk_i,   // тактовая частота шины
   input                  wb_rst_i,   // сброс
   input    [5:0]         wb_adr_i,   // адрес 
   input    [15:0]        wb_dat_i,   // входные данные
   output reg [15:0]      wb_dat_o,   // выходные данные
   input                  wb_cyc_i,   // начало цикла шины
   input                  wb_we_i,    // разрешение записи (0 - чтение)
   input                  wb_stb_i,   // строб цикла шины
   input    [1:0]         wb_sel_i,   // выбор конкретных байтов для записи - старший, младший или оба
   output                 wb_ack_o,   // подтверждение выбора устройства

// обработка прерывания   
   output reg             irq,         // запрос
   input                  iack,        // подтверждение
   
// DMA
   output reg             dma_req,    // запрос DMA
   input                  dma_gnt,    // подтверждение DMA
   output reg[21:0]       dma_adr_o,  // выходной адрес при DMA-обмене
   input[15:0]            dma_dat_i,  // входная шина данных DMA
   output reg[15:0]       dma_dat_o,  // выходная шина данных DMA
   output reg             dma_stb_o,  // строб цикла шины DMA
   output reg             dma_we_o,   // направление передачи DMA (0 - память->диск, 1 - диск->память) 
   input                  dma_ack_i,  // Ответ от устройства, с которым идет DMA-обмен
   
// интерфейс SD-карты
   output                 sdcard_cs, 
   output                 sdcard_mosi, 
   output                 sdcard_sclk, 
   input                  sdcard_miso, 
   output reg             sdreq,      // запрос доступа к карте
   input                  sdack,      // подтверждение доступа к карте
   input                  sdmode,     // режим SDSPI
   
// тактирование SD-карты
   input                  sdclock,   

// Адрес начала банка на карте
   input [26:0]           start_offset
   ); 

   
// Геометрия диска
//--------------------------------------------------
// Число секторов на дорожке, SPT
assign spt =  8'd22;   
// число дорожек на цилиндр (головок)                                
assign track_per_cyl = 8'd19;    
// число цилиндров
assign cyl_limit = 16'd815 ; 

// Регистры устройства
//=================================================================================================== 
//  17776700  RHCS1 - регистр управления/состояния 1
//               0    R/W  GO         запуск команды
//               1-5  R/W  FUNC       код команды
//               6    R/W  IE         разрешение прерывания
//               7    R/O  RDY        готовность контроллера
//               8-9  R/W  BAE        расширение адреса для 18-битного DMA
//               10   R/W  PSEL       выбор порта в двухпортовой конфигурации
//               11   R/O  DVA        признак наличия привода с выбранным номером
//               12        0
//               13   R/O  MCPE       ошибка четности
//               14   R/O  TRE        сборный сигнал ошибок передачи данных
//               15   R/O  SC         признак ошибки или сигнала внимание. Является сборкой трех сигналов -  TRE, ATTN, MCPE
//
//  17776702  RHWC - счетчик слов для обмена
//
//  17776704  RHBA - адрес буфера в памяти для DMA
//
//  17776706  rpda - регистр головки и сектора                       
//              0-4  R/W   SA  сектор
//              8-11 R/W   TA  головка   
//
//  17776710  RHCS2 - регистр управления/состояния 2
//              0-2 R/W    UNIT  выбор номера устройства
//              3   R/W    BAI   запрет инкремента адреса при DMA-обмене
//              4   R/W    PAT   тест проверки четности
//              5   W/O    CLR   сброс контроллера
//              6   R/O    IR    готовность буфера SILO к записи слова
//              7   R/W    OR    готовность буфера SILO к чтению слова
//              8   R/O    MDPE  ошибка четности на шине massbus
//              9   R/O    MXF   превышено время начала передачи данных устройством
//             10   R/O    PGE   попытка выполнить передачу данных при незавершенной предыдущей
//             11   R/O    NEM   зависание шины
//             12   R/O    NED   устройство не подключено к контроллеру
//             13   R/O    PE    ошибка четности на устройстве
//             14   R/O    WCE   ошибка при проверке записи
//             15   R/O    DLT   ошибка переполнения/переопустшения буфера
//
//  17776712  RHDS - регистр состояния привода   
//            0     R/O   OM    режим смещения от центра дорожки
//            1-5         0
//            6     R/O   VV    подтверждение установки тома (volume valid)   
//            7     R/O   DRDY  готовность устройства
//            8     R/O   DPR   устройство не занято другим контроллером в двухпортовой конфигурации. В однопортовом режиме всегда 1.
//            9     R/O   PGM   только для двухпортового режима - устройство доступно для обоих контроллеров
//            10    R/O   LST   произошел обмен с последним сектором устройства
//            11    R/O   WRL   диск аппаратно защищен от записи
//            12    R/O   MOL   диск раскручен и готов к работе
//            13    R/O   PIP   идет позиционирование
//            14    R/O   ERR   сборный сигнал всех флагов ошибок
//            15    R/O   ATA   устройство подняло сигнал "внимание"
//
//  17776714 RHER1 - регистр ошибок 1 
//            0     R/O   ILF   ошибочный код команды
//            1     R/O   ILR   обращение к несуществующему регистру
//            2     R/O   RMR   попытка записи в регистр в процессе обработки команды
//            3     R/O   PAR   ошибка чтености
//            4     R/O   FER   ошибка формата тома
//            5     R/O   WCF   нет синхросигнала записи
//            6     R/O   ECH   ошибка ECC
//            7     R/O   HCE   информация из заголовка сектора не соответствует ожидаемой
//            8     R/O   HCRC  ошибка CRC заголовка сектора
//            9     R/O   AOE   выход за пределы диска в процессе обмена данными
//            10    R/O   IAE   запрошенный CHS выходит за границу диска
//            11    R/O   WLE   попытка записи на защищенный диск
//            12    R/O   DTE   ошибка синхронизации с устройством
//            13    R/O   OPI   таймаут выполнения операции
//            14    R/O   UNS   авария на устройстве - устройство находится в небезопасном состоянии
//            15    R/O   DCK   ошибка чтения данных
//
//  17776716 RHAS - Регистр сигналов "внимание" 
//            0-7   R/W   ATTN0-ATTN7 - сигнал "внимание" приводов 0-7.
//
//  17776720 RHLA - регистр упреждающего доступа
//            0-3   0
//            4-5   R/O  EXT0,EXT1 - текущая фаза вращения диска (00 - от 0 до 90 градусов, 01 - от 90 до 180 градусов, итд)
//            6-10  R/O  номер сектора, над которым проходит головка в данный момент
//
//  17776722 RHDB - Интерфейс к буферу данных SILO
//
//  17776724 RHMR1 - Регистр диагностики 
//
//  17776726 RHDT - регистр типа устройства :
//                     020021 - однопортовый RP05
//                     024021 - двухпортовый RP05
//                     020022 - однопортовый RP06
//                     024022 - двухпортовый RP06
//           
//  17776730 RHSN - серийный номер 
//
//  17776732 RHOF - регистр смещения 
//            0-7   R/W  OF  смещение от центра дорожки
//            8-9   0
//            10    R/W  HCI  запрет проверки заголовков секторов
//            11    R/W  ECI  отключение ECC
//            12    R/W  FMT  формат диска (18 или 22 сектора)
//            13-14  0
//            15    R/W  SGN  направление смещения головок от центра дорожки
//
//  17776734  регистр цилиндра RHDC  
//            0-9   R/W  CYL  номер цилиндра
//
//  17776740 RHMR2 - Регистр диагностики 2 
//
//  17776742 RHER2 - регистр ошибок 2 
//           0-2   0
//           3     R/O   DPE  ошибка чтености данных
//           4-6   0
//           7     R/O   DVC  
//           8-9   0
//           10    R/O   LBC 
//           11    R/O   LSC  потеря системного тактового сигнала
//           12    R/O   IVC  неправильная команда
//           13    R/O   OPE
//           14    R/O   SKI  позиционирование не завершено
//           15    R/O   BSE  плохой сектор
//
// 17776744 RHEC - регистр ECC 1 
// 
// 17776746 RHEC - регистр ECC 2
//
// ---- Следующие регистры доступны только в режиме massbus ------
//
// 17776750 RHBAE - 22-битное расширение адреса 
//          0-5    R/W  биты 16-21 адреса
//
// 17776752 RHCS3 - регистр управления/состояния 3
//          0      R/O  IPCK
//          1-2     0
//          3      R/W  IE  разрешение прерывания 
//          4-6     0
//          7      R/W  DBL
//          8      R/O  WCE
//          9      R/O  DPE
//         10      R/O  APE 
//
//======================================================================
//  
//   Коды команд:
//
//  00 - NOP
//  01 - разгрузка тома
//  02 - позиционирование
//  03 - рекалибровка, переход к CHS 0 - 0 - 0
//  04 - сброс ошибок
//  05 - освобождение устройства в духпортовой конфигурации
//  06 - установка смещения
//  07 - отключение смещения
//  10 - подготовка к начальной загрузке (vv=1, chs=000, fmt=0, hci=0, eci=0)
//  11 - подтверждение установки тома
//  14 - поиск сектора
//  24 - проверка записи данных
//  25 - проверка записи заголовка и данных
//  30 - запись данных
//  31 - запись ззаголовка и данных
//  34 - чтение данных
//  35 - чтение заголовка и данных
//======================================================================


// машина состояний обработки прерываний
parameter[1:0] i_idle = 0; 
parameter[1:0] i_req = 1; 
parameter[1:0] i_wait = 2; 
reg[1:0] interrupt_state; 
reg interrupt_trigger;      // триггер запроса прерывания

// Регистры контроллера
//-------------------------------------

wire rhcs1_sc;      
wire rhcs1_tre;     
//                  
wire[1:0] rhcs1_bae;
reg rhcs1_rdy;     
reg rhcs1_ie;       
reg[4:0] rhcs1_fnc;
reg rhcs1_go;    

reg[15:0] rhwc; 

reg[15:0] rhba; 

reg[7:0] rpda_hd; // головка
reg[7:0] rpda_sa; // сектор

reg rhcs2_pe;   
reg rhcs2_nem;  
reg rhcs2_pge;  
reg rhcs2_mxf;  
reg rhcs2_or;   
reg rhcs2_ir;   
reg rhcs2_clr; 
reg rhcs2_pat; 
reg rhcs2_bai; 
reg[2:0] unit; 

reg rhds_ata[7:0];
wire rhds_err; 
reg rhds_wrl; 
reg rhds_lst; 
reg rhds_dry; 
reg rhds_vv[7:0]; 
reg rhds_om; 

reg rher1_dck;
reg rher1_uns;
reg rher1_opi;
reg rher1_dte;
reg rher1_wle;
reg rher1_iae;
reg rher1_aoe;
reg rher1_hcrc;
reg rher1_hce; 
reg rher1_ech; 
reg rher1_wcf; 
reg rher1_fer; 
reg rher1_par; 
reg rher1_rmr; 
reg rher1_ilr; 
reg rher1_ilf; 

reg[4:0] rhla_sc; 

reg [15:0] rhdb;

reg[15:0] rhmr1; 

reg rhof_fmt; 
reg rhof_eci; 
reg rhof_hci; 
reg rhof_ofd; 

reg[15:0] rhdc;

reg[15:0] rhmr2; 

reg rher2_dpe; 
reg rher2_dvc; 
reg rher2_lbc; 
reg rher2_lsc; 
reg rher2_ivc; 
reg rher2_ope; 
reg rher2_ski; 
reg rher2_bse; 

reg[5:0] rhbae; 

reg rhcs3_ape; 
reg[1:0] rhcs3_dpe; 
reg[1:0] rhcs3_wce; 
reg rhcs3_dbl; 
wire rhcs3_ie; 
reg[3:0] rhcs3_ipck; 

reg update_rhwc;   // требование обновить содержимое wcp
reg[15:0] wcp;     // счетчик слов, неинвертированный
reg error_reset; 
reg[11:0] rmclock; 
wire[1:0] rmclock_piptimer; 
reg rhcs1_rdyset; 
reg[21:1] ram_phys_addr; 
wire[7:0] spt; 
wire[7:0] track_per_cyl; 
wire[15:0] cyl_limit; 
reg write_start; 
reg read_start; 
reg iocomplete;          // признак завершения работы DMA-контроллера
reg [5:0] reply_count;   // таймер ожидания ответа при DMA-обмене

// интерфейс контроллера DMA
reg nxm; 
reg[8:0] sector_data_index; // counter within sector

// машина состояний контроллера
parameter[3:0] dma_idle = 0; 
parameter[3:0] dma_read = 1; 
parameter[3:0] dma_readh = 2; 
parameter[3:0] dma_readh2 = 3; 
parameter[3:0] dma_preparebus = 4; 
parameter[3:0] dma_read_done = 5; 
parameter[3:0] dma_write1 = 6; 
parameter[3:0] dma_write = 7; 
parameter[3:0] dma_write_fill = 8; 
parameter[3:0] dma_write_wait = 9; 
parameter[3:0] dma_write_done = 10; 
parameter[3:0] dma_wait = 11; 
parameter[3:0] dma_write_delay = 12; 
parameter[3:0] dma_readsector = 13; 

reg[3:0] dma_state; 

// Сборка регистров для чтения
assign rhcs1_sc = rhcs1_tre;
assign rhcs1_tre = rhcs2_pe | rhcs2_nem | rhcs2_mxf | rhcs2_pge | rher1_iae | rher2_ivc ;
assign rhcs1_bae = rhbae[1:0] ;  // 2 младших бита расширения адреса
assign rhcs3_ie = rhcs1_ie ;

wire v_iae;

// признак ошибки - собирается из всех битов ошибок
assign rhds_err = rher1_dck | rher1_uns | rher1_opi | rher1_dte | rher1_wle | rher1_iae | rher1_aoe 
                | rher1_hcrc | rher1_hce | rher1_ech | rher1_wcf | rher1_fer | rher1_par | rher1_rmr 
                | rher1_ilr  | rher1_ilf | rher2_dpe | rher2_dvc | rher2_lbc | rher2_lsc | rher2_ivc 
                | rher2_ope  | rher2_ski | rher2_bse ;

//***********************************************
//*  Контроллер SD-карты
//***********************************************
// интерфейс к SDSPI
wire [26:0] sdaddr;            // полный SD-адрес, вычисляемый из цилиндра, головки, сектора
reg  [26:0] sdcard_addr;       // буфер адреса сектора карты
wire sdcard_error;             // флаг ошибки
wire [15:0] sdbuf_dataout;     // слово; читаемое из буфера чтения
wire sdcard_idle;              // признак готовности контроллера
reg [7:0] sdbuf_addr;          // адрес в буфере чтния/записи
reg sdbuf_we;                  // строб записи буфера
reg [15:0] sdbuf_datain;       // слово; записываемое в буфер записи
reg sdspi_start;               // строб запуска sdspi
reg sdspi_write_mode;          // 0-чтение, 1-запись
wire sdspi_io_done;            // флаг заверщение операции обмена с картой


sdspi sd1 (
      // интерфейс к карте
      .sdcard_cs(sdcard_cs), 
      .sdcard_mosi(sdcard_mosi), 
      .sdcard_miso(sdcard_miso),
      .sdcard_sclk(sdcard_sclk),
      
      .sdcard_addr(sdcard_addr),                  // адрес блока на карте
      .sdcard_idle(sdcard_idle),                  // сигнал готовности модуля к обмену
      .sdcard_error(sdcard_error),                // флаг ошибки
      
      // сигналы управления чтением - записью
      .sdspi_start(sdspi_start),                // строб запуска ввода вывода
      .sdspi_io_done(sdspi_io_done),            // флаг окончания обмена данными
      .sdspi_write_mode(sdspi_write_mode),      // режим: 0 - чтение, 1 - запись

      // интерфейс к буферной памяти контроллера
      .sdbuf_addr(sdbuf_addr),                 // текущий адрес в буферах чтения и записи
      .sdbuf_dataout(sdbuf_dataout),           // слово, читаемое из буфера чтения
      .sdbuf_datain(sdbuf_datain),             // слово, записываемое в буфер записи
      .sdbuf_we(sdbuf_we),                     // строб записи буфера

      .mode(sdmode),                               // режим ведущего-ведомого контроллера
      .controller_clk(wb_clk_i),                   // синхросигнал общей шины
      .reset(wb_rst_i),                            // сброс
      .sdclk(sdclock)                              // синхросигнал SD-карты
); 

//**************************************
//*  Сигнал ответа 
//**************************************
reg reply;

always @(posedge wb_clk_i or posedge wb_rst_i)
    if (wb_rst_i == 1) reply <= 1'b0;
    else if (wb_stb_i) reply <= 1'b1;
    else reply <= 1'b0;

assign wb_ack_o=reply & wb_stb_i;

//***************************************************
//*  Проверка выхода CHS за пределы диска
//***************************************************
assign v_iae=~(               
              (rhdc < cyl_limit)          // цилиндр
            & (rpda_hd < track_per_cyl)  // головка
            & (rpda_sa < spt));          // сектор
    
//**********************************************
// обработка прерываний и шинных транзакций
//**********************************************
always @(posedge wb_clk_i) begin 
      
      // сброс контроллера
      if (wb_rst_i | rhcs2_clr)   begin
         irq <= 1'b0 ; 
         interrupt_trigger <= 1'b0 ; 
         interrupt_state <= i_idle ; 
         rhcs2_clr <= 1'b0 ; 
         error_reset <= 1'b1 ; 
         read_start <= 1'b0 ; 
         write_start <= 1'b0 ; 
         rhcs1_rdy <= 1'b1 ; 
         rhcs1_rdyset <= 1'b0 ; 
         sdreq <= 1'b0;
         rhcs1_rdy <= 1'b1 ; 
         rhcs1_ie <= 1'b0 ; 
         rhcs1_fnc <= 5'b00000 ; 
         rhcs1_go <= 1'b0 ; 
         rhwc <= {16{1'b0}} ; 
         update_rhwc <= 1'b1 ; 
         rhba <= {16{1'b0}} ; 
         rpda_hd <= {8{1'b0}} ; 
         rpda_sa <= {8{1'b0}} ; 
         rhcs2_pe <= 1'b0 ; 
         rhcs2_nem <= 1'b0 ; 
         rhcs2_pge <= 1'b0 ; 
         rhcs2_mxf <= 1'b0 ; 
         rhcs2_or <= 1'b0 ; 
         rhcs2_ir <= 1'b1 ;
         rhdb <= 15'o0;
         rhcs2_pat <= 1'b0 ; 
         rhcs2_bai <= 1'b0 ; 
         unit <= 3'b000 ; 
         rhds_ata[0] <= 1'b0 ; 
         rhds_ata[1] <= 1'b0 ; 
         rhds_ata[2] <= 1'b0 ; 
         rhds_ata[3] <= 1'b0 ; 
         rhds_ata[4] <= 1'b0 ; 
         rhds_ata[5] <= 1'b0 ; 
         rhds_ata[6] <= 1'b0 ; 
         rhds_ata[7] <= 1'b0 ; 
         rhds_wrl <= 1'b0 ; 
         rhds_lst <= 1'b0 ; 
         rhds_dry <= 1'b1 ; 
         rhds_vv[0] <= 1'b1 ;
         rhds_vv[1] <= 1'b1 ;
         rhds_vv[2] <= 1'b1 ;
         rhds_vv[3] <= 1'b1 ;
         rhds_vv[4] <= 1'b1 ;
         rhds_vv[5] <= 1'b1 ;
         rhds_vv[6] <= 1'b1 ;
         rhds_vv[7] <= 1'b1 ;
         rhds_om <= 1'b0 ; 
         rhmr1 <= {16{1'b0}} ; 
         rhdc <= {16{1'b0}} ;
         rhbae <= {6{1'b0}} ; 
         rhcs3_ape <= 1'b0 ; 
         rhcs3_dpe <= {2{1'b0}} ; 
         rhcs3_wce <= {2{1'b0}} ; 
         rhcs3_dbl <= 1'b0 ; 
         rhcs3_ipck <= {4{1'b0}} ; 
         error_reset <= 1'b1 ; 
         sdreq <= 1'b0;
      end

      else  begin
        
        //******************************
        //* обработка прерывания
        //******************************
            case (interrupt_state)
                // нет активного прерывания
              i_idle :
                        begin
                           // Прерывание по готовности устройства
                           if (rhcs1_ie  & (rhcs1_rdyset  | rhds_ata[unit] )) begin
                              if (interrupt_trigger == 1'b0) begin
                                 interrupt_state <= i_req ; 
                                 irq <= 1'b1 ; 
                                 interrupt_trigger <= 1'b1 ; 
                              end                        
                              else    irq <= 1'b0 ;    // условий нет - снимаем запрос на прерывания                           
                           end                           
                           else interrupt_trigger <= 1'b0 ;                            
                        end
               // Формирование запроса на прерывание         
               i_req :
                        begin
                           if (rhcs1_ie == 1'b1) begin                           
                              // если прерывания вообще разрешены
                              if (iack == 1'b1) begin
                                 // если получено подтверждение прерывания от процессора
                                 irq <= 1'b0 ;               // снимаем запрос
                                 interrupt_state <= i_wait ; // переходим к ожиданию окончания обработки
                              end 
                           end
                           
                           else begin                           
                             // если прерывания запрещены
                              irq <= 1'b0;
                              interrupt_trigger <= 1'b0 ; 
                              interrupt_state <= i_idle ; 
                           end 
                        end
                        
                        
               // Ожидание окончания обработки прерывания         
               i_wait :   if (iack == 1'b0)  begin
                             interrupt_state <= i_idle ; 
                             rhcs1_ie <= 1'b0 ;
                          end 
             endcase
//========================================================================

//***********************************            
// Обработка транзакций общей шины
//***********************************            
            // чтение регистров
            if (wb_stb_i & ~wb_we_i)  begin
               case (wb_adr_i[5:1])
                  // rhcs1 17776701
                  5'b00000 :  wb_dat_o <= {rhcs1_sc, rhcs1_tre, 1'b0, 1'b0, 1'b1, 1'b0, rhcs1_bae, rhcs1_rdy, rhcs1_ie, rhcs1_fnc, rhcs1_go} ; 
                  // rhwc  17776702                                          
                  5'b00001 :  wb_dat_o <= (~wcp) + 1'b1 ; 
                  // rhba  17776704                                        
                  5'b00010 :  wb_dat_o <= rhba ; 
                  // rpda  17776706
                  5'b00011 :  wb_dat_o <= {3'b000, rpda_hd[4:0], 3'b000, rpda_sa[4:0]} ; 
                  // rhcs2 17776710
                  5'b00100 :  wb_dat_o <= {1'b0, 1'b0, rhcs2_pe, 1'b0, rhcs2_nem, rhcs2_pge, rhcs2_mxf, 1'b0, rhcs2_or, rhcs2_ir, rhcs2_clr, rhcs2_pat, rhcs2_bai, unit} ; 
                  // rhds  17776712
                  5'b00101 :  wb_dat_o <= {rhds_ata[unit], rhds_err, 1'b0, 1'b1, rhds_wrl, rhds_lst, 1'b0, 1'b1, rhds_dry, rhds_vv[unit], 5'b00000, rhds_om} ; 
                  // rher1 17776714
                  5'b00110 :  wb_dat_o <= {rher1_dck, rher1_uns, rher1_opi, rher1_dte, rher1_wle, rher1_iae, rher1_aoe, rher1_hcrc, rher1_hce, rher1_ech, rher1_wcf, rher1_fer, rher1_par, rher1_rmr, rher1_ilr, rher1_ilf} ; 
                  // rhas  17776716
                  5'b00111 :  wb_dat_o <= {8'b00000000, rhds_ata[7], rhds_ata[6], rhds_ata[5], rhds_ata[4], rhds_ata[3], rhds_ata[2], rhds_ata[1], rhds_ata[0] } ; 
                  // rhla  17776720 
                  5'b01000 :  wb_dat_o <= {5'b00000, rhla_sc, 6'b000000} ; 
                  // RHDB 17776722 - эмуляция FIFO
                  5'b01001 :  begin
                               wb_dat_o <= rhdb;  // текущее содержимое регистра
                               rhdb <= 15'o0;     // обнуляем его
                               rhcs2_or <= 1'b0;  // снимаем готовность выходных данных
                               rhcs2_ir <= 1'b1;  // поднимаем готовность входных данных
                              end 
                  // rhmr1  17776724
                  5'b01010 :  wb_dat_o <= rhmr1 ; 
                  // rhdt  17776726
                  5'b01011 :  wb_dat_o <= {1'b0, 15'o20022} ; 
                  // rmsn  17776730                                      
                  5'b01100 :  wb_dat_o <= {1'b0, 15'o20040} ; 
                  // rhof  17776732 
                  5'b01101 :  wb_dat_o <= {3'b000, rhof_fmt, rhof_eci, rhof_hci, 2'b00, rhof_ofd, 7'b0000000} ; 
                  // rhdc  17776734
                  5'b01110 :  wb_dat_o <= {6'b000000, rhdc[9:0]} ; 
                  // rmhr  17776736   
                  5'b01111 :  wb_dat_o <= {6'b000000, rhdc[9:0]} ; 
                  // rhmr2 17776740
                  5'b10000 :  wb_dat_o <= rhmr2 ; 
                  // rher2 17776742            15        14         13         12          11        10         9-8    7          6-4      3
                  5'b10001 :  wb_dat_o <= {rher2_bse, rher2_ski, rher2_ope, rher2_ivc, rher2_lsc, rher2_lbc, 2'b00, rher2_dvc, 3'b000, rher2_dpe, 3'b000} ; 
                  // rhec1 17776744
                  5'b10010 :  wb_dat_o <= {16{1'b0}} ; 
                  // rhec2 17776746
                  5'b10011 :  wb_dat_o <= {16{1'b0}} ; 
            //=== Следующие регистры присутствуют только в massbus-конфигурации (rh-11;)      
`ifdef massbus                  
                  // rhbae 17776750
                  5'b10100, 5'b11110 : wb_dat_o <= {10'b0000000000, rhbae} ; 
                  // rhcs3 17776752
                  5'b10101 : wb_dat_o <= {rhcs3_ape, rhcs3_dpe, rhcs3_wce, rhcs3_dbl, 3'b000, rhcs3_ie, 2'b00, rhcs3_ipck} ; 
`endif                  
                  // несуществующие регистры
                  default:    begin
                             rher1_ilr <= 1'b1 ;       // ошибка ILR
                            wb_dat_o <= {16{1'b0}} ; 
                    end   
               endcase 
            end 
            
            // Запись регистров
            else if (wb_stb_i & wb_we_i & ~reply) begin
               // блокировка записи регистров на время обработки команды
               if (rhcs1_go & (wb_adr_i[5:1] != 5'b00111) & (wb_adr_i[5:1] != 5'b01010))   rher1_rmr <= 1'b1 ; 
               else begin
                  // запись четных байтов
                  if (wb_sel_i[0]) begin
                     case (wb_adr_i[5:1])
                        // rhcs1 1777670
                        5'b00000 :
                                 begin
                                    rhcs1_rdyset <= wb_dat_i[7] ; 
                                    rhcs1_ie <= wb_dat_i[6] ;        // разрешение прерываний
                                    rhcs1_fnc <= wb_dat_i[5:1] ;     // код команды
                                    if (rhcs1_sc == 1'b0)  begin     // запуск - только при отсутствии ошибок
                                       rhcs1_go <= wb_dat_i[0] ;     // go - запуск команды
                                       if (rhds_err == 1'b0) rhds_ata[unit] <= 1'b0 ; // если ошибок нет - снимаем сигнал внимание
                                    end 
                                 end
                        // rhwc  17776702
                        5'b00001 :
                                 begin
                                    rhwc[7:0] <= wb_dat_i[7:0] ;  // ~размер передаваемых данных
                                    update_rhwc <= 1'b1 ;         // флаг обновления счетчика данных
                                 end
                        // rhba  17776704
                        5'b00010 :  rhba[7:0] <= {wb_dat_i[7:1],1'b0} ; 
                        // rpda  17776706
                        5'b00011 :  rpda_sa <= wb_dat_i[7:0] ; 
                        // rhcs2 17776710
                        5'b00100 :
                                 begin
                                    if ((wb_dat_i[5]) == 1'b1) rhcs2_clr <= 1'b1 ;  // общий сброс контроллера
                                    rhcs2_pat <= wb_dat_i[4] ; 
                                    rhcs2_bai <= wb_dat_i[3] ; 
                                    unit <= wb_dat_i[2:0] ; 
                                 end
                        // rher1 17776714 - принудительная установка признаков ошибки
                        5'b00110 :
                                 begin
                                    rher1_hce <= wb_dat_i[7] ; 
                                    rher1_ech <= wb_dat_i[6] ; 
                                    rher1_wcf <= wb_dat_i[5] ; 
                                    rher1_fer <= wb_dat_i[4] ; 
                                    rher1_par <= wb_dat_i[3] ; 
                                    rher1_rmr <= wb_dat_i[2] ; 
                                    rher1_ilr <= wb_dat_i[1] ; 
                                    rher1_ilf <= wb_dat_i[0] ; 
												if (|wb_dat_i[7:0]) rhds_ata[unit] <= 1'b1;
                                 end
                        // rhas  17776716 - установка сигналов внимание
                        5'b00111 :  
                                 begin
                                    if (wb_dat_i[0]) rhds_ata[0] <= 1'b0 ; 
                                    if (wb_dat_i[1]) rhds_ata[1] <= 1'b0 ; 
                                    if (wb_dat_i[2]) rhds_ata[2] <= 1'b0 ; 
                                    if (wb_dat_i[3]) rhds_ata[3] <= 1'b0 ; 
                                    if (wb_dat_i[4]) rhds_ata[4] <= 1'b0 ; 
                                    if (wb_dat_i[5]) rhds_ata[5] <= 1'b0 ; 
                                    if (wb_dat_i[6]) rhds_ata[6] <= 1'b0 ; 
                                    if (wb_dat_i[7]) rhds_ata[7] <= 1'b0 ; 
                                 end
                         // RHDB 17 776 722 - эмуляция буфера FIFO
                        5'b01001 :  begin
                               rhdb <= wb_dat_i;   // вводим данные в буфер
                               rhcs2_or <= 1'b1;   // поднимаем готовность выдачи данных
                               rhcs2_ir <= 1'b0;   // снимаем готовность приема данных
                              end 
                        
                        // rhmr1  17776724
                        5'b01010 :
                                 begin
                                    if ((wb_dat_i[3]) == 1'b1) rhds_wrl <= 1'b1 ;         // программная защита записи
                                    if ((wb_dat_i[0]) == 1'b1) rhds_vv[unit] <= 1'b0 ;    // признак установки тома VV
                                    rhmr1[0] <= wb_dat_i[0] ;                             // диагностический режим
                                    if ((wb_dat_i[0]) == 1'b0)   rhmr1 <= {16{1'b0}} ;    // выход из режима диагностики
                                 end
                        // 176726 - только для коррекции странного поведения RSTS, этого регистра реально не существует
                        5'b01011 :   ;      
                        // rhof  17776732
                        5'b01101 :  rhof_ofd <= wb_dat_i[7] ; 
                        // rhdc  17776734
                        5'b01110 :
                                 begin
                                    rhdc[7:0] <= wb_dat_i[7:0] ; 
                                    rhds_om <= 1'b0 ; 
                                 end
                        // rher2 17776742
                        5'b10001 :
                                 begin
                                    rher2_dvc <= wb_dat_i[7] ; 
                                    rher2_dpe <= wb_dat_i[3] ; 
                                 end
`ifdef massbus                  											
                        // rhbae 17776750
                        5'b10100, 5'b11110 :
                                    rhbae <= wb_dat_i[5:0] ; 
                        // rhcs3 17776752
                        5'b10101: begin
                                    rhcs3_ape <= wb_dat_i[15];
                                    rhcs3_dpe <= wb_dat_i[14:13];
                                    rhcs3_wce <= wb_dat_i[12:11];
                                    rhcs3_dbl <= wb_dat_i[10];
                                  end
`endif											 
                        // несуществующие регистры
                        default:      rher1_ilr <= 1'b1 ; 
                     endcase 
                  end 
                  
                  // запись нечетных байтов
                  if (wb_sel_i[1]) begin
                     case (wb_adr_i[5:1])
                        // rhcs1 17776700
                        5'b00000 :   rhbae[1:0] <= wb_dat_i[9:8] ; 
                        // rhwc   17776702
                        5'b00001 :
                                 begin
                                    rhwc[15:8] <= wb_dat_i[15:8] ; 
                                    update_rhwc <= 1'b1 ; 
                                 end
                        // rhba   17776704
                        5'b00010 :  rhba[15:8] <= wb_dat_i[15:8] ; 
                        // rpda   17776706
                        5'b00011 :  rpda_hd <= wb_dat_i[15:8] ; 
                        // rhcs2 17776710
                        5'b00100 :
                                 begin
                                    rhcs2_pe <= wb_dat_i[13] ; 
                                    rhcs2_mxf <= wb_dat_i[9] ; 
                                 end
                        // rher1 17776714 - принудительная установка флагов ошибок
                        5'b00110 :
                                 begin
                                    rher1_dck <= wb_dat_i[15] ; 
                                    rher1_uns <= wb_dat_i[14] ; 
                                    rher1_opi <= wb_dat_i[13] ; 
                                    rher1_dte <= wb_dat_i[12] ; 
                                    rher1_wle <= wb_dat_i[11] ; 
                                    rher1_iae <= wb_dat_i[10] ; 
                                    rher1_aoe <= wb_dat_i[9] ; 
                                    rher1_hcrc <= wb_dat_i[8] ; 
                                 end
                        // rhof  17776732
                        5'b01101 :
                                 begin
                                    rhof_fmt <= wb_dat_i[12] ; 
                                    rhof_eci <= wb_dat_i[11] ; 
                                    rhof_hci <= wb_dat_i[10] ; 
                                 end
                        // rhdc  17776734
                        5'b01110 :
                                 begin
                                    rhdc[15:8] <= wb_dat_i[15:8] ; 
                                    rhds_om <= 1'b0 ; 
                                 end
                        // rher2 17776742 - принудительная установка флагов ошибок
                        5'b10001 :
                                 begin
                                    rher2_bse <= wb_dat_i[15] ; 
                                    rher2_ski <= wb_dat_i[14] ; 
                                    rher2_ope <= wb_dat_i[13] ; 
                                    rher2_ivc <= wb_dat_i[12] ; 
                                    rher2_lsc <= wb_dat_i[11] ; 
                                    rher2_lbc <= wb_dat_i[10] ; 
                                 end
`ifdef massbus                  											
                        // rhcs3 17776752
                        5'b10101: begin
                                    rhcs1_ie <= wb_dat_i[6];
                                    rhcs3_ipck <= wb_dat_i[3:0];
                                  end              
`endif											 
                     endcase 
                  end 
               end 
            end

//******************************************            
//* импульсы последовательности секторов
//******************************************            
            rmclock <= rmclock + 1'b1 ;   // счетчик-делитель частоты следования секторов
            if (|rmclock == 1'b0)  rhla_sc <= rhla_sc + 5'b00001 ;  // счетчик секторов
            
            
//******************************************            
//*  Установка сигнала готовности RDY
//******************************************            
            if (rhcs1_rdyset == 1'b1) begin
               rhcs1_rdy <= 1'b1 ; 
               rhcs1_rdyset <= 1'b0 ; 
            end 

//******************************************            
//*  Обновление счетчика передаваемых слов
//******************************************            
            if (update_rhwc == 1'b1)  begin
               update_rhwc <= 1'b0 ; 
               wcp <= (~rhwc) + 1'b1 ; 
            end 

//******************************************            
//* генерация бита четности
//******************************************            
            if (rhcs2_pat & 
               (wb_dat_i[15] ^ wb_dat_i[14] ^ wb_dat_i[13] ^ wb_dat_i[12] ^ wb_dat_i[11] ^ 
                wb_dat_i[10] ^ wb_dat_i[9] ^ wb_dat_i[8] ^ wb_dat_i[7] ^ wb_dat_i[6] ^ 
                wb_dat_i[5] ^ wb_dat_i[4] ^ wb_dat_i[3] ^ wb_dat_i[2] ^ wb_dat_i[1] ^ 
                wb_dat_i[0]))   rher1_par <= 1'b1 ; 

//******************************************            
//* запуск выполнения команды    
//******************************************            
            if (rhcs1_go)  begin
               // сбрасываем текущие ошибки
               rhcs2_pe <= 1'b0 ; 
               rhcs2_nem <= 1'b0 ; 
               rhcs2_mxf <= 1'b0 ; 
               rhcs2_pge <= 1'b0 ; 
               if ((rhds_vv[unit] == 1'b0 | rhds_dry == 1'b0) & (rhcs1_fnc != 5'o10) & (rhcs1_fnc != 5'o11) & (rhcs1_fnc != 5'o00)) begin
                  // Для неустановленного тома допустимы только команды NOP, "валидация" и "подготовка к начальной загрузке"
                  // для остальных команд - ошибка IVC
                  rher2_ivc <= 1'b1 ; 
                  rhcs1_go <= 1'b0 ; 
                  rhds_ata[unit] <= 1'b1 ; 
               end
                              
               else  begin
                  // разбор поля команды
                  case (rhcs1_fnc)
                     5'o0 :
                              begin
                                 // NOP - пустая команда
                                 rhcs1_go <= 1'b0 ; 
                                 rhds_dry <= 1'b1 ; 
                              end
                     5'o2 :
                              begin
                                 // позиционирование
                                 if (v_iae)  rher1_iae <= 1'b1 ;   // некорректный CHS
                                 rhds_ata[unit] <= 1'b1 ;          // поднимаем сигнал внимание
                                 rhcs1_go <= 1'b0 ;                // завершаем команду
                                 rhds_dry <= 1'b1 ;                // поднимаем флаг готовности
                                 rhcs1_rdyset <= 1'b1 ; 
                              end
                     5'o3 :
                              begin
                                 // рекалибровка
                                 // переходим к CHS=0
                                 rpda_hd <= {8{1'b0}} ;  
                                 rpda_sa <= {8{1'b0}} ; 
                                 rhdc <= {16{1'b0}} ; 
                                 rhds_vv[unit] <= 1'b1 ;        // валидация тома
                                 rhds_om <= 1'b0 ;              // снимаем режим смещения дорожки
                                 rhof_ofd <= 1'b0 ;   
                                 rhof_hci <= 1'b0 ; 
                                 rhof_eci <= 1'b0 ; 
                                 rhof_fmt <= 1'b0 ; 
                                 rhcs1_go <= 1'b0 ;             // завершаем команду
                                 rhds_dry <= 1'b1 ;             // поднимаем флаг готовности
                                 rhds_ata[unit] <= 1'b1 ;       // поднимаем сигнал внимание
                              end
                     5'o4 :
                              begin
                                 // сброс ошибок устройства
                                 error_reset <= 1'b1 ; 
                                 rhcs1_go <= 1'b0 ; 
                                 rhds_dry <= 1'b1 ; 
                              end
                     5'o5 :
                              begin
                                 // освобождение тома
                                 rhcs1_go <= 1'b0 ; 
                                 rhds_dry <= 1'b1 ; 
                              end
                     5'o6 :
                              begin
                                 // установка режима смещения
                                 rhds_om <= 1'b1 ;              // флаг режима смещения
                                 rhcs1_go <= 1'b0 ;             // завершаем команду
                                 rhds_dry <= 1'b1 ;             // поднимаем флаг готовности
                                 rhds_ata[unit] <= 1'b1 ;       // поднимаем сигнал внимание
                              end
                     5'o7 :
                              begin
                                 // отключение режима смещения
                                 rhds_om <= 1'b0 ;              // флаг режима смещения
                                 rhcs1_go <= 1'b0 ;             // завершаем команду
                                 rhds_dry <= 1'b1 ;             // поднимаем флаг готовности
                                 rhds_ata[unit] <= 1'b1 ;       // поднимаем сигнал внимание
                              end
                     5'o10 :
                              begin
                                 // подготовка к начальной загрузке
                                 
                                 //   установка CHS=0
                                 rpda_hd <= {8{1'b0}} ; 
                                 rpda_sa <= {8{1'b0}} ; 
                                 rhdc <= {16{1'b0}} ; 
                                 
                                 rhds_vv[unit] <= 1'b1 ;   // валидация тома
                                 rhds_om <= 1'b0 ; 
                                 rhof_ofd <= 1'b0 ; 
                                 rhof_hci <= 1'b0 ; 
                                 rhof_eci <= 1'b0 ; 
                                 rhof_fmt <= 1'b0 ; 
                                 rhcs1_go <= 1'b0 ; 
                                 rhds_dry <= 1'b1 ; 
                              end
                     5'o11 :
                              begin
                                 // подтверждение установки тома
                                 rhds_vv[unit] <= 1'b1 ; 
                                 rhcs1_go <= 1'b0 ; 
                                 rhds_dry <= 1'b1 ; 
                              end
                     5'o14 :
                              begin
                                 // поиск дорожки
                                 if (v_iae)  rher1_iae <= 1'b1 ; // при некорректном CHS - поднимаем ошибку IAE
                                 rhcs1_go <= 1'b0 ;             // завершаем команду
                                 rhds_dry <= 1'b1 ;             // поднимаем флаг готовности
                                 rhds_ata[unit] <= 1'b1 ;       // поднимаем сигнал внимание
                              end
                     
                     5'o30, 5'o31 :
                              begin
                                 // запись данных
                                 //----------------------------------------------------------------
                                 rhcs1_rdy <= 1'b0 ;    // снимаем сигнал готовности
                                 rhds_om <= 1'b0 ; 
                                 if (rhcs1_go == 1'b1) sdreq <= 1'b1;  // запрос на доступ к карте
                                 
                                 // доступ к карте предоставлен, запись еще не запущена
                                 if (sdack & sdcard_idle & (write_start == 1'b0)) begin
                                    if (v_iae == 1'b1) begin 
                                       // если CHS корректен - ошибка IAE и завершаем команду
                                       rher1_iae <= 1'b1 ; 
                                       rhcs1_go <= 1'b0 ; 
                                       rhds_dry <= 1'b1 ; 
                                       rhcs1_rdyset <= 1'b1 ; 
                                       rhds_ata[unit] <= 1'b1 ; 
                                       sdreq <= 1'b0;
                                    end
                                    else if (rhds_wrl == 1'b1) begin
                                       // Установлен режим только чтение - блокируем запись
                                       rher1_wle <= 1'b1 ; 
                                       rhds_ata[unit] <= 1'b1 ; 
                                       rhcs1_go <= 1'b0 ; 
                                       rhds_dry <= 1'b1 ; 
                                       rhcs1_rdyset <= 1'b1 ; 
                                       sdreq <= 1'b0;
                                    end
                                    else begin 
                                       // проверка окончена - запускаем запись
                                       write_start <= 1'b1 ;    // запускаем DMA-контроллер на прием данных
                                       if ((rhcs1_fnc[0] == 1'b1) & (wcp >= 16'o2)) wcp <= wcp - 2'd2 ;  // режим записи заголовка - отрезаем 4 байта заголовка от счетчика
                                    end 
                                 end
                                 
                                 // запись данных окончена
                                 else if (write_start == 1'b1 & iocomplete == 1'b1) begin
                                    write_start <= 1'b0 ;  // освобождаем DMA-контроллер
                                     
                                    if (nxm == 1'b0 & sdcard_error == 1'b0)  begin
                                       // ошибок не обнаружено
                                       if (rpda_sa == spt - 1) begin
                                          // переход на новую головку
                                          rpda_sa <= {8{1'b0}} ; 
                                          if (rpda_hd == track_per_cyl - 1)  begin
                                             // переход на новый цилиндр
                                             rpda_hd <= {8{1'b0}} ; 
                                             if (rhdc == cyl_limit)   rher1_aoe <= 1'b1 ; // ошибка - выход за границу диска
                                             else  begin
                                                rhdc <= rhdc + 1'b1 ;  // цилиндр ++
                                                if (rhdc == cyl_limit - 1)   rhds_lst <= 1'b1 ; // признак последнего цилиндра
                                             end 
                                          end
                                          else  rpda_hd <= rpda_hd + 1'b1 ;  // головка++
                                       end

                                       else  rpda_sa <= rpda_sa + 1'b1 ;  // сектор++
                                       
                                       // установка регистров текущего адреса в памяти rhba
                                       rhbae <= ram_phys_addr[21:16] ;         
                                       rhba <= {ram_phys_addr[15:1], 1'b0} ; 

                                       // вычитаем размер сектора из счетчика запрошенных данных
                                       if ((wcp) > (16'b0000000100000000))  wcp <= (wcp) - (16'b0000000100000000) ; 
                                       else  begin
                                          // счетчик данных исчерпан, меньше размера сектора - завершаем процесс записи
                                          wcp <= {16{1'b0}} ;    // обнуляем счетчик данных
                                          rhcs1_go <= 1'b0 ;     // снимаем бит активности команды
                                          rhds_dry <= 1'b1 ;     // признак готовности контроллера
                                          rhcs1_rdyset <= 1'b1 ; 
                                          sdreq <= 1'b0;         // освобождаем SD-карту
                                       end 
                                    end
                                    
                                    // обработка ошибок записи
                                    else  begin
                                       // выводим адрес ошибки в регистры rhba
                                       rhbae <= ram_phys_addr[21:16] ; 
                                       rhba <= {ram_phys_addr[15:1], 1'b0} ; 
                                       rhcs1_go <= 1'b0 ;        // снимаем бит активности команды
                                       rhds_dry <= 1'b1 ;        // признак готовности контроллера
                                       if (nxm == 1'b1)   rhcs2_nem <= 1'b1 ;  // признак таймаута шины
                                       if (sdcard_error == 1'b1)  rher1_dck <= 1'b1 ; // признак ошибок ввода-вывода 
                                       rhds_ata[unit] <= 1'b1 ; 
                                       rhcs1_rdyset <= 1'b1 ; 
                                       sdreq <= 1'b0;
                                    end 
                                 end 
                              end

                     5'o34, 5'o35, 5'o24, 5'o25 :
                              begin
                                 // чтение/верификация
                                 //----------------------------------------------------------------
                                 rhcs1_rdy <= 1'b0 ;    // снимаем сигнал готовности
                                 if (rhcs1_go == 1'b1) sdreq <= 1'b1;  // запрос на доступ к карте
                                 if (sdack & sdcard_idle & ~read_start & ~iocomplete)  begin
                                    // доступ к карте получен, SDSPI готов к обмену
                                    if (v_iae == 1'b1)  begin
                                         // некорректный CHS - ошибка IAE
                                       rher1_iae <= 1'b1 ; 
                                       rhcs1_go <= 1'b0 ; 
                                       rhds_dry <= 1'b1 ; 
                                       rhcs1_rdyset <= 1'b1 ; 
                                       rhds_ata[unit] <= 1'b1 ; 
                                       sdreq <= 1'b0;
                                    end
                                    else  begin
                                       // проверка окончена - запускаем DMA-контроллер на передачу данных
                                       read_start <= 1'b1 ; 
                                       if ((rhcs1_fnc[0]) == 1'b1 & (wcp) >= (16'b0000000000000010)) wcp <= wcp - 2'd2 ;  // режим чтения заголовка - отрезаем 4 байта заголовка от счетчика
                                    end 
                                 end
                                 // чтение окончено
                                 else if (read_start == 1'b1 & iocomplete == 1'b1) begin
                                    read_start <= 1'b0;
                                    if (nxm == 1'b0 & sdcard_error == 1'b0)   begin
                                       // ошибок не было
                                       if (rpda_sa == spt - 1)  begin
                                          // переход на новую головку
                                          rpda_sa <= {8{1'b0}} ; 
                                          if (rpda_hd == track_per_cyl - 1)  begin
                                             // переход на новый цилиндр
                                             rpda_hd <= {8{1'b0}} ; 
                                             if (rhdc == cyl_limit) rher1_aoe <= 1'b1 ;  // признак выхода за границу диска
                                             else begin
                                                rhdc <= rhdc + 1'b1 ;    // цилиндр++
                                                if (rhdc == cyl_limit - 1)  rhds_lst <= 1'b1 ;  // признак последнего цилиндра
                                             end 
                                          end
                                          else   rpda_hd <= rpda_hd + 1'b1 ;   // головка++
                                       end

                                       else    rpda_sa <= rpda_sa + 1'b1 ;  // сектор++
                                       // передача текущего адреса физической памяти в регистр rhba
                                       rhbae <= ram_phys_addr[21:16] ; 
                                       rhba <= {ram_phys_addr[15:1], 1'b0} ; 
                                       // вычитаем размер сектора из счетчика запрашиваемых данных
                                       if ((wcp) > (16'b0000000100000000))  wcp <= (wcp) - (16'b0000000100000000) ; 
                                       // счетчик меньше размера сектора - завершаем чтение
                                       else  begin
                                          wcp <= {16{1'b0}} ; 
                                          rhcs1_go <= 1'b0 ; 
                                          rhds_dry <= 1'b1 ; 
                                          rhcs1_rdyset <= 1'b1 ; 
                                          sdreq <= 1'b0;
                                       end 
                                    end
                                    
                                    // обработка ошибок чтения
                                    else  begin
                                       // заносим адрес ошибки в регистр rhba
                                       rhbae <= ram_phys_addr[21:16] ; 
                                       rhba <= {ram_phys_addr[15:1], 1'b0} ; 
                                       rhcs1_go <= 1'b0 ; 
                                       rhds_dry <= 1'b1 ; 
                                       if (nxm == 1'b1) rhcs2_nem <= 1'b1 ;   // таймаут шины
                                       if (sdcard_error == 1'b1) rher1_dck <= 1'b1 ; // ошибка ввода-вывода
                                       rhds_ata[unit] <= 1'b1 ; 
                                       rhcs1_rdyset <= 1'b1 ; 
                                       sdreq <= 1'b0;
                                    end 
                                 end 
                              end
                     default :
                              begin
                              // Ошибочные коды команд
                              //---------------------------------------
                                 rher1_ilf <= 1'b1 ; 
                                 rher2_ivc <= 1'b1 ; 
                                 rhds_ata[unit] <= 1'b1 ; 
                                 rhcs1_go <= 1'b0 ; 
                                 rhds_dry <= 1'b1 ; 
                                 sdreq <= 1'b0;
                              end
                  endcase 
               end 
            end 

//*********************************************
//*    Сброс ошибок
//*********************************************
            if (error_reset == 1'b1)  begin
               error_reset <= 1'b0 ; 
               rher1_dck <= 1'b0 ; 
               rher1_uns <= 1'b0 ; 
               rher1_opi <= 1'b0 ; 
               rher1_dte <= 1'b0 ; 
               rher1_wle <= 1'b0 ; 
               rher1_iae <= 1'b0 ; 
               rher1_aoe <= 1'b0 ; 
               rher1_hcrc <= 1'b0 ; 
               rher1_hce <= 1'b0 ; 
               rher1_ech <= 1'b0 ; 
               rher1_wcf <= 1'b0 ; 
               rher1_fer <= 1'b0 ; 
               rher1_par <= 1'b0 ; 
               rher1_rmr <= 1'b0 ; 
               rher1_ilr <= 1'b0 ; 
               rher1_ilf <= 1'b0 ; 
               rher2_dpe <= 1'b0 ; 
               rher2_dvc <= 1'b0 ; 
               rher2_lbc <= 1'b0 ; 
               rher2_lsc <= 1'b0 ; 
               rher2_ivc <= 1'b0 ; 
               rher2_ope <= 1'b0 ; 
               rher2_ski <= 1'b0 ; 
               rher2_bse <= 1'b0 ; 
               rhmr1 <= 16'b0000000000001000 ; 
               rhmr2 <= {1'b0, 15'o11777} ; 
            end 
      end  
   end 

//**********************************        
//*   Контроллер DMA
//**********************************        
always @(posedge wb_clk_i)    begin
      if (wb_rst_i == 1'b1) begin
      
         // сброс системы
         dma_state <= dma_idle ; 
         dma_req <= 1'b0 ; 
         dma_we_o <= 1'b0;
         dma_stb_o <= 1'b0;
         dma_adr_o <= 22'o0;
         sdspi_write_mode <= 1'b0 ; 
         sdspi_start <= 1'b0;
         nxm <= 1'b0 ; 
         iocomplete <= 1'b0;
      end
      
      else   begin
            case (dma_state)
               // ожидание запроса
               dma_idle :
                        begin
                           nxm <= 1'b0 ;       //  снимаем флаг ошибки nxm
                           iocomplete <= 1'b0; // снимаем флаг завершения ввода-вывода
                           dma_we_o <= 1'b0;   // снимает флаг записи
                           dma_stb_o <= 1'b0;  // снимаем строб данных
                           dma_adr_o <= 22'o0; // обнуляем адрес
                           
                           // старт процедуры записи
                           if (write_start == 1'b1) begin
                              sdcard_addr <= sdaddr;                   // получаем адрес SD-сектора                
                              dma_req <= 1'b1 ;                        // поднимаем запрос DMA
                              if (dma_gnt == 1'b1) begin               // ждем подтверждения DMA
                                 dma_state <= dma_write1 ; // переходим к этапу 1 записи
                                 if (rhcs1_fnc == 5'b11001) ram_phys_addr <= ({rhbae, rhba[15:1]}) + 2'd2 ; // запись с заголовком
                                 else                       ram_phys_addr <= {rhbae, rhba[15:1]} ;          // запись без заголовка
                                 
                                 // вычисление количества байтов в текущем секторе (передача может быть неполной)
                                 if (wcp >= 16'o400) sector_data_index <= 9'o400;               // запрошен полный сектор или больше
                                 else if (wcp == 16'o0) sector_data_index <= 9'b000000000 ;     // запрошено 0 байт данных
                                 else                sector_data_index <= {1'b0, wcp[7:0]} ;    // запрошено меньше сектора
                                 sdbuf_addr <= 8'b11111111 ;                              // адрес в буфере sd-контроллера
                              end 
                           end
                           // старт процедуры чтения
                           else if (read_start == 1'b1) begin
                                 sdcard_addr <= sdaddr;                   // получаем адрес SD-сектора                
                                 // проверка на режим чтения заголовков
                                 ram_phys_addr <= {rhbae, rhba[15:1]} ; 
                                 if (rhcs1_fnc[0] == 1'b1) begin
                                    dma_req <= 1'b1 ;                        // поднимаем запрос DMA
                                    if (dma_gnt == 1'b1)  begin              // ждем подтверждения DMA
                                       dma_state <= dma_readh ; // переходим к чтению заголовков
                                    end   
                                 end 
                                 // режим чтения данных
                                 else  dma_state <= dma_readsector;                 // переходим к чтению данных
                                 // коррекция счетчика читаемых слов
                                 if (wcp >= 16'o400)  sector_data_index <= 9'o400;             // запрошен сектор и больше
                                 else                 sector_data_index <= {1'b0, wcp[7:0]} ;  // запрошено меньше сектора
                                 sdbuf_addr <= 0 ;                                       // начальный адрес в буфере SD-контроллера
                           end 
                        end
           
               // Чтение заголовков- слово 1
               dma_readh :
                        begin
                           dma_adr_o <= {ram_phys_addr, 1'b0} ; 
                           dma_dat_o <= {3'b110, rhof_fmt, rhdc[11:0]} ;  // сборка слова 1 заголовка
                           dma_stb_o <= 1'b1 ;
                           dma_we_o <= 1'b1;
                           ram_phys_addr <= ram_phys_addr + 1'b1 ; 
                           if (dma_ack_i == 1'b1) begin
                              dma_state <= dma_readh2; 
                              dma_stb_o <= 1'b0;
                           end   
                        end
                        
               // Чтение заголовков- слово 2
               dma_readh2 :
                        begin
                           dma_adr_o <= {ram_phys_addr, 1'b0} ; 
                           dma_dat_o <= {rpda_hd, rpda_sa} ;   // сборка слова 2 заголовка
                           dma_stb_o <= 1'b1 ; 
                           ram_phys_addr <= ram_phys_addr + 1'b1 ; 
                           if (dma_ack_i == 1'b1) begin
                              dma_state <= dma_readsector; 
                              dma_stb_o <= 1'b0;
                           end   
                        end
                        
               // Чтение сектора данных в буфер SDSPI     
               dma_readsector :
                        begin
                           sdspi_start <= 1'b1;          // запускаем SDSPI
                           sdspi_write_mode <= 1'b0;     // режим чтения
                           if (sdspi_io_done == 1'b1) begin
                              // чтение сектора закончено
                              dma_req <= 1'b1 ;                        // поднимаем запрос DMA
                              ram_phys_addr <= {rhbae, rhba[15:1]} ;   // выставляем физический адрес на шину
                              // ждем подтверждения DMA
                              if (dma_gnt == 1'b1)  dma_state <= dma_preparebus; 
                           end 
                        end   
                        // чтение данных - подготовка шины к DMA
               dma_preparebus :
                        begin
                           sdspi_start <= 1'b0;
                           dma_state <= dma_read ; 
                           dma_adr_o <= {ram_phys_addr, 1'b0} ; // выставляем адрес на шину
                           dma_stb_o <= 1'b0 ;                        // снимаем строб данных 
                           dma_we_o <= 1'b0 ;                         // снимаем строб записи
                           reply_count <= 6'b111111;                  // взводим таймер ожидания шины
                           dma_state <= dma_read ; // переходим к чтению заголовков
                        end
                        // чтение данных - обмен по шине
               dma_read :
                        begin
                           if (sector_data_index != 9'o0)  begin
                              // передача данных сектора
                              dma_dat_o <= sdbuf_dataout ;    // выставляем данные
                              dma_we_o <= 1'b1;               // режим записи
                              dma_stb_o <= 1'b1 ;             // строб записи на шину
                              reply_count <= reply_count - 1'b1; // таймер ожидания ответа
                              if (|reply_count == 1'b0) begin
                                // таймаут шины
                                nxm <= 1'b1;
                                dma_state <= dma_read_done ; 
                              end  
                              if (dma_ack_i == 1'b1) begin   // устройство подтвердило обмен
                                  dma_state <= dma_preparebus; 
                                  dma_stb_o <= 1'b0 ;                        // снимаем строб данных 
                                  dma_we_o <= 1'b0 ;                         // снимаем строб записи
                                  ram_phys_addr <= ram_phys_addr + 1'b1 ; // если разрешено, увеличиваем физический адрес
                                  sector_data_index <= sector_data_index - 1'b1 ;       // уменьшаем счетчик данных сектора
                                  sdbuf_addr <= sdbuf_addr + 1'b1 ;         // увеличиваем адрес буфера SD
                              end    
                           end
                           else begin
                              // все сектора прочитаны 
                              dma_state <= dma_read_done ; 
                              dma_stb_o <= 1'b0 ; 
                              dma_we_o <= 1'b0 ; 
                           end 
                        end
               dma_read_done :
                        begin
                           dma_req <= 1'b0 ;        // освобождаем шину
                           dma_stb_o <= 1'b0 ; 
                           dma_we_o <= 1'b0 ; 
                           // ждем подтверждения завершения операции
                           if (read_start == 1'b0) begin
                              dma_state <= dma_idle ; // переходим в состояние ожидания команды
                              iocomplete <= 1'b0;                 // снимаем подтверждение окончания работы
                           end 
                           else iocomplete <= 1'b1;  // подтверждаем окончание обмена
                        end
                        

               // этап 1 записи - подготовка шины к DMA
               dma_write1 :
                        begin
                              sector_data_index <= sector_data_index - 1'b1 ; // уменьшаем счетчик записанных данных
                              dma_we_o <= 1'b0 ;   // режим передачи DMA - из памяти к устройству
                              dma_stb_o <= 1'b1 ;  // поднимаем строб чтения
                              dma_adr_o <= {ram_phys_addr, 1'b0} ; // выставляем адрес на шину
                              ram_phys_addr <= ram_phys_addr + 1'b1 ;  //  физический адрес в памяти ++
                              sdbuf_we <= 1'b1 ;         // поднимаем флаг режима записи sdspi
                              sdbuf_addr <= sdbuf_addr + 1'b1 ; // адрес буфера sdspi++
                              reply_count <= 6'b111111;  // взводим таймер ожидания ответа
                              
                              dma_state <= dma_write ;  
                        end
                        
               // перепись данных сектора из памяти в буфер контроллера через DMA         
               dma_write :
                        begin
                           reply_count <= reply_count - 1'b1; // таймер ожидания ответа
                           
                             // таймаут шины
                           if (|reply_count == 1'b0) begin
                                nxm <= 1'b1;    // ошибка NXM
                                dma_stb_o <= 1'b0 ;      // снимаем строб записи
                                dma_state <= dma_write_done ; // прекращаем обработку DMA 
                           end  
                           // ожидание ответа шины
                           else if (dma_ack_i) begin   // шина подтвердила готовность данных
                                 sdbuf_datain <= dma_dat_i ; // передаем байт данные с шины на вход sdspi
//                                 dma_adr_o <= {ram_phys_addr, 1'b0} ; // выставляем адрес на шину
                                 dma_stb_o <= 1'b0 ;      // снимаем строб записи
                                 if (sector_data_index == 9'o0) begin
                                    // конец данных - освобождаем шину
                                    if (sdbuf_addr == 255) dma_state <= dma_write_wait ;  // сектор дописан полностью
                                    else                   dma_state <= dma_write_fill;   // сектор недописан
                                    dma_req <= 1'b0 ;   // снимаем запрос DMA
                                  end 
                                 else  dma_state <= dma_write_delay ;  
                           end   
                        end
               // задержка 1 такт между операциями DMA-чтения         
               dma_write_delay: dma_state <= dma_write1;         
               // дописывание нулей в конец неполного сектора         
               dma_write_fill :
                        begin
                           if (sdbuf_addr == 255)  dma_state <= dma_write_wait ; 
                           else   begin
                              sdbuf_datain <= {16{1'b0}} ; 
                              sdbuf_addr <= sdbuf_addr + 1'b1 ; 
                              sdbuf_we <= 1'b1 ; 
                           end 
                        end
                        
               dma_write_wait :
                        begin
                           sdspi_start <= 1'b1 ; 
                           sdspi_write_mode <= 1'b1;
                           sdbuf_we <= 1'b0 ; 
                           if (sdspi_io_done == 1'b1)   begin
                              dma_state <= dma_write_done ; 
                              sdspi_start <= 1'b0 ; 
                              sdspi_write_mode <= 1'b0;
                              iocomplete <= 1'b1;
                           end 
                        end
               dma_write_done :
                        begin
                           if (write_start == 1'b0)  begin
                              iocomplete <= 1'b0;
                              dma_state <= dma_idle ; 
                           end 
                        end
                        
            endcase 
      end  
   end 

//********************************************************************
// Формирователь адреса SD-карты, 23-битный адрес
// rhdc * 608
//********************************************************************
wire[22:0] ca_offset; 
wire[22:0] dn_offset; 
wire[22:0] sd_addr; 
wire[22:0] lowsdoffset; 
wire[22:0] udn_offset; 

// смещение до цилиндра - cyl*418
assign ca_offset = 
     ({5'b00000, rhdc[9:0], 8'b00000000}) +    // cyl*256
     ({6'b000000, rhdc[9:0], 7'b0000000}) +    // cyl*128
     ({8'b00000000, rhdc[9:0], 5'b00000}) +    // cyl*32
     ({12'b000000000000, rhdc[9:0], 1'b0});    // cyl*2

// смещение до начала диска на SD-карте для дисков 0-3
assign dn_offset = (unit[1:0] == 2'b00) ? 23'h000000 : 
                  (unit[1:0] == 2'b01) ? 23'h060000 :
                  (unit[1:0] == 2'b10) ? 23'h0c0000 :
                                            23'h120000 ;
// дополнительное смещение для дисков 4-7
assign udn_offset = ((unit[2]) == 1'b1) ? 23'h180000 : 23'h000000;

// полный SD-адрес - цилиндр, головка, сектор  =  dev_offset+cyl*418+hd*22+sc
assign sdaddr = 
     start_offset +
     udn_offset +
     dn_offset + 
     ca_offset + 
     ({14'b00000000000000, rpda_hd[4:0], 4'b0000}) + 
     ({16'b0000000000000000, rpda_hd[4:0], 2'b00}) + 
     ({17'b00000000000000000, rpda_hd[4:0], 1'b0}) + 
     ({18'b000000000000000000, rpda_sa[4:0]});

endmodule
