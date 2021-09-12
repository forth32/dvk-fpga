//======================================================================================================
//   Ядро процессора PDP2011
//======================================================================================================

// длительность программного сброса командой reset в тактах
`define initcycles_reset 100
// Время ожидание ответа шины в тактах
`define busdelay 50

module cpu2011 (

   // общая шина
   output reg [15:0] wbm_adr_o,   // шина виртуального адреса
   input      [15:0] wbm_dat_i,   // входная шина данных (bus -> cpu)
   output reg [15:0] wbm_dat_o,   // выходная шина данных (cpu -> bus)
   output reg        wbm_we_o,    // признак цикла записи на шину
   output reg [1:0]  wbm_sel_o,   // выбор байтов для записи
   output reg        wbm_stb_o,   // строб обмена по шине
   input             wbm_ack_i,   // подтверждение транзакции (REPLY)
   input clk,                     // тактовый синхросигнал
   input reset,                   // сброс процессора
   output reg init,               // выход сброса для всей периферии на шине

   // Информация от типе цикла шины   
   output reg cp,                 // 1 - обмен по шине происходит в предыдущем режиме процессора
   output reg ifetch,             // признак выборки инструкции
   output reg id,                 // 1 - обращение к данным

   // Векторные прерывания
   input [7:4]       virq,       // линии запросов прерываний
   input [8:0]       vector,     // ввод адреса вектора
   output reg [7:4]  vstb,       // строб запроса вектора 
   input             vack,       // подтверждение приема вектора
   input[15:0]       pirq,       // Регистр PIRQ, формирователь программных прерываний
    
    // линии прерываний от MMU
   input mmutrap,                // запрос прерывания от MMU по окончании обработки текущей инструкции
   output reg ack_mmutrap,       // подтверждение прерывания от MMU
   input mmuabort,               // запрос от MMU на отмену обработки текущей инструкции
   output reg ack_mmuabort,      // подтверждение отмены обработки инструкции

   // Индикаторы состояния процессора 
   output reg iwait,             // 1 - процессор стоит на команде wait и ждет прерывания
   output reg run,               // индикатор работы процессора
   
   // Сигналы ручного управления
   input  sw_cont,               // кнопка выхода из состояния HALT
   input cpuslow,                // Включение замедленного режима процессора
   
   // DMA
   input npr,                    // запрос DMA
   output reg npg,               // подтверждение DMA
   output oddabort,              // признак словного обращения по нечетному адресу
   
   // Флаги ошибок
   output reg illhalt,           // признак недопустимости инструкции HALT в текущем режиме процессора
   output     bus_timeout,       // флаг таймаута шины
   output ysv,                   // желтое состояние стека
   output rsv,                   // красное состояние стека
   input[15:0] cpu_stack_limit,  // значение нижней границы стека 
   output reg sr0_ic,            // sr0/mmr0 - флаг завершения декодирования инструкции
   output[15:0] sr1,             // sr1/mmr1 - адрес текущей инструкции
   output reg[15:0] sr2,         // sr2/MMR2 - информация об автоинкременте и автодекременте
   output reg dstfreference,     // 1 - финальная адресация приемника
   output dw8,                   // признак байтового обмена 
   
   // PSW
   input[15:0] psw_in,           // Ввод PSW , записываемого под адресу 177776
   input psw_in_we_even,         // Разрешение записи младшего байта PSW
   input psw_in_we_odd,          // Разрешение записи старшего байта PSW
   output[15:0] psw_out,         // Вывод PSW , читаемого под адресу 177776

   // конфигурация 
   input fpu_enable,             // Признак наличия FPU в схеме процессора
   input[15:0] startup_adr,      // Адрес старта по сбросу
   input[15:0] startup_psw       // Стартовое PSW
);
//*********************************************************
//*   Состояния секвенсора
//*********************************************************
parameter[7:0] sq_init = 0 ;
parameter[7:0] sq_ifetch = 1 ;
parameter[7:0] sq_idecode = 2 ;
parameter[7:0] sq_src0 = 3 ;
parameter[7:0] sq_src1 = 4 ;
parameter[7:0] sq_src1l = 5 ;
parameter[7:0] sq_src2 = 6 ;
parameter[7:0] sq_src2l = 7 ;
parameter[7:0] sq_src2w = 8 ;
parameter[7:0] sq_src3 = 9 ;
parameter[7:0] sq_src3l = 10 ;
parameter[7:0] sq_src3a = 11 ;
parameter[7:0] sq_src3al = 12 ;
parameter[7:0] sq_src4 = 13 ;
parameter[7:0] sq_src4l = 14 ;
parameter[7:0] sq_src4w = 15 ;
parameter[7:0] sq_src5 = 16 ;
parameter[7:0] sq_src5l = 17 ;
parameter[7:0] sq_src5a = 18 ;
parameter[7:0] sq_src5al = 19 ;
parameter[7:0] sq_src6 = 20 ;
parameter[7:0] sq_src6l = 21 ;
parameter[7:0] sq_src6a = 22 ;
parameter[7:0] sq_src6al = 23 ;
parameter[7:0] sq_src7 = 24 ;
parameter[7:0] sq_src7l = 25 ;
parameter[7:0] sq_src7a = 26 ;
parameter[7:0] sq_src7al = 27 ;
parameter[7:0] sq_src7b = 28 ;
parameter[7:0] sq_src7bl = 29 ;
parameter[7:0] sq_dst0 = 30 ;
parameter[7:0] sq_dst1 = 31 ;
parameter[7:0] sq_dst1l = 32 ;
parameter[7:0] sq_dst2 = 33 ;
parameter[7:0] sq_dst2l = 34 ;
parameter[7:0] sq_dst3 = 35 ;
parameter[7:0] sq_dst3l = 36 ;
parameter[7:0] sq_dst3a = 37 ;
parameter[7:0] sq_dst3al = 38 ;
parameter[7:0] sq_dst4 = 39 ;
parameter[7:0] sq_dst4l = 40 ;
parameter[7:0] sq_dst5 = 41 ;
parameter[7:0] sq_dst5l = 42 ;
parameter[7:0] sq_dst5a = 43 ;
parameter[7:0] sq_dst5al = 44 ;
parameter[7:0] sq_dst6 = 45 ;
parameter[7:0] sq_dst6l = 46 ;
parameter[7:0] sq_dst6a = 47 ;
parameter[7:0] sq_dst6al = 48 ;
parameter[7:0] sq_dst7 = 49 ;
parameter[7:0] sq_dst7l = 50 ;
parameter[7:0] sq_dst7a = 51 ;
parameter[7:0] sq_dst7al = 52 ;
parameter[7:0] sq_dst7b = 53 ;
parameter[7:0] sq_dst7bl = 54 ;
parameter[7:0] sq_sob = 55 ;
parameter[7:0] sq_jmp = 56 ;
parameter[7:0] sq_jsr = 57 ;
parameter[7:0] sq_jsra = 58 ;
parameter[7:0] sq_jsrb = 59 ;
parameter[7:0] sq_jsrc = 60 ;
parameter[7:0] sq_rts = 61 ;
parameter[7:0] sq_rtsa = 62 ;
parameter[7:0] sq_rtsal = 63 ;
parameter[7:0] sq_mark = 64 ;
parameter[7:0] sq_marka = 65 ;
parameter[7:0] sq_markb = 66 ;
parameter[7:0] sq_markbl = 67 ;
parameter[7:0] sq_mfp = 68 ;
parameter[7:0] sq_mfpa = 69 ;
parameter[7:0] sq_mtp = 70 ;
parameter[7:0] sq_mtpl = 71 ;
parameter[7:0] sq_mtpa = 72 ;
parameter[7:0] sq_dopr = 73 ;
parameter[7:0] sq_dopra = 74 ;
parameter[7:0] sq_doprb = 75 ;
parameter[7:0] sq_mul = 76 ;
parameter[7:0] sq_mula = 77 ;
parameter[7:0] sq_mulb = 78 ;
parameter[7:0] sq_div = 79 ;
parameter[7:0] sq_diva = 80 ;
parameter[7:0] sq_divb = 81 ;
parameter[7:0] sq_ash = 82 ;
parameter[7:0] sq_ashb = 83 ;
parameter[7:0] sq_ashc = 84 ;
parameter[7:0] sq_ashd = 85 ;
parameter[7:0] sq_ashe = 86 ;
parameter[7:0] sq_xor = 87 ;
parameter[7:0] sq_ldfps = 88 ;
parameter[7:0] sq_halt = 89 ;
parameter[7:0] sq_fptrap = 90 ;
parameter[7:0] sq_fpao = 91 ;
parameter[7:0] sq_fpso2 = 92 ;
parameter[7:0] sq_fpwr = 93 ;
parameter[7:0] sq_fpwr1 = 94 ;
parameter[7:0] sq_fpwr2 = 95 ;
parameter[7:0] sq_fpd0 = 96 ;
parameter[7:0] sq_fpir1 = 97 ;
parameter[7:0] sq_fpir2 = 98 ;
parameter[7:0] sq_fpiwr = 99 ;
parameter[7:0] sq_fpiww = 100 ;
parameter[7:0] sq_fpiw1 = 101 ;
parameter[7:0] sq_fpiw2 = 102 ;
parameter[7:0] sq_fpr1 = 103 ;
parameter[7:0] sq_fpr2 = 104 ;
parameter[7:0] sq_fpr3 = 105 ;
parameter[7:0] sq_fpr4 = 106 ;
parameter[7:0] sq_fpww = 107 ;
parameter[7:0] sq_fpw1 = 108 ;
parameter[7:0] sq_fpw2 = 109 ;
parameter[7:0] sq_fpw3 = 110 ;
parameter[7:0] sq_fpw4 = 111 ;
parameter[7:0] sq_fprun = 112 ;
parameter[7:0] sq_fprunao = 113 ;
parameter[7:0] sq_tstset = 114 ;
parameter[7:0] sq_wrtlck = 115 ;
parameter[7:0] sq_wrtlcka = 116 ;
parameter[7:0] sq_rsv = 117 ;
parameter[7:0] sq_trap = 118 ;
parameter[7:0] sq_trapa = 119 ;
parameter[7:0] sq_trapal = 120 ;
parameter[7:0] sq_trapb = 121 ;
parameter[7:0] sq_trapc = 122 ;
parameter[7:0] sq_trapw = 123 ;
parameter[7:0] sq_trapd = 124 ;
parameter[7:0] sq_trape = 125 ;
parameter[7:0] sq_trapf = 126 ;
parameter[7:0] sq_trapfl = 127 ;
parameter[7:0] sq_rti = 128 ;
parameter[7:0] sq_rtia = 129 ;
parameter[7:0] sq_rtial = 130 ;
parameter[7:0] sq_rtib = 131 ;
parameter[7:0] sq_rtibl = 132 ;
parameter[7:0] sq_illegalop = 133 ;
parameter[7:0] sq_mmuabort = 134 ;
parameter[7:0] sq_mmutrap = 135 ;
parameter[7:0] sq_stststore = 136 ;
parameter[7:0] sq_edf = 137 ;
parameter[7:0] sq_virq7 = 138 ;
parameter[7:0] sq_virq6 = 139 ;
parameter[7:0] sq_virq5 = 140 ;
parameter[7:0] sq_virq4 = 141 ;
parameter[7:0] sq_store_alu_p = 142 ;
parameter[7:0] sq_store_alu_w = 143 ;
parameter[7:0] sq_store_alu_r = 144 ;
parameter[7:0] sq_dma = 145 ;
parameter[7:0] sq_wbread = 146 ;
parameter[7:0] sq_wbread_wait = 147 ;
parameter[7:0] sq_wbwrite = 148 ;
parameter[7:0] sq_wbwrite_wait = 149 ;
parameter[7:0] sq_nxmabort = 150 ;
parameter[7:0] sq_fpir1l = 151;
parameter[7:0] sq_fpir2l = 152;
parameter[7:0] sq_fpr1l = 153;
parameter[7:0] sq_fpr2l = 154;
parameter[7:0] sq_fpr3l = 155;
parameter[7:0] sq_fpr4l = 156;
parameter[7:0] sq_fpiw1l = 157;
parameter[7:0] sq_fpw1l = 158;
parameter[7:0] sq_fpw2l = 159;
parameter[7:0] sq_fpw3l = 160;
parameter[7:0] sq_rsv1 = 161;

reg [15:0] datain;

// Регистры  состояния секвенсера   
reg[7:0] state;      // текущее состояние
reg[7:0] nextstate;  // состояние для возврата из субблока   
reg[7:0] dst_mode_processor; // обработчик режима адресации приемника
reg[7:0] src_mode_processor; // обработчик режима адресации источника


reg[15:0] ir;       // хранит очередную инструкцию, выбранную декодером из памяти
reg[15:0] ir_addr;  // адрес выбранной декодером инструкции
   
reg trap_flag;   
   
// Флажки характеристик декодируемой инструкции   

reg ir_sop;      // однооперандная инструкция
reg ir_dop;      // двухоперандная инструкция
reg ir_jmp;      // инструкция JMP
reg ir_jsr;      // инструкция JSR
reg ir_mfpi;     // инструкция MFPI
reg ir_mfpd;     // инструкция MFPD 
wire ir_mf;      // MFPI/MFPD
reg ir_mtpi;     // инструкция MTPI 
reg ir_mtpd;     // инструкция MTPD 
wire ir_mt;      // MTPI/MTPD
reg ir_rtt;      // инструкция RTT
reg ir_dopr;     // инструкция регистр-регистр
reg ir_fpsop1;   // однооперандная инструкция FPU типа 1
reg ir_fpsop2;   // однооперандная инструкция FPU типа 2 
reg ir_fpao;     // инструкция FPU  аккумулятор-операнд
reg ir_facdst;   // инструкция FPU  аккумулятор-операнд с передачией в CPU
reg ir_facsrc;   // инструкция FPU  аккумулятор-операнд с передачией из CPU 
reg ir_facfdst;  // инструкция FPU  аккумулятор-операнд с передачией в FPU
reg ir_facfsrc;  // инструкция FPU  аккумулятор-операнд с передачией из FPU 
wire ir_fpma48; 
reg ir_fpmai;    // инструкция FPU, формат данных- integer
reg ir_fpmaf;    // инструкция FPU, формат данных- float
reg ir_byte;     // признак байтового результата АЛУ
reg ir_store;    // Разрешение от АЛУ на запись результата
wire ir_srcr7;   // Истоник - регистр РС
wire ir_dstr7;   // Приемник - регистр РС
wire ir_dstm2r7; // Приемник или единстрвенный операнд - (R7)+
reg v_sop; 
reg v_dop; 
reg v_jmp; 
reg v_jsr; 
reg v_mfpi; 
reg v_mfpd; 
reg v_mtpi; 
reg v_mtpd; 
reg v_dopr; 
reg v_fpsop1; 
reg v_fpsop2; 
reg v_fpao; 

// хранилище вектора текущего прерывания
reg[8:0] trap_vector; 

// psw
reg[15:0] temp_psw; 
reg[15:0] psw; 
wire[15:8] pswmf; 
reg[15:0] psw_delayedupdate; 
reg psw_delayedupdate_even; 
reg psw_delayedupdate_odd; 
reg spl_psw_update;
// pc
reg[15:0] r7; 

// Интерфейс к АЛУ
reg[15:0] alu_input;             // вход АЛУ для однооперандных инструкций или второй операнд двухоперандных инструкций
reg[15:0] alus_input;            // вход АЛУ для первого операнда двухоперандных инструкций
reg[15:0] alut_input;            // вход EIS АЛУ для второго операнда
reg[15:0] alu_output;            // выход АЛУ - результат обработки
wire[15:0] alu_output_signext;   // байтовый выход АЛУ с расширенным на старший байт знаком
reg[3:0] alu_psw;                // обновленное PSW после обработки операндов в АЛУ

// Шины регистрового файла
reg[5:0] rbus_waddr;           // адрес записи в регистровом файле
reg[15:0] rbus_d;              // входная шина регистрового файла   
wire[15:0] rbus_o;             // выходная шина регистрового файла
reg rbus_we;                   // разрешение записи в регистровый файл
reg[2:0] rbus_ix;              // регистр-источник текущей инструкции
reg[1:0] rbus_cpu_mode;        // текущий режим работы процессора
reg rbus_cpu_psw11;            // выбор набора регистров
wire[15:0] rbus_data;          // содержимое выбранного регистра
wire[15:0] rbus_data_m2; 
wire[15:0] rbus_data_mv;       // Содержимое регистра после автодекремента --
wire[15:0] rbus_data_pv;       // Содержимое регистра после автоинкремента ++
reg rsv_flag;
reg [15:0] prev_sp;

// SR1/MMR1
wire[7:0] sr1_dst;    // описатель приемника
wire[7:0] sr1_src;    // описатель источника
reg[4:0] sr1_dstd;    // размер автоинкремента приемника       
reg[4:0] sr1_srcd;    // размер автоинкремента источника
wire[4:0] sr1_p2;     // инкремент +2
wire[4:0] sr1_pv;     // инкремент, специфичный для инструкции
wire[4:0] sr1_m2;     // декремент -2
wire[4:0] sr1_mv;     // декремент, специфичный для инструкции

// Флаги текущего/предыдущего режима
wire cp_req; 
wire cp_mf; 
wire cp_mt; 
reg cp_latch;

// Признаки режима инструкций/данных
wire id_current;
reg id_latch;

// Элементы формирователя игнала чтения rd
wire rd_select;    // Сигнал RD, определяемый состоянием секвенсера
wire rs_mt;        // сигналы подавления чтения
wire rs_jj; 

// Вычисляемые адреса операндов
reg[15:0] dest_addr;        // вычисленный адрес записи результата
reg[15:0] addr_indirect;    // вычисленный адрес при косвенной адресации
reg finalreference; 

// Регистры АЛУ EIS-инструкций
reg[15:0] eis_output; 
reg[15:0] eis_output32; 
reg[15:0] eis_temp; 
reg[31:0] eis_temp1; 
reg[31:0] eis_temp2; 
reg[4:0] eis_sequencer; 
reg[3:0] eis_psw; 
reg eis_flag1; 
reg eis_flag2; 
integer initcycles; 

// Сигналы контроля уровня стека
reg yellow_stack_event; 
reg yellow_stack_latch; 
wire yellow_stack_event_trigger; 
wire yellow_stack_immediate_states; 
reg yellow_trap_inhibit; 
reg red_stack_trap; 
wire red_stack_trap_trigger; 
wire red_stack_immediate_states; 
wire stack_check_states; 

reg trap_force_kernel;

// Регистры АЛУ FPP
reg[15:0] fps; 
reg[3:0] fec; 
reg[15:0] fea; 

reg[63:0] falu_input; 
reg[63:0] falus_input; 
reg[63:0] falu_output; 
reg[63:0] falu_output2; 
reg[3:0] falu_fps; 
reg falu_load; 
reg falu_done; 
reg falu_flag1; 
reg[3:0] falu_fsm; 
reg[9:0] falu_ccw; 
reg[7:0] falu_state; 
reg[58:0] falu_work1; 
reg[58:0] falu_work2; 
reg falu_pending_clear; 
reg falu_pending_fic; 
reg falu_pending_fiu; 
reg falu_pending_fiv; 
reg falu_pending_divz; 

// Состоояния АЛУ плавающей точки
parameter[3:0] falu_idle = 0; 
parameter[3:0] falu_align = 1; 
parameter[3:0] falu_mult = 2; 
parameter[3:0] falu_div = 3; 
parameter[3:0] falu_addsub = 4; 
parameter[3:0] falu_shift = 5; 
parameter[3:0] falu_shift2 = 6; 
parameter[3:0] falu_shifte = 7; 
parameter[3:0] falu_norm = 8; 
parameter[3:0] falu_rt = 9; 
parameter[3:0] falu_rtc = 10; 
parameter[3:0] falu_sep = 11; 
parameter[3:0] falu_sep2 = 12; 
parameter[3:0] falu_sep3 = 13; 
parameter[3:0] falu_zres = 14; 
parameter[3:0] falu_res = 15; 

// Регистры блока FPU
reg[2:0] fbus_raddr; 
reg[2:0] fbus_waddr; 
reg[63:0] fbus_d; 
wire[63:0] fbus_o; 
reg fbus_we; 
reg fbus_fd; 

// флаги таймаута шины
reg nxmabort;
reg nxmabort_pend;
reg [5:0] bus_timer;  // таймер шинных транзакций

assign bus_timeout=nxmabort;

// Таймер замедления процессора
reg [6:0] cpudelay;
   
//********************************************************
//*   Блок регистров процессора , шина RBUS
//********************************************************
   cpuregs cpuregs0 (
      //       режим cpu      набор регистров   #регистра
      .raddr({rbus_cpu_mode, rbus_cpu_psw11,    rbus_ix}),  // адрес чтения 
      .waddr(rbus_waddr),  // адрес записи
      .d(rbus_d),          // данные для записи
      .o(rbus_o),          // выходные данные чтения
      .we(rbus_we),        // разрешение записи
      .clk(clk)            // тактовый сигнал
   ); 
   
//********************************************************
//*   Блок регистров FPU, шина FBUS
//********************************************************
   fpuregs fpuregs0 (
      .raddr(fbus_raddr),  // адрес чтения 
      .waddr(fbus_waddr),  // адрес записи
      .d(fbus_d),          // входная шина данных
      .o(fbus_o),          // выходная шина данных
      .fpmode(fbus_fd), 
      .we(fbus_we), 
      .clk(clk)
   );
   
//********************************************************
//*    Подсистема контроля границы стека
//********************************************************
   
//  Состояния, в которых желтая граница стека возникает по адресам 0-376
assign yellow_stack_immediate_states = (state == sq_dst4)     // -(R)
                                     | (state == sq_jsra)     // Запись в стек при вызове подпрограммы
                                     | (state == sq_trapc)    // Запись PSW при прерывании
                                     | (state == sq_trapd);   // Запись PC при прерывании

//  Состояния, в которых производится проверка желтой и красной границы через регистр SL
assign stack_check_states =             (state == sq_dst1)     // (R)
                                      | (state == sq_dst2)     // (R)+
                                      | (state == sq_dst4)     // -(R)
                                      | (state == sq_jsra)     //  JSR - запись SP в стек
                                      | (state == sq_trapc)    // прерывание, запись PSW в стек
                                      | (state == sq_trapd) ;  // прерывание, запись PC в стек

// Детектор желтой границы стека                                                                  
assign yellow_stack_event_trigger = 
            (rbus_ix == 3'b110)                    // адресация через SP 
            & (psw[15:14] == 2'b00)                // режим KERNEL
            & (yellow_trap_inhibit == 1'b0)        // предотвращение повторного прерывания
            & (red_stack_trap == 1'b0)             // красная граница не достигнута
            & (red_stack_trap_trigger == 1'b0) 
            &(
            // Стек попадает в область 0-377
             (yellow_stack_immediate_states & (rbus_data_m2[15:8] == 8'b00000000))    
            // Стек опустился ниже верхнего значения границы
          |  (stack_check_states & ({cpu_stack_limit[15:8], 8'b11111110} > rbus_data_m2)));
            
assign ysv = yellow_stack_event_trigger ;

// Состояния, в которых детектор срабатываеит при ошибках нечетной адресации или таймауте шины
// В этих состояниях регистр SL не учитывается и прерывание происходит всегда
assign red_stack_immediate_states = (state == sq_trapc)    // прерывание, запись PSW в стек
                                  | (state == sq_trapd);   // прерывание, запись PС в стек
                                              
assign red_stack_trap_trigger = 
                                // Ошибки записи в стек в момент входа в прерывание 
                                (red_stack_immediate_states & (oddabort | nxmabort)) |
                                // Уход стека ниже нижней границы SL
                                (stack_check_states   
                                & rbus_ix == 3'b110            // SP
                                & psw[15:14] == 2'b00          // KERNEL
                                & red_stack_trap == 1'b0       // прерывание еще не произошло
                                &  (oddabort | nxmabort |              // ошибки таймаута шины или нечетной адресации
                                   ({cpu_stack_limit[15:8], 8'b11100000} > (rbus_data_m2)) |   // стек уехал ниже нижней границы, определяемой SL
                                   (rbus_data[15:1] == 15'o77777)));   // стек уехал на страницу ввода-вывода

assign rsv = red_stack_trap_trigger ;

//-------------------------------------------------------------
// dw8 : признак байтового доступа к шине
assign dw8 = finalreference & ir_byte;
                     
// флаг CP - адрес относится к предыдущему режиму
  // mfpi/mfpd - чтение
assign cp_mf = finalreference & (state != sq_store_alu_w) & (ir_mfpi | ir_mfpd) ;
  // mtpi/mtpd - запись
assign cp_mt = (state == sq_store_alu_w) & (ir_mtpi | ir_mtpd );
assign cp_req = cp_mf | cp_mt ;

// rs - подавление строба чтения в некоторых состояниях:
assign rs_mt = ((ir_mtpi == 1'b1 | ir_mtpd == 1'b1) 
               & finalreference == 1'b1 
               & state != sq_store_alu_w)    // не в состоянии store_alu_w
               ? 1'b1 : 1'b0 ;
assign rs_jj = ((ir_jmp == 1'b1 | ir_jsr == 1'b1) // подавление в командах переходов
               & finalreference == 1'b1) 
               ? 1'b1 : 1'b0 ;
               
wire rd_inhibit = (rs_mt | rs_jj // подавление RD сигналами rs
                   | iwait);     // также в состоянии wait чтение не производится

// Формирования флагов - индикаторов использования R7 в качестве операнда
assign ir_dstm2r7 = (ir[5:0] == 6'b010111) ? 1'b0 : 1'b1 ;  // (R7)+
assign ir_srcr7 = (ir[8:6] == 3'b111) ? 1'b0 : 1'b1 ;       // R7 - источник
assign ir_dstr7 = (ir[2:0] == 3'b111) ? 1'b0 : 1'b1 ;       // R7 - приемник

// Формирование прзнака I/D текущей инструкции
assign id_current = (state == sq_idecode) ? 1'b0 : 
                   (state == sq_ifetch) ? 1'b0 : 
                   (state == sq_dst1) ? ir_dstr7 : 
                   (state == sq_dst2) ? ir_dstr7 : 
                   (state == sq_dst3) ? ir_dstr7 : 
                   (state == sq_dst3a) ? 1'b1 : 
                   (state == sq_dst4) ? ir_dstr7 : 
                   (state == sq_dst5) ? ir_dstr7 : 
                   (state == sq_dst5a) ? 1'b1 : 
                   (state == sq_dst6) ? 1'b0 : 
                   (state == sq_dst6a) ? 1'b1 : 
                   (state == sq_dst7) ? 1'b0 : 
                   (state == sq_dst7a) ? 1'b1 : 
                   (state == sq_dst7b) ? 1'b1 : 
                   (state == sq_src1) ? ir_srcr7 : 
                   (state == sq_src2) ? ir_srcr7 : 
                   (state == sq_src3) ? ir_srcr7 : 
                   (state == sq_src3a) ? 1'b1 : 
                   (state == sq_src4) ? ir_srcr7 : 
                   (state == sq_src5) ? ir_srcr7 : 
                   (state == sq_src5a) ? 1'b1 : 
                   (state == sq_src6) ? 1'b0 : 
                   (state == sq_src6a) ? 1'b1 : 
                   (state == sq_src7) ? 1'b0 : 
                   (state == sq_src7a) ? 1'b1 : 
                   (state == sq_src7b) ? 1'b1 : 
                   (state == sq_fpir1) ? ir_dstm2r7 : 
                   (state == sq_fpir2) ? 1'b1 : 
                   (state == sq_fpiw1) ? ir_dstm2r7 : 
                   (state == sq_fpiw2) ? 1'b1 : 
                   (state == sq_fpr1) ? ir_dstm2r7 : 
                   (state == sq_fpr2 || state == sq_fpr3 || state == sq_fpr4) ? 1'b1 : 
                   (state == sq_fpw1) ? ir_dstm2r7 : 
                   (state == sq_fpw2 || state == sq_fpw3 || state == sq_fpw4) ? 1'b1 : 
                   (state == sq_stststore) ? 1'b1 : 
                   (state == sq_mfpa) ? 1'b1 : 
                   (state == sq_mtp) ? 1'b1 : 
                   (state == sq_trapa) ? 1'b1 : 
                   (state == sq_trapb) ? 1'b1 : 
                   (state == sq_trapc) ? 1'b1 : 
                   (state == sq_trapw) ? 1'b1 : 
                   (state == sq_trapd) ? 1'b1 : 
                   (state == sq_trapf) ? 1'b1 : 
                   (state == sq_jsrb) ? 1'b1 : 
                   (state == sq_rtsa) ? 1'b1 : 
                   (state == sq_mark) ? 1'b0 : 
                   (state == sq_marka) ? 1'b0 : 
                   (state == sq_rtia) ? 1'b1 : 
                   (state == sq_rtib) ? 1'b1 : 
                   (state == sq_store_alu_w) ? ir_dstm2r7 : 
                                            1'b0 ; 

// выходное PSW - передается в блок MMU
assign psw_out = {rbus_cpu_mode, pswmf[13:8], psw[7:0]} ;
assign pswmf[15:11] = psw[15:11];
assign pswmf[10:8] = 3'b000 ;

// Выходная шина данных RBUS - интерфейс к регистровому файлу
assign rbus_data = (rbus_ix == 3'b111) ? r7 : rbus_o ; // R7 хранится в отдельном регистре, остальное - в регистровом файле

// Вычисление значений автоинкремента/автодекремента
assign ir_fpma48 = (((fps[7]) == 1'b1 & ir[11:8] != 4'b1111) | ((fps[7]) == 1'b0 & ir[11:8] == 4'b1111)) ? 1'b1 : 1'b0 ;
assign rbus_data_m2 = rbus_data - 16'd2 ;

// Содержимое регистра после автодекремента - с учетом разрядности обрабатываемых операндов
assign rbus_data_mv = (ir_fpmaf == 1'b1 & ir_fpma48 == 1'b1 & rbus_ix != 3'b111) ? rbus_data-16'd8 : 
                      (ir_fpmaf == 1'b1 & ir_fpma48 == 1'b0 & rbus_ix != 3'b111) ? rbus_data-16'd4 : 
                      (ir_fpmai == 1'b1 & (fps[6]) == 1'b1 & rbus_ix != 3'b111) ? rbus_data-16'd4 : 
                      (ir_byte == 1'b1 & rbus_ix[2:1] != 2'b11) ? rbus_data-16'd1 : 
                                                                            rbus_data-16'd2 ;

// Содержимое регистра после автоинкремента - с учетом разрядности обрабатываемых операндов
assign rbus_data_pv = (ir_fpmaf == 1'b1 & ir_fpma48 == 1'b1 & rbus_ix != 3'b111) ? rbus_data+16'd8 : 
                      (ir_fpmaf == 1'b1 & ir_fpma48 == 1'b0 & rbus_ix != 3'b111) ? rbus_data+16'd4 : 
                      (ir_fpmai == 1'b1 & (fps[6]) == 1'b1 & rbus_ix != 3'b111) ? rbus_data+16'd4 : 
                      (ir_byte == 1'b1 & rbus_ix[2:1] != 2'b11) ? rbus_data+16'd1 :       // байтовая адресация - кроме SP и РС
                                                                         rbus_data+16'd2 ;  // все остальные случаи - словная адресация
                                                                         
// Сборка регистра SR1/MMR1

assign ir_mf = ir_mfpi | ir_mfpd;    // инструкции чтения из пространства предыдущего режима
assign ir_mt = ir_mtpi | ir_mtpd;    // инструкции записи в пространство предыдущего режима

assign sr1_dst = ((sr1_dstd != 5'b00000)) ? {sr1_dstd, ir[2:0]} : 8'b00000000 ;

assign sr1_src = ((ir_mt | ir_mf | ir_jsr) & (sr1_srcd != 5'b00000)) ? {sr1_srcd, 3'b110} : 
                 ((sr1_srcd != 5'b00000) & & ir_dop) ? {sr1_srcd, ir[8:6]} : 
                                                                            8'b00000000 ;
                                                                                    
assign sr1 = ((ir_dop | ir_jsr) & sr1_srcd != 5'b00000) ? {sr1_dst, sr1_src} : 
             (sr1_dstd == 5'b00000) ? {sr1_dst, sr1_src} : 
                                      {sr1_src, sr1_dst} ;
                                     
assign sr1_p2 = 5'o2;   // добавка автоинкремента на 2
// вычисление значения автоинкремента в зависимости от формата инструкции   
assign sr1_pv = (ir_fpmaf & ir_fpma48 & (rbus_ix != 3'b111)) ? 5'd8 :  
                (ir_fpmaf & ir_fpma48 & (rbus_ix != 3'b111)) ? 5'd4 : 
                (ir_fpmai & (fps[6] == 1'b1) & (rbus_ix != 3'b111)) ? 5'd4 : 
                (ir_byte  & (rbus_ix[2:1] != 2'b11)) ? 5'o1 : 
                                                       5'o2 ;
                                                              
assign sr1_m2 = 5'b11110 ; // добавка автодекремента на 2

// вычисление значения автодекремента в зависимости от формата инструкции   
assign sr1_mv = (ir_fpmaf == 1'b1 & ir_fpma48 == 1'b1 & rbus_ix != 3'b111) ? 5'b11000 :   // -8
                (ir_fpmaf == 1'b1 & ir_fpma48 == 1'b0 & rbus_ix != 3'b111) ? 5'b11100 :   // -4
                (ir_fpmai == 1'b1 & (fps[6]) == 1'b1 & rbus_ix != 3'b111) ? 5'b11100 :    // -4
                (ir_byte == 1'b1 & rbus_ix[2:1] != 2'b11) ?                 5'b11111 :    // -1
                                                                               5'b11110 ;    // -2
// Признак нечетного адреса при словном обмене                    
assign oddabort = ~dw8 & wbm_adr_o[0] & wbm_stb_o; 
                                                                             
//*******************************************************                                                                           
//*  Потактовая обработка состояний процессора
//*******************************************************                                                                             
   always @(posedge clk)   begin
      if (reset == 1'b1) begin
      //*********************************************
      //*   Сброс системы
      //*********************************************
         r7 <= startup_adr ;                    // Засылаем стартовый адрес в R7
         psw <= startup_psw ;                  // Начальное PSW
         rbus_cpu_mode <= startup_psw[15:14] ; // Начальный режим работы процессора - psw[15:14]
         rbus_cpu_psw11 <= startup_psw[11] ;   // Начальный набор регистров - psw[11]
         ir_rtt <= 1'b0 ; 
         wbm_stb_o <= 1'b0;
         iwait <= 1'b0 ;  // снимаем флаг ожидания  подтверждения прерывания
         finalreference <= 1'b0;
         // Снятие сигналов подтверждения прерывания
         vstb[7] <= 1'b0 ; 
         vstb[6] <= 1'b0 ; 
         vstb[5] <= 1'b0 ; 
         vstb[4] <= 1'b0 ; 
         npg <= 1'b0 ; 
         // Очистка флагов контроля стековых границ
         red_stack_trap <= 1'b0 ; 
         yellow_stack_event <= 1'b0 ; 
         yellow_stack_latch <= 1'b0 ; 
         // Регистры FP-11
         fps <= 16'h0000;   // регистр состояния
         fea <= 16'h0000;   // регистр адреса ошибки
         fec <= 4'b0000 ;   // регистр кода ошибки
         falu_load <= 1'b0 ; 
         psw_delayedupdate_even <= 1'b0 ;
         psw_delayedupdate_odd <= 1'b0 ; 
         spl_psw_update <= 1'b0;
         cpudelay <= 5'd`CPUSLOW;  // счетчик замедления процессора         
         state <= sq_init ; // Начальное состояние секвенсера
         initcycles <= 7 ;     // Число тактов задержки перед снятием сброса - ширина выходного импульса сброса шины
         init <= 1'b1 ;        // выставляем сигнал сброса шины
         rbus_waddr <= 6'b000000 ; // Начальный адрес в регистровом файле  - R0 набора 0
         rbus_d <= 16'd70;     // тип процессора - 11/70
         rbus_we <= 1'b1 ;     // Записываем тип процессора в регистр 0 набора 0
         dst_mode_processor <= sq_store_alu_r;
         src_mode_processor <= sq_store_alu_r;
         run <= 1'b1;           // признак запуска секвенсара команд
         nxmabort <= 1'b0;
         nxmabort_pend <= 1'b0;
         dstfreference <= 1'b0;
         trap_flag <= 1'b0;
         id_latch <= 1'b0;
         id <= 1'b0;
         cp <= 1'b0;
         cp_latch <= 1'b0;
         rsv_flag <= 1'b0;
         trap_force_kernel <= 1'b0;
      end
      
      else begin
      //***********************************************************
      //* Начальные состояния сигналов  в каждом такте секвенсора
      //*********************************************************** 
         id_latch <= id_current;
         cp_latch <= cp_req;
         rbus_we <= 1'b0 ;        // Снимаем флаг записи в основной регистровый файл
         fbus_we <= 1'b0 ;        // Снимаем флаг записи в регистровый файл FP-11
         ifetch <= 1'b0 ;         // Снимаем флаг выбора инструкции
         // Снятие флагов подтверждения прерывания от MMU
         ack_mmuabort <= 1'b0 ;   
         ack_mmutrap <= 1'b0 ; 
         // Снятие флага ошибки исполнения HALT в режимах, отлиных от KERNEL
         illhalt <= 1'b0 ; 
         falu_pending_clear <= 1'b0 ; 

         // Обработка триггеров запроса прерывания от схем контроля границ стека
         if (yellow_stack_event_trigger == 1'b1)    yellow_stack_event <= 1'b1 ; // Желтая граница
         if (red_stack_trap_trigger & ~trap_force_kernel)  begin
            // Красная граница
            red_stack_trap <= 1'b1 ; // установка флага прерывания
            state <= sq_rsv ;     // переключаем секвенсер  на обработку этого прерывания
         end
         
         // Обработка запросов прерывания по ошибке от MMU, с отменой обработки текущей инструкции
         else if (mmuabort == 1'b1)  begin
            wbm_stb_o <= 1'b0;    // снимаем строб шины
            wbm_we_o <= 1'b0;     // снимаем признак записи
            state <= sq_mmuabort ;  // переходим к обработке прерывания
            ack_mmuabort <= 1'b1 ;     // устанавливаем признак обработки прерывания
         end
         
         else if (nxmabort == 1'b1)  begin
            trap_vector <= 9'o004 ;    // Выставляем вектор 4
            state <= sq_trap ;      // переходим к обычной обработке прерывания
            nxmabort <= 1'b0;
            nxmabort_pend <= 1'b0;
         end
         
      //********************************************************************************
      //*  Рабочие состояния секвенсера - если не было условий внутренних прерываний
      //********************************************************************************
         else   begin
            case (state)
               //*********************************************************         
               //  Состояние после сброса
               //  Любая выполняемая инструкция сразу прерывается на любом этапе выполнения
               //*********************************************************         
               sq_init :
                        begin
                           // Таймаут сброса закончен
                           if (initcycles == 0)  begin
                              state <= sq_ifetch ;  // переходим к выборке первой инструкции 
                              init <= 1'b0 ;           // снимаем сигнал сброса с шины
                           end
                           // таймер сигнала сброса шины
                           else begin
                              init <= 1'b1 ;           // выставляем сигнал сброса шины
                              initcycles <= initcycles-1'b1 ;  // выдержка перед снятием сброса
                           end 
                        end
               //*********************************************************         
               // Подготовка к выборке очередной инструкции из памяти
               //*********************************************************         
               sq_ifetch :
                         // Пропуск тактов в режиме замедления процессора
                         if (cpuslow & (|cpudelay != 1'b0)) cpudelay <= cpudelay-1'b1;
                         else begin
                           cpudelay <= 7'd`CPUSLOW ;  
                           rbus_we <= 1'b0;
                           rbus_cpu_mode <= pswmf[15:14] ; // Установка текущего режима работы процессора из PSW
                           rbus_cpu_psw11 <= pswmf[11] ;   // Выбор текущего набора регистров по биту 11 PSW
                           red_stack_trap <= 1'b0 ;        // снимаем флаг красной границы стека
                           yellow_trap_inhibit <= 1'b0 ;  //  снимаем флаг подавления подвторного определения желтой границы
                           yellow_stack_event <= 1'b0;    // снимаем флаг события желтой границы - при выборке новой инструкции становится неактуальным
                           dstfreference <= 1'b0;         // снимаем флаг доступа к операнду-приемнику
                           id <= 1'b0;                    // ID=0 - идет выболрка инструкции
                           
                           // Обновление PSW
                           if (spl_psw_update) begin
                                 psw[7:5] <= psw_delayedupdate[7:5]; 
                                 spl_psw_update <= 1'b0;
                           end      
                           
                           if (psw_delayedupdate_even | psw_delayedupdate_odd) begin
                                 // Обновление PSW - малдший байт
                                 if (psw_delayedupdate_even == 1'b1) begin
                                       psw[7:5] <= psw_delayedupdate[7:5] ; // Обновление приоритета процессора
                                       psw[3:0] <= psw_delayedupdate[3:0] ; // обновление флагов
                                 end 
                                 
                                 // Обновление PSW - старший байт
                                 if (psw_delayedupdate_odd == 1'b1)  begin
                                       psw[15:8] <= psw_delayedupdate[15:8] ; 
                                       rbus_cpu_mode <= psw_delayedupdate[15:14] ; // новый режим процессора
                                 end 
                                 // Снимаем флаг запроса обновления
                                 psw_delayedupdate_even <= 1'b0 ; 
                                 psw_delayedupdate_odd <= 1'b0 ; 
                                 state <= sq_ifetch;
                           end                            
                           
                           // Обработка DMA-запрсов
                           else if (npr == 1'b1)  begin
                              state <= sq_dma ;         // переходим в состояние ожидания завершения DMA
                           end
                           
                           // Поиск источника прерываний
                           //-----------------------------------------
                           // Обработка ошибок MMU
                           else if (mmutrap == 1'b1)  begin
                              state <= sq_mmutrap ;     // переходим к прерыванию по вектору 150
                              ack_mmutrap <= 1'b1 ;        // подтверждаем прием прерывания
                           end
                           
                           // Обработка желтой границы стека
                           else if (yellow_stack_latch)  begin
                              yellow_stack_latch <= 1'b0 ;  // снимаем флаг 
                              yellow_trap_inhibit <= 1'b1 ;  // предотвращаем повторную установку флага
                              trap_vector <= 9'o004;       // вектор 4
                              state <= sq_trap ;        // переходим к обработке вектора
                           end
                           
                           // Обработка прерываний от FPU
                           else if (falu_pending_fic | falu_pending_fiu | falu_pending_fiv | falu_pending_divz ) state <= sq_fptrap ; 

                           // Обработка маскируемых векторных прерываний  и PIRQ
                                                       
                           // Обработка программных прерываний PIRQ 7
                           else if ((pirq[15] == 1'b1) & (psw[7:5] < 3'b111)) begin
                                 trap_vector <= 9'o240; 
                                 state <= sq_trap ; 
                           end
                           
                           // Обработка векторных прерываний уровня 7
                           else if (virq[7] == 1'b1 & (psw[7:5]) < (3'b111))  begin
                              state <= sq_virq7 ; // переходим в состояния обработки векторных прерываний, приоритет 7
                           end
                           
                           // Обработка программных прерываний PIRQ 6                           
                           else if ((pirq[14] == 1'b1) & (psw[7:5] < 3'b110)) begin
                                 trap_vector <= 9'o240; 
                                 state <= sq_trap ; 
                           end
                           
                           // Обработка векторных прерываний уровня 6
                           else if (virq[6] == 1'b1 & (psw[7:5]) < (3'b110)) begin
                              state <= sq_virq6 ;
                           end

                           // Обработка программных прерываний PIRQ 5
                           else if ((pirq[13] == 1'b1) & (psw[7:5] < 3'b101)) begin
                                trap_vector <= 9'o240; 
                                state <= sq_trap ; 
                           end
                           
                           // Обработка векторных прерываний уровня 5
                           else if (virq[5] == 1'b1 & (psw[7:5]) < (3'b101))  begin
                              state <= sq_virq5 ; 
                           end
                           
                           // Обработка программных прерываний PIRQ 4
                           else if ((pirq[12] == 1'b1) & (psw[7:5] < 3'b100)) begin
                                trap_vector <= 9'o240; 
                                state <= sq_trap ; 
                           end
                           
                           // Обработка векторных прерываний уровня 4
                           else if (virq[4] == 1'b1 & (psw[7:5]) < (3'b100))  begin
                              state <= sq_virq4 ; 
                           end
                           
                           // Обработка программных прерываний PIRQ 3
                           else if ((pirq[11] == 1'b1) & (psw[7:5] < 3'b011)) begin
                                trap_vector <= 9'o240; 
                                state <= sq_trap ; 
                           end
                           
                           // Обработка программных прерываний PIRQ 2
                           else if ((pirq[10] == 1'b1) & (psw[7:5] < 3'b010)) begin
                                trap_vector <= 9'o240; 
                                state <= sq_trap ; 
                           end
                           
                           // Обработка программных прерываний PIRQ 1
                           else if ((pirq[9] == 1'b1) & (psw[7:5] < 3'b001)) begin
                                trap_vector <= 9'o240; 
                                state <= sq_trap ; 
                           end
                           
                           // Обработка прерываний по биту Т
                           else if ((psw[4]) == 1'b1 & ir_rtt == 1'b0 & iwait == 1'b0)  begin
                              // если поднят бит Т, и не выполнялась RTT, и мы не ждем прерывания
                              trap_vector <= 9'o014;   // вектор 14
                              state <= sq_trap ; 
                           end
                           // Активных прерываний нет - начинаем выборку инструкции
                           else begin
                              run <= 1'b1 ;            // признак запуска секвенсера
                              // сброс компонентов SR/MMR
                              sr1_srcd <= 5'b00000 ; // источник
                              sr1_dstd <= 5'b00000 ; // приемник
                              sr0_ic <= 1'b1 ;       // IC - флаг завершения инструкции
                              sr2 <= r7 ;            // SR2 - адрес текущей инструкции
                              if (iwait == 1'b0)  begin
                                 // Если не выполняется инструкция wait
                                 ifetch <= 1'b1 ;         // Поднимаем признак выборки инструкции
                                 nextstate <= sq_idecode ; // Переходим к выборке инструкции
                                 state <= sq_wbread_wait;
                                 wbm_adr_o <= r7;
                                 bus_timer <= 6'd`busdelay;   // взводим таймер ожидания
                                 wbm_stb_o <= 1'b1;
                                 wbm_we_o <= 1'b0;
                                 wbm_sel_o <= 2'b11;
                                 state <= sq_wbread_wait;
                              end
                           end 
                           
                        end
               //*********************************************************         
               //* Выборка и декодирование инструкции
               //*********************************************************         
               //*  addr=r7
               //*  rd+
               sq_idecode :  begin
                        if (nxmabort_pend == 1'b1)  begin
                                trap_vector <= 9'o004 ;    // Выставляем вектор 4
                                state <= sq_trap ;      // переходим к обычной обработке прерывания
                                nxmabort <= 1'b0;
                                nxmabort_pend <= 1'b0;
                        end
                        else begin
                           r7 <= r7+16'd2 ;       // Инкремент PC - инструкцию выбираем по PC+2
                           ir <= datain ;     // Выбираем инструкцию из памяти и помещаем в регистр ir
                           ir_addr <= r7 ;    // сохраняем адрес инструкции
                           sr0_ic <= 1'b0 ;   // признак незавершенности инструкции
                           ir_fpmaf <= 1'b0 ; // not a fp 4- or 8-byte memory access
                           ir_fpmai <= 1'b0 ; // not a fp integer mode memory access
                           
                           // Сброс флагов типов инструкций
                           ir_sop <= 1'b0 ; 
                           ir_dop <= 1'b0 ; 
                           ir_jmp <= 1'b0 ; 
                           ir_jsr <= 1'b0 ; 
                           ir_mfpi <= 1'b0 ;
                           ir_mfpd <= 1'b0 ;
                           ir_mtpi <= 1'b0 ;
                           ir_mtpd <= 1'b0 ;
                           ir_dopr <= 1'b0 ;
                           ir_fpsop1 <= 1'b0 ;
                           ir_fpsop2 <= 1'b0 ;
                           ir_fpao <= 1'b0 ; 
                           ir_facdst <= 1'b0 ;
                           ir_facsrc <= 1'b0 ;
                           ir_facfdst <= 1'b0 ;
                           ir_facfsrc <= 1'b0 ;
                           ir_rtt <= 1'b0 ; 
                           
                           fbus_fd <= fps[7] ; // fpu-флаг FPS 
                           
                           falu_pending_clear <= 1'b1 ; // Очистка запроса прерывания от FPU
                           
                           state <= sq_illegalop ;   // по умолчанию следующее состояние - неправильная инструкция
                           
                           // * Установка флагов характеристик выбранной инструкции
                           
                           // Флаг однооперандной инструкции
                           if (datain[14:9] == 7'b0000101               // один операнд, слово или байт (x05xxx)
                              | datain[14:8] == 8'b00001100             // один операнд, слово или байт (x06xxx), первая половина диапазона инструкций
                              | datain[15:6] == 10'b0000000011          // swab
                              | (datain[15:6] == 10'b0000110111 ))      // sxt
                             v_sop = 1'b1; 
                           else v_sop = 1'b0; 
                           
                           // Флаг двухоперандной инструкции
                           if (datain[14:12] != 3'b000 & datain[14:12] != 3'b111)  v_dop = 1'b1; 
                           else  v_dop = 1'b0;
                           
                           // jmp
                           if (datain[15:6] == 10'b0000000001)     v_jmp = 1'b1; 
                           else v_jmp = 1'b0; 

                           // jsr
                           if (datain[15:9] == 7'b0000100)         v_jsr = 1'b1; 
                           else   v_jsr = 1'b0; 

                           // mfpi
                           if (datain[15:6] == 10'b0000110101)  v_mfpi = 1'b1; 
                           else     v_mfpi = 1'b0; 
                           // mfpd
                           if (datain[15:6] == 10'b1000110101)  v_mfpd = 1'b1; 
                           else                      v_mfpd = 1'b0; 
                           // mtpi
                           if (datain[15:6] == 10'b0000110110)  v_mtpi = 1'b1; 
                           else                              v_mtpi = 1'b0; 
                           // mtpd
                           if (datain[15:6] == 10'b1000110110)  v_mtpd = 1'b1; 
                           else                              v_mtpd = 1'b0; 
                           // двухоперандная с регистром - eis и команда xor
                              // mul, div, ash, ashc
                              // xor
                           if ((datain[15:11] == 5'b01110) | (datain[15:9] == 7'b0111100))    v_dopr = 1'b1; 
                           else   v_dopr = 1'b0; 
                           
                           // fpu, однооперандная группы 1 - не использующие аккумулятор
                           // dfps, stfps, stst
                           if (datain[15:8] == 8'b11110000 & datain[7:6] != 2'b00 & fpu_enable == 1)  v_fpsop1 = 1'b1; 
                           else    v_fpsop1 = 1'b0; 

                           // fpu, однооперандная группы 2 - использующие аккумулятор
                           // lr(f/d), tst(f/d), abs(f/d), neg(f/d)
                           if (datain[15:8] == 8'b11110001 & fpu_enable == 1)    v_fpsop2 = 1'b1; 
                           else                              v_fpsop2 = 1'b0; 

                           // fpu, двухоперандные с аккумулятором
                           if (datain[15:12] == 4'b1111 & datain[11:9] != 3'b000 & fpu_enable == 1)  v_fpao = 1'b1; 
                           else           v_fpao = 1'b0; 

                           //
                           // Копирование флагов в регистры хранения
                           //
                           if (v_sop == 1'b1)  ir_sop <= 1'b1 ; 
                           if (v_dop == 1'b1)  ir_dop <= 1'b1 ; 
                           if (v_jmp == 1'b1)  ir_jmp <= 1'b1 ; 
                           if (v_jsr == 1'b1)  ir_jsr <= 1'b1 ; 
                           if (v_mfpi == 1'b1) ir_mfpi <= 1'b1 ; 
                           if (v_mfpd == 1'b1) ir_mfpd <= 1'b1 ; 
                           if (v_mtpi == 1'b1) ir_mtpi <= 1'b1 ; 
                           if (v_mtpd == 1'b1) ir_mtpd <= 1'b1 ; 
                           if (v_dopr == 1'b1) ir_dopr <= 1'b1 ; 
                           if (v_fpsop1 == 1'b1)  ir_fpsop1 <= 1'b1 ; 
                           if (v_fpsop2 == 1'b1)  begin
                              ir_fpsop2 <= 1'b1 ; 
                              ir_fpmaf <= 1'b1 ; 
                           end 
                           if (v_fpao == 1'b1) begin
                              ir_fpao <= 1'b1 ; 
                              ir_facdst <= 1'b0 ; 
                              ir_facsrc <= 1'b0 ; 
                              ir_facfdst <= 1'b0 ; 
                              ir_facfsrc <= 1'b0 ; 
                              if (datain[11:9] == 3'b101)  begin
                                 // stexp, stc(f|d)(i|l)
                                 ir_facdst <= 1'b1 ; 
                                    // stc(f|d)(i|l)
                                 if ((datain[8]) == 1'b1)  ir_fpmai <= 1'b1 ; // Требуется доступ к 2 или 4 байтам памяти
                              end
                                 // ldexp
                              else if (datain[11:8] == 4'b1101) ir_facsrc <= 1'b1 ; 
                              else if (datain[11:8] == 4'b1110) begin
                                 // ldc(i|l)(f|d)
                                 ir_facsrc <= 1'b1 ; 
                                 ir_fpmai <= 1'b1 ; // Требуется доступ к 2 или 4 байтам памяти
                              end
                              else if (datain[11:8] == 4'b1000) begin
                                 // st(f|d)
                                 ir_facfdst <= 1'b1 ; 
                                 ir_fpmaf <= 1'b1 ; // Требуется доступ к 4 или 8 байтам памяти
                              end
                              else if (datain[11:8] == 4'b1100) begin
                                 // stc(f|d)
                                 ir_facfdst <= 1'b1 ; 
                                 ir_fpmaf <= 1'b1 ; // Требуется доступ к 4 или 8 байтам памяти
                              end
                              else begin
                                 // Для всех fp-команд, не попавших в одну из вышеопределенных категорий
                                 ir_facfsrc <= 1'b1 ; 
                                 ir_fpmaf <= 1'b1 ; // Требуется доступ к 4 или 8 байтам памяти
                              end 
                           end 
                           
                           //*********************************************************************
                           //*  Обработка групп инструкций:
                           //*   - однооперандных
                           //*   - двухоперандных
                           //*   - JMP, JSR, MFPI, MFPD, MTPI, MTPD
                           //*   - EIS-инструкций и команды XOR
                           //*   - fp-инструкций, работающих с шиной
                           //*********************************************************************
                           if (v_sop == 1'b1    | v_dop == 1'b1    | v_jmp == 1'b1 
                             | v_jsr == 1'b1    | v_mfpi == 1'b1   | v_dopr == 1'b1 
                             | v_mfpd == 1'b1   | v_mtpi == 1'b1   | v_mtpd == 1'b1 
                             | v_fpsop1 == 1'b1 | v_fpsop2 == 1'b1 | v_fpao == 1'b1)  begin
                             
                              case (datain[5:3])
                                 //  Определение режимов адресации операнда - приемника или единственного
                                 //  установка соответствующего обработчика
                                 3'b000 : src_mode_processor <= sq_dst0 ;   // R
                                 3'b001 : src_mode_processor <= sq_dst1 ;   // @R   
                                 3'b010 : src_mode_processor <= sq_dst2 ;   // (R)+
                                 3'b011 : src_mode_processor <= sq_dst3 ;   // @(R)+
                                 3'b100 : src_mode_processor <= sq_dst4 ;   // -(R)
                                 3'b101 : src_mode_processor <= sq_dst5 ;   // @-(R)
                                 3'b110 : src_mode_processor <= sq_dst6 ;   // X(R) (индексный)
                                 3'b111 : src_mode_processor <= sq_dst7 ;   // @X(R) (косвенно-индексный)
                              endcase 
                              
                              // Установка обработчика регистра-приемника для АЛУ
                              if (datain[5:3] == 3'b000)  dst_mode_processor <= sq_store_alu_r ; // приемник - регистр
                              else                        dst_mode_processor <= sq_store_alu_p ; // все остальные способы адресации приемника
                              
                              //*********************************************************************
                              // дальнейшая обработка двухоперандных инструкций
                              //*********************************************************************
                              if (v_dop == 1'b1)   begin  
                                 // определение режима адресации источника
                                 case (datain[11:9])
                                    3'b000 :            // R
                                             begin
                                                rbus_ix <= datain[8:6] ;   // # регистра-источника
                                                state <= sq_src0 ;      
                                             end
                                    3'b001 :            // @R
                                             begin
                                                rbus_ix <= datain[8:6] ; 
                                                state <= sq_src1 ; 
                                             end
                                    3'b010 :            // (R)+
                                             begin
                                                rbus_ix <= datain[8:6] ; 
                                                state <= sq_src2 ; 
                                             end
                                    3'b011 :            // @(R)+
                                             begin
                                                rbus_ix <= datain[8:6] ; 
                                                state <= sq_src3 ; 
                                             end
                                    3'b100 :            // -(R)
                                             begin
                                                rbus_ix <= datain[8:6] ; 
                                                state <= sq_src4 ; 
                                             end
                                    3'b101 :            // @-(R)
                                             begin
                                                rbus_ix <= datain[8:6] ; 
                                                state <= sq_src5 ; 
                                             end
                                    3'b110 :            // X(R)
                                             begin
                                                rbus_ix <= datain[8:6] ; 
                                                state <= sq_src6 ; 
                                             end
                                    3'b111 :            // @X(R)
                                             begin
                                                rbus_ix <= datain[8:6] ; 
                                                state <= sq_src7 ; 
                                             end
                                 endcase 
                              end
                              else  begin
                              //*********************************************************************
                              //* Обработка однооперандных и безоперандных инструкций
                              //*********************************************************************
                                 // определение режима адресации операнда
                                 case (datain[5:3])
                                    3'b000 :  // R
                                             begin
                                                if ((v_jmp|v_jsr) == 1'b1) state <= sq_illegalop ; // JMP при регистровой адресации недопустим - трап 10 
                                                
                                                // mfpi/mfpd
                                                else if (v_mfpi == 1'b1 | v_mfpd == 1'b1)  begin
                                                   if (datain[2:0] == 3'b110)  rbus_cpu_mode <= psw[13:12] ; // получаем текущий режим процессора
                                                   rbus_ix <= datain[2:0] ;   // выделяем номер регистра
                                                   state <= sq_dst0 ;    // уходим на обработку приемника (R)
                                                end
                                                
                                                // mtpi/mtpd
                                                else if (v_mtpi == 1'b1 | v_mtpd == 1'b1)  begin
                                                   // в регистровом режиме - просто читаем регистр
                                                   rbus_ix <= 3'b110 ; // SP
                                                   state <= sq_mtp ; 
                                                end
                                                
                                                else if (fpu_enable == 1 & v_fpao == 1'b1)begin
                                                   if (datain[11:9] != 3'b101 & datain[11:8] != 4'b1101 & datain[11:8] != 4'b1110) begin
                                                      if (datain[2:1] == 2'b11)  begin
                                                         // ac6 and ac7 do not exist
                                                         fec <= 4'b0010 ; 
                                                         state <= sq_fptrap ; 
                                                      end
                                                      else begin
                                                         // stexp, stc(f/d)(i/l)
                                                         // ldexp
                                                         // ldc(i/l)(f/d)
                                                         if ({datain[11], datain[9:8]} == 3'b100)  fbus_raddr <= {1'b0, datain[7:6]} ; 
                                                         else                                      fbus_raddr <= datain[2:0] ; 
                                                         state <= sq_fpao ; 
                                                      end 
                                                   end
                                                   else  begin
                                                      fbus_raddr <= {1'b0, datain[7:6]} ; // ldexp, ldc(i/l)(f/d), stexp, stc(f/d)(i/l) get ac
                                                      rbus_ix <= datain[2:0] ; 
                                                      state <= sq_dst0 ; 
                                                   end 
                                                end
                                                
                                                else if (fpu_enable == 1 & v_fpsop2 == 1'b1) begin
                                                   if (datain[2:1] == 2'b11)  begin
                                                      // Регистров AC6 и 7 не существует - трап при попытке работы с ними
                                                      fec <= 4'b0010 ; 
                                                      state <= sq_fptrap ; 
                                                   end
                                                   else begin
                                                      fbus_raddr <= datain[2:0] ; 
                                                      state <= sq_fpso2 ; 
                                                   end 
                                                end
                                                
                                                else begin
                                                   rbus_ix <= datain[2:0] ; // выбираем номер регистр-приемника
                                                   state <= sq_dst0 ; 
                                                end 
                                             end
                                    3'b001 :   // @R
                                             begin
                                                state <= sq_dst1 ; 
                                                rbus_ix <= datain[2:0] ; 
                                             end
                                    3'b010 :   // (R)+
                                             begin
                                                state <= sq_dst2 ; 
                                                rbus_ix <= datain[2:0] ; 
                                             end
                                    3'b011 :   // @(R)+
                                             begin
                                                state <= sq_dst3 ; 
                                                rbus_ix <= datain[2:0] ; 
                                             end
                                    3'b100 :   // -(R)
                                             begin
                                                state <= sq_dst4 ; 
                                                rbus_ix <= datain[2:0] ; 
                                             end
                                    3'b101 :   // @-(R)
                                             begin
                                                state <= sq_dst5 ; 
                                                rbus_ix <= datain[2:0] ; 
                                             end
                                    3'b110 :   // X(R)
                                             begin
                                                state <= sq_dst6 ; 
                                                rbus_ix <= datain[2:0] ; 
                                             end
                                    3'b111 :   // @X(R)
                                             begin
                                                state <= sq_dst7 ; 
                                                rbus_ix <= datain[2:0] ; 
                                             end
                                 endcase 
                                 // для однооперандных инструкций - определяем обработчик результата
                                 if (v_sop == 1'b1)  begin
                                    if (datain[5:3] == 3'b000)   dst_mode_processor <= sq_store_alu_r ;   // регистр
                                    else                         dst_mode_processor <= sq_store_alu_p ;   // память
                                 end
                                 
                                 else if (v_jmp == 1'b1)         dst_mode_processor <= sq_jmp ; 
                                 else if (v_jsr == 1'b1)         dst_mode_processor <= sq_jsr ; 
                                 else if (v_mfpi == 1'b1 | v_mfpd == 1'b1) dst_mode_processor <= sq_mfp ; 
                                 else if (v_mtpi == 1'b1 | v_mtpd == 1'b1)  begin
                                    rbus_ix <= 3'b110 ; 
                                    state <= sq_mtp ; 
                                 end
                                 else if (v_dopr == 1'b1)       dst_mode_processor <= sq_dopr ; 
                                 
                                 else if (fpu_enable == 1 & v_fpsop1 == 1'b1)   begin
                                    case (datain[7:6])
                                       2'b01 :
                                                   // ldfps
                                                   dst_mode_processor <= sq_ldfps ; 
                                       2'b10 :
                                                begin
                                                   // stfps
                                                   if (datain[5:3] == 3'b000)  dst_mode_processor <= sq_store_alu_r ; 
                                                   else                        dst_mode_processor <= sq_store_alu_p ; 
                                                end
                                       2'b11 :
                                                begin
                                                   // stst
                                                   if (datain[5:3] == 3'b000)  dst_mode_processor <= sq_store_alu_r ; 
                                                   else                        dst_mode_processor <= sq_store_alu_p ; 
                                                end
                                    endcase 
                                 end
                                 else if (fpu_enable == 1 & v_fpsop2 == 1'b1)  dst_mode_processor <= sq_fpso2 ; // clr(f|d),tst(f|d),abs(f|d),neg(f|d)
                                 else if (fpu_enable == 1 & v_fpao == 1'b1)  begin
                                    dst_mode_processor <= sq_fpao ; 
                                       // ldexp
                                       // stexp
                                       // st(f|d), stc(f|d)(d|f)
                                       // stc(f|d)(i|l)
                                    if (datain[11:8] == 4'b1101 | datain[11:8] == 4'b1010 | {datain[11], datain[9:8]} == 3'b100 | datain[11:8] == 4'b1011) 
                                       fbus_raddr <= {1'b0, datain[7:6]} ; // st(f|d), stc(f|d)(d|f), ldexp, stexp
                                 end
                                 else  dst_mode_processor <= sq_illegalop ; 
                              end 
                           end 
                           
                           if (datain[14:11] == 4'b0000)  begin
                              if ((datain[15]) == 1'b0 & datain[10:8] == 3'b000)  begin
                                 if (datain[7:0] == 8'b00000000)  begin
                                    // HALT
                                    if (pswmf[15:14] != 2'b00 )  begin
                                       // в режимах, отличных от kernel, команда вызывает трап 4
                                       illhalt <= 1'b1 ; 
                                       trap_vector <= 9'o004; 
                                       state <= sq_trap ; 
                                    end
                                    else  begin
                                       // в режиме kernel - отключаем секвенсор
                                       run <= 1'b0 ; 
                                       state <= sq_halt ; 
                                    end
                                 end 
                                 if (datain[7:0] == 8'b00000001)   begin
                                    // wait
                                    iwait <= 1'b1 ; 
                                    state <= sq_ifetch ; 
                                 end 
                                 if (datain[7:0] == 8'b00000010)   begin
                                    // rti
                                    state <= sq_rti ; 
                                    rbus_ix <= 3'b110 ; 
                                 end 
                                 if (datain[7:0] == 8'b00000011)  begin
                                    // bpt
                                    trap_vector <= 9'o014 ; // bpt, vector = 014
                                    state <= sq_trap ; 
                                 end 
                                 if (datain[7:0] == 8'b00000100)  begin
                                    // IOT
                                    trap_vector <= 9'o020; // iot, vector = 020
                                    state <= sq_trap ; 
                                 end 
                                 if (datain[7:0] == 8'b00000101) begin
                                    // RESET
                                    if (pswmf[15:14] == 2'b00) begin
                                       initcycles <= `initcycles_reset ; 
                                       state <= sq_init ; 
                                    end
                                    else  state <= sq_ifetch ; // Не в kernel эта команда игнорируется
                                 end 
                                 if (datain[7:0] == 8'b00000110)   begin
                                    // RTT
                                    state <= sq_rti ; 
                                    rbus_ix <= 3'b110 ; 
                                    ir_rtt <= 1'b1 ; 
                                 end 
                                 if (datain[7:3] == 5'b10000) begin
                                    // RTS
                                    state <= sq_rts ; 
                                    rbus_ix <= 3'b110 ; 
                                 end 
                                 if (datain[7:3] == 5'b10011)  begin
                                    // SPL
                                    if (pswmf[15:14] == 2'b00)  begin
                                       psw_delayedupdate[7:5] <= datain[2:0] ; 
                                       spl_psw_update <= 1'b1;
                                    end   
                                    state <= sq_ifetch ; 
                                 end 
                                 if (datain[7:4] == 4'b1010)   begin
                                    // clear cc
                                    psw[3:0] <= psw[3:0] & (~datain[3:0]) ; 
                                    state <= sq_ifetch ; 
                                 end 
                                 if (datain[7:4] == 4'b1011)  begin
                                    // set cc
                                    psw[3:0] <= psw[3:0] | datain[3:0] ; 
                                    state <= sq_ifetch ; 
                                 end 
                              end
                              else begin
                                 // Команды относительных переходов.
                                 state <= sq_ifetch ; 
                                 case ({datain[15], datain[10:8]})
                                    // Вычисление адреса перехода
                                    4'b0001 :
                                                // br
                                                r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b0010 :
                                                // bne
                                                if ((psw[2]) == 1'b0)  r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b0011 :
                                                // beq
                                                if ((psw[2]) == 1'b1)  r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b0100 :
                                                // bge
                                                if (psw[3] == psw[1])  r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b0101 :
                                                // blt
                                                if (psw[3] != psw[1])  r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b0110 :
                                                // bgt
                                                if ((psw[2]) == 1'b0 & psw[3] == psw[1]) r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b0111 :
                                                // ble
                                                if ((psw[2]) == 1'b1 | psw[3] != psw[1])  r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b1000 :
                                                // bpl
                                                if ((psw[3]) == 1'b0)  r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b1001 :
                                                // bmi
                                                if ((psw[3]) == 1'b1)  r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b1010 :
                                                // bhi
                                                if ((psw[2]) == 1'b0 & (psw[0]) == 1'b0)  r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b1011 :
                                                // blos
                                                if ((psw[2]) == 1'b1 | (psw[0]) == 1'b1)  r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b1100 :
                                                // bvc
                                                if ((psw[1]) == 1'b0)  r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b1101 :
                                                // bvs
                                                if ((psw[1]) == 1'b1)  r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b1110 :
                                                // bhis
                                                if ((psw[0]) == 1'b0)  r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                    4'b1111 :
                                                // blo
                                                if ((psw[0]) == 1'b1)  r7 <= r7+16'd2 + ({datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7], datain[7:0], 1'b0}) ; 
                                 endcase 
                              end 
                           end 
                           if (datain[15:9] == 7'b1000100)  begin
                              // trap, emt
                              if ((datain[8]) == 1'b0)    trap_vector <= 9'o030; // emt, vector = 030
                              else                        trap_vector <= 9'o034; // trap, vector = 034
                              state <= sq_trap ; 
                           end 
                           
                           if (datain[15:9] == 7'b0111111)  begin
                              // sob
                              rbus_ix <= datain[8:6] ; 
                              state <= sq_sob ; 
                           end 
                           
                           if (datain[15:6] == 10'b0000110100)  begin
                              // mark
                              rbus_waddr <= {pswmf[15:14], 4'b0110} ; 
                              rbus_d <= r7+16'd2 + ({datain[5:0], 1'b0}) ; 
                              rbus_we <= 1'b1 ; 
                              rbus_ix <= 3'b101 ; 
                              state <= sq_mark ; 
                           end 
                           
                           if (fpu_enable == 1 & datain[15:6] == 10'b1111000000)  begin
                              // Команды FPP
                              case (datain[5:0])
                                 6'b000000 :
                                          begin
                                             // cfcc
                                             psw[3:0] <= fps[3:0] ; 
                                             state <= sq_ifetch ; 
                                          end
                                 6'b000001 :
                                          begin
                                             // setf
                                             fps[7] <= 1'b0 ; 
                                             state <= sq_ifetch ; 
                                          end
                                 6'b000010 :
                                          begin
                                             // seti
                                             fps[6] <= 1'b0 ; 
                                             state <= sq_ifetch ; 
                                          end
                                 6'b001001 :
                                          begin
                                             // setd
                                             fps[7] <= 1'b1 ; 
                                             state <= sq_ifetch ; 
                                          end
                                 6'b001010 :
                                          begin
                                             // setl
                                             fps[6] <= 1'b1 ; 
                                             state <= sq_ifetch ; 
                                          end
                                 default :
                                          begin
                                                if (datain[5:3] == 3'b000)  state <= sq_ifetch ; 
                                                else  begin
                                                   fec <= 4'b0010 ;
                                                   state <= sq_fptrap ; 
                                                end 
                                          end
                              endcase 
                           end 
                        end
                     end   
               //*****************************************************
               // halt : остановка секвенсера до сигнала CONT
               //*****************************************************
               sq_halt :
                        begin
                           if (sw_cont == 1'b0) run <= 1'b0 ; // отключаем секвенсер
                           else begin
                             // кнопка CONT нажата
                             run <= 1'b1;         // включаем секвенсер
                             state <= sq_ifetch;  // запускаем выборку команд
                           end  
                        end   
               //*****************************************************
               // неправильный код операции
               //*****************************************************
               sq_illegalop :
                        begin
                           // неправильный КОП для FP-команд
                           if (fpu_enable == 1 & ir[15:12] == 4'b1111)   begin
                              fec <= 4'b0010 ; 
                              state <= sq_fptrap ;  
                           end
                           
                           // остальные ошибки команд
                           else   begin
                              trap_vector <= 9'o010; // недопустимая инструкция, трап 10
                              state <= sq_trap ; 
                           end 
                        end
               //*****************************************************
               // jmp 
               //*****************************************************
               sq_jmp :
                        begin
                           r7 <= dest_addr ;    // просто записываем в R7 новый адрес
                           state <= sq_ifetch ; // и продолжаем выборку команд
                        end
               //*****************************************************
               // npg : подтверждение DMA
               //*****************************************************
               sq_dma :
                        begin
                           run <= 1'b0 ;    // останавливаем секвенсор 
                           if (npr == 1'b0)  begin
                              // запрос DMA снят
                              state <= sq_ifetch ; // включаем секвенсор и переходим к следующей инструкции
                              npg <= 1'b0 ;  // снимаем сигнал подтверждения
                           end
                           else   npg <= 1'b1 ;  // выставляем сигнал подтверждени
                        end
               //*******************************************************************
               // mmuabort : mmu требует прекратить обработку текущей инструкции
               //*******************************************************************
               sq_mmuabort:
                        begin
                           wbm_stb_o <= 1'b0;    // снимаем строб шины
                           wbm_we_o <= 1'b0;     // снимаем признак записи
                           finalreference <= 1'b0;
                           dstfreference <= 1'b0;
                           sr0_ic <= 1'b0;
                           cp <= 1'b0;
                           if (trap_flag == 1'b1) begin
                              psw <= temp_psw;
                              sr2 <= r7;
                              trap_flag <= 1'b0;
                           end   
                           if (mmuabort == 1'b0)  begin  // ждем снятия сигнала прерывания со стороны MMU
                              ack_mmuabort <= 1'b0 ; // снимаем сигнал подтверждения 
                              trap_vector <= 9'o250; // прерывание по вектору 250
                              state <= sq_trap ; 
                           end 
                        end
               //*****************************************************************************
               // mmutrap : mmu требует запрос прерывания по окончании текущей инструкции
               //*****************************************************************************
               sq_mmutrap :
                        begin
                           if (mmutrap == 1'b0)  begin
                              ack_mmutrap <= 1'b0 ; 
                              trap_vector <= 9'o250; // mmu, vector = 250
                              state <= sq_trap ; 
                           end 
                        end
               //*****************************************************         
               // Запрос векторного прерывания уровня 7
               //*****************************************************         
               sq_virq7 :
                        begin
                              // проверяем, не снят ли запрос прерывания
                              if (virq[7] == 1'b0) begin
                                 state <= sq_ifetch;
                                 vstb[7] <= 1'b0;
                              end   
                              // если не снят, поднимаем строб запроса вектора
                              else vstb[7] <= 1'b1;    
                              
                              // получение вектора прерывания
                              if (virq[7] == 1'b1) trap_vector <= vector ;  // получаем вектор прерывания
                              // получено подтверждение
                              if (vack == 1'b1) begin
                                vstb[7] <= 1'b0;             // 
                                state <= sq_trap ;  // переходим к обработке прерывания
                                sr2 <= {7'b0000000, vector} ; // записываем вектор в SR2
                                sr0_ic <= 1'b0 ; // снимаем флаг завершения выполнения инструкции
                             end 
                        end
               //*****************************************************         
               // Запрос векторного прерывания уровня 6
               //*****************************************************         
               sq_virq6 :
                        begin
                              // проверяем, не снят ли запрос прерывания
                              if (virq[6] == 1'b0) begin
                                 state <= sq_ifetch;
                                 vstb[6] <= 1'b0;
                              end   
                              // если не снят, поднимаем строб запроса вектора
                              else vstb[6] <= 1'b1;    
                              
                              // получение вектора прерывания
                              if (virq[6] == 1'b1) trap_vector <= vector ;  // получаем вектор прерывания
                              // получено подтверждение
                              if (vack == 1'b1) begin
                                vstb[6] <= 1'b0;             // 
                                state <= sq_trap ;  // переходим к обработке прерывания
                                sr2 <= {7'b0000000, vector} ; // записываем вектор в SR2
                                sr0_ic <= 1'b0 ; // снимаем флаг завершения выполнения инструкции
                             end 
                        end
               //*****************************************************         
               // Запрос векторного прерывания уровня 5
               //*****************************************************         
               sq_virq5 :
                        begin
                              // проверяем, не снят ли запрос прерывания
                              if (virq[5] == 1'b0)  begin
                                 state <= sq_ifetch;
                                 vstb[5] <= 1'b0;
                              end   
                              // если не снят, поднимаем строб запроса вектора
                              else vstb[5] <= 1'b1;    
                              
                              // получение вектора прерывания
                              if (virq[5] == 1'b1) trap_vector <= vector ;  // получаем вектор прерывания
                              // получено подтверждение
                              if (vack == 1'b1) begin
                                vstb[5] <= 1'b0;             // 
                                state <= sq_trap ;  // переходим к обработке прерывания
                                sr2 <= {7'b0000000, vector} ; // записываем вектор в SR2
                                sr0_ic <= 1'b0 ; // снимаем флаг завершения выполнения инструкции
                             end 
                        end
               //*****************************************************         
               // Запрос векторного прерывания уровня 4
               //*****************************************************         
               sq_virq4 :
                        begin
                              // проверяем, не снят ли запрос прерывания
                              if (virq[4] == 1'b0) begin
                                 state <= sq_ifetch;
                                 vstb[4] <= 1'b0;
                              end   
                              // если не снят, поднимаем строб запроса вектора
                              else vstb[4] <= 1'b1;    
                              
                              // получение вектора прерывания
                              if (virq[4] == 1'b1) trap_vector <= vector ;  // получаем вектор прерывания
                              // получено подтверждение
                              if (vack == 1'b1) begin
                                vstb[4] <= 1'b0;             // 
                                state <= sq_trap ;  // переходим к обработке прерывания
                                sr2 <= {7'b0000000, vector} ; // записываем вектор в SR2
                                sr0_ic <= 1'b0 ; // снимаем флаг завершения выполнения инструкции
                             end 
                        end
               //*****************************************************         
               // Обработка прерываний FPU
               //*****************************************************         
               sq_fptrap :
                        begin
                           if (falu_pending_fic == 1'b1)  fec <= 4'b0110 ; 
                           if (falu_pending_fiu == 1'b1)  fec <= 4'b1010 ; 
                           if (falu_pending_fiv == 1'b1)  fec <= 4'b1000 ; 
                           if (falu_pending_divz == 1'b1) fec <= 4'b0100 ; 
                           fea <= ir_addr ; 
                           fps[15] <= 1'b1 ; 
                           if ((fps[14]) == 1'b0)  begin
                              trap_vector <= 9'o244; // вектор 244
                              state <= sq_trap ;  // переходим к обработке прерывания
                           end
                           else  begin
                              // ждем очистки флагов ожидания прерывания
                              if (falu_pending_fic == 1'b0 & 
                                  falu_pending_fiu == 1'b0 & 
                                  falu_pending_fiv == 1'b0 & 
                                  falu_pending_divz == 1'b0) state <= sq_ifetch ; 
                           end 
                           falu_pending_clear <= 1'b1 ; 
                        end
               //*****************************************************         
               // Ошибка контроля границ стека - красное состояние
               //*****************************************************         
               sq_rsv :
                        begin 
                              rbus_ix <= 3'o6;
                              state <= sq_rsv1;
                              rsv_flag <= 1'b1;
                        end
                  
               sq_rsv1:      
                        begin 
                              prev_sp <= rbus_o;
                              psw <= temp_psw ; 
                              rbus_waddr <= {2'b00, 4'b0110} ;  // SP режима kernel
                              rbus_d <= 16'h0004;               // устанавливаем в 4
                              rbus_we <= 1'b1 ;                 // поднимаем бит записи
                              trap_vector <= 9'o004;            // вектор 4
                              state <= sq_trap ;             // уходим в обработку прерывания
                              trap_flag <= 1'b0;
                              trap_force_kernel <= 1'b1;
                        end
                              
               //*****************************************************         
               // Обработка векторных прерываний - подготовка
               //*****************************************************
               sq_trap :
                        begin
                           // если это - повторное прерывание, произошло при обработке предыдущего
                           if (trap_flag == 1'b1) state <= sq_rsv;
                           else begin
                              trap_flag <= 1'b1;
                              finalreference <= 1'b0;                        
                              iwait <= 1'b0;
                              temp_psw <= psw ;     // сохранияем PSW
                              psw[15:14] <= 2'b00 ; // устанавливаем режим kernel
                              psw[13:12] <= pswmf[15:14] ; // устанавливаем биты предыдущиего режима
                              rbus_cpu_mode <= 2'b00 ; // переходим в режим kernel
                              state <= sq_trapa ; 
                           end   
                        end
               //**********************************************************      
               //*  Обработка векторных прерываний -  чтение PSW из вектора
               //*  и формирование рабочего PSW
               //*  Адрес на шине - вектор+2
               //**********************************************************
               //*  addr = trap_vector+2 - вектор+2
               //*  RD+
               sq_trapa : 
                        begin
                           rbus_we <= 1'b0;
                           wbm_adr_o <= trap_vector+16'd2;
                           nextstate <= sq_trapal;
                           state <= sq_wbread;
                        end

               sq_trapal :
                        begin
                           nxmabort <= nxmabort_pend;
                           rbus_ix <= 3'b110 ;    // регистр SP
                           if (trap_force_kernel | red_stack_trap)  rbus_cpu_mode <= 2'b00 ; 
                           else                                     rbus_cpu_mode <= datain[15:14] ; 
                           if (~trap_force_kernel) psw[15:14] <= datain[15:14] ;  // подавляем смену режима при красной ошибке стека
                           psw[11:0] <= datain[11:0] ; 
                           state <= sq_trapb ; 
                        end
               //******************************************************************      
               //*  Обработка векторных прерываний - запись в стек старого PSW
               //******************************************************************
               //*  addr = rbus_data-2 - SP-2
               //*  dataout=temp_psw
               //*  WR+
               sq_trapb :  
                       begin
                           wbm_adr_o <= rbus_data-16'd2;
                           wbm_dat_o <= temp_psw;
                           state <= sq_wbwrite;
                           nextstate <= sq_trapc ; 
                       end    
               //**********************************************************      
               //* Обработка векторных прерываний - декремент SP
               //**********************************************************
               sq_trapc :
                        begin
                           nxmabort <= nxmabort_pend;
                           if (red_stack_trap == 1'b1)  rbus_waddr <= {2'b00, 4'b0110} ; 
                           else                         rbus_waddr <= {pswmf[15:14], 4'b0110} ; 
                           rbus_d <= rbus_data-16'd2 ;    // sp-2
                           rbus_we <= 1'b1 ;          // включаем запись - обновляем SP
                           state <= sq_trapw;      
                        end
               //**********************************************************      
               //*  Обработка векторных прерываний - запись PC в стек
               //*  Адрес на шине - значение предыдущего SP-4
               //************************************************************************
               //*  addr = rbus_data-2 - SP-2
               //*  dataout=R7
               //*  WR+
               sq_trapw :
                        begin
                           wbm_adr_o <= rbus_data-16'd4;    
                           wbm_dat_o <= r7;
                           state <= sq_wbwrite;
                           nextstate <= sq_trapd ; 
                        end   
               //************************************************************************
               //* Обработка векторных прерываний -  декремент SP и установка режима ядра
               //* Чтение адреса обработчика из вектора
               //************************************************************************
               //*  addr = trap_vector - вектор, адрес перехода к обработчику прерывания
               //*  RD+
               sq_trapd :
                        begin
                           nxmabort <= nxmabort_pend;
                           if (red_stack_trap == 1'b1) rbus_waddr <= {2'b00, 4'b0110} ;           // обработка красной ошибки стека - используем стек ядра
                           else                        rbus_waddr <= {pswmf[15:14], 4'b0110} ;    // иначе - стек текущего режима
                           rbus_d <= rbus_data-16'd2 ;  // установка SP-2
                           rbus_we <= 1'b1 ; 
                           rbus_cpu_mode <= 2'b00 ; // устанавливаем режим KERNEL
                           wbm_adr_o <= trap_vector; // адрес вектора прерывания
                           state <= sq_wbread;
                           nextstate <= sq_trapf;
                        end
               //******************************************************************
               //* Обработка векторных прерываний - переход на адрес обработчика
               //******************************************************************
               sq_trapf :
                        begin
                           nxmabort <= nxmabort_pend;
                           trap_force_kernel <= 1'b0;
                           r7 <= datain ;         // новый PC
                           state <= sq_ifetch ;   // с этого РС начинаем выбирать команды
                           trap_flag <= 1'b0;
                           if (rsv_flag) begin
                             rsv_flag <= 1'b0;
                             rbus_waddr <= {2'b00, 4'b0110} ;  // SP режима kernel
                             rbus_d <= prev_sp;               // возвращаем старый SP
                             rbus_we <= 1'b1 ;                 // поднимаем бит записи
                           end  
                        end
               //**********************************************************
               //*  RTI - возврат из прерывания - инкремент SP
               //**********************************************************
               sq_rti :
                        begin
                           state <= sq_rtia ; 
                           rbus_waddr <= {pswmf[15:14], 4'b0110} ; 
                           rbus_d <= rbus_data+16'd2 ; // SP__
                           rbus_we <= 1'b1 ; 
                        end
               //**********************************************************
               //*  RTI - возврат из прерывания - чтение РС возврата
               //**********************************************************
               //*  addr = rbus_data - содержимое стека
               //*  RD+
               sq_rtia :
                        begin
                           nextstate <= sq_rtial;
                           wbm_adr_o <= rbus_data;
                           state <= sq_wbread;
                        end   
               sq_rtial :
                        begin
                           nxmabort <= nxmabort_pend;
                           state <= sq_rtib ; 
                           r7 <= datain ; 
                        end
               //**********************************************************
               //*  RTI - возврат из прерывания - чтение PSW 
               //**********************************************************
               //*  addr = rbus_data - содержимое стека
               //*  RD+
               sq_rtib :
                        begin
                           nextstate <= sq_rtibl;
                           wbm_adr_o <= rbus_data;
                           state <= sq_wbread;
                        end   
               sq_rtibl :
                        begin
                           nxmabort <= nxmabort_pend;
                           state <= sq_ifetch ; 
                           rbus_waddr <= {pswmf[15:14], 4'b0110} ; 
                           rbus_d <= rbus_data+16'd2 ;  // SP++
                           rbus_we <= 1'b1 ; 
                              psw[4:0] <= datain[4:0] ; 
                              if (pswmf[15:14] == 2'b00)  psw[7:5] <= datain[7:5] ; 
                              psw[10:8] <= datain[10:8] ; 
                              if (pswmf[15:14] == 2'b00)  psw[15:11] <= datain[15:11] ; 
                              else                        psw[15:11] <= datain[15:11] | pswmf[15:11] ; 
                        end
               //**********************************************************
               //*  SOB
               //**********************************************************
               sq_sob :
                        begin
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[8:6]} ; 
                              rbus_d <= rbus_data-16'd1 ; 
                              rbus_we <= 1'b1 ; 
                              if (rbus_data == 16'b0000000000000001)  state <= sq_ifetch ; 
                              else  begin
                                 r7 <= r7 - ({ir[5:0], 1'b0}) ; 
                                 state <= sq_ifetch ; 
                              end 
                        end
               //**********************************************************
               //*  MFPI/MFPD - чтение кода/данных из предыдущего режима
               //**********************************************************
               sq_mfp :
                        begin
                           rbus_ix <= 3'b110 ;      // SP
                           state <= sq_mfpa ; 
                        end
               //* addr = rbus_data-2
               //* RD +         
               sq_mfpa :
                        begin
                           dest_addr <= rbus_data-16'd2;      // адрес записи - SP-2
                           state <= sq_store_alu_p ;  // следующий этап - сохранение результатов АЛУ
                           sr1_srcd <= sr1_m2 ; 
                           rbus_waddr <= {psw[15:14], 1'b0, 3'b110} ;  // запись SP
                           rbus_d <= rbus_data-16'd2 ;                     // sp-2
                           rbus_we <= 1'b1 ; 
                        end
               //***********************************************************************
               //*  MFPI/MFPD - запсь кода/данных в пространство предыдущего режима
               //***********************************************************************
               //*  addr=rbus_data  - SP
               //*  RD +
               sq_mtp : 
                        begin
                           wbm_adr_o <= rbus_data;
                           nextstate <= sq_mtpl;
                           state <= sq_wbread;
                        end
               sq_mtpl :
                        begin
                           nxmabort <= nxmabort_pend;
                           sr1_srcd <= sr1_p2 ; 
                           rbus_waddr <= {psw[15:14], 1'b0, 3'b110} ; 
                           rbus_d <= rbus_data+16'd2 ; 
                           rbus_we <= 1'b1 ; 
                           state <= sq_mtpa ; 
                        end
               sq_mtpa :
                        begin
                           alus_input <= datain ; 
                           rbus_ix <= ir[2:0] ; 
                           state <= src_mode_processor ; 
                        end
               //**********************************************************
               //*  Загрузка 32-битных операндов EIS-инструкций
               //**********************************************************
               sq_dopr :
                        begin
                           rbus_ix <= ir[8:6] ;    // выбираем номер регистра-операнда
                           state <= sq_dopra ; 
                        end
                        
               sq_dopra :
                        begin
                           // загрузка старшей части операнда
                             // Для случая, когда операнд 1 - PC, и используется индексная адресация, надо сдвинуть значение на слово назад
                           if ((ir[8:6] == 3'b111) && (ir[5:4] == 2'b11)) alus_input <= rbus_data-2 ;
                             // остальные случаи
                           else                   alus_input <= rbus_data ; 
                           rbus_ix <= {ir[8:7], 1'b1} ;        // выставляем номер следующего нечетного регистра
                           
                           if (ir[11:9] == 3'b000) begin
                              // MUL
                              eis_sequencer <= 5'b11111 ; 
                              state <= sq_mul ; 
                           end
                           else if (ir[11:9] == 3'b010) begin
                              // ASH
                              eis_sequencer <= 5'b11111 ; 
                              state <= sq_ash ; 
                           end
                              // XOR
                           else if (ir[11:9] == 3'b100) state <= sq_xor ; 
                              // остальные инструкции
                           else    state <= sq_doprb ; 
                        end
               // Дополнительный шаг - загрузка младшей части операнда
               sq_doprb :
                        begin
                           alut_input <= rbus_data ;  // младшая часть из нечетного регистра
                           
                           // DIV
                           if (ir[11:9] == 3'b001) begin
                              if (ir[6] == 1'b1) begin
                                 // Ошибка - регистр-операнд должен быть четным
                                 state <= sq_ifetch ;  // прекращаем обработку инструкции
                                 psw[3:0] <= 4'b0010 ; // V=1, остальные флаги - 0
                              end
                              
                              else  begin
                                 eis_sequencer <= 5'b10000 ; 
                                 state <= sq_div ;     // переходим непосредственно к делению
                              end 
                           end
                           
                           // ASHC
                           else if (ir[11:9] == 3'b011)  begin
                              eis_sequencer <= 5'b11111 ; 
                              state <= sq_ashc ; 
                           end
                           else    state <= sq_illegalop ; // неподдерживаемые EIS-коды
                        end
                        
               //************************************************************         
               // MUL: умножение. Ожидание секвенсера EIS 
               //************************************************************         
               sq_mul :
                        begin
                           if (eis_sequencer == 5'b00001)  state <= sq_mula ; 
                           eis_sequencer <= eis_sequencer + 5'd1 ; 
                        end
               //************************************************************         
               // MUL: умножение. Запись младшей части результата
               //************************************************************         
               sq_mula :
                        begin
                           if (ir[8:6] != 3'b111)   begin
                              // для регистров-приемников кроме PC
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[8:6]} ; 
                              rbus_d <= eis_output ; 
                              rbus_we <= 1'b1 ; 
                           end
                           // для PC
                           else  r7 <= eis_output ; 
                           state <= sq_mulb ; 
                        end
               //************************************************************         
               // MUL: умножение. Запись старшей части результата
               //************************************************************         
               sq_mulb :
                        begin
                           if (ir[8:7] != 2'b11)  begin
                              // кроме PC
                              //                                               !- принудительно выставляем нечетный регистр
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[8:7], 1'b1} ; 
                              rbus_d <= eis_output32 ; 
                              rbus_we <= 1'b1 ; 
                           end
                           // для PC 
                           else r7 <= eis_output32 ; 
                           psw[3:0] <= eis_psw ; 
                           state <= sq_ifetch ; 
                        end
               //************************************************************         
               //* DIV: деление. 
               //************************************************************         

               // Счетчик шагов EIS-секвенсера
               sq_div :
                        begin
                           if (eis_sequencer == 5'b11111)   state <= sq_diva ; 
                           eis_sequencer <= eis_sequencer - 5'd1 ; 
                        end
                        
               // Запись частного         
               sq_diva :
                        begin
                           if (eis_psw[1:0] == 2'b00)  begin
                              if (ir[8:6] != 3'b111)  begin
                                 rbus_waddr <= {pswmf[15:14], pswmf[11], ir[8:6]} ; 
                                 rbus_d <= eis_output ; 
                                 rbus_we <= 1'b1 ; 
                              end
                              else r7 <= eis_output ; 
                           end 
                           state <= sq_divb ; 
                        end
                        
               // запись остатка         
               sq_divb :
                        begin
                           if (eis_psw[1:0] == 2'b00)  begin
                              if (ir[8:7] != 2'b11) begin
                                 rbus_waddr <= {pswmf[15:14], pswmf[11], ir[8:7], 1'b1} ; 
                                 rbus_d <= eis_output32 ; 
                                 rbus_we <= 1'b1 ; 
                              end
                              else  r7 <= eis_output32 ; 
                           end
                           psw[3:0] <= eis_psw[3:0];   
                           /*
                           if (eis_psw[1] == 1'b0) psw[3:0] <= eis_psw ; // V=0, нормальное завершение
                           else psw[1:0]=eis_psw[1:0];                   // V=1 - не трогаем Z и N, чтобы EKBB не придирался
                           */
                           state <= sq_ifetch ; 
                        end
               //************************************************************         
               //*  ASH: арифметический сдвиг
               //************************************************************         
               sq_ash :
                        begin
                           if (eis_sequencer == 5'b11111)  eis_sequencer <= eis_sequencer + 5'd1 ; 
                           else if (eis_flag2 == 1'b1)  state <= sq_ashb ; 
                        end
               sq_ashb :
                        begin
                           if (ir[8:6] != 3'b111)  begin
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[8:6]} ; 
                              rbus_d <= eis_output ; 
                              rbus_we <= 1'b1 ; 
                           end
                           else  r7 <= eis_output ; 
                           psw[3:0] <= eis_psw ; 
                           state <= sq_ifetch ; 
                        end
               // ashc through ashe: handle ashc insn
               sq_ashc :
                        begin
                           if (eis_sequencer == 5'b11111)    eis_sequencer <= eis_sequencer + 5'd1 ; 
                           else if (eis_flag2 == 1'b1) state <= sq_ashd ; 
                        end
               sq_ashd :
                        begin
                           if (ir[8:6] != 3'b111)  begin
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[8:6]} ; 
                              rbus_d <= eis_output ; 
                              rbus_we <= 1'b1 ; 
                           end
                           else  r7 <= eis_output ; 
                           state <= sq_ashe ; 
                        end
               sq_ashe :
                        begin
                           if (ir[8:7] != 2'b11)  begin
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[8:7], 1'b1} ; 
                              rbus_d <= eis_output32 ; 
                              rbus_we <= 1'b1 ; 
                           end
                           else   r7 <= eis_output32 ; 
                           psw[3:0] <= eis_psw ; 
                           state <= sq_ifetch ; 
                        end
               // xor
               sq_xor :
                        begin
                           if (ir[5:3] == 3'b000)  state <= sq_store_alu_r ; 
                           else                    state <= sq_store_alu_p ; 
                        end
               // ldfps - чтение состояния FPU
               sq_ldfps :
                        begin
                           fps <= alu_output ; 
                           state <= sq_ifetch ; 
                        end
               //*******************************************         
               // STST - запись состояния FPU
               //******************************************* 
               //* adr = dest_addr
               //* dataout = fea
               //* WR +
               sq_stststore :  
                 begin
                  wbm_adr_o <= dest_addr;
                  wbm_dat_o <= fea;
                  nextstate <= sq_ifetch ; 
                  state <= sq_wbwrite;
                 end
                           
               sq_fpso2 :
                        begin
                           addr_indirect <= dest_addr ; 
                           if (ir[5:3] != 3'b000)  begin
                                 // clr(f|d)
                              if (ir[7:6] == 2'b00) state <= sq_fprun ; // don't need to read for clear
                              else                  state <= sq_fpr1 ; 
                           end
                           else    begin
                              falu_input <= fbus_o ; // fbus read already done in ifetch for mode 0
                              state <= sq_fprun ; 
                           end 
                        end
               sq_fpao :
                        begin
                           if (ir[5:3] != 3'b000)   begin
                              addr_indirect <= dest_addr ; 
                              if (ir_facfsrc == 1'b1)  begin
                                 fbus_raddr <= {1'b0, ir[7:6]} ; 
                                 state <= sq_fpr1 ; 
                              end
                              else if (ir_facfdst == 1'b1)   begin
                                 falu_input <= fbus_o ; 
                                 state <= sq_fprun ; 
                              end
                              else if (ir_facdst == 1'b1)    begin
                                 falu_input <= fbus_o ; 
                                 state <= sq_fprun ; 
                              end
                              else if (ir_facsrc == 1'b1)    begin
                                 falu_input <= fbus_o ; 
                                 state <= sq_fpir1 ; 
                              end
                              // FIXME, go into some cpu error state? 
                           end
                           else  begin
                              if (ir_facfsrc == 1'b1)  begin
                                 falu_input <= fbus_o ; 
                                 fbus_raddr <= {1'b0, ir[7:6]} ; 
                                 state <= sq_fprun ; 
                              end
                              else if (ir_facfdst == 1'b1) begin
                                 falu_input <= fbus_o ; 
                                 state <= sq_fprun ; 
                              end
                              else if (ir_facdst == 1'b1)  begin
                                 falu_input <= fbus_o ; 
                                 state <= sq_fprun ; 
                              end
                              else if (ir_facsrc == 1'b1)  begin
                                 // mode 0, so input from register!!!
                                 if ((ir[8]) == 1'b1) begin
                                    // ldexp
                                    falu_input <= fbus_o ; 
                                    falus_input[55:40] <= rbus_data ; 
                                 end
                                 else  begin
                                    // ldc(i|l)(f|d)
                                    falu_input[55:40] <= 16'b0000000000000000 ; 
                                    falu_input[39:24] <= rbus_data ; 
                                    falu_input[23:0] <= 24'b000000000000000000000000 ; 
                                 end 
                                 state <= sq_fprun ; 
                              end 
                           end 
                        end
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               sq_fpir1 :
                        //*  addr = addr_indirect
                        //*  RD+
                        begin
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_fpir1l;
                            state <= sq_wbread;
                        end
                        
               sq_fpir1l :
                        begin
                           if ((ir[8]) == 1'b1)  begin
                              // state is reachable only for ldexp and ldc(i|l)(f|d); ir(8) = 1 means ldexp
                              state <= sq_fprun ; // ldexp
                              falus_input[55:40] <= datain ; // FIXME, it does not really make sense to put the input value here?
                           end
                           else  begin
                              falu_input[23:0] <= 24'b000000000000000000000000 ; 
                              if ((fps[6]) == 1'b1 & ir[5:0] != 6'b010111) begin
                                 // ldc(i|l)(f|d) mode 2, reg 7 : then only 1 word to be read
                                 falu_input[55:40] <= datain ; 
                                 addr_indirect <= addr_indirect + 16'd2 ; 
                                 state <= sq_fpir2 ; 
                              end
                              else begin
                                 falu_input[55:40] <= 16'b0000000000000000 ; 
                                 falu_input[39:24] <= datain ; 
                                 state <= sq_fprun ; 
                              end 
                           end 
                        end
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               //*  addr = addr_indirect
               //*  RD+
               sq_fpir2 :
                        begin
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_fpir2l;
                            state <= sq_wbread;
                        end
               sq_fpir2l :
                        begin
                           falu_input[39:24] <= datain ; 
                           state <= sq_fprun ; 
                        end
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               //*  addr = addr_indirect
               //*  RD+
               sq_fpr1 :
                        begin
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_fpr1l;
                            state <= sq_wbread;
                        end
               sq_fpr1l :
                        begin
                           if (datain[15:7] == 9'b100000000 & (fps[11]) == 1'b1 & (fps[14]) == 1'b0)  begin
                              // do we need to trigger the fiuv trap for -0, undefined variable?
                              state <= sq_fptrap ; // cause trap
                              fps[15] <= 1'b1 ; // set error flag
                              fec <= 4'b1100 ; // fiuv code
                           end
                           else  begin
                              if (datain[15:7] == 9'b100000000 & (fps[11]) == 1'b1) begin
                                 // if interrupts are disabled, we still signal the error... FIXME, is this required at all?
                                 fps[15] <= 1'b1 ; // set error flag
                                 fec <= 4'b1100 ; // fiuv code
                              end 
                              falu_input[63:48] <= datain ; 
                              addr_indirect <= addr_indirect + 16'd2 ; 
                              if (ir[5:0] == 6'b010111)  begin
                                 // mode 2, reg 7 : then only 1 word to be loaded
                                 falu_input[47:0] <= 48'b000000000000000000000000000000000000000000000000 ; 
                                 state <= sq_fprun ; 
                              end
                              else  state <= sq_fpr2 ; 
                           end 
                        end
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               //*  addr = addr_indirect
               //*  RD+
               sq_fpr2 :
                        begin
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_fpr2l;
                            state <= sq_wbread;
                        end
                        
               sq_fpr2l :
                        begin
                           falu_input[47:32] <= datain ; 
                           if ((fps[7]) == 1'b1 | ((fps[7]) == 1'b0 & ir[11:8] == 4'b1111))   begin
                              // if mode is d
                              // or if mode is f, and the insn is ldcfd
                              // then we need to read the next two words
                              state <= sq_fpr3 ; 
                              addr_indirect <= addr_indirect + 16'd2 ; 
                           end
                           else  begin
                              falu_input[31:0] <= 32'b00000000000000000000000000000000 ; // if mode is f, insn is not ldcfd, zero out the low 32 bits of the input
                              state <= sq_fprun ; 
                           end 
                        end
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               //*  addr = addr_indirect
               //*  RD+
               sq_fpr3 :
                        begin
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_fpr3l;
                            state <= sq_wbread;
                        end
               sq_fpr3l :
                        begin
                           falu_input[31:16] <= datain ; 
                           addr_indirect <= addr_indirect + 16'd2 ; 
                           state <= sq_fpr4 ; 
                        end
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               //*  addr = addr_indirect
               //*  RD+
               sq_fpr4 :
                        begin
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_fpr4l;
                            state <= sq_wbread;
                        end
               sq_fpr4l :
                        begin
                           falu_input[15:0] <= datain ; 
                           state <= sq_fprun ; 
                        end
               sq_fpwr :
                        begin
                           fbus_d <= falu_output ; 
                           fps[4] <= 1'b0 ; // this appears to be needed to pass zkdl; always setting the bit to zero makes one of the other tests complain.
                           fps[3:0] <= falu_fps ; 
                           fbus_waddr <= {1'b0, ir[7:6]} ; 
                           fbus_we <= 1'b1 ; 
                           state <= sq_ifetch ; 
                              // mod with even ac, need to store ac+1
                           if (ir[11:8] == 4'b0011 & (ir[6]) == 1'b0)  state <= sq_fpwr1 ; 
                        end
               sq_fpwr1 :
                        begin
                           state <= sq_fpwr2 ; 
                        end
               sq_fpwr2 :
                        begin
                           fbus_d <= falu_output2 ; 
                           fbus_waddr <= {1'b0, ir[7], 1'b1} ; 
                           fbus_we <= 1'b1 ; 
                           state <= sq_ifetch ; 
                        end
               sq_fpd0 :
                        begin
                           fps[4] <= 1'b0 ; // this appears to be needed to pass zkdl; always setting the bit to zero makes one of the other tests complain.
                           fps[3:0] <= falu_fps ; 
                           if (ir_fpsop2 == 1'b1 & ir[7:6] == 2'b01)  begin
                              // tst(f/d)
                              state <= sq_ifetch ; 
                           end
                           else if (ir[2:1] != 2'b11)  begin
                              fbus_d <= falu_output ; 
                              fbus_waddr <= ir[2:0] ; 
                              fbus_we <= 1'b1 ; 
                           end 
                           state <= sq_ifetch ; 
                        end
               sq_fpiwr :
                        begin
                           if (ir[2:0] != 3'b111) begin
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[2:0]} ; 
                              rbus_d <= falu_output[63:48] ; 
                              rbus_we <= 1'b1 ; 
                           end
                           else   r7 <= falu_output[63:48] ; // FIXME, check what real pdp's do?
                           fps[4] <= 1'b0 ; // this appears to be needed to pass zkdl; always setting the bit to zero makes one of the other tests complain.
                           fps[3:0] <= falu_fps ; 
                           psw[3:0] <= falu_fps ; 
                           state <= sq_ifetch ; 
                        end
               sq_fpiww :
                        begin
                           addr_indirect <= dest_addr ; 
                           fps[4] <= 1'b0 ; // this appears to be needed to pass zkdl; always setting the bit to zero makes one of the other tests complain.
                           fps[3:0] <= falu_fps ; 
                           psw[3:0] <= falu_fps ; 
                           state <= sq_fpiw1 ; 
                        end
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               //*  addr = addr_indirect
               //*  dataout=falu_output
               //*  WR+
               sq_fpiw1 :
                        begin
                           wbm_adr_o <= addr_indirect;
                           wbm_dat_o <= falu_output[63:48];
                           nextstate <= sq_fpiw1l; 
                           state <= sq_wbwrite;
                        end
               sq_fpiw1l :
                        begin
                              // stc(f|d)(i|l) mode 2, reg 7 : then only 1 word to be written
                              // stc(f|d)(i|l), short integer mode
                              // stexp insn
                           if (ir[5:0] == 6'b010111 | (fps[6]) == 1'b0 | ir[11:8] == 4'b1010)  state <= sq_ifetch ; 
                           else  begin
                              addr_indirect <= addr_indirect + 16'd2 ; 
                              state <= sq_fpiw2 ; 
                           end 
                        end
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               //*  addr = addr_indirect
               //*  dataout=falu_output
               //*  WR+
               sq_fpiw2 :
                        begin
                           wbm_adr_o <= addr_indirect;
                           wbm_dat_o <= falu_output[47:32];
                           nextstate <= sq_ifetch; 
                           state <= sq_wbwrite;
                        end
                        
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               sq_fpww :
                        begin
                           addr_indirect <= dest_addr ; 
                           fps[4] <= 1'b0 ; // this appears to be needed to pass zkdl; always setting the bit to zero makes one of the other tests complain.
                           fps[3:0] <= falu_fps ; 
                              // tst(f/d)
                           if (ir_fpsop2 == 1'b1 & ir[7:6] == 2'b01)  state <= sq_ifetch ; 
                           else                              state <= sq_fpw1 ; 
                        end
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               //*  addr = addr_indirect
               //*  dataout=falu_output
               //*  WR+
               sq_fpw1 :
                        begin
                           wbm_adr_o <= addr_indirect;
                           wbm_dat_o <= falu_output[63:48];
                           nextstate <= sq_fpw1l; 
                           state <= sq_wbwrite;
                        end
               sq_fpw1l :
                        begin
                              // mode 2, reg 7 : then only 1 word to be written
                           if (ir[5:0] == 6'b010111)  state <= sq_ifetch ; 
                           else
                           begin
                              addr_indirect <= addr_indirect + 16'd2 ; 
                              state <= sq_fpw2 ; 
                           end 
                        end
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               //*  addr = addr_indirect
               //*  dataout=falu_output
               //*  WR+
               sq_fpw2 :
                        begin
                           wbm_adr_o <= addr_indirect;
                           wbm_dat_o <= falu_output[47:32];
                           nextstate <= sq_fpw2l; 
                           state <= sq_wbwrite;
                        end
               sq_fpw2l :
                        begin
                           if (((fps[7]) == 1'b1 & ir[11:8] != 4'b1100) | ((fps[7]) == 1'b0 & ir[11:8] == 4'b1100))  begin
                              // reverse sense of fps D bit when insn is stc(f|d)(d|f)
                              state <= sq_fpw3 ; 
                              addr_indirect <= addr_indirect + 16'd2 ; 
                           end
                           else   state <= sq_ifetch ; 
                        end
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               //*  addr = addr_indirect
               //*  dataout=falu_output
               //*  WR+
               sq_fpw3 :
                        begin
                           wbm_adr_o <= addr_indirect;
                           wbm_dat_o <= falu_output[31:16];
                           nextstate <= sq_fpw3l; 
                           state <= sq_wbwrite;
                        end
               sq_fpw3l :
                        begin
                           addr_indirect <= addr_indirect + 16'd2 ; 
                           state <= sq_fpw4 ; 
                        end
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               //*  addr = addr_indirect
               //*  dataout=falu_output
               //*  WR+
               sq_fpw4 :
                        begin
                           wbm_adr_o <= addr_indirect;
                           wbm_dat_o <= falu_output[15:0];
                           nextstate <= sq_ifetch; 
                           state <= sq_wbwrite;
                        end
               //**********************************************************************************         
               //*
               //**********************************************************************************         
               sq_fprun :
                        begin
                           if (ir_fpao == 1'b1)  begin
                              if (ir_facfsrc == 1'b1)  falus_input <= fbus_o ; 
                              state <= sq_fprunao ; 
                              falu_load <= 1'b1 ; 
                              falu_state <= 0 ; 
                           end
                           else if (ir_fpsop2 == 1'b1)  begin
                              if (ir[5:3] == 3'b000)  state <= sq_fpd0 ; 
                              else                    state <= sq_fpww ; 
                           end
                           else                       state <= sq_ifetch ; 
                        end
               sq_fprunao :
                        begin
                           falu_state <= falu_state + 8'd1 ; 
                           falu_load <= 1'b0 ; 
                           if (falu_state > 160)  state <= sq_ifetch ; 
                           if (falu_done == 1'b1)  begin
                              falu_state <= 0 ; 
                              case (ir[11:8])
                                 4'b1000 :
                                          begin
                                             // st(f|d)
                                             if (ir[5:3] == 3'b000)  state <= sq_fpd0 ; 
                                             else                    state <= sq_fpww ; 
                                          end
                                 4'b1010 :
                                          begin
                                             // stexp
                                             if (ir[5:3] == 3'b000)  state <= sq_fpiwr ; 
                                             else                    state <= sq_fpiww ; 
                                          end
                                 4'b1011 :
                                          begin
                                             // stc(f|d)(i|l)
                                             if (ir[5:3] == 3'b000)  state <= sq_fpiwr ; 
                                             else                    state <= sq_fpiww ; 
                                          end
                                 4'b1100 :
                                          begin
                                             // stc(f|d)(d|f)
                                             fbus_fd <= 1'b1 ; // enable full access to fp register bank
                                             if (ir[5:3] == 3'b000)  state <= sq_fpd0 ; 
                                             else                    state <= sq_fpww ; 
                                          end
                                 4'b1111 :
                                          begin
                                             // ldc(d|f)(f|d)
                                             fbus_fd <= 1'b1 ; // enable full access to fp register bank
                                             state <= sq_fpwr ; 
                                          end
                                 default :
                                          begin
                                             state <= sq_fpwr ; 
                                          end
                              endcase 
                           end 
                        end
               sq_tstset :
                        begin
                           rbus_waddr <= {pswmf[15:14], pswmf[11], 3'b000} ; 
                           rbus_d <= alu_input ; 
                           rbus_we <= 1'b1 ; 
                           state <= sq_store_alu_p ; 
                        end
               sq_wrtlck :
                        begin
                           rbus_ix <= 3'b000 ; 
                           state <= sq_wrtlcka ; 
                        end
               sq_wrtlcka :
                        begin
                           alu_input <= rbus_data ; 
                           state <= sq_store_alu_p ; 
                        end
               //****************************************
               //*   MARK         
               //****************************************
               sq_mark :
                        begin
                           r7 <= rbus_data ; 
                           rbus_ix <= 3'b110 ; 
                           state <= sq_marka ; 
                        end
               sq_marka :
                        begin
                           rbus_waddr <= {pswmf[15:14], pswmf[11], 3'b110} ; 
                           rbus_d <= rbus_data+16'd2 ; 
                           rbus_we <= 1'b1 ; 
                           state <= sq_markb ; 
                        end
               //*                        
               //*  addr = rbus_data
               //*  RD +
               sq_markb : 
                        begin
                            wbm_adr_o <= rbus_data;
                            nextstate <= sq_markbl;
                            state <= sq_wbread;
                        end
                            
               sq_markbl:
                        begin
                           nxmabort <= nxmabort_pend;
                           rbus_waddr <= {pswmf[15:14], pswmf[11], 3'b101} ; 
                           rbus_d <= datain ; 
                           rbus_we <= 1'b1 ; 
                           state <= sq_ifetch ; 
                        end
               //***********************************         
               //*   JSR
               //***********************************         
               // выбор регистра стека
               sq_jsr :
                        begin
                           rbus_ix <= 3'b110 ;    // регистр SP
                           state <= sq_jsra ; 
                        end
               // SP прочитан, уменьшаем его на 2
               sq_jsra :
                        begin
                           addr_indirect <= rbus_data-16'd2 ;   // SP-2 - адрес для сохранения регистра
                           rbus_waddr <= {pswmf[15:14], 4'b0110} ;  // адрес записи SP - текущий режим, набор регистров 0 
                           rbus_d <= rbus_data-16'd2 ;                  // SP <= SP-2
                           rbus_we <= 1'b1 ;                        // включаем запись 
                           sr1_srcd <= sr1_m2 ;            
                           rbus_ix <= ir[8:6] ;                     // регистр для сохранения
                           state <= sq_jsrb ; 
                        end
               // такт записи SP.
               //*  addr = addr_indirect - вычисленный ранее адрес ячейки стека
               //*  dataout = rbus_data - выбранный для сохранения регистр
               //*  wr +         
               sq_jsrb :
                        begin
                           wbm_adr_o <= addr_indirect;
                           wbm_dat_o <= rbus_data;
                           nextstate <= sq_jsrc ; 
                           state <= sq_wbwrite;
                        end
               // установка нового PC         
               sq_jsrc :
                        begin
                           nxmabort <= nxmabort_pend;
                           if (ir[8:6] != 3'b111)  begin
                              // для JSR Rx кроме PC
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[8:6]} ;  // адрес для записи регистра в файл
                              rbus_d <= r7 ;                                      // записываем текущий PC в выбранный регистр
                              rbus_we <= 1'b1 ; 
                           end 
                           
                           r7 <= dest_addr ;           // адрес подпрограммы
                           state <= sq_ifetch ; 
                        end
               //********************************************         
               //*    RTS
               //********************************************         
               sq_rts :
                        begin
                           addr_indirect <= rbus_data ;    // читаем SP и устанавливаем как адрес на шине
                           if (ir[2:0] != 3'b110) begin
                              // Для всех регистров кроме SP
                              rbus_waddr <= {pswmf[15:14], 4'b0110} ;  // адрес записи - SP, набор 0
                              rbus_d <= rbus_data+16'd2 ;                  // SP=SP+2
                              rbus_we <= 1'b1 ;                        
                           end
                           sr1_dstd <= sr1_p2;         
                           rbus_ix <= ir[2:0] ;    // регистр-приемник
                           state <= sq_rtsa ; 
                        end
               // RTS: - чтение сохраненных данных из стека
               //*  addr = addr_indirect - вычисленный ранее адрес ячейки стека
               //*  RD +         
               sq_rtsa : 
                        begin
                           wbm_adr_o <= addr_indirect;
                           nextstate <= sq_rtsal;
                           state <= sq_wbread;
                        end   
               sq_rtsal :
                        begin
                           nxmabort <= nxmabort_pend;
                           if (ir[2:0] != 3'b111)  begin
                             // для всех кроме РС
                              r7 <= rbus_data ;    // восстанавливаем РС из регистра
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[2:0]} ; // перезаписывем регистр из ячейки стека
                              rbus_d <= datain ; 
                              rbus_we <= 1'b1 ; 
                           end
                           else   r7 <= datain ;  // для RTS PC
                           state <= sq_ifetch ; 
                        end
               //*******************************************************************************
               //*  Чтение первого операнда из регистра - R
               //*******************************************************************************
               sq_dst0 :
                        begin
                           alu_input <= rbus_data ; 
                           state <= dst_mode_processor ; 
                           rbus_cpu_mode <= pswmf[15:14] ; 
                        end
               //***************************************************************************************
               //*  Чтение второго операнда из регистра - R
               //***************************************************************************************
               sq_src0 : 
                        begin
                              alus_input <= rbus_data ; // получаем содержимое регистра-источника
                              rbus_ix <= ir[2:0] ;      // теперь адресуем регистр=приемник
                              state <= src_mode_processor ;      // уходим на обработку режима адресации приемника
                        end
               //****************************************************************************************
               //* Чтение первого операнда при косвенной адресации (R)
               //****************************************************************************************
               //*  addr = rbus_data - содержимое регистра-приемника
               //*  rd +
               sq_dst1: 
                        begin
                            finalreference <= 1'b1;
                            dstfreference <= 1'b1;
                            wbm_adr_o <= rbus_data;
                            nextstate <= sq_dst1l;
                            state <= sq_wbread;
                        end
               sq_dst1l :
                        begin
                           nxmabort <= nxmabort_pend;
                           dest_addr <= rbus_data ;     // адрес для записи результата
                           alu_input <= datain ;   // операнд - на вход АЛУ
                           state <= dst_mode_processor ; 
                        end
               //*********************************************************
               // Чтение второго операнда при косвенной адресации (R)
               //*********************************************************
               //*  addr = rbus_data - содержимое регистра-источника
               //*  rd +
               sq_src1: 
                        begin
                            finalreference <= 1'b1;
                            wbm_adr_o <= rbus_data;
                            nextstate <= sq_src1l;
                            state <= sq_wbread;
                        end
                
               sq_src1l :  // @R
                        begin
                           nxmabort <= nxmabort_pend;
                           alus_input <= datain ;          // вход АЛУ - (R), читается из памяти
                           state <= src_mode_processor ;   // переходим к обработчику результата 
                           rbus_ix <= ir[2:0] ;            // устанавливаем регистр результата
                        end
               //****************************************************************
               // Чтение первого операнда при автоинкрементной адресации (R)+
               //****************************************************************
               //*  addr = rbus_data - содержимое регистра-приемника
               //*  rd +
               sq_dst2: 
                        begin
                            finalreference <= 1'b1;
                            dstfreference <= 1'b1;
                            wbm_adr_o <= rbus_data;
                            nextstate <= sq_dst2l;
                            state <= sq_wbread;
                        end
               sq_dst2l :
                        begin
                           nxmabort <= nxmabort_pend;
                           dest_addr <= rbus_data ; 
                           alu_input <= datain ; 
                           state <= dst_mode_processor ; 
                           sr1_dstd <= sr1_pv ; 
                           if (ir[2:0] != 3'b111)  begin
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[2:0]} ; 
                              rbus_d <= rbus_data_pv ; 
                              rbus_we <= 1'b1 ; 
                           end
                           else  r7 <= rbus_data+16'd2 ; 
                        end
               //***************************************************************
               // Чтение второго операнда при автоинкрементной адресации (R)+
               //***************************************************************
               //*  addr = rbus_data - содержимое регистра-источника
               //*  rd +
               sq_src2: 
                        begin
                            finalreference <= 1'b1;
                            wbm_adr_o <= rbus_data;
                            nextstate <= sq_src2l;
                            state <= sq_wbread;
                        end
               sq_src2l :   // (R)+
                        begin
                           nxmabort <= nxmabort_pend;
                           alus_input <= datain ;   // вход АЛУ - (R), читается из памяти

                                // Для двухоперандных, если регистры источника и приемника совпадают, и приемник - не РС
                              // вставляем такт ожидания на обновление содержимого регистра
                           if ((ir_dop == 1'b1) & (ir[8:6] == ir[2:0]) & (ir[2:0] != 3'b111))   state <= sq_src2w ;
                              // для остальных случаев - сразу переходим к обработке результата   
                           else                                                           state <= src_mode_processor ; 

                           rbus_ix <= ir[2:0] ;   // регистр-приемник
                           sr1_srcd <= sr1_pv ; 
                           if (ir[8:6] != 3'b111)  begin
                              // регистр - не РС
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[8:6]} ;  // формируем адрес регистра-источника
                              rbus_d <= rbus_data_pv ;           // обновляем содержимое регистра в соответствии с шагом автоинкремента
                              rbus_we <= 1'b1 ;                  // разрешаем запись в регистровый файл
                           end
                              // регистр - РС
                           else      r7 <= rbus_data+16'd2 ;   // просто делаем приращение регистра
                        end
               //**********************************************
               //* Такт ожидания записи при обновлении регистра
               //**********************************************
               sq_src2w :
                        begin
                           state <= src_mode_processor ; 
                        end
               //*****************************************************************************************
               // Формирование адреса первого операнда при косвенной автоинкрементной адресации @(R)+
               //*****************************************************************************************
               //*  addr = rbus_data - содержимое регистра-источника
               //*  rd +
               sq_dst3: 
                        begin
                            wbm_adr_o <= rbus_data;
                            nextstate <= sq_dst3l;
                            state <= sq_wbread;
                        end
               sq_dst3l :
                        begin
                           nxmabort <= nxmabort_pend;
                           addr_indirect <= datain ; 
                           sr1_dstd <= sr1_p2 ; 
                           if (ir[2:0] != 3'b111)  begin
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[2:0]} ; 
                              rbus_d <= rbus_data+16'd2 ; 
                              rbus_we <= 1'b1 ; 
                           end
                           else   r7 <= rbus_data+16'd2 ; 
                           state <= sq_dst3a ; 
                        end
               //**************************************************************************
               // Чтение первого операнда при косвенной автоинкрементной адресации @(R)+
               //**************************************************************************
               //*  addr = addr_indirect
               //*  rd +
               sq_dst3a: 
                        begin
                            finalreference <= 1'b1;
                            dstfreference <= 1'b1;
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_dst3al;
                            state <= sq_wbread;
                        end
               sq_dst3al :
                        begin
                           nxmabort <= nxmabort_pend;
                           dest_addr <= addr_indirect ; 
                           alu_input <= datain ; 
                           state <= dst_mode_processor ; 
                        end
               //***************************************************************************************
               // Формирование адреса первого  операнда при косвенной автоинкрементной адресации @(R)+
               //***************************************************************************************
               //*  addr = rbus_data - содержимое регистра-источника
               //*  rd +
               sq_src3: 
                        begin
                            wbm_adr_o <= rbus_data;
                            nextstate <= sq_src3l;
                            state <= sq_wbread;
                        end
               sq_src3l :     
                        begin
                           nxmabort <= nxmabort_pend;
                           addr_indirect <= datain ;   // адрес операнда из регистра 
                           rbus_ix <= ir[2:0] ;        // регистр-приемник
                           sr1_srcd <= sr1_p2 ; 
                           if (ir[8:6] != 3'b111)    begin
                              // для всех кроме РС
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[8:6]} ;   // адрес записи
                              rbus_d <= rbus_data+16'd2 ;                              // +2 к содержимому регистра
                              rbus_we <= 1'b1 ;                    // включаем запись регистрового файла
                           end
                              // обновление РС
                           else    r7 <= rbus_data+16'd2 ;             
                           state <= sq_src3a ; 
                        end
               //**************************************************************************
               // Чтение второго операнда при косвенной автоинкрементной адресации @(R)+
               //**************************************************************************
               //*  addr = addr_indirect
               //*  rd +
               sq_src3a:
                        begin
                            finalreference <= 1'b1;
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_src3al;
                            state <= sq_wbread;
                        end
               sq_src3al :
                        begin
                           nxmabort <= nxmabort_pend;
                           alus_input <= datain ;    // читаем содежимое памяти по сформированному адресу
                           state <= src_mode_processor ; 
                        end
               //**************************************************************************
               //* Чтение первого операнда при автодекрементной адресации -(R)
               //**************************************************************************
               //*  addr = rbus_data_mv - содержимое регистра-источника, уменьшенное на значение декремена
               //*  rd +
               sq_dst4: 
                        begin
                            finalreference <= 1'b1;
                            dstfreference <= 1'b1;
                            wbm_adr_o <= rbus_data_mv;  // выставляем адрес - регистр-декремент
                            nextstate <= sq_dst4l;
                            state <= sq_wbread;         // читаем данные по выбранному адресу
                        end
               sq_dst4l :
                        begin
                           nxmabort <= nxmabort_pend;
                           dest_addr <= rbus_data_mv ;  // адрес для записи результата
                           alu_input <= datain ; 
                           state <= dst_mode_processor ; 
                           sr1_dstd <= sr1_mv ; 
                           if (ir[2:0] != 3'b111)  begin
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[2:0]} ; 
                              rbus_d <= rbus_data_mv ; 
                              rbus_we <= 1'b1 ; 
                           end
                           else  r7 <= rbus_data-16'd2 ; 
                        end
               //***************************************************************
               // Чтение второго операнда при автодекрементной адресации -(R)
               //***************************************************************
               //*  addr = rbus_data_mv - содержимое регистра-источника, уменьшенное на значение декремена
               //*  rd +
               sq_src4: 
                        begin
                            finalreference <= 1'b1;
                            wbm_adr_o <= rbus_data_mv;
                            nextstate <= sq_src4l;
                            state <= sq_wbread;
                        end
               sq_src4l : // -(R)
                        begin
                           nxmabort <= nxmabort_pend;
                           alus_input <= datain ;              // читаем из памяти операнд
                                // Для двухоперандных, если регистры источника и приемника совпадают, и приемник - не РС
                              // вставляем такт ожидания на обновление содержимого регистра
                           if (ir_dop == 1'b1 & ir[8:6] == ir[2:0] & ir[2:0] != 3'b111)  state <= sq_src4w ; 
                              // для остальных случаев - сразу переходим к обработке результата   
                           else                             state <= src_mode_processor ; 
                           
                           rbus_ix <= ir[2:0] ;     // регистр приемника
                           sr1_srcd <= sr1_mv ; 
                           if (ir[8:6] != 3'b111)    begin
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[8:6]} ;   // формируем адрес регистра-приемника
                              rbus_d <= rbus_data_mv ;                             // значение регистра с учетом коррекции декремента
                              rbus_we <= 1'b1 ;           // включаем запись в регистровый файл
                           end
                           // обновление РС
                           else  r7 <= rbus_data-16'd2 ; 
                        end
               //*************************************************
               //* Такт ожидания записи при обновлении регистра
               //*************************************************
               sq_src4w :
                        begin
                           state <= src_mode_processor ; 
                        end
               //*************************************************************************************
               // Формирование адреса первого операнда при косвенной автодекрементной адресации @-(R)
               //*************************************************************************************
               //*  addr = rbus_data-2
               //*  rd +
               sq_dst5: 
                        begin
                            wbm_adr_o <= rbus_data-16'd2;
                            nextstate <= sq_dst5l;
                            state <= sq_wbread;
                        end
               sq_dst5l :
                        begin
                           nxmabort <= nxmabort_pend;
                           addr_indirect <= datain ; 
                           sr1_dstd <= sr1_m2 ; 
                           if (ir[2:0] != 3'b111)   begin
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[2:0]} ; 
                              rbus_d <= rbus_data-16'd2 ; 
                              rbus_we <= 1'b1 ; 
                           end
                           else  r7 <= rbus_data-16'd2 ; 
                           state <= sq_dst5a ; 
                        end
               //***************************************************************************
               //* Чтение второго операнда при косвенной автодекрементной адресации @-(R)
               //***************************************************************************
               //*  addr = addr_indirect
               //*  rd +
               sq_dst5a: begin
                            finalreference <= 1'b1;
                            dstfreference <= 1'b1;
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_dst5al;
                            state <= sq_wbread;
                        end
               sq_dst5al :
                        begin
                           nxmabort <= nxmabort_pend;
                           dest_addr <= addr_indirect ; 
                           alu_input <= datain ; 
                           state <= dst_mode_processor ; 
                        end
               //***************************************************************************************
               // Формирование адреса второго операнда при косвенной автодекрементной адресации @-(R)
               //***************************************************************************************
               //*  addr = rbus_data-2
               //*  rd +
               sq_src5: 
                        begin
                            wbm_adr_o <= rbus_data-16'd2;
                            nextstate <= sq_src5l;
                            state <= sq_wbread;
                        end
               sq_src5l :       // @-(R)
                        begin
                           nxmabort <= nxmabort_pend;
                           addr_indirect <= datain ;   // адрес, прочитанный из памяти
                           rbus_ix <= ir[2:0] ;        // регистр-приемник
                           sr1_srcd <= sr1_m2 ; 
                           if (ir[8:6] != 3'b111)   begin
                              rbus_waddr <= {pswmf[15:14], pswmf[11], ir[8:6]} ; 
                              rbus_d <= rbus_data-16'd2 ;   // коррекция декремента
                              rbus_we <= 1'b1 ; 
                           end
                           else r7 <= rbus_data-16'd2 ; 
                           state <= sq_src5a ; 
                        end
               //*******************************************************************
               // Чтение операнда при косвенной автодекрементной адресации @-(R)
               //*******************************************************************
               //*  addr = addr_indirect
               //*  rd +
               sq_src5a: 
                        begin
                            finalreference <= 1'b1;
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_src5al;
                            state <= sq_wbread;
                        end
               sq_src5al :
                        begin
                           nxmabort <= nxmabort_pend;
                           alus_input <= datain ; 
                           state <= src_mode_processor ; 
                        end
               //********************************************************************
               //* Формирование адреса первого операнда при индексной адресации X(R)
               //********************************************************************
               //*  addr = R7
               //*  rd +
               sq_dst6: 
                        begin
                            wbm_adr_o <= r7;
                            nextstate <= sq_dst6l;
                            state <= sq_wbread;
                        end
               sq_dst6l :
                        begin
                           nxmabort <= nxmabort_pend;
                           r7 <= r7+16'd2 ; 
                           if (ir[2:0] == 3'b111)  addr_indirect <= datain + rbus_data + 16'd2 ; 
                           else                    addr_indirect <= datain + rbus_data ; 
                           state <= sq_dst6a ; 
                        end
               //*******************************************************************
               // Чтение первого операнда при индексной адресации X(R)
               //*******************************************************************
               //*  addr = addr_indirect
               //*  rd +
               sq_dst6a: 
                        begin
                            finalreference <= 1'b1;
                            dstfreference <= 1'b1;
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_dst6al;
                            state <= sq_wbread;
                        end
               sq_dst6al :
                        begin
                           nxmabort <= nxmabort_pend;
                           dest_addr <= addr_indirect ; 
                           alu_input <= datain ; 
                           state <= dst_mode_processor ; 
                        end
               //********************************************************************
               //* Формирование адреса второго операнда при индексной адресации X(R)
               //********************************************************************
               //*  addr = R7
               //*  rd +
               sq_src6: 
                        begin
                            wbm_adr_o <= r7;
                            nextstate <= sq_src6l;
                            state <= sq_wbread;
                        end
               sq_src6l :     // X(R)
                        begin
                           nxmabort <= nxmabort_pend;
                           r7 <= r7+16'd2 ;   // продвигаем РС для пропуска адреса
                           // формирование адреса операнда
                           if (ir[8:6] == 3'b111)  addr_indirect <= datain + rbus_data + 16'd2;  // для адресации через РС
                           else                    addr_indirect <= datain + rbus_data ;     // для адресации через остальные регистры
                           state <= sq_src6a ; 
                        end
               //*******************************************************************
               // Чтение второго операнда при индексной адресации X(R)
               //*******************************************************************
               //*  addr = addr_indirect
               //*  rd +
               sq_src6a: 
                        begin
                            finalreference <= 1'b1;
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_src6al;
                            state <= sq_wbread;
                        end
               sq_src6al :
                        begin
                           nxmabort <= nxmabort_pend;
                           alus_input <= datain ;    // аргумент из памяти
                           state <= src_mode_processor ;  
                           rbus_ix <= ir[2:0] ;      // регистр-приемник
                        end
               //***************************************************************************************************
               //* Формирование первого адреса (индекса) первого операнда при косвенной индексной адресации @X(R)
               //***************************************************************************************************
               //*  addr = R7
               //*  rd +
               sq_dst7: 
                        begin
                            wbm_adr_o <= r7;
                            nextstate <= sq_dst7l;
                            state <= sq_wbread;
                        end
               sq_dst7l :
                        begin
                           nxmabort <= nxmabort_pend;
                           r7 <= r7+16'd2 ; 
                           if (ir[2:0] == 3'b111)  addr_indirect <= datain + rbus_data + 16'd2 ; 
                           else                    addr_indirect <= datain + rbus_data ; 
                           state <= sq_dst7a ; 
                        end
               //*********************************************************************************
               // Чтение первого адреса первого операнда при косвенной индексной адресации @X(R)
               //*********************************************************************************
               //*  addr = addr_indirect
               //*  rd +
               sq_dst7a: 
                        begin
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_dst7al;
                            state <= sq_wbread;
                        end
               sq_dst7al :
                        begin
                           nxmabort <= nxmabort_pend;
                           addr_indirect <= datain ; 
                           state <= sq_dst7b ; 
                        end
               //*****************************************************************************************
               //* Формирование второго адреса первого операнда при косвенной индексной адресации @X(R)
               //*****************************************************************************************
               //*  addr = addr_indirect
               //*  rd +
               sq_dst7b: 
                        begin
                            finalreference <= 1'b1;
                            dstfreference <= 1'b1;
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_dst7bl;
                            state <= sq_wbread;
                        end
               sq_dst7bl :
                        begin
                           nxmabort <= nxmabort_pend;
                           dest_addr <= addr_indirect ; 
                           alu_input <= datain ; 
                           state <= dst_mode_processor ; 
                           rbus_ix <= 3'b110 ; 
                        end
               //***************************************************************************************************
               //* Формирование первого адреса (индекса) второго операнда при косвенной индексной адресации @X(R)
               //***************************************************************************************************
               //*  addr = R7
               //*  rd +
               sq_src7: 
                        begin
                            wbm_adr_o <= r7;
                            nextstate <= sq_src7l;
                            state <= sq_wbread;
                        end
               sq_src7l :    // @X(R)
                        begin
                           nxmabort <= nxmabort_pend;
                           r7 <= r7+16'd2 ; 
                           if (ir[8:6] == 3'b111)   addr_indirect <= datain + rbus_data + 16'd2; 
                           else                     addr_indirect <= datain + rbus_data ; 
                           state <= sq_src7a ; 
                        end
               //*********************************************************************************
               // Чтение первого адреса второго операнда при косвенной индексной адресации @X(R)
               //*********************************************************************************
               //*  addr = addr_indirect
               //*  rd +
               sq_src7a: 
                        begin
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_src7al;
                            state <= sq_wbread;
                        end
               sq_src7al :
                        begin
                           nxmabort <= nxmabort_pend;
                           addr_indirect <= datain ; 
                           state <= sq_src7b ; 
                        end
               //*****************************************************************************************
               //* Формирование второго адреса второго операнда при косвенной индексной адресации @X(R)
               //*****************************************************************************************
               //*  addr = addr_indirect
               //*  rd +
               sq_src7b: 
                        begin
                            finalreference <= 1'b1;
                            wbm_adr_o <= addr_indirect;
                            nextstate <= sq_src7bl;
                            state <= sq_wbread;
                        end
               sq_src7bl :
                        begin
                           nxmabort <= nxmabort_pend;
                           alus_input <= datain ;     // операнд подаем на АЛУ
                           state <= src_mode_processor ; 
                           rbus_ix <= ir[2:0] ;    // регистр-приемник
                        end
               //*********************************************************************
               //* Запись результата АЛУ в память - такт ожидания обмена с памятью
               //*********************************************************************
               sq_store_alu_p :
                        begin
                           state <= sq_store_alu_w ; 
                        end
               //*********************************************************************
               //* Запись результата АЛУ в память -
               //*********************************************************************
               //*  addr = dest_addr
               //*  dataout = alu_output
               //*  wr = ir_store
               sq_store_alu_w :
                        begin
                           finalreference <= 1'b1;
                           dstfreference <= 1'b1;
                           psw[3:0] <= alu_psw ;                 // обновленные флаги
                           
                           if (ir_store) begin
                              state <= sq_wbwrite;
                              wbm_adr_o <= dest_addr;
                              wbm_dat_o <= alu_output;
                              nextstate <= sq_ifetch ;    // переходим к обработке следующей инструкции
                           end   
                           else state <= sq_ifetch;
                           
                           // команда STST - запись состояния FPU
                           if (ir[15:6] == 10'b1111000011)  begin   //  1 111 000 011
                              if (ir[5:0] != 6'b010111)   begin
                                 // не для (R7)+
                                 state <= sq_stststore ; 
                                 dest_addr <= dest_addr + 16'd2 ; 
                              end 
                           end 
                        end
               //**********************************************
               //*  Запись результата работы АЛУ в регистр
               //**********************************************
               sq_store_alu_r :
                        begin
                           // только в случае разрешения записи
                           if (ir_store == 1'b1)   begin
                              if (ir[2:0] != 3'b111)   begin
                                 // Все регистры кроме PC
                                 // формирование адреса записи в регистровом файле
                                 if (ir_mtpi == 1'b1 | ir_mtpd == 1'b1) rbus_waddr <= {pswmf[13:12], pswmf[11], ir[2:0]} ; // адресация предыдущего режима - для MTP*
                                 else                                   rbus_waddr <= {pswmf[15:14], pswmf[11], ir[2:0]} ; // адресация текущуго режима
                                 // расширение знака для команды MOVB
                                 if (ir[15:12] == 4'b1001)      rbus_d <= alu_output_signext ; 
                                    // для остальных байтовых команд копируем старший байт без изменений
                                 else if (ir_byte == 1'b1)      rbus_d <= {alu_input[15:8], alu_output[7:0]} ; 
                                    // для словных команд записываем в регистр все 16 бит результата
                                 else                           rbus_d <= alu_output ; 
                                 rbus_we <= 1'b1 ; 
                              end
                              
                              // Для регистра РС - просто записываем результат в регистр
                              else                                 r7 <= alu_output ; 
                           end 
                           psw[3:0] <= alu_psw ;    // обновляем флаги PSW
                           state <= sq_ifetch ;  // переходим к обработке следующей инструкции
                        end

               //**********************************************
               //*  Чтение данных с общей шины - подготовка
               //**********************************************
               sq_wbread:  
                     begin
                      if (rd_inhibit) state <= nextstate;
                      else begin
                        id <= (ir_mfpi & (psw[15:12] == 4'b1111)) ? 1'b1 : 
                              (ir_mfpi & (cp_latch|cp_req) ) ? 1'b0 : 
                              (ir_mfpd & (cp_latch|cp_req) ) ? 1'b1 : id_latch;
                               
                        cp <= cp_latch|cp_req;
                      
                        bus_timer <= 6'd`busdelay;   // взводим таймер ожидания
                        wbm_stb_o <= 1'b1;
                        wbm_we_o <= 1'b0;
                        wbm_sel_o <= 2'b11;
                        state <= sq_wbread_wait;
                      end   
                     end 
               //**************************************************
               //*  Чтение данных с общей шины - ожидание ответа
               //**************************************************
               sq_wbread_wait:  
                   begin
                     bus_timer <= bus_timer - 1'b1;
                     if (bus_timer == 6'd0) begin
                        id <= 1'b0;
                       cp <= 1'b0;
                       state <= nextstate;
                       nxmabort_pend <= 1'b1;
                       wbm_stb_o <= 1'b0;
                     end 
                     if (oddabort) begin
                        trap_vector <= 9'o004 ;    // Выставляем вектор 4
                        state <= sq_trap ;      // переходим к обычной обработке прерывания
                     end
                                 
                     if (wbm_ack_i) begin
                        wbm_stb_o <= 1'b0;
                        state <= nextstate;
                        finalreference <= 1'b0;
                        id <= 1'b0;
                          cp <= 1'b0;
                        if (ir_byte == 0)               datain <= wbm_dat_i;
                        else if (wbm_adr_o[0] == 1'b0)  datain <= wbm_dat_i;
                        else                            datain <= {wbm_dat_i[15:8], wbm_dat_i[15:8]};
                     end
                   end  
               //**********************************************
               //*  Вывод данных на общую шину - подготовка
               //**********************************************
               sq_wbwrite:  begin
                      // Активация детектора желтой границы стека - это делается только при попытке записи
                      if (yellow_stack_event) begin
                         yellow_stack_event <= 1'b0;
                         yellow_stack_latch <= 1'b1;
                      end    
                      bus_timer <= 6'd`busdelay;   // взводим таймер ожидания
                      wbm_stb_o <= 1'b1;
                      wbm_we_o <= 1'b1;
                      
                      id <=  (ir_mtpi & cp_latch ) ? 1'b0 : 
                             (ir_mtpd & cp_latch ) ? 1'b1 : id_latch;
                             
                      cp <= cp_latch;
                      state <= sq_wbwrite_wait;
                      if (dw8 == 1'b0) wbm_sel_o <= 2'b11;
                      else begin
                        if (wbm_adr_o[0] == 1'b0) wbm_sel_o <= 2'b01;
                        else begin
                           wbm_sel_o <= 2'b10;
                           wbm_dat_o[15:8] <= wbm_dat_o[7:0];
                        end
                      end   
                     end 
               //**************************************************
               //*  Чтение данных с общей шины - ожидание ответа
               //**************************************************
               sq_wbwrite_wait:  begin
                     bus_timer <= bus_timer - 1'b1;
                     if (bus_timer == 6'd0) begin
                     // таймаут шины
                       state <= nextstate;
                         id <= 1'b0;
                       cp <= 1'b0;
                       nxmabort_pend <= 1'b1;
                       wbm_stb_o <= 1'b0;
                       wbm_we_o <= 1'b0;     // снимаем признак записи
                       finalreference <= 1'b0;
                       dstfreference <= 1'b0;
                     end  

                     // получен ответ с шины
                     if (wbm_ack_i) begin
                        wbm_stb_o <= 1'b0;    // снимаем строб 
                        wbm_we_o <= 1'b0;     // снимаем признак записи
                        finalreference <= 1'b0;
                        dstfreference <= 1'b0;
                        id <= 1'b0;
                        cp <= 1'b0;
                        state <= nextstate;

                        // режим прямой записи PSW - перезаписываем psw из внешнего источника
                        if (psw_in_we_even == 1'b1) begin
                           // младший байт
                              psw_delayedupdate_even <= 1'b1 ; 
                              psw_delayedupdate[7:0] <= psw_in[7:0] ; 
                        end 
                        if (psw_in_we_odd == 1'b1)   begin
                           // старший байт
                              psw_delayedupdate_odd <= 1'b1 ; 
                              psw_delayedupdate[15:8] <= psw_in[15:8] ; 
                        end 
                        
                     end
                   end  
                      
            endcase 
         end 
        //** конец блока секвенсеора******************************************************************************

      end  
   end 
//===============================================================================================================================================================================

//**********************************************
//*  АЛУ базового набор инструкций
//**********************************************

// Знакорасширенный выход АЛУ для байтовых операций
assign alu_output_signext = {alu_output[7], alu_output[7], alu_output[7], alu_output[7], alu_output[7], alu_output[7], alu_output[7], alu_output[7], alu_output[7:0]} ;
// 16-битный выход
reg[15:0] result; 
// 8-битный выход
reg[7:0] result8; 

always @(*) begin
   ir_byte <= 1'b0 ; 
   ir_store <= 1'b1 ; 
   //***********************************************
   //* Обработка однооперандных инструкций
   //***********************************************
   if (ir_sop == 1'b1)   begin
      case (ir[15:6])
         10'b0000000011 :
                  begin
                     // swab
                     result[15:8] = alu_input[7:0]; 
                     result[7:0] = alu_input[15:8]; 
                     alu_output <= result ; 
                     alu_psw[3] <= alu_input[15] ; 
                     if (alu_input[15:8] == 8'b00000000)  alu_psw[2] <= 1'b1 ; 
                     else                                 alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= 1'b0 ; 
                  end
         10'b0000101000 :
                  begin
                     // clr
                     result = 16'b0000000000000000; 
                     alu_output <= result ; 
                     alu_psw[3:0] <= 4'b0100 ; 
                  end
         10'b1000101000 :
                  begin
                     // clrb
                     ir_byte <= 1'b1 ; 
                     result = 16'b0000000000000000; 
                     alu_output <= result ; 
                     alu_psw[3:0] <= 4'b0100 ; 
                  end
         10'b0000101001 :
                  begin
                     // com
                     result = ~alu_input; 
                     alu_output <= result ; 
                     alu_psw[3] <= ~alu_input[15] ; 
                     if (~alu_input == 16'b0000000000000000)   alu_psw[2] <= 1'b1 ; 
                     else                                      alu_psw[2] <= 1'b0 ; 
                     alu_psw[1:0] <= 2'b01 ; 
                  end
         10'b1000101001 :
                  begin
                     // comb
                     ir_byte <= 1'b1 ; 
                     result = ~alu_input; 
                     alu_output <= result ; 
                     alu_psw[3] <= ~alu_input[7] ; 
                     if (~alu_input[7:0] == 8'b00000000)       alu_psw[2] <= 1'b1 ; 
                     else                                      alu_psw[2] <= 1'b0 ; 
                     alu_psw[1:0] <= 2'b01 ; 
                  end
         10'b0000101010 :
                  begin
                     // inc
                     result = alu_input + 16'd1; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)     alu_psw[2] <= 1'b1 ; 
                     else                                    alu_psw[2] <= 1'b0 ; 
                     if (alu_input == 16'b0111111111111111)  alu_psw[1] <= 1'b1 ; 
                     else                                    alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= psw[0] ; 
                  end
         10'b1000101010 :
                  begin
                     // incb
                     ir_byte <= 1'b1 ; 
                     result = alu_input + 16'd1; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[7] ; 
                     if (result[7:0] == 8'b00000000)       alu_psw[2] <= 1'b1 ; 
                     else                                  alu_psw[2] <= 1'b0 ; 
                     if (alu_input[7:0] == 8'b01111111)    alu_psw[1] <= 1'b1 ; 
                     else                                  alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= psw[0] ; 
                  end
         10'b0000101011 :
                  begin
                     // dec
                     result = alu_input - 16'd1; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000) alu_psw[2] <= 1'b1 ; 
                     else                                alu_psw[2] <= 1'b0 ; 
                     if (alu_input == 16'b1000000000000000)  alu_psw[1] <= 1'b1 ; 
                     else                                 alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= psw[0] ; 
                  end
         10'b1000101011 :
                  begin
                     // decb
                     ir_byte <= 1'b1 ; 
                     result = alu_input - 16'd1; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[7] ; 
                     if (result[7:0] == 8'b00000000)  alu_psw[2] <= 1'b1 ; 
                     else                             alu_psw[2] <= 1'b0 ; 
                     if (alu_input[7:0] == 8'b10000000)   alu_psw[1] <= 1'b1 ; 
                     else                                 alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= psw[0] ; 
                  end
         10'b0000101100 :
                  begin
                     // neg
                     result = (~alu_input) + 16'd1; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)  begin
                        alu_psw[2] <= 1'b1 ; 
                        alu_psw[0] <= 1'b0 ; 
                     end
                     else  begin
                        alu_psw[2] <= 1'b0 ; 
                        alu_psw[0] <= 1'b1 ; 
                     end 
                     if (result == 16'b1000000000000000)   alu_psw[1] <= 1'b1 ; 
                     else                                  alu_psw[1] <= 1'b0 ; 
                  end
         10'b1000101100 :
                  begin
                     // negb
                     ir_byte <= 1'b1 ; 
                     result = (~alu_input) + 16'd1; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[7] ; 
                     if (result[7:0] == 8'b00000000)   begin
                        alu_psw[2] <= 1'b1 ; 
                        alu_psw[0] <= 1'b0 ; 
                     end
                     else  begin
                        alu_psw[2] <= 1'b0 ; 
                        alu_psw[0] <= 1'b1 ; 
                     end 
                     if (result[7:0] == 8'b10000000)  begin
                        alu_psw[1] <= 1'b1 ; 
                     end
                     else   alu_psw[1] <= 1'b0 ; 
                  end
         10'b0000101101 :
                  begin
                     // adc
                     result = alu_input + psw[0]; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)   alu_psw[2] <= 1'b1 ; 
                     else                                  alu_psw[2] <= 1'b0 ; 
                     if (alu_input == 16'b0111111111111111 & (psw[0]) == 1'b1)    alu_psw[1] <= 1'b1 ; 
                     else                                                         alu_psw[1] <= 1'b0 ; 
                     if (alu_input == 16'b1111111111111111 & (psw[0]) == 1'b1)    alu_psw[0] <= 1'b1 ; 
                     else                                                         alu_psw[0] <= 1'b0 ; 
                  end
         10'b1000101101 :
                  begin
                     // adcb
                     ir_byte <= 1'b1 ; 
                     result = alu_input + psw[0]; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[7] ; 
                     if (result[7:0] == 8'b00000000)  alu_psw[2] <= 1'b1 ; 
                     else    alu_psw[2] <= 1'b0 ; 
                     if (alu_input[7:0] == 8'b01111111 & (psw[0]) == 1'b1)  alu_psw[1] <= 1'b1 ; 
                     else                                                   alu_psw[1] <= 1'b0 ; 
                     if (alu_input[7:0] == 8'b11111111 & (psw[0]) == 1'b1)  alu_psw[0] <= 1'b1 ; 
                     else                                                   alu_psw[0] <= 1'b0 ; 
                  end
         10'b0000101110 :
                  begin
                     // sbc
                     result = alu_input - psw[0]; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)  alu_psw[2] <= 1'b1 ; 
                     else                                 alu_psw[2] <= 1'b0 ; 
                     if (alu_input == 16'b1000000000000000 & (psw[0]) == 1'b1)    alu_psw[1] <= 1'b1 ; 
                     else                                                         alu_psw[1] <= 1'b0 ; 
                     if (alu_input == 16'b0000000000000000 & (psw[0]) == 1'b1)    alu_psw[0] <= 1'b1 ; 
                     else                                                         alu_psw[0] <= 1'b0 ; 
                  end
         10'b1000101110 :
                  begin
                     // sbcb
                     ir_byte <= 1'b1 ; 
                     result = alu_input - psw[0]; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[7] ; 
                     if (result[7:0] == 8'b00000000)  alu_psw[2] <= 1'b1 ; 
                     else                             alu_psw[2] <= 1'b0 ; 
                     if (alu_input[7:0] == 8'b10000000 & (psw[0]) == 1'b1)    alu_psw[1] <= 1'b1 ; 
                     else                                                     alu_psw[1] <= 1'b0 ; 
                     if (alu_input[7:0] == 8'b00000000 & (psw[0]) == 1'b1)    alu_psw[0] <= 1'b1 ; 
                     else                                                     alu_psw[0] <= 1'b0 ; 
                  end
         10'b0000101111 :
                  begin
                     // tst
                     result = alu_input; 
                     alu_output <= result ; 
                     ir_store <= 1'b0 ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)   alu_psw[2] <= 1'b1 ; 
                     else                                  alu_psw[2] <= 1'b0 ; 
                     alu_psw[1:0] <= 2'b00 ; 
                  end
         10'b1000101111 :
                  begin
                     // tstb
                     ir_byte <= 1'b1 ; 
                     result = alu_input; 
                     alu_output <= result ; 
                     ir_store <= 1'b0 ; 
                     alu_psw[3] <= result[7] ; 
                     if (result[7:0] == 8'b00000000)   alu_psw[2] <= 1'b1 ; 
                     else                              alu_psw[2] <= 1'b0 ; 
                     alu_psw[1:0] <= 2'b00 ; 
                  end
         10'b0000110000 :
                  begin
                     // ror
                     result = {psw[0], alu_input[15:1]}; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)  alu_psw[2] <= 1'b1 ; 
                     else                                 alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= alu_input[0] ^ result[15] ; 
                     alu_psw[0] <= alu_input[0] ; 
                  end
         10'b1000110000 :
                  begin
                     // rorb
                     ir_byte <= 1'b1 ; 
                     result8 = {psw[0], alu_input[7:1]}; 
                     alu_output[7:0] <= result8 ; 
                     alu_output[15:8] <= 8'bXXXXXXXX ; 
                     alu_psw[3] <= result8[7] ; 
                     if (result8 == 8'b00000000)  alu_psw[2] <= 1'b1 ; 
                     else                         alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= alu_input[0] ^ result8[7] ; 
                     alu_psw[0] <= alu_input[0] ; 
                  end
         10'b0000110001 :
                  begin
                     // rol
                     result = {alu_input[14:0], psw[0]}; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)  alu_psw[2] <= 1'b1 ; 
                     else                                 alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= alu_input[15] ^ result[15] ; 
                     alu_psw[0] <= alu_input[15] ; 
                  end
         10'b1000110001 :
                  begin
                     // rolb
                     ir_byte <= 1'b1 ; 
                     result8 = {alu_input[6:0], psw[0]}; 
                     alu_output[7:0] <= result8 ; 
                     alu_output[15:8] <= 8'bXXXXXXXX ; 
                     alu_psw[3] <= result8[7] ; 
                     if (result8 == 8'b00000000)  alu_psw[2] <= 1'b1 ; 
                     else                         alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= alu_input[7] ^ result8[7] ; 
                     alu_psw[0] <= alu_input[7] ; 
                  end
         10'b0000110010 :
                  begin
                     // asr
                     result = {alu_input[15], alu_input[15:1]}; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)   alu_psw[2] <= 1'b1 ; 
                     else                                  alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= alu_input[0] ^ result[15] ; 
                     alu_psw[0] <= alu_input[0] ; 
                  end
         10'b1000110010 :
                  begin
                     // asrb
                     ir_byte <= 1'b1 ; 
                     result8 = {alu_input[7], alu_input[7:1]}; 
                     alu_output[7:0] <= result8 ; 
                     alu_output[15:8] <= 8'bXXXXXXXX ; 
                     alu_psw[3] <= result8[7] ; 
                     if (result8 == 8'b00000000)  alu_psw[2] <= 1'b1 ; 
                     else                         alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= alu_input[0] ^ result8[7] ; 
                     alu_psw[0] <= alu_input[0] ; 
                  end
         10'b0000110011 :
                  begin
                     // asl
                     result = {alu_input[14:0], 1'b0}; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)    alu_psw[2] <= 1'b1 ; 
                     else                                   alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= alu_input[15] ^ result[15] ; 
                     alu_psw[0] <= alu_input[15] ; 
                  end
         10'b1000110011 :
                  begin
                     // aslb
                     ir_byte <= 1'b1 ; 
                     result8 = {alu_input[6:0], 1'b0}; 
                     alu_output[7:0] <= result8 ; 
                     alu_output[15:8] <= 8'bXXXXXXXX ; 
                     alu_psw[3] <= result8[7] ; 
                     if (result8 == 8'b00000000)   alu_psw[2] <= 1'b1 ; 
                     else                          alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= alu_input[7] ^ result8[7] ; 
                     alu_psw[0] <= alu_input[7] ; 
                  end
         10'b0000110111 :
                  begin
                     // sxt
                     if ((psw[3]) == 1'b0)   begin
                        result = 16'b0000000000000000; 
                        alu_psw[2] <= 1'b1 ; 
                     end
                     else begin
                        result = 16'b1111111111111111; 
                        alu_psw[2] <= 1'b0 ; 
                     end 
                     alu_output <= result ; 
                     alu_psw[3] <= psw[3] ; 
                     alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= psw[0] ; 
                  end
         default :
                  begin
                     alu_output <= 16'bXXXXXXXXXXXXXXXX ; 
                     alu_psw <= 4'bXXXX ; 
                  end
      endcase 
   end
   //***********************************************
   //* Обработка двухперандных инструкций
   //***********************************************
   else if (ir_dop == 1'b1)  begin
      case (ir[15:12])
         4'b0001 :
                  begin
                     // mov
                     result = alus_input; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)   alu_psw[2] <= 1'b1 ; 
                     else                                  alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= psw[0] ; 
                  end
         4'b1001 :
                  begin
                     // movb
                     ir_byte <= 1'b1 ; 
                     result = alus_input; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[7] ; 
                     if (result[7:0] == 8'b00000000)  alu_psw[2] <= 1'b1 ; 
                     else                             alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= psw[0] ; 
                  end
         4'b0010 :
                  begin
                     // cmp
                     result = alus_input - alu_input; 
                     alu_output <= alu_input ; 
                     ir_store <= 1'b0 ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000) alu_psw[2] <= 1'b1 ; 
                     else                                alu_psw[2] <= 1'b0 ; 
                     if ((alu_input[15] != alus_input[15]) & (alus_input[15] != result[15]))  alu_psw[1] <= 1'b1 ; 
                     else                                                                     alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= ((~alus_input[15]) & alu_input[15]) | ((~alus_input[15]) & result[15]) | (alu_input[15] & result[15]) ; 
                  end
         4'b1010 :
                  begin
                     // cmpb
                     ir_byte <= 1'b1 ; 
                     result8 = alus_input[7:0] - alu_input[7:0]; 
                     alu_output <= alu_input ; 
                     ir_store <= 1'b0 ; 
                     alu_psw[3] <= result8[7] ; 
                     if (result8 == 8'b00000000)  alu_psw[2] <= 1'b1 ; 
                     else                         alu_psw[2] <= 1'b0 ; 
                     if ((alu_input[7] != alus_input[7]) & (alus_input[7] != result8[7]))   alu_psw[1] <= 1'b1 ; 
                     else                                                                   alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= ((~alus_input[7]) & alu_input[7]) | ((~alus_input[7]) & result8[7]) | (alu_input[7] & result8[7]) ; 
                  end
         4'b0011 :
                  begin
                     // bit
                     result = alus_input & alu_input; 
                     alu_output <= alu_input ; 
                     ir_store <= 1'b0 ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)    alu_psw[2] <= 1'b1 ; 
                     else                                   alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= psw[0] ; 
                  end
         4'b1011 :
                  begin
                     // bitb
                     ir_byte <= 1'b1 ; 
                     result = alus_input & alu_input; 
                     alu_output <= alu_input ; 
                     ir_store <= 1'b0 ; 
                     alu_psw[3] <= result[7] ; 
                     if (result[7:0] == 8'b00000000)    alu_psw[2] <= 1'b1 ; 
                     else                               alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= psw[0] ; 
                  end
         4'b0100 :
                  begin
                     // bic
                     result = (~alus_input) & alu_input; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)    alu_psw[2] <= 1'b1 ; 
                     else                                   alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= psw[0] ; 
                  end
         4'b1100 :
                  begin
                     // bicb
                     ir_byte <= 1'b1 ; 
                     result = (~alus_input) & alu_input; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[7] ; 
                     if (result[7:0] == 8'b00000000)  alu_psw[2] <= 1'b1 ; 
                     else                             alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= psw[0] ; 
                  end
         4'b0101 :
                  begin
                     // bis
                     result = alus_input | alu_input; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)  alu_psw[2] <= 1'b1 ; 
                     else                                 alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= psw[0] ; 
                  end
         4'b1101 :
                  begin
                     // bisb
                     ir_byte <= 1'b1 ; 
                     result = alus_input | alu_input; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[7] ; 
                     if (result[7:0] == 8'b00000000)      alu_psw[2] <= 1'b1 ; 
                     else                                 alu_psw[2] <= 1'b0 ; 
                     alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= psw[0] ; 
                  end
         4'b0110 :
                  begin
                     // add
                     result = alu_input + alus_input; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)   alu_psw[2] <= 1'b1 ; 
                     else                                  alu_psw[2] <= 1'b0 ; 
                     if ((alu_input[15] == alus_input[15]) & (alus_input[15] != result[15]))  alu_psw[1] <= 1'b1 ; 
                     else                                                                     alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= (alu_input[15] & alus_input[15]) | (alu_input[15] & ~result[15]) | (alus_input[15] & ~result[15]) ; 
                  end
         4'b1110 :
                  begin
                     // sub
                     result = alu_input - alus_input; 
                     alu_output <= result ; 
                     alu_psw[3] <= result[15] ; 
                     if (result == 16'b0000000000000000)   alu_psw[2] <= 1'b1 ; 
                     else                                  alu_psw[2] <= 1'b0 ; 
                     if ((alu_input[15] != alus_input[15]) & (alu_input[15] != result[15]))  alu_psw[1] <= 1'b1 ; 
                     else                                                                    alu_psw[1] <= 1'b0 ; 
                     alu_psw[0] <= ((~alu_input[15]) & alus_input[15]) | ((~alu_input[15]) & result[15]) | (alus_input[15] & result[15]) ; 
                  end
         default :
                  begin
                     alu_output <= 16'bXXXXXXXXXXXXXXXX ; 
                     alu_psw <= 4'bXXXX ; 
                  end
      endcase 
   end
   // misc insns
   else if (ir_mfpi == 1'b1 | ir_mfpd == 1'b1)  begin
      // mfpi, mfpd
      alu_output <= alu_input ; 
      alu_psw[3] <= alu_input[15] ; 
      if (alu_input == 16'b0000000000000000)  alu_psw[2] <= 1'b1 ; 
      else                                    alu_psw[2] <= 1'b0 ; 
      alu_psw[1] <= 1'b0 ; 
      alu_psw[0] <= psw[0] ; 
   end
   else if (ir_mtpi == 1'b1 | ir_mtpd == 1'b1) begin
      // mtpi, mtpd
      alu_output <= alus_input ; 
      alu_psw[3] <= alus_input[15] ; 
      if (alus_input == 16'b0000000000000000)  alu_psw[2] <= 1'b1 ; 
      else                                     alu_psw[2] <= 1'b0 ; 
      alu_psw[1] <= 1'b0 ; 
      alu_psw[0] <= psw[0] ; 
   end
   else if (ir_dopr == 1'b1 & ir[11:9] == 3'b100)  begin
      // xor
      result = alu_input ^ alus_input; // xor is handled here, not in the eis alu
      alu_output <= result ; 
      alu_psw[3] <= result[15] ; 
      if (result == 16'b0000000000000000)  alu_psw[2] <= 1'b1 ; 
      else                                 alu_psw[2] <= 1'b0 ; 
      alu_psw[1] <= 1'b0 ; 
      alu_psw[0] <= psw[0] ; 
   end
   // FPU с целым 16-битным результатом 
   else if (ir_fpsop1 == 1'b1)   begin
      alu_psw[3:0] <= psw[3:0] ; 
      case (ir[7:6])
         2'b01 :
                  begin
                     // ldfps
                     result = alu_input; 
                     alu_output <= result ; 
                  end
         2'b10 :
                  begin
                     // stfps
                     result[15:14] = fps[15:14]; 
                     result[13:12] = 2'b00; 
                     result[11:0] = fps[11:0]; 
                     alu_output <= result ; 
                  end
         2'b11 :
                  begin
                     // stst
                     result = {12'b000000000000, fec}; 
                     alu_output <= result ; 
                  end
         default :
                  begin
                     alu_output <= 16'bXXXXXXXXXXXXXXXX ; 
                     alu_psw <= 4'bXXXX ; 
                  end
      endcase 
   end
   else begin
      alu_output <= 16'bXXXXXXXXXXXXXXXX ; 
      alu_psw <= 4'bXXXX ; 
   end 
end 

//*************************************************
// АЛУ блока инструкций EIS: MUL, DIV , ASH , ASHC  
//*************************************************
always @(posedge clk) begin
   if (ir_dopr == 1'b1 & (ir[11]) == 1'b0)  begin
      case (ir[10:9])
         //*************************************************
         //*  MUL:   Умножение
         //*************************************************
         2'b00 :
                  begin
                     // Вычисление результата - используется аппаратный блок умножения
                     if (eis_sequencer == 5'b11111)   eis_temp1 <= $signed(alu_input) * $signed(alus_input) ; 
                     // Вывод результата на выходы АЛУ
                     else if (eis_sequencer == 5'b00000)  begin
                        eis_output <= eis_temp1[31:16] ;  // старшее слово
                        eis_output32 <= eis_temp1[15:0] ; // младшее слово
                        eis_psw[3] <= eis_temp1[31] ;     // бит знака N PSW
                        // Установка флага Z
                        if (eis_temp1 == 32'b00000000000000000000000000000000)   eis_psw[2] <= 1'b1 ; 
                        else                                                     eis_psw[2] <= 1'b0 ; 
                        eis_psw[1] <= 1'b0 ; // PSW V=0, переполнения не бывает
                        // Флаг PSW C
                        if (((eis_temp1[31]) == 1'b1 & eis_temp1[30:15] != 16'b1111111111111111) | 
                            ((eis_temp1[31]) == 1'b0 & eis_temp1[30:15] != 16'b0000000000000000))   eis_psw[0] <= 1'b1 ; 
                        else                                                                        eis_psw[0] <= 1'b0 ; 
                     end 
                  end
         //*************************************************
         //*  DIV: Деление
         //*************************************************
         2'b01 :
                  begin
                     if (eis_sequencer == 5'b10000)    begin
                        // load seq. code
                        if ((alu_input[15]) == 1'b1)   begin
                           // if input negative
                           eis_temp1 <= {1'b0, ((~alu_input) + 1'b1), {15{1'b0}}} ; // take two's complement
                           eis_flag1 <= 1'b1 ; 
                        end
                        else   begin
                           eis_temp1 <= {1'b0, alu_input, {15{1'b0}}} ; 
                           eis_flag1 <= 1'b0 ; 
                        end 
                        // div
                        if ((alus_input[15]) == 1'b1)  begin
                           eis_temp2 <= (~({alus_input, alut_input})) + 1 ; 
                           eis_flag2 <= 1'b1 ; 
                        end
                        else  begin
                           eis_temp2 <= {alus_input, alut_input} ; 
                           eis_flag2 <= 1'b0 ; 
                        end 
                        eis_psw <= 4'b0000 ; 
                     end
                     // main div loop
                     else if ((eis_sequencer[4]) == 1'b0)  begin
                        if ((eis_temp1) <= (eis_temp2))  begin
                           if (eis_sequencer[3:0] == 4'b1111)  begin
                              if ((eis_temp1) <= (eis_temp2))   begin
                                 eis_psw[1] <= 1'b1 ; 
                                 eis_psw[2] <= 1'b1 ; 
                              end 
                           end 
                           eis_temp[eis_sequencer[3:0]] <= 1'b1 ; 
                           eis_temp2 <= eis_temp2 - eis_temp1 ; 
                        end
                        else   eis_temp[eis_sequencer[3:0]] <= 1'b0 ; 
                        eis_temp1[30:0] <= eis_temp1[31:1] ; 
                     end
                     else   begin
                        // post processing
                        // setting the flags after the div instruction is the tricky part. A division by zero causes
                        // the result not to be stored - which is handled by the state machine, results are only
                        // stored if the v and c flags are 00. Still a very tricky thing considering all the
                        // border cases. I believe the current model is correct - and also, it passes all the tests
                        // I can find. Specifically, fkac, and zkdj - and the results make sense as well.
                        if (eis_flag2 == 1'b1)   
                           // if 2nd op was negative
                           eis_output32 <= (~eis_temp2[15:0]) + 1'b1 ; // sign adjust remainder
                        else    eis_output32 <= eis_temp2[15:0] ; // or just the positive
                        if (eis_flag1 != eis_flag2)   begin
                           // if signs were different
                           eis_psw[3] <= 1'b1 ; // set N
                           eis_output <= (~eis_temp) + 1'b1 ; // sign adjust result
                        end
                        else  begin
                           eis_psw[3] <= 1'b0 ; // clear n
                           eis_output <= eis_temp ; // copy result
                        end 
                        if (eis_temp[14:0] == {15{1'b0}})  begin
                           // результат равен 0
                           if ((eis_temp[15]) == 1'b0)  begin
                              eis_psw[3] <= 1'b0 ; 
                              eis_psw[2] <= 1'b1 ; 
                              eis_output[15] <= 1'b0 ; 
                           end
                           else            eis_psw[2] <= 1'b0 ; 
                           if ((eis_temp[15]) == 1'b1 & eis_flag1 != eis_flag2)   eis_psw[1] <= 1'b0 ; // special case: quotient is negative maxint - that isn't an overflow
                        end
                        //       деление на 0
                        if (alu_input == {16{1'b0}})  begin
                           // установка  V и C
                           eis_psw[2] <= psw[2] ;
                           eis_psw[1] <= 1'b1 ; 
                           eis_psw[0] <= 1'b1 ; 
                        end 
                     end 
                  end
         2'b10 :
                  begin
                     if (eis_sequencer == 5'b11111)  begin
                        eis_output <= alus_input ; 
                        eis_flag2 <= 1'b0 ; 
                        eis_psw[1] <= 1'b0 ; 
                        eis_psw[0] <= 1'b0 ; 
                        eis_temp[15:6] <= 10'b0000000000 ; // for easier debugging
                        eis_flag1 <= alu_input[5] ; 
                        if (alu_input[4:0] == 5'b11111)  begin
                           // ash
                              // see EK-1184E-TM-001_Dec87.pdf, page B-17
                                 // Speculative - see ashc case
                           eis_flag1 <= 1'b1 ; 
                        end 
                        if ((alu_input[5]) == 1'b1)   eis_temp[5:0] <= (~alu_input[5:0]) + 1'b1 ; 
                        else                          eis_temp[5:0] <= alu_input[5:0] ; 
                     end
                     else   begin
                        if (eis_temp[5:0] != 6'b000000)  begin
                           if (eis_flag1 == 1'b1)  begin
                              eis_output <= {eis_output[15], eis_output[15:1]} ; 
                              eis_psw[0] <= eis_output[0] ; 
                           end
                           else  begin
                              eis_output <= {eis_output[14:0], 1'b0} ; 
                              if (eis_output[15:14] == 2'b10 | eis_output[15:14] == 2'b01)  eis_psw[1] <= 1'b1 ; 
                              eis_psw[0] <= eis_output[15] ; 
                           end 
                           eis_temp[5:0] <= eis_temp[5:0] - 1'b1 ; 
                        end
                        else   begin
                           eis_flag2 <= 1'b1 ; 
                           eis_psw[3] <= eis_output[15] ; 
                           if (eis_output == 16'b0000000000000000)   eis_psw[2] <= 1'b1 ; 
                           else                                      eis_psw[2] <= 1'b0 ; 
                        end 
                     end 
                  end
         2'b11 :
                  begin
                     if (eis_sequencer == 5'b11111)  begin
                        eis_temp1 <= {alus_input, alut_input} ; 
                        eis_flag2 <= 1'b0 ; 
                        eis_psw[1] <= 1'b0 ; 
                        eis_psw[0] <= 1'b0 ; 
                        eis_temp[15:6] <= 10'b0000000000 ; // for easier debugging
                        eis_flag1 <= alu_input[5] ; 
                        if (alu_input[4:0] == 5'b11111)    begin
                           // ashc
                        end 
                        if ((alu_input[5]) == 1'b1)    eis_temp[5:0] <= ({1'b0, (~alu_input[4:0])}) + 1'b1 ; 
                        else  begin
                           eis_temp[4:0] <= alu_input[4:0] ; 
                           eis_temp[5] <= 1'b0 ; 
                        end 
                     end
                     else  begin
                        if (eis_temp[5:0] != 6'b000000)  begin
                           if (eis_flag1 == 1'b1)  begin
                              eis_temp1 <= {eis_temp1[31], eis_temp1[31:1]} ; 
                              eis_psw[0] <= eis_temp1[0] ; 
                           end
                           else  begin
                              eis_temp1 <= {eis_temp1[30:0], 1'b0} ; 
                              if (eis_temp1[31:30] == 2'b10 | eis_temp1[31:30] == 2'b01)   eis_psw[1] <= 1'b1 ; 
                              eis_psw[0] <= eis_temp1[31] ; 
                           end 
                           eis_temp[5:0] <= eis_temp[5:0] - 1'b1 ; 
                        end
                        else  begin
                           eis_flag2 <= 1'b1 ; 
                           eis_output <= eis_temp1[31:16] ; 
                           eis_output32 <= eis_temp1[15:0] ; 
                           eis_psw[3] <= eis_temp1[31] ; 
                           if (eis_temp1 == 32'b00000000000000000000000000000000)    eis_psw[2] <= 1'b1 ; 
                           else                                                      eis_psw[2] <= 1'b0 ; 
                        end 
                     end 
                  end
      endcase 
   end  
end 

reg[3:0] v_caseworkaround; 

//**********************************************
// АЛУ плавающей точки
//**********************************************
always @(posedge clk)   begin
   if (fpu_enable == 1 & reset == 1'b1)  begin
      falu_done <= 1'b0 ; 
      falu_fsm <= falu_idle ; 
      falu_fps <= 4'b0000 ; 
      falu_flag1 <= 1'b0 ; 
      falu_pending_fiu <= 1'b0 ; 
      falu_pending_fiv <= 1'b0 ; 
      falu_pending_fic <= 1'b0 ; 
      falu_pending_divz <= 1'b0 ; 
   end
   else if (fpu_enable == 1 & iwait == 1'b0)   begin
      if (falu_pending_clear == 1'b1)    begin
         falu_pending_fiu <= 1'b0 ; 
         falu_pending_fiv <= 1'b0 ; 
         falu_pending_fic <= 1'b0 ; 
         falu_pending_divz <= 1'b0 ; 
      end 
      if (ir_fpao == 1'b1)   begin
         if (falu_load == 1'b1)    begin
            falu_done <= 1'b0 ; 
            falu_fps <= 4'b0000 ; 
            case (ir[11:8])
               4'b0010, 4'b0011 :
                        begin
                           // mul(f|d), mod(f|d)
                              // set sign - positive if both operands are same sign, negative otherwise
                           if (falu_input[63] == falus_input[63])   falu_fps[3] <= 1'b0 ; 
                           else                                     falu_fps[3] <= 1'b1 ; 
                           falu_fps[2:0] <= 3'b000 ; // set default for fps bits
                           falu_fsm <= falu_mult ; 
                           falu_work1 <= {59{1'b0}} ; 
                           falu_work2 <= {1'b0, 1'b1, falus_input[54:0], 1'b0, 1'b0} ; 
                           // if the falu_load bit is one, load the work registers and the initial state for the falu state machine.
                           // both of which are dependent on exactly which instruction we need to process - the sequence in the
                           // state machine needs to be started at a specific point, which is not the same for all insn - and
                           // definitely all insn have their own initialization requirements and special cases.
                           //
                           // also, the main cpu state machine includes
                           if (falu_input[62:55] == 8'b00000000 | falus_input[62:55] == 8'b00000000)  begin
                              // if either input exponent is zero, we don't need to multiply at all
                              falu_output <= {64{1'b0}} ; 
                              falu_output2 <= {64{1'b0}} ; 
                              falu_fps <= 4'b0100 ; 
                              falu_fsm <= falu_idle ; 
                              falu_done <= 1'b1 ; 
                           end 
                           falu_ccw <= ({2'b00, falu_input[62:55]}) + ({2'b00, falus_input[62:55]}) - 10'b0010000001 ; 
                        end
               4'b0100, 4'b0110 :
                        begin
                           // add(f|d), sub(f|d)
                           falu_fsm <= falu_align ; 
                           falu_work1 <= {1'b0, 1'b1, falu_input[54:0], 1'b0, 1'b0} ; 
                           falu_work2 <= {1'b0, 1'b1, falus_input[54:0], 1'b0, 1'b0} ; 
                           falu_fps[3:0] <= 4'b0000 ; // set default for fps bits
                           if (falu_input[62:55] == 8'b00000000)
                           begin
                              // if the primary input exponent is zero, we don't need to add (or subtract) at all
                              falu_output <= falus_input ; 
                              falu_fps[3] <= falus_input[63] ; 
                              if (falus_input[62:55] == 8'b00000000)   falu_fps[2] <= 1'b1 ; 
                              else                                     falu_fps[2] <= 1'b0 ; 
                              falu_fsm <= falu_idle ; 
                              falu_done <= 1'b1 ; 
                           end
                           else if (falus_input[62:55] == 8'b00000000)  begin
                              // if the secondary input exponent is zero, we don't need to add (or subtract) at all
                              falu_output[62:0] <= falu_input[62:0] ; 
                              if ((ir[9]) == 1'b0)   begin
                                 falu_fps[3] <= falu_input[63] ; 
                                 falu_output[63] <= falu_input[63] ; 
                              end
                              else begin
                                 falu_fps[3] <= ~falu_input[63] ; 
                                 falu_output[63] <= ~falu_input[63] ; 
                              end 
                              falu_fsm <= falu_idle ; 
                              falu_done <= 1'b1 ; 
                           end
                           else if ((falu_input[62:55]) < (falus_input[62:55])) begin
                              falu_ccw <= {2'b00, ((falus_input[62:55]) - (falu_input[62:55]))} ; 
                              falu_flag1 <= 1'b1 ; 
                           end
                           else if ((falu_input[62:55]) == (falus_input[62:55]))    begin
                              falu_ccw <= {10{1'b0}} ; 
                              if ((falu_input[54:0]) < (falus_input[54:0]))  falu_flag1 <= 1'b1 ; 
                              else    falu_flag1 <= 1'b0 ; 
                           end
                           else  begin
                              falu_ccw <= {2'b00, ((falu_input[62:55]) - (falus_input[62:55]))} ; 
                              falu_flag1 <= 1'b0 ; 
                           end 
                        end
               4'b0101 :
                        begin
                           // ld(f|d)
                           falu_output <= falu_input ; 
                           falu_fps[3] <= falu_input[63] ; 
                           if (falu_input[62:55] == 8'b00000000)  falu_fps[2] <= 1'b1 ; 
                           else                                   falu_fps[2] <= 1'b0 ; 
                           falu_fps[1:0] <= 2'b00 ; // set default for fps bits
                           falu_fsm <= falu_idle ; 
                           falu_done <= 1'b1 ; 
                        end
               4'b0111 :
                        begin
                           // cmp(f|d)
                           falu_output <= falus_input ; 
                           falu_fps[3:0] <= 4'b0000 ; // set default for fps bits
                           if ((falu_input[63]) == 1'b1 & (falus_input[63]) == 1'b0)  falu_fps[3] <= 1'b1 ; 
                           else if ((falu_input[63]) == 1'b0 & (falus_input[63]) == 1'b0)    begin
                              if ((falu_input[62:55]) < (falus_input[62:55])) falu_fps[3] <= 1'b1 ; 
                              else if ((falu_input[62:55]) == (falus_input[62:55]))
                              begin
                                 if ((falu_input[54:0]) < (falus_input[54:0]))   falu_fps[3] <= 1'b1 ; 
                                 else if ((falu_input[54:0]) == (falus_input[54:0]))   falu_fps[2] <= 1'b1 ; 
                                 // n=0, z=0 
                              end
                              // n=0, z=0 
                           end
                           else if ((falu_input[63]) == 1'b1 & (falus_input[63]) == 1'b1)   begin
                              if ((falus_input[62:55]) < (falu_input[62:55]))     falu_fps[3] <= 1'b1 ; 
                              else if ((falus_input[62:55]) == (falu_input[62:55]))    begin
                                 if ((falus_input[54:0]) < (falu_input[54:0]))   falu_fps[3] <= 1'b1 ; 
                                 else if ((falus_input[54:0]) == (falu_input[54:0]))     falu_fps[2] <= 1'b1 ; 
                                 // n=0, z=0 
                              end
                              // n=0, z=0 
                           end 
                           if (falu_input[62:55] == 8'b00000000 & falus_input[62:55] == 8'b00000000)   begin
                              falu_fps <= 4'b0100 ; 
                              falu_output <= {64{1'b0}} ; 
                           end 
                           falu_fsm <= falu_idle ; 
                           falu_done <= 1'b1 ; 
                        end
               4'b1000 :
                        begin
                           // st(f|d)
                           falu_output <= falu_input ; 
                           falu_fps <= fps[3:0] ; 
                           falu_fsm <= falu_idle ; 
                           falu_done <= 1'b1 ; 
                        end
               4'b1001 :
                        begin
                           // div(f|d)
                           if (falu_input[63] == falus_input[63])   begin
                              // set sign - positive if both operands are same sign, negative otherwise
                              falu_fps[3] <= 1'b0 ; 
                           end
                           else                                 falu_fps[3] <= 1'b1 ; 
                           falu_fps[2:0] <= 3'b000 ; // set default for other fps bits
                           falu_fsm <= falu_div ; 
                           falu_work1 <= {59{1'b0}} ; 
                           falu_work2 <= {1'b0, 1'b1, falus_input[54:0], 1'b0, 1'b0} ; 
                           if (falus_input[62:55] == 8'b00000000)      begin
                              // check ac operand first, then if fsrc is zero, those settings will take precedence over these
                              falu_output <= {64{1'b0}} ; 
                              falu_fps <= 4'b0100 ; 
                              falu_fsm <= falu_idle ; 
                              falu_done <= 1'b1 ; 
                           end 
                           if (falu_input[62:55] == 8'b00000000)
                           begin
                              falu_pending_divz <= 1'b1 ; 
                              falu_output <= falus_input ; 
                              falu_fps <= fps[3:0] ; // the doc is unspecific... but xxdp jfpa seems to expect no updates to fps
                              falu_fsm <= falu_idle ; 
                              falu_done <= 1'b1 ; 
                           end 
                           falu_ccw <= 10'b0000111010 ; 
                        end
               4'b1010 :
                        begin
                           // stexp
                           falu_output[55:48] <= falu_input[62:55] - 8'b10000000 ; 
                           if ((falu_input[62:55]) < (8'b10000000))   begin
                              falu_fps[3] <= 1'b1 ; 
                              falu_output[63:56] <= {8{1'b1}} ; 
                           end
                           else begin
                              falu_fps[3] <= 1'b0 ; 
                              falu_output[63:56] <= {8{1'b0}} ; 
                           end 
                           if (falu_input[62:55] == 8'b10000000)   falu_fps[2] <= 1'b1 ; 
                           else                                    falu_fps[2] <= 1'b0 ; 
                           falu_fps[1] <= 1'b0 ; 
                           falu_fps[0] <= 1'b0 ; 
                           falu_fsm <= falu_idle ; 
                           falu_done <= 1'b1 ; 
                        end
               4'b1011 :
                        begin
                           // stc(f|d)(i|l)
                           falu_fsm <= falu_shift ; 
                           falu_fps[3] <= falu_input[63] ; // n is set from input
                           falu_fps[2:0] <= 3'b000 ; // set default for other fps bits
                           falu_work1 <= {59{1'b0}} ; // the idea to use work1 here is that synthesis may reuse the shifter we already have for it
                           if ((fps[6]) == 1'b0)    begin
                              // if short integer mode
                              falu_work1[58:43] <= {1'b1, falu_input[54:40]} ; 
                              falu_ccw <= (10'b0010010000) - ({2'b00, falu_input[62:55]}) ; // exponent minus the bias
                           end
                           else     begin
                                 // if in long integer mode, we need to check if we're in float mode, because then we can only copy 23 bits of fraction
                              if ((fps[7]) == 1'b0)   falu_work1[58:35] <= {1'b1, falu_input[54:32]} ; 
                              else                    falu_work1[58:26] <= {1'b1, falu_input[54:23]} ; 
                                 // reg or mode 2, reg 7
                              if (ir[5:3] == 3'b000 | ir[5:0] == 6'b010111)   falu_ccw <= (10'b0010010000) - ({2'b00, falu_input[62:55]}) ; // exponent minus the bias
                              else                                    falu_ccw <= (10'b0010100000) - ({2'b00, falu_input[62:55]}) ; // exponent minus the bias
                           end 
                           if ((falu_input[62:55]) < (8'b10000001))  begin
                              // it is not entirely clear in the manuals, but if the input is less than 1, the output is zero, and only the Z flag is set. It is not a conversion error!
                              falu_output <= {64{1'b0}} ; 
                              falu_fps[3] <= 1'b0 ; 
                              falu_fps[2] <= 1'b1 ; 
                              falu_fps[1] <= 1'b0 ; 
                              falu_fps[0] <= 1'b0 ; 
                              falu_fsm <= falu_idle ; 
                              falu_done <= 1'b1 ; 
                           end 
                        end
               4'b1100, 4'b1111 :
                        begin
                           // stc(f|d)(d|f), ldc(d|f)(f|d)
                           falu_fps[3] <= falu_input[63] ; // n bit is in most cases a direct copy of the input
                           falu_output[63:55] <= falu_input[63:55] ; // right in most cases
                           falu_fps[2:0] <= 3'b000 ; // set default for other fps bits
                           if (falu_input[62:55] == 8'b00000000)    begin
                              // if the input exponent is zero, then the z bit in fps must be set
                              falu_fps[2] <= 1'b1 ; 
                              falu_fps[3] <= 1'b0 ; // negative zero exp is ignored
                              falu_output <= {64{1'b0}} ; 
                           end
                           else  begin
                              falu_fps[2] <= 1'b0 ; 
                              if (((fps[7]) == 1'b0 & ir[11:8] == 4'b1100) | ((fps[7]) == 1'b1 & ir[11:8] == 4'b1111))     begin
                                 // convert to a double, or to a float?
                                 falu_output[54:32] <= falu_input[54:32] ; // just copy the high part if converting f to d
                                 falu_output[31:0] <= 32'b00000000000000000000000000000000 ; // and set the low part to zeroes
                              end
                              else  begin
                                 if ((fps[5]) == 1'b1)   begin
                                    // on d to f conversion, if round/trunc is trunc
                                    falu_output[54:32] <= falu_input[54:32] ; // just copy the high part
                                    falu_output[31:0] <= 32'b00000000000000000000000000000000 ; // and set the low part to zeroes
                                 end
                                 else  begin
                                    if (falu_input[62:31] == 32'b11111111111111111111111111111111)  begin
                                       // this bit pattern causes overflow to occur
                                       falu_output[62:32] <= 31'b0000000000000000000000000000000 ; // result after overflow is zeroes
                                       falu_fps[2] <= 1'b1 ; // set z bit, because of zeroes we just set!
                                       falu_fps[1] <= 1'b1 ; // set v bit to signal overflow
                                          // if fiv enabled
                                       if ((fps[9]) == 1'b1)   falu_pending_fiv <= 1'b1 ; // then signal the pending interrupt
                                    end
                                    else falu_output[62:31] <= falu_input[62:31] + 1'b1 ; // normal case, round bit added. Note that I count on normal arithmetic to handle increasing the exponent, if that is necessary to handle an overflow of the fraction part
                                    falu_output[31:0] <= 32'b00000000000000000000000000000000 ; // in all cases, the low part is cleared
                                 end 
                              end 
                           end 
                           falu_fsm <= falu_idle ; 
                           falu_done <= 1'b1 ; 
                        end
               4'b1101 :
                        begin
                           // ldexp
                           falu_output[63] <= falu_input[63] ; // setup sign, in all cases a copy of the input
                           falu_output[54:0] <= falu_input[54:0] ; // fraction is in all cases same as input
                           falu_fps[3] <= falu_input[63] ; // setup n bit
                           falu_fps[2:0] <= 3'b000 ; // set default for other fps bits
                           if ((falus_input[55]) == 1'b1)   begin
                              if (falus_input[54:47] == 8'b11111111 & falus_input[47:40] != 8'b10000000)  begin
                                 // if yes, then the next 8 bits need to be ones too, else it is an overflow
                                 // would produce an overflow as well - special case
                                 falu_output[62:55] <= falus_input[47:40] + 8'b10000000 ; // not an overflow --> assign the new exponent, biased w. 200 oct
                              end
                              else  begin
                                 if ((fps[10]) == 1'b1)  begin
                                    // if fiu enabled
                                    falu_output[62:55] <= falus_input[47:40] + 8'b10000000 ; 
                                    // sign bit on, ie. is this a negative 2-complement integer
                                    if (falus_input[47:40] + 8'b10000000 == 8'b00000000)  falu_fps[2] <= 1'b1 ; 
                                    falu_pending_fiu <= 1'b1 ; 
                                 end
                                 else   begin
                                    falu_output <= {64{1'b0}} ; // if fiu disabled, just set the output to zeroes
                                    falu_fps[2] <= 1'b1 ; // and dont forget to set the z bit either
                                    falu_fps[3] <= 1'b0 ; // and also dont forget zero is not negative
                                 end 
                              end 
                           end
                           else  begin
                              if (falus_input[54:47] == 8'b00000000)   begin
                                 // for a positive exponent, the high 8 bits must be clear, otherwise it is an overflow
                                 falu_output[62:55] <= falus_input[47:40] + 8'b10000000 ; // not overflow - assign new exponent biased w. 200 oct
                              end
                              else    begin
                                 falu_fps[1] <= 1'b1 ; // v bit is set only when exponent > 177
                                 if ((fps[9]) == 1'b1)  begin
                                    // if fiv is enabled
                                    falu_output[62:55] <= falus_input[47:40] + 8'b10000000 ; 
                                    // positive exponent
                                    if (falus_input[47:40] + 8'b10000000 == 8'b00000000)  falu_fps[2] <= 1'b1 ; 
                                    falu_pending_fiv <= 1'b1 ; 
                                 end
                                 else  begin
                                    // if fiv is disabled
                                    falu_output <= {64{1'b0}} ; // set the output to all zeroes
                                    falu_fps[2] <= 1'b1 ; // set z bit as well
                                    falu_fps[3] <= 1'b0 ; 
                                 end 
                              end 
                           end 
                           falu_fsm <= falu_idle ; 
                           falu_done <= 1'b1 ; 
                        end
               4'b1110 :
                        begin
                           // ldc(i|l)(f|d)
                           falu_fsm <= falu_norm ; 
                           falu_fps[2:0] <= 3'b000 ; // set default for fps bits
                           if ((fps[6]) == 1'b0 | ir[5:3] == 3'b000 | ir[5:0] == 6'b010111)  begin
                              // mode 2, reg 7 only
                                 // if fl is set ie long mode, mode must be 0 or mode 2, reg 7, and the strange exception to use the single 16bit word as the upper applies.
                              if ((fps[6]) == 1'b1)   falu_ccw <= 10'b0010011111 ; // 37(8) or 31(10), max number of shifts for long mode; special case
                              else                    falu_ccw <= 10'b0010001111 ; // 17(8) or 15(10), max number of shifts for integer mode
                              falu_work1 <= {59{1'b0}} ; 
                              if ((falu_input[39]) == 1'b1)   begin
                                 falu_fps[3] <= 1'b1 ; 
                                 falu_work1[58:43] <= (~falu_input[39:24]) + 1'b1 ; 
                              end
                              else  begin
                                 falu_fps[3] <= 1'b0 ; 
                                 falu_work1[58:43] <= falu_input[39:24] ; 
                              end 
                           end
                           else
                           begin
                              falu_ccw <= 10'b0010011111 ; // 37(8) or 31(10), max number of shifts for long mode
                              falu_work1 <= {59{1'b0}} ; 
                              if ((falu_input[55]) == 1'b1)
                              begin
                                 falu_fps[3] <= 1'b1 ; 
                                 falu_work1[58:27] <= (~falu_input[55:24]) + 1 ; 
                              end
                              else
                              begin
                                 falu_fps[3] <= 1'b0 ; 
                                 falu_work1[58:27] <= falu_input[55:24] ; 
                              end 
                           end 
                        end
            endcase 
         end
         else   begin
            case (falu_fsm)
               // multiply, ie. shifting and adding
               // this does not deal with the fd bit - all mult operations are full precision, regardless of the bit. The core
               // would be significantly faster for single prec if we would deal with the fd bit. FIXME!
               falu_mult :
                        begin
                           if (falu_work2[57:2] != 56'b00000000000000000000000000000000000000000000000000000000) begin
                                 // if lowest order bit is a one
                              if ((falu_work2[2]) == 1'b1) falu_work1 <= ({1'b0, falu_work1[58:1]} + ({1'b0, 1'b1, falu_input[54:0], 2'b00})) ; // then shift right and add
                              else                         falu_work1 <= {1'b0, falu_work1[58:1]} ; // if not set, then only shift right
                              falu_work2 <= {1'b0, falu_work2[58:1]} ; // shift right for next round
                           end
                           else falu_fsm <= falu_norm ; // if all bits done, then go into normalize state
                        end
               // align the operands for addition or subtraction
               // flag1 has which one of the operands needs to be shifted - and also, check the fd bit to see what the maximum value of the shift should be
               // falu_ccw has the difference - if it is 0, or shift- and decrement to 0, the addition/subtraction state is next up
               falu_align :
                        begin
                           if (falu_ccw != 10'b0000000000)   begin
                              if (falu_flag1 == 1'b1)  falu_work1 <= {1'b0, falu_work1[58:1]} ; 
                              else                     falu_work2 <= {1'b0, falu_work2[58:1]} ; 
                              if ((fps[7]) == 1'b1 & (falu_ccw) > (10'b0000111001)) begin
                                 // > 57 ??
                                 falu_ccw <= 10'b0000000000 ; 
                                 if (falu_flag1 == 1'b1)  falu_work1 <= {59{1'b0}} ; 
                                 else                     falu_work2 <= {59{1'b0}} ; 
                                 falu_fsm <= falu_addsub ; 
                              end
                              else if ((fps[7]) == 1'b0 & (falu_ccw) > (10'b0000011001))   begin
                                 // > 25 ??
                                 falu_ccw <= 10'b0000000000 ; 
                                 if (falu_flag1 == 1'b1)    falu_work1 <= {59{1'b0}} ; 
                                 else                       falu_work2 <= {59{1'b0}} ; 
                                 falu_fsm <= falu_addsub ; 
                              end
                              else      falu_ccw <= falu_ccw - 1'b1 ; 
                           end
                           else    falu_fsm <= falu_addsub ; 
                        end
               falu_addsub :
                        begin
                           // this statement:
                           //                     case ir(9) & falu_input(63) & falus_input(63) & falu_flag1  is
                           // would be a nice and elegant way to express what I would like
                           // alas, ISE cannot translate it. See:
                           // AR #22098 - 8.2i XST-"ERROR:HDLParsers:818 - Cannot determine the type of the selector &"
                           v_caseworkaround = {ir[9], falu_input[63], falus_input[63], falu_flag1}; 
                           case (v_caseworkaround)
                              4'b0000, 4'b0001 :
                                       begin
                                          // add, +|+
                                          falu_work1 <= falu_work1 + falu_work2 ; 
                                          falu_fps[3] <= 1'b0 ; 
                                       end
                              4'b0100 :
                                       begin
                                          // add, !work1<work2, -|+
                                          falu_work1 <= falu_work1 - falu_work2 ; 
                                          falu_fps[3] <= 1'b1 ; 
                                       end
                              4'b0101 :
                                       begin
                                          // add, work1<work2, -|+
                                          falu_work1 <= falu_work2 - falu_work1 ; 
                                          falu_fps[3] <= 1'b0 ; 
                                       end
                              4'b0010 :
                                       begin
                                          // add, !work1<work2, +|-
                                          falu_work1 <= falu_work1 - falu_work2 ; 
                                          falu_fps[3] <= 1'b0 ; 
                                       end
                              4'b0011 :
                                       begin
                                          // add, work1<work2, +|-
                                          falu_work1 <= falu_work2 - falu_work1 ; 
                                          falu_fps[3] <= 1'b1 ; 
                                       end
                              4'b0110, 4'b0111 :
                                       begin
                                          // add, -|-
                                          falu_work1 <= falu_work1 + falu_work2 ; 
                                          falu_fps[3] <= 1'b1 ; 
                                       end
                              4'b1000 :
                                       begin
                                          // sub, !work1<work2, +|+
                                          falu_work1 <= falu_work1 - falu_work2 ; 
                                          falu_fps[3] <= 1'b1 ; 
                                       end
                              4'b1001 :
                                       begin
                                          // sub, work1<work2, +|+
                                          falu_work1 <= falu_work2 - falu_work1 ; 
                                          falu_fps[3] <= 1'b0 ; 
                                       end
                              4'b1100, 4'b1101 :
                                       begin
                                          // sub, -|+
                                          falu_work1 <= falu_work2 + falu_work1 ; 
                                          falu_fps[3] <= 1'b0 ; 
                                       end
                              4'b1010, 4'b1011 :
                                       begin
                                          // sub, +|-
                                          falu_work1 <= falu_work2 + falu_work1 ; 
                                          falu_fps[3] <= 1'b1 ; 
                                       end
                              4'b1110 :
                                       begin
                                          // sub, !work1<work2, -|-
                                          falu_work1 <= falu_work1 - falu_work2 ; 
                                          falu_fps[3] <= 1'b0 ; 
                                       end
                              4'b1111 :
                                       begin
                                          // sub, work1<work2, -|-
                                          falu_work1 <= falu_work2 - falu_work1 ; 
                                          falu_fps[3] <= 1'b1 ; 
                                       end
                              default :
                                       begin
                                       end
                           endcase 
                           if (falu_flag1 == 1'b1)   falu_ccw <= {2'b00, falus_input[62:55]} ; 
                           else                      falu_ccw <= {2'b00, falu_input[62:55]} ; 
                           falu_fsm <= falu_norm ; 
                        end
               falu_div :
                        begin
                           if ((falu_work2) >= ({1'b0, 1'b1, falu_input[54:0], 2'b00}))   begin
                              falu_work1 <= {falu_work1[57:0], 1'b1} ; 
                              falu_work2 <= ({falu_work2[57:0], 1'b0}) - ({1'b1, falu_input[54:0], 3'b000}) ; 
                           end
                           else  begin
                              falu_work1 <= {falu_work1[57:0], 1'b0} ; 
                              falu_work2 <= {falu_work2[57:0], 1'b0} ; 
                           end 
                           if (falu_ccw != 10'b0000000000)  falu_ccw <= falu_ccw - 1'b1 ; 
                           else    begin
                              falu_fsm <= falu_norm ; 
                              falu_ccw <= ({2'b00, falus_input[62:55]}) - ({2'b00, falu_input[62:55]}) + (10'b0010000000) ; 
                           end 
                        end
               falu_shift :
                        begin
                           if (falu_ccw != 10'b0000000000)   begin
                              falu_work1 <= {1'b0, falu_work1[58:1]} ; 
                              falu_ccw <= falu_ccw - 1'b1 ; 
                           end
                           else  begin
                              falu_output <= {64{1'b0}} ; 
                              if ((falu_input[63]) == 1'b1)   falu_output[63:32] <= (~falu_work1[58:27]) + 1 ; 
                              else                            falu_output[63:32] <= falu_work1[58:27] ; 
                              falu_fsm <= falu_shift2 ; 
                           end 
                           if ((fps[6]) == 1'b0) begin
                              if ((falu_ccw) > (10'b0000001111))   falu_fsm <= falu_shifte ; 
                           end
                           else  begin
                              if ((falu_ccw) > (10'b0000011111))   falu_fsm <= falu_shifte ; 
                           end 
                        end
               falu_shift2 :
                        begin
                           if (falu_output[63:48] == 16'b0000000000000000)   begin
                              if ((fps[6]) == 1'b0)  begin
                                 falu_fps[3] <= 1'b0 ; 
                                 falu_fps[2] <= 1'b1 ; 
                              end
                              else  begin
                                 if (falu_output[47:32] == 16'b0000000000000000)   begin
                                    falu_fps[3] <= 1'b0 ; 
                                    falu_fps[2] <= 1'b1 ; 
                                 end 
                              end 
                           end 
                           if (falu_output[63] != falu_input[63])   begin
                              falu_fsm <= falu_shifte ; 
                           end
                           else begin
                              falu_fsm <= falu_idle ; 
                              falu_done <= 1'b1 ; 
                           end 
                        end
               falu_shifte :
                        begin
                           falu_fps[3] <= 1'b0 ; // on error, result is not negative
                           falu_fps[2] <= 1'b1 ; 
                           falu_fps[1] <= 1'b0 ; // V bit is not used
                           falu_fps[0] <= 1'b1 ; 
                           falu_output <= {64{1'b0}} ; 
                           if ((fps[8]) == 1'b1)   falu_pending_fic <= 1'b1 ; 
                           falu_fsm <= falu_idle ; 
                           falu_done <= 1'b1 ; 
                        end
               falu_norm :
                        begin
                           if (falu_work1[58:57] == 2'b01)   begin
                              // hidden bit in the right place, overflow bit clear?
                              if (ir[11:8] == 4'b0011)  falu_fsm <= falu_sep ; 
                              else                      falu_fsm <= falu_rt ; 
                           end
                           else if ((falu_work1[58]) == 1'b1)
                           begin
                              // is the overflow bit set?
                              falu_work1 <= {1'b0, falu_work1[58:1]} ; // shift right
                              falu_ccw <= falu_ccw + 1'b1 ; // increase exponent
                              if (ir[11:8] == 4'b0011)
                              begin
                                 falu_fsm <= falu_sep ; 
                              end
                              else
                              begin
                                 falu_fsm <= falu_rt ; 
                              end 
                           end
                           else
                           begin
                              //                                                        76543210987654321098765432109876543210987654321098765432
                              if (falu_work1[57:2] != 56'b00000000000000000000000000000000000000000000000000000000)  begin
                                 falu_work1 <= {falu_work1[57:0], 1'b0} ; // shift left
                                 falu_ccw <= falu_ccw - 1'b1 ; // decrease exponent
                              end
                              else begin
                                 // coming here, we have lost all ones from the fraction; the output is zero
                                 falu_fps[3] <= 1'b0 ; // make sure that the n bit is cleared
                                 falu_fsm <= falu_zres ; // result is zero
                              end 
                           end 
                        end
               falu_sep :
                        begin
                           if ((falu_ccw) <= (10'b0010000000))   begin
                              falu_output2 <= {64{1'b0}} ; 
                              falu_fsm <= falu_rt ; 
                           end
                           else if (((falu_ccw) > (10'b0010011000) & (fps[7]) == 1'b0) | ((falu_ccw) > (10'b0010111000) & (fps[7]) == 1'b1))   begin
                              falu_fsm <= falu_sep3 ; 
                           end
                           else  begin
                              falu_output2[63] <= falu_fps[3] ; 
                              falu_output2[62:55] <= falu_ccw[7:0] ; 
                              falu_output2[54:0] <= falu_work1[56:2] ; 
                              falu_fsm <= falu_sep2 ; 
                              falu_work2 <= {59{1'b0}} ; 
                              falu_work2[58:57] <= 2'b10 ; 
                           end 
                        end
               falu_sep2 :
                        begin
                           if ((falu_ccw) > (10'b0010000000))  begin
                              falu_work1 <= {falu_work1[57:0], 1'b0} ; // shift left
                              falu_work2 <= {1'b1, falu_work2[58:1]} ; // shift right
                              falu_ccw <= falu_ccw - 1'b1 ; 
                           end
                           else if ((falu_work1[57]) != 1'b1 & falu_ccw != 10'b0000000000)  begin
                              falu_work1 <= {falu_work1[57:0], 1'b0} ; // shift left
                              falu_ccw <= falu_ccw - 1'b1 ; 
                              if (falu_work1[57:2] == 56'b00000000000000000000000000000000000000000000000000000000)  falu_ccw <= 10'b0000000000 ; 
                           end
                           else  begin
                              falu_output2[54:0] <= falu_output2[54:0] & falu_work2[56:2] ; 
                              falu_fsm <= falu_res ; 
                              if (falu_ccw == 10'b0000000000)  falu_fsm <= falu_zres ; // zero result handled directly, because res would wrongly raise an underflow
                           end 
                        end
               falu_sep3 :
                        begin
                           falu_output <= {64{1'b0}} ; // set fraction output to zero
                           falu_fps[3] <= 1'b0 ; // if the fraction is zero, so is its sign
                           falu_fps[2] <= 1'b1 ; // set z for fraction
                           falu_output2[63] <= falu_fps[3] ; 
                           falu_output2[62:55] <= falu_ccw[7:0] ; 
                           falu_output2[54:0] <= falu_work1[56:2] ; 
                           if ((falu_ccw[8]) == 1'b1 & (falu_ccw[9]) != 1'b1)   begin
                              // overflow?
                              falu_fps[1] <= 1'b1 ; // set the flag
                                 // are overflow traps enabled?
                              if ((fps[9]) == 1'b1)   falu_pending_fiv <= 1'b1 ; // yes, set flag
                              else                    falu_output2 <= {64{1'b0}} ; 
                           end 
                           falu_done <= 1'b1 ; 
                           falu_fsm <= falu_idle ; 
                        end
               falu_rt :
                        begin
                           if ((fps[5]) == 1'b0)   begin
                              if ((fps[7]) == 1'b0)  falu_work1 <= {((falu_work1[58:33]) + (26'b00000000000000000000000001)), 33'b000000000000000000000000000000000} ; 
                              else                   falu_work1 <= falu_work1 + 2'b10 ; 
                           end 
                           falu_fsm <= falu_rtc ; 
                        end
               falu_rtc :
                        begin
                           if ((falu_work1[58]) == 1'b1)
                           begin
                              falu_work1 <= {1'b0, falu_work1[58:1]} ; 
                              falu_ccw <= falu_ccw + 1'b1 ; 
                           end 
                           falu_fsm <= falu_res ; 
                        end
               falu_res :
                        begin
                           falu_output[63] <= falu_fps[3] ; 
                           falu_output[62:55] <= falu_ccw[7:0] ; 
                           falu_output[54:0] <= falu_work1[56:2] ; 
                           falu_done <= 1'b1 ; 
                           falu_fsm <= falu_idle ; 
                           if (falu_ccw[7:0] == 8'b00000000) begin
                              falu_fps[2] <= 1'b1 ; 
                           end
                           else  falu_fps[2] <= 1'b0 ; 
                           if ((falu_ccw[9]) == 1'b1 | falu_ccw[9:0] == 10'b0000000000)  begin
                                 // are underflow traps enabled?
                              if ((fps[10]) == 1'b1)  falu_pending_fiu <= 1'b1 ; // yes, set flag
                              else                    falu_fsm <= falu_zres ; // traps are not enabled, output is zero
                           end
                           else if ((falu_ccw[8]) == 1'b1)  begin
                              falu_fps[1] <= 1'b1 ; // set the flag
                                 // are overflow traps enabled?
                              if ((fps[9]) == 1'b1)  falu_pending_fiv <= 1'b1 ; // yes, set flag
                              else                   falu_fsm <= falu_zres ; // traps are not enabled, output is zero
                           end 
                        end
               falu_zres :
                        begin
                           falu_output <= {64{1'b0}} ; 
                           falu_fps[3] <= 1'b0 ; 
                           falu_fps[2] <= 1'b1 ; 
                           falu_fps[0] <= 1'b0 ; 
                           falu_done <= 1'b1 ; 
                           falu_fsm <= falu_idle ; 
                        end
               falu_idle :
                        begin
                           falu_done <= 1'b0 ; 
                           falu_ccw <= {10{1'b0}} ; 
                           falu_work1 <= {59{1'b0}} ; 
                           falu_work2 <= {59{1'b0}} ; 
                           falu_flag1 <= 1'b0 ; 
                        end
            endcase 
         end 
      end
      else if (ir_fpsop2 == 1'b1) begin
         case (ir[7:6])
            2'b00 :
                     begin
                        // clr(f/d)
                        falu_output <= {64{1'b0}} ; 
                        falu_fps[3:0] <= 4'b0100 ; 
                     end
            2'b01 :
                     begin
                        // tst(f/d)
                        falu_output <= falu_input ; 
                        if (falu_input[62:55] == 8'b00000000)  begin
                           falu_fps[2] <= 1'b1 ; 
                           falu_output <= {64{1'b0}} ; 
                        end
                        else  falu_fps[2] <= 1'b0 ; 
                        falu_fps[3] <= falu_input[63] ; 
                        falu_fps[1] <= 1'b0 ; 
                        falu_fps[0] <= 1'b0 ; 
                     end
            2'b10 :
                     begin
                        // abs(f/d)
                        falu_output <= {1'b0, falu_input[62:0]} ; 
                        if (falu_input[62:55] == 8'b00000000)   begin
                           falu_fps[2] <= 1'b1 ; 
                           falu_output <= {64{1'b0}} ; 
                        end
                        else  falu_fps[2] <= 1'b0 ; 
                        falu_fps[3] <= 1'b0 ; 
                        falu_fps[1] <= 1'b0 ; 
                        falu_fps[0] <= 1'b0 ; 
                     end
            2'b11 :
                     begin
                        // neg(f/d)
                        if ((falu_input[63]) == 1'b0)    begin
                           falu_output <= {1'b1, falu_input[62:0]} ; 
                           falu_fps[3] <= 1'b1 ; 
                        end
                        else   begin
                           falu_output <= {1'b0, falu_input[62:0]} ; 
                           falu_fps[3] <= 1'b0 ; 
                        end 
                        if (falu_input[62:55] == 8'b00000000)    begin
                           falu_output <= {64{1'b0}} ; 
                           falu_fps[2] <= 1'b1 ; 
                           falu_fps[3] <= 1'b0 ; 
                        end
                        else  falu_fps[2] <= 1'b0 ; 
                        falu_fps[1] <= 1'b0 ; 
                        falu_fps[0] <= 1'b0 ; 
                     end
            default :
                     begin
                        falu_output <= {64{1'bx}} ; 
                        falu_fps[3:0] <= 4'bXXXX ; 
                     end
         endcase 
         falu_output2 <= {64{1'bx}} ; 
      end
      else  begin
         falu_output <= {64{1'bx}} ; 
         falu_output2 <= {64{1'bx}} ; 
         falu_fps[3:0] <= 4'bXXXX ; 
      end 
   end  
end 
endmodule
