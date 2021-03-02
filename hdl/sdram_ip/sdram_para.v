`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Module Name   : sdram_para
//------------------------------------------------------------------------------

   // Варианты состояний инициализации
`define      I_NOP       4'd0      //Waiting for power-on 200us end of stable period
`define      I_PRE        4'd1      //Precharge state
`define      I_TRP        4'd2      //Waiting for precharge to complete tRP
`define      I_AR1        4'd3      //The first self-refresh
`define      I_TRF1       4'd4      //Waiting for the end of the first self-refresh   tRFC
`define      I_AR2        4'd5      //2nd self-refresh
`define      I_TRF2        4'd6      //Waiting for the second self-refresh to end   tRFC   
`define      I_MRS       4'd7      //Mode register setting
`define      I_TMRD       4'd8      //Waiting mode register setting is complete   tMRD
`define      I_DONE       4'd9      //loading finished


   //Delay parameter
`define   end_trp         cnt_clk_r   == TRP_CLK
`define   end_trfc      cnt_clk_r   == TRFC_CLK
`define   end_tmrd      cnt_clk_r   == TMRD_CLK
`define   end_trcd      cnt_clk_r   == TRCD_CLK-1
`define end_tcl         cnt_clk_r   == `cas_latency-1
`define end_rdburst      cnt_clk      == sdrd_byte-4//TREAD_CLK-4      //Issue a burst read interrupt command
`define   end_tread      cnt_clk_r   == sdrd_byte+2//TREAD_CLK+2      //TREAD_CLK+2
`define end_wrburst      cnt_clk      == sdwr_byte-1//TWRITE_CLK-1   //Issue a burst write interrupt command
`define   end_twrite      cnt_clk_r   == sdwr_byte-1//TWRITE_CLK-1
`define   end_tdal      cnt_clk_r   == TDAL_CLK   
`define   end_trwait      cnt_clk_r   == TRP_CLK

   //команды смены состояния
   //                   CKE CS RAS CAS WE
`define      CMD_INIT     5'b01111   // инициализация - CKE=0 CS=1
`define      CMD_NOP       5'b10111   // ожидание команды - CKE=1 CS=0
`define      CMD_ACTIVE    5'b10011   // начало команды(RAS) - RAS=0 CAS=1 WE=1
`define      CMD_READ    5'b10101      // чтение RAS=1 CAS=0 WE=1
`define      CMD_WRITE    5'b10100   // запись  RAS=1 CAS=0 WE=0
`define      CMD_B_STOP    5'b10110   // конец цепочки обмена RAS=1 CAS=1 WE=0
`define      CMD_PRGE    5'b10010      // закрытие строки - RAS=0 CAS=1 WE=0
`define      CMD_A_REF    5'b10001   // саморегенераци - RAS=0 CAS=0 WE=1
`define      CMD_LMR       5'b10000   // загрузка регистра управления - RAS=0 CAS=0 WE=0


//endmodule
