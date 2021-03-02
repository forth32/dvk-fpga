`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Description   : Обработка команд модуля SDRAM, вызываемых переключением текущего состояния
//            
// Revision      : V1.0
// Additional Comments   :  
// 
////////////////////////////////////////////////////////////////////////////////
module sdram_cmd(
            clk,rst_n,
            sdram_byteenable,      
            sdram_cke,sdram_cs_n,sdram_ras_n,sdram_cas_n,sdram_we_n,sdram_ba,sdram_addr,
            sys_wraddr,sys_rdaddr,sdwr_byte,sdrd_byte,
            init_state,work_state,sys_r_wn,cnt_clk
         );
   //System signal
input clk;               //50MHz
input rst_n;            //Low level reset signal
   // SDRAM hardware interface
output sdram_cke;         // SDRAM clock valid signal
output sdram_cs_n;         //   SDRAM chip select signal
output sdram_ras_n;         //   SDRAM row address strobe
output sdram_cas_n;         //   SDRAM column address strobe
output sdram_we_n;         //   SDRAM write enable bit
output[1:0] sdram_ba;      //   SDRAM L-Bank address line
output[12:0] sdram_addr;   // SDRAM address bus

   // SDRAM package interface
input[1:0] sdram_byteenable;   
input[22:0] sys_wraddr;      // Address Scratchpad when writing SDRAM, (bit21-20) L-Bank address: (bit19-8) is the row address, (bit7-0) is the column address 
input[22:0] sys_rdaddr;      // Address Scratchpad when reading SDRAM, (bit21-20) L-Bank address: (bit19-8) is the row address, (bit7-0) is the column address 
input[8:0] sdwr_byte;      //Burst write SDRAM bytes (1-256)
input[8:0] sdrd_byte;      //Burst read SDRAM bytes (1-256)
   // SDRAM internal interface
input[3:0] init_state;      // SDRAM initialization status register
input[3:0] work_state;      // SDRAM read and write status register
input sys_r_wn;         // SDRAM read/write control signal
input[8:0] cnt_clk;      //Clock count   


`include "sdram_para.v"      // Contains SDRAM parameter definition module
parameter [3:0] latency=`cas_latency;
//-------------------------------------------------------------------------------
//-------------------------------------------------------------------------------
reg[4:0] sdram_cmd_r;   //   SDRAM operation command
reg[1:0] sdram_ba_r;    // выбор банка
reg[12:0] sdram_addr_r; // шина адреса SDRAM

// Распределение активной команды по шинам управления SDRAM
assign {sdram_cke,sdram_cs_n,sdram_ras_n,sdram_cas_n,sdram_we_n} = sdram_cmd_r;
assign sdram_ba = sdram_ba_r;
assign sdram_addr = sdram_addr_r;

wire[22:0] sys_addr;      //внутренняя шина адреса: (bit22-21) выбор банка, (bit20-8) адрес колонки RAS, (bit7-0) адрес строки CAS    
// Выбор источника адреса SDRAM 
assign sys_addr = sys_r_wn ? sys_rdaddr:sys_wraddr;
   
//******************************************************************************
//* Установка сигналов управления согласно выполняемой команде
//******************************************************************************
// Варианты рабочих состояний 
parameter[3:0]      W_ACTIVE=    4'd1   ;   //Valid, read and write
parameter[3:0]      W_TRCD   =    4'd2   ;   //Waiting effectively
parameter[3:0]      W_IDLE   =    4'd0   ;   //Idle state
//***********************************************************
parameter[3:0]      W_READ=       4'd3   ;   //Read data status
parameter[3:0]      W_CL   =    4'd4;      //Waiting for latency
parameter[3:0]      W_RD   =    4'd5   ;   //Reading data
parameter[3:0]      W_RWAIT=       4'd6   ;   //Precharge wait state after reading is completed
//************************************************************
parameter[3:0]      W_WRITE   =    4'd7   ;   //Write data status
parameter[3:0]      W_WD      = 4'd8   ;   //Write data
parameter[3:0]      W_TDAL   =    4'd9   ;   //Waiting to write data and end self-refresh
//*************************************************************
parameter[3:0]      W_AR      = 4'd10   ;   //Self refresh
parameter[3:0]      W_TRFC   =    4'd11;      //Self-refresh waiting

always @ (posedge clk or negedge rst_n) begin
   if(!rst_n) begin
    // начальное состояние автомата - после сброса
         sdram_cmd_r <= `CMD_INIT;  // состояние сброса-отключения
         sdram_ba_r <= 2'b11;
         sdram_addr_r <= 13'h1fff;
      end
   else
      case (init_state)
            `I_NOP,`I_TRP,`I_TRF1,`I_TRF2,`I_TMRD: begin
                  sdram_cmd_r <= `CMD_NOP;
                  sdram_ba_r <= 2'b11;
                  sdram_addr_r <= 13'h1fff;   
               end
            `I_PRE: begin
                  sdram_cmd_r <= `CMD_PRGE;
                  sdram_ba_r <= 2'b11;
                  sdram_addr_r <= 13'h1fff;
               end 
            `I_AR1,`I_AR2: begin
                  sdram_cmd_r <= `CMD_A_REF;
                  sdram_ba_r <= 2'b11;
                  sdram_addr_r <= 13'h1fff;                  
               end              
            `I_MRS: begin   
                  // настройка управляющего регистра
                  sdram_cmd_r <= `CMD_LMR;
                  sdram_ba_r <= 2'b00;   //Operating mode setting
                  sdram_addr_r <= {
                            3'b000,         //Operating mode setting
                            1'b1,         //Operation mode setting (here set to A9=0, ie burst read/burst write)
                            2'b00,         //Operating mode setting ({A8, A7}=00), the current operation is the mode register setting
                            latency,   // CAS latency 
                            1'b0,         //Burst transfer mode (set here as order, A3=b0)
                            3'b000         //Burst length (here set to 256, {A2, A1, A0} = 1111)
                        };
               end   
            `I_DONE:
             // команды рабочего режима
               case (work_state)
                     W_IDLE,W_TRCD,W_CL,W_TRFC,W_TDAL: begin
                           sdram_cmd_r <= `CMD_NOP;
                           sdram_ba_r <= 2'b11;
                           sdram_addr_r <= 13'h1fff;
                        end
                     W_ACTIVE: begin
                           sdram_cmd_r <= `CMD_ACTIVE;
                           sdram_ba_r <= sys_addr[22:21];   //L-Bank address
                           sdram_addr_r <= sys_addr[20:8];   //Row address
                        end
                     W_READ: begin
                           sdram_cmd_r <= `CMD_READ;
                           sdram_ba_r <= sys_addr[22:21];   //L-Bank address
                           sdram_addr_r <= {
                                       5'b00100,      // A10=1, set write completion to allow precharge
                                       sys_addr[7:0]   //Column address
                                    };
                        end
                     W_RD: begin
                           if(`end_rdburst) sdram_cmd_r <= `CMD_B_STOP;
                           else begin
                              sdram_cmd_r <= `CMD_NOP;
                              sdram_ba_r <= 2'b11;
                              sdram_addr_r <= 13'h1fff;                        
                           end
                        end                        
                     W_WRITE: begin
                           sdram_cmd_r <= `CMD_WRITE;
                           sdram_ba_r <= sys_addr[22:21];   //L-Bank address
                           sdram_addr_r <= {
                                       5'b00100,      // A10=1, set write completion to allow precharge
                                       sys_addr[7:0]   //Column address  
                                    };
                        end      
                     W_WD: begin
                           if(`end_wrburst) sdram_cmd_r <= `CMD_B_STOP;
                           else begin
                              sdram_cmd_r <= `CMD_NOP;
                              sdram_ba_r <= 2'b11;
                              sdram_addr_r <= 13'h1fff;                        
                           end
                        end                                       
                     W_AR: begin
                           sdram_cmd_r <= `CMD_A_REF;
                           sdram_ba_r <= 2'b11;
                           sdram_addr_r <= 13'h1fff;   
                        end
                     default: begin
                           sdram_cmd_r <= `CMD_NOP;
                           sdram_ba_r <= 2'b11;
                           sdram_addr_r <= 13'h1fff;   
                        end
                  endcase
            default: begin
                     sdram_cmd_r <= `CMD_NOP;
                     sdram_ba_r <= 2'b11;
                     sdram_addr_r <= 13'h1fff;   
                  end
         endcase
end

endmodule

