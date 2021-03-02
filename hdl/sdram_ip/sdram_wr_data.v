`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Description   : SDRAM data read and write module
////////////////////////////////////////////////////////////////////////////////
module sdram_wr_data(
               clk,rst_n,
               /*sdram_clk,*/sdram_data,
               sys_data_in,sys_data_out,
               work_state,cnt_clk
            );
   //System signal
input clk;      //System clock, 100MHz
input rst_n;   //Reset signal, active low
   // SDRAM hardware interface
//output sdram_clk;         // SDRAM clock signal
inout[15:0] sdram_data;      // SDRAM data bus
   // SDRAM package interface
input[15:0] sys_data_in;   //Data register when writing SDRAM
output[15:0] sys_data_out;   //Data register when reading SDRAM

   // SDRAM internal interface
input[3:0] work_state;   //Data status register when reading and writing SDRAM
input[8:0] cnt_clk;      //Clock count

`include "sdram_para.v"      // Contains SDRAM parameter definition module

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

//------------------------------------------------------------------------------
//Data write control
//------------------------------------------------------------------------------
reg[15:0] sdr_din;   //Burst data write register
reg sdr_dlink;      // SDRAM data bus input and output control

   //Send the data to be written to the SDRAM data bus
always @ (posedge clk or negedge rst_n) 
   if(!rst_n) sdr_din <= 16'd0;   //Burst data write register reset
   else if((work_state == W_WRITE) | (work_state == W_WD)) sdr_din <= sys_data_in;   //Continuously write 256 16-bit data stored in wrFIFO

   //Generate bidirectional data line direction control logic
always @ (posedge clk or negedge rst_n) 
   if(!rst_n) sdr_dlink <= 1'b0;
   else if((work_state == W_WRITE) | (work_state == W_WD)) sdr_dlink <= 1'b1;
   else sdr_dlink <= 1'b0;

assign sdram_data = sdr_dlink ? sdr_din:16'hzzzz;

//------------------------------------------------------------------------------
//Data readout control
//------------------------------------------------------------------------------
reg[15:0] sdr_dout;   //Burst data read register

   //Read data from SDRAM
always @ (posedge clk or negedge rst_n)
   if(!rst_n) sdr_dout <= 16'd0;   //Burst data read register reset
   else if((work_state == W_RD)/* & (cnt_clk > 9'd0) & (cnt_clk < 9'd10)*/) sdr_dout <= sdram_data;   //Continuously read 8B 16bit data storage to rdFIFO

assign sys_data_out = sdr_dout;

//------------------------------------------------------------------------------

endmodule
