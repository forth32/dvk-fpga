`timescale 1ns / 1ps
////////////////////////////////////////////////////////////////////////////////
// Description   : SDRAM package control top module
// Revision      : V1.0
// Additional Comments   :  
// 
////////////////////////////////////////////////////////////////////////////////
/*-----------------------------------------------------------------------------
SDRAM interface description:
On power-on reset, SDRAM will automatically wait for 200us and then initialize, the specific mode register
See the sdram_ctrl module for settings.
SDRAM operation:
Control sys_en=1, sys_r_wn=0, sys_addr, sys_data_in for SDRAM data write
Operation; control sys_en=1, sys_r_wn=1, sys_addr can read data from sys_data_out.
At the same time, you can check whether the reading and writing is completed by querying the status of sdram_busy.
-----------------------------------------------------------------------------*/
module sdram_top(
            clk,rst_n,sdram_byteenable,
            sdram_wr_req,sdram_rd_req,sdram_wr_ack,sdram_rd_ack,
            sys_wraddr,sys_rdaddr,
            sys_data_in,sys_data_out,sdwr_byte,sdrd_byte,//sdram_busy,sys_dout_rdy,
            /*sdram_clk,*/sdram_cke,sdram_cs_n,sdram_ras_n,sdram_cas_n,
            sdram_we_n,sdram_ba,sdram_addr,sdram_data,sdram_init_done
         );

input clk;      //System clock, 100MHz
input rst_n;   //Reset signal, active low

   // SDRAM package interface
input[1:0] sdram_byteenable;   
input sdram_wr_req;         //System write SDRAM request signal
input sdram_rd_req;         //System read SDRAM request signal
output sdram_wr_ack;      //The system writes the SDRAM response signal as the output valid signal of wrFIFO
output sdram_rd_ack;      //System read SDRAM response signal
input[22:0] sys_wraddr;      // When writing SDRAM, the address register, (bit22-21) L-Bank address: (bit20-8) is the row address, and (bit7-0) is the column address.
input[22:0] sys_rdaddr;      // Address Scratchpad when reading SDRAM, (bit22-21) L-Bank address: (bit20-8) is the row address, (bit7-0) is the column address 
input[15:0] sys_data_in;   //When writing SDRAM, the data register, 4 burst read and write word data, the default is 00 address bit15-0; 01 address bit31-16; 10 address bit47-32; 11 address bit63-48
output[15:0] sys_data_out;   //Data Scratchpad when reading SDRAM, (format is the same as above)
input[8:0] sdwr_byte;      //Burst write SDRAM bytes (1-256)
input[8:0] sdrd_byte;      //Burst read SDRAM bytes (1-256)
//output sdram_busy;         // SDRAM busy flag, high indicates that SDRAM is in operation
//output sys_dout_rdy;      // SDRAM data output completion flag
output   sdram_init_done;   //System initialization signal

   // FPGA and SDRAM hardware interface
//output sdram_clk;         // SDRAM clock signal
output sdram_cke;         // SDRAM clock valid signal
output sdram_cs_n;         // SDRAM chip select signal
output sdram_ras_n;         // SDRAM row address strobe
output sdram_cas_n;         // SDRAM column address strobe
output sdram_we_n;         // SDRAM write enable bit
output[1:0] sdram_ba;      // SDRAM L-Bank address line
output[12:0] sdram_addr;   // SDRAM address bus
inout[15:0] sdram_data;      // SDRAM data bus

   // SDRAM internal interface
wire[3:0] init_state;   // SDRAM initialization register
wire[3:0] work_state;   // SDRAM working status register
wire[8:0] cnt_clk;      //Clock count
wire sys_r_wn;         // SDRAM read/write control signal
            
sdram_ctrl      module_001(      // SDRAM state control module
                           .clk(clk),                  
                           .rst_n(rst_n),
                           .sdram_wr_req(sdram_wr_req),
                           .sdram_rd_req(sdram_rd_req),
                           .sdram_wr_ack(sdram_wr_ack),
                           .sdram_rd_ack(sdram_rd_ack),
                           .sdwr_byte(sdwr_byte),
                           .sdrd_byte(sdrd_byte),                     
                     //      .sdram_busy(sdram_busy),
                     //      .sys_dout_rdy(sys_dout_rdy),
                           .sdram_init_done(sdram_init_done),
                           //
                           .init_state(init_state),
                           .work_state(work_state),
                           .cnt_clk(cnt_clk),
                           .sys_r_wn(sys_r_wn)
                        );

sdram_cmd      module_002(      // SDRAM command module
                           .clk(clk),
                           .rst_n(rst_n),
                           .sdram_byteenable(sdram_byteenable),
                           .sdram_cke(sdram_cke),      
                           .sdram_cs_n(sdram_cs_n),   
                           .sdram_ras_n(sdram_ras_n),   
                           .sdram_cas_n(sdram_cas_n),   
                           .sdram_we_n(sdram_we_n),   
                           .sdram_ba(sdram_ba),         
                           .sdram_addr(sdram_addr),                           
                           .sys_wraddr(sys_wraddr),   
                           .sys_rdaddr(sys_rdaddr),
                           .sdwr_byte(sdwr_byte),
                           .sdrd_byte(sdrd_byte),      
                           .init_state(init_state),   
                           .work_state(work_state),
                           .sys_r_wn(sys_r_wn),
                           .cnt_clk(cnt_clk)
                        );

sdram_wr_data   module_003(      // SDRAM data read and write module
                           .clk(clk),
                           .rst_n(rst_n),
                     //      .sdram_clk(sdram_clk),
                           .sdram_data(sdram_data),
                           .sys_data_in(sys_data_in),
                           .sys_data_out(sys_data_out),
                           .work_state(work_state),
                           .cnt_clk(cnt_clk)
                        );
endmodule

