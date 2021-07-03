## Generated SDC file "top.sdc"

## Copyright (C) 2018  Intel Corporation. All rights reserved.
## Your use of Intel Corporation's design tools, logic functions 
## and other software and tools, and its AMPP partner logic 
## functions, and any output files from any of the foregoing 
## (including device programming or simulation files), and any 
## associated documentation or information are expressly subject 
## to the terms and conditions of the Intel Program License 
## Subscription Agreement, the Intel Quartus Prime License Agreement,
## the Intel FPGA IP License Agreement, or other applicable license
## agreement, including, without limitation, that your use is for
## the sole purpose of programming logic devices manufactured by
## Intel and sold by Intel or its authorized distributors.  Please
## refer to the applicable agreement for further details.


## VENDOR  "Altera"
## PROGRAM "Quartus Prime"
## VERSION "Version 18.0.0 Build 614 04/24/2018 SJ Standard Edition"

## DATE    "Tue Mar 23 12:00:10 2021"

##
## DEVICE  "EP4CE55F23C8"
##


#**************************************************************
# Time Information
#**************************************************************

set_time_format -unit ns -decimal_places 3



#**************************************************************
# Create Clock
#**************************************************************

create_clock -name {clk} -period 20.000 -waveform { 0.000 10.000 } [get_ports {clk50}]


#**************************************************************
# Create Generated Clock
#**************************************************************

derive_pll_clocks

#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************

set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}]  0.030  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}]  0.030  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}]  0.030  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}]  0.030  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {clk}] -setup 0.100  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {clk}] -hold 0.070  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {clk}] -setup 0.100  
set_clock_uncertainty -rise_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {clk}] -hold 0.070  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[2]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[1]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {clk}] -setup 0.100  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -rise_to [get_clocks {clk}] -hold 0.070  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {clk}] -setup 0.100  
set_clock_uncertainty -fall_from [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -fall_to [get_clocks {clk}] -hold 0.070  
set_clock_uncertainty -rise_from [get_clocks {clk}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -setup 0.070  
set_clock_uncertainty -rise_from [get_clocks {clk}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -hold 0.100  
set_clock_uncertainty -rise_from [get_clocks {clk}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -setup 0.070  
set_clock_uncertainty -rise_from [get_clocks {clk}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -hold 0.100  
set_clock_uncertainty -rise_from [get_clocks {clk}] -rise_to [get_clocks {clk}]  0.020  
set_clock_uncertainty -rise_from [get_clocks {clk}] -fall_to [get_clocks {clk}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {clk}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -setup 0.070  
set_clock_uncertainty -fall_from [get_clocks {clk}] -rise_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -hold 0.100  
set_clock_uncertainty -fall_from [get_clocks {clk}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -setup 0.070  
set_clock_uncertainty -fall_from [get_clocks {clk}] -fall_to [get_clocks {pll1|altpll_component|auto_generated|pll1|clk[0]}] -hold 0.100  
set_clock_uncertainty -fall_from [get_clocks {clk}] -rise_to [get_clocks {clk}]  0.020  
set_clock_uncertainty -fall_from [get_clocks {clk}] -fall_to [get_clocks {clk}]  0.020  


#**************************************************************
# Set Input Delay
#**************************************************************



#**************************************************************
# Set Output Delay
#**************************************************************



#**************************************************************
# Set Clock Groups
#**************************************************************



#**************************************************************
# Set False Path
#**************************************************************

set_false_path -from [get_keepers {topboard:kernel|wbc_rst:reset|key_down}] -to [get_keepers {topboard:kernel|wbc_rst:reset|key_syn[0]}]
set_false_path -from {topboard:kernel|wbc_rst:reset|pwr_event} -to {topboard:kernel|wbc_rst:reset|count_dc[*]}
set_false_path -from {topboard:kernel|wbc_rst:reset|pwr_event} -to {topboard:kernel|wbc_rst:reset|count_ac[*]}
set_false_path -from {topboard:kernel|wbc_rst:reset|pwr_event} -to {topboard:kernel|wbc_rst:reset|count_pw[*]}
set_false_path -from {topboard:kernel|wbc_rst:reset|pwr_event} -to {topboard:kernel|wbc_rst:reset|sys_dclo}
set_false_path -from {topboard:kernel|wbc_rst:reset|pwr_event} -to {topboard:kernel|wbc_rst:reset|sys_rst}
set_false_path -from {topboard:kernel|wbc_rst:reset|pwr_event} -to {topboard:kernel|wbc_rst:reset|sys_aclo}
set_false_path -from {topboard:kernel|wbc_rst:reset|pwr_event} -to {topboard:kernel|wbc_rst:reset|pwr_rst}

set_false_path -from {topboard:kernel|ksm:terminal|vga:video|row[*]} -to {topboard:kernel|kgd:graphics|wb_dat_o[*]}
set_false_path -from {topboard:kernel|ksm:terminal|vga:video|col[*]} -to {topboard:kernel|kgd:graphics|wb_dat_o[*]}

set_false_path -from {topboard:kernel|ksm:terminal|wbc_uart:uart|tx_csr_brk} -to {topboard:kernel|wbc_uart:uart1|rx_rdata_reg[0]}
set_false_path -from {topboard:kernel|ksm:terminal|wbc_uart:uart|tx_csr_brk} -to {topboard:kernel|wbc_uart:uart2|rx_rdata_reg[0]}
set_false_path -from {topboard:kernel|wbc_uart:uart1|tx_csr_brk} -to {topboard:kernel|ksm:terminal|wbc_uart:uart|rx_rdata_reg[0]}
set_false_path -from {topboard:kernel|wbc_uart:uart2|tx_csr_brk} -to {topboard:kernel|ksm:terminal|wbc_uart:uart|rx_rdata_reg[0]}

set_false_path -from {topboard:kernel|ksm:terminal|wbc_uart:uart|tx_shr[0]} -to {topboard:kernel|wbc_uart:uart1|rx_rdata_reg[0]}
set_false_path -from {topboard:kernel|ksm:terminal|wbc_uart:uart|tx_shr[0]} -to {topboard:kernel|wbc_uart:uart2|rx_rdata_reg[0]}
set_false_path -from {topboard:kernel|wbc_uart:uart1|tx_shr[0]} -to {topboard:kernel|ksm:terminal|wbc_uart:uart|rx_rdata_reg[0]}
set_false_path -from {topboard:kernel|wbc_uart:uart2|tx_shr[0]} -to {topboard:kernel|ksm:terminal|wbc_uart:uart|rx_rdata_reg[0]}

set_false_path -from {topboard:kernel|ksm:terminal|vregs:videoreg|vtcsr[0]} -to {topboard:kernel|wbc_uart:uart1|rx_rdata_reg[0]}
set_false_path -from {topboard:kernel|ksm:terminal|vregs:videoreg|vtcsr[0]} -to {topboard:kernel|wbc_uart:uart2|rx_rdata_reg[0]}

set_false_path -from {topboard:kernel|ksm:terminal|vregs:videoreg|vtcsr[*]} -to {topboard:kernel|wbc_uart:uart1|baud_div[*]}
set_false_path -from {topboard:kernel|ksm:terminal|vregs:videoreg|vtcsr[*]} -to {topboard:kernel|wbc_uart:uart2|baud_div[*]}

set_false_path -from {topboard:kernel|wbc_rst:reset|sys_aclo} -to {topboard:kernel|mc1201_02:cpu|vm2_wb:cpu|io_st[*]}
#**************************************************************
# Set Multicycle Path
#**************************************************************



#**************************************************************
# Set Maximum Delay
#**************************************************************



#**************************************************************
# Set Minimum Delay
#**************************************************************



#**************************************************************
# Set Input Transition
#**************************************************************

