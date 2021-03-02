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

## DATE    "Fri Feb 19 07:45:51 2021"

##
## DEVICE  "EP4CE22F17C7"
##

derive_clock_uncertainty

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
create_generated_clock -source [get_pins {cpu|corepll|altpll_component|auto_generated|pll1|inclk[0]}] -divide_by 1 -multiply_by 2 [get_pins {cpu|corepll|altpll_component|auto_generated|pll1|clk[0]}]
create_generated_clock -source [get_pins {cpu|corepll|altpll_component|auto_generated|pll1|inclk[0]}] -divide_by 1 -multiply_by 2 -phase 180 [get_pins {cpu|corepll|altpll_component|auto_generated|pll1|clk[1]}]
create_generated_clock -source [get_pins {cpu|corepll|altpll_component|auto_generated|pll1|inclk[0]}] -divide_by 4 -multiply_by 1 [get_pins {cpu|corepll|altpll_component|auto_generated|pll1|clk[2]}]


#**************************************************************
# Set Clock Latency
#**************************************************************



#**************************************************************
# Set Clock Uncertainty
#**************************************************************



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

