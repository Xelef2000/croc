# Copyright (c) 2025 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# Synthesis script template for VLSI-2 EX04     
# To use this script in Yosys (from the `yosys` directory):                  
# % > source scripts/yosys_flow.tcl
#
# Authors:
# - Bowen Wang      <bowwang@iis.ee.ethz.ch>
# - Enrico Zelioli  <ezelioli@iis.ee.ethz.ch>
# Last Modification: 09.03.2025

#######################################
###### Read Technology Libraries ######
#######################################

# TODO: student task 2
# Read liberty files for standard cells, SRAM macros, and I/O pads

yosys read_liberty -lib ../technology/lib/RM_IHPSG13_1P_256x64_c2_bm_bist_typ_1p20V_25C.lib
yosys read_liberty -lib ../technology/lib/sg13g2_io_typ_1p2V_3p3V_25C.lib
yosys read_liberty -lib ../technology/lib/sg13g2_stdcell_typ_1p20V_25C.lib

#########################
###### Load Design ######
#########################

# TODO: student task 3 & 4
# 3.1: Enable Yosys SystemVerilog frontend
# 3.2: Load Croc chip design

yosys plugin -i slang.so
yosys read_slang --top croc_chip -F ../croc.flist --allow-use-before-declare --ignore-unknown-modules --keep-hierarchy

#########################
###### Elaboration ######
#########################

# TODO: student task 5
# 5.1 Resolve design hierarchy 
# 5.2 Convert processes to netlists
# 5.3 Export report and netlist

yosys hierarchy -check -top  croc_chip
yosys proc
yosys tee -q -o "reports/croc_parsed.rpt" stat
yosys write_verilog "croc.analysys.v" 

####################################
###### Coarse-grain Synthesis ######
####################################

# TODO: student task 6
# 6.1 Early-stage design check
# 6.2 First opt pass (no FF)
# 6.3 Extract FSM and write report
# 6.4 Perform wreduce
# 6.3 Infer memories and optimize register-files
# 6.4 Optimize flip-flops

yosys check
yosys opt -noff
yosys fsm
yosys tee -q -o "reports/croc_opt_fsm.rpt" stat
yosys wreduce
yosys peepopt
yosys opt -full
yosys memory
yosys opt_dff
yosys tee -q -o "reports/croc_opt_ff.rpt" stat

###########################################
###### Define target clock frequency ######
###########################################

# TODO: student task 7
# 7.1 Define clock period variable

set period_ps 10000

##################################
###### Fine-grain synthesis ######
##################################

# TODO: student task 9
# 9.1 Generic cell substitution
# 9.2 Generate report

yosys techmap
yosys tee -q -o "reports/croc_techmap.rpt" stat

############################
###### Flatten design ######
############################

# Before flattening the hierarchy to allow cross-module optimizations,
# preserve hierarchy of selected modules/instances.
# 't' means type as in select all instances of this type/module
# yosys-slang uniquifies all modules with the naming scheme:
# <module-name>$<instance-name> -> match for t:<module-name>$$
# Examples:
# yosys setattr -set keep_hierarchy 1 "t:croc_soc$*"
# yosys setattr -set keep_hierarchy 1 "t:cdc_*$*"

# TODO: student task 12 & 13
# 12.1 Flatten design
yosys select  -list tc_clk* tc_sram soc_ctrl_reg_top
yosys setattr -set keep_hierarchy 1 tc_clk* tc_sram soc_ctrl_reg_top
yosys flatten

################################
###### Technology Mapping ######
################################

# TODO: student task 10
# 10.1 Register mapping
# 10.2 Generate a report
# 10.3 Combinational logic mapping
# 10.4 Export netlist

yosys dfflibmap -liberty ../technology/lib/sg13g2_stdcell_typ_1p20V_25C.lib
yosys tee -q -o "reports/liberty.rpt" stat -liberty ../technology/lib/sg13g2_stdcell_typ_1p20V_25C.lib
yosys abc -liberty ../technology/lib/sg13g2_stdcell_typ_1p20V_25C.lib -D ${period_ps} -constr src/abc.constr -script scripts/abc-opt.script
yosys write_verilog "out/croc.v" 


#######################################
###### Prepare for OpenROAD flow ######
#######################################

# TODO: student task 14
# 14.1 Split multi-bit nets
# 14.2 Replace undefined constants
# 14.3 Replace constant bits with driver cells
# 14.4 Export
yosys splitnets
yosys setundef -zero
yosys hilomap
yosys write_verilog "out/final-croc.v" 
exit
