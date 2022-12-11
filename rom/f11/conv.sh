#!/bin/sh
srec_cat boot_diag_rom.bin -binary --byte-swap 2 -o boot_diag_rom.mif -Memory_Initialization_File 16 -obs=2

