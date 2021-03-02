#!/bin/sh
./macro11 -o ksm-firmware.obj -l ksm-firmware.lst ksm-firmware.mac
./rt11obj2bin ksm-firmware.obj > ksm-firmware.map
srec_cat ksm-firmware.obj.bin -binary --byte-swap 2 -o ksm-firmware.mif -Memory_Initialization_File 16 -obs=2
#srec_cat ksm-firmware.obj.bin -binary --byte-swap 2 -o ksm-firmware.mem -Vmem 16 -obs=2
