#!/bin/sh
./macro11 -o m9312-bootloaders.obj -l m9312-bootloaders.lst m9312-bootloaders.mac
./rt11obj2bin-t m9312-bootloaders.obj>m9312-bootloaders.map

./macro11 -o m9312-console.obj -l m9312-console.lst m9312-console.mac
./rt11obj2bin-t m9312-console.obj>m9312-console.map

cat m9312-console.obj.bin m9312-bootloaders.obj.bin > bootrom.bin
srec_cat bootrom.bin -binary --byte-swap 2 -fill 0x00 0x0000 0x400 -o bootrom.mif -Memory_Initialization_File 16 -obs=2
srec_cat bootrom.bin -binary -fill 0x00 0x0000 0x400 -byte-swap 2 -o bootrom.mem --VMem 16
