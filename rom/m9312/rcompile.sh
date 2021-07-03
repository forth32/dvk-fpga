#!/bin/sh
#./macro11 -o m9312-bootloaders.obj -l m9312-bootloaders.lst m9312-bootloaders.mac
#./rt11obj2bin-t m9312-bootloaders.obj>m9312-bootloaders.map
#srec_cat m9312-bootloaders.obj.bin -binary --byte-swap 2 -o m9312-bootloaders.mif -Memory_Initialization_File 16 -obs=2
#srec_cat m9312-bootloaders.obj.bin -binary -fill 0x00 0x0000 0x4000 -byte-swap 2 -o m9312-bootloaders.mem --VMem 16

#./macro11 -o m9312-console.obj -l m9312-console.lst m9312-console.mac
#./rt11obj2bin-t m9312-console.obj>m9312-console.map
#srec_cat m9312-console.obj.bin -binary --byte-swap 2 -o m9312-console.mif -Memory_Initialization_File 16 -obs=2
#srec_cat m9312-console.obj.bin -binary -fill 0x00 0x0000 0x4000 -byte-swap 2 -o m9312-console.mem --VMem 16

./macro11 -o bootrom.obj -l bootrom.lst bootrom.mac
./rt11obj2bin-t bootrom.obj > bootrom.map
srec_cat bootrom.obj.bin -binary -fill 0x00 0x0000 0x400 --byte-swap 2 -o bootrom.mif -Memory_Initialization_File 16 -obs=2

