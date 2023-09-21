#!/bin/bash
progname=mboot
base=$((0173252))
ov=4
chunk=256
./macro11 -o $progname.obj -l $progname.lst $progname.mac
./rt11obj2bin $progname.obj > $progname.map
cp $out.orig rom.bin
start=$(($ov*$chunk+$base-0173000))
dd conv=notrunc if=$progname.obj.bin of=rom.bin bs=1 seek=$start
# Коррекция DD на DM
echo -n "M" | dd conv=notrunc of=rom.bin  bs=1 seek=$((0x1e3))
# Коррекция DL на DB
echo -n "B" | dd conv=notrunc of=rom.bin  bs=1 seek=$((0x1cb))
# коррекция адреса перехода и номеров оверлеев - off=1CE  adr=173372  ov l=4 h=5
echo -e -n "\0372\0366\0004\0005" | dd conv=notrunc of=rom.bin  bs=1 seek=$((0x1ce))
# Вписываем MY  173472 = 367 072
echo -e -n "MY\0300\0000\0072\0367\0004\0005" | dd conv=notrunc of=rom.bin  bs=1 seek=$((0x1ea))
cp $out.bin ../../boot_diag_rom.bin
