#include <stdio.h>
#include <stdint.h>

void main(int argc,char* argv[]) {

uint32_t pos;
uint32_t y,x,bit,byte,off,r,i;
uint8_t ch;
int code=-1;

uint8_t font[3072];
    
if (argc<2) {
 printf(" Программа для просмотра образов шрифтов, содержащихся в двоичном шрифтовом файле\n\
Запуск: %s <font bin file> [code]\n\
code - позиция просматриваемого символа, если не указан - выводятся образы всех символов\n",argv[0]);
 return;
} 
FILE* in=fopen(argv[1],"r"); // font.dat
if (in == 0) {
    printf("Ошибка открытия входного файла %s\n",argv[1]);
    return;
}    
fread(font,1,3072,in);
fclose(in);

if (argc>2) sscanf(argv[2],"%x",&code);
//
//                   bit        byte
// R R R R C C C   d d d d d   d d d
//
//  R - row 0 - 11
//  C - col 0 - 7
//  d - char 0 - 256

//sscanf(argv[1],"%x",&pos);
//if(pos>255) return;


for(pos=0;pos<256;pos++) {

if (pos == code || code == -1) {    
  printf("\n\n --- Позиция %02x ---",pos);
  printf("\n     01234567");
  for(y=0;y<12;y++) {
    printf("\n %02i: ",y);
    for(x=0;x<8;x++) {
      bit=(y<<11)|(x<<8)|pos;
      byte=bit>>3;
      ch=font[byte];
      off=bit&7;
      r=(ch>>off)&1;
      if (r != 0) printf("@");
      else printf(".");
    }
  }
printf("\n");
}
}
}
