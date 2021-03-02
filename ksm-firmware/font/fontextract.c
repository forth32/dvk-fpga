#include <stdio.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>

void main(int argc,char* argv[]) {
  
FILE* in;
FILE* out;
uint32_t pos;
uint32_t y,x,bit,byte,off,r,i;
uint8_t ch;
char filename[20];
int32_t code=-1;

uint8_t font[3072];

if (argc<2) {
    printf(" Программа предназначена для преобразования двоичного файла шрифтов в набор текстовых образов\n\
Все образы складываются в каталог out/ под именем xx.fnt, xx - позиция шрифта в исходном файле\n\
Запуск: %s <font bin file> [code]\n\
code - позиция исзвлекаемого символа, если не указано - извлекаются все символы\n",argv[0]);
    return;
}    
in=fopen(argv[1],"r");
if (in == 0) {
    printf("Ошибка открытия фходного файла %s\n",argv[1]);
    return;
}    
fread(font,1,3072,in);
fclose(in);
//
//                   bit        byte
// R R R R C C C   d d d d d   d d d
//
//  R - row 0 - 11
//  C - col 0 - 7
//  d - char 0 - 256

mkdir ("out",0777);
if (argc > 2) sscanf(argv[2],"%x",&code);

for(pos=0;pos<256;pos++) {
  if ((pos != code) && (code != -1)) continue;  
  sprintf(filename,"out/%02x.fnt",pos);
  out=fopen(filename,"w");
  for(y=0;y<12;y++) {
    for(x=0;x<8;x++) {
      bit=(y<<11)|(x<<8)|pos;
      byte=bit>>3;
      ch=font[byte];
      off=bit&7;
      r=(ch>>off)&1;
      if (r != 0) fprintf(out,"O");
      else fprintf(out,".");
    }
    fprintf(out,"\n");
  }
  fclose(out);
}
}
