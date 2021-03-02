#include <stdio.h>
#include <stdint.h>

void main(int argc,char* argv[]) {
  
FILE* in;
FILE* fch;
uint32_t pos;
uint32_t y,x,bit,byte,off,r,i;
uint8_t ch;
char filename[20];

uint8_t font[3072];
int code=-1;
char str[20];

if (argc<2) {
    printf(" Программа для замены отдельных символов в двоичном шрифтовом файле.\n\
Запуск: %s <font bin file> [code]\n\n\
code - позиция заменяемого символа, если не указан - заменяются символы всех найденных файлов-образов\n\n\
Файлы-образы должны лежать в текущем каталоге. Они имеют имя xx.fnt, где xx - hex-номер позиции символа\n\n",argv[0]);
    return;
}    


in=fopen(argv[1],"r+");
if (in == 0) {
    printf("Ошибка открытия входного файла %s\n",argv[1]);
    return;
}    

if (argc>2) sscanf(argv[2],"%x",&code);

fread(font,1,3072,in);
//
//                   bit        byte
// R R R R C C C   d d d d d   d d d
//
//  R - row 0 - 11
//  C - col 0 - 7
//  d - char 0 - 256

for(pos=0;pos<256;pos++) {
  if (code != pos && code != -1) continue;  
  sprintf(filename,"%02x.fnt",pos);
  fch=fopen(filename,"r");
  if (fch == 0) {
      if (code != -1) {
          printf("Файл %s не найден\n",filename);
          return;
      }    
      continue;
  }    
  printf("Замена шрифта %02x\n",pos);
  for(y=0;y<12;y++) {
    fgets(str,20,fch);
    if (feof(in)) {
      printf("\n Неожиданный конец файла %s\n",filename);
      return;
    }
    for(x=0;x<8;x++) {
      bit=(y<<11)|(x<<8)|pos;
      byte=bit>>3;
      ch=font[byte];
      off=bit&7;
      if ((str[x] != '.')&&(str[x] != ' ')) font[byte] |= (1<<off); // устанавливаем в 1
      else font[byte] &= ~(1<<off); // сбрасываем бит в 0
    }
  }
}  
fseek(in,0,SEEK_SET);
fwrite(font,1,3072,in);
fclose(in);
}
