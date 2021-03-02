#include <stdio.h>
#include <stdint.h>
void main(int argc, char* argv[]) {
  
uint8_t buf[8];
uint8_t ch;
int bit,bitpos;

uint32_t adr;
uint8_t prev=2;

if (argc<3) {
  printf("\n Преобразователь двоичного файла шрифтов в формат Altera MIF для загрузки в память FPGA\n\
 Формат команды:\n  %s infile.bin outfile.mif\n\n",argv[0]);
  return;
}  

FILE* in=fopen(argv[1],"r");
if (in == 0) {
    printf("Ошибка открытия входного файла %s\n",argv[1]);
    return;
}    
FILE* out=fopen(argv[2],"w");
if (out == 0) {
    printf("Ошибка открытия выходного файла %s\n",argv[1]);
    return;
}    

// MIF-заголовок
fprintf(out,"WIDTH=1;\nDEPTH=24576;\nADDRESS_RADIX=HEX;\nDATA_RADIX=BIN;\n\nCONTENT BEGIN");

for(adr=0;adr<24576;adr++) {
  bitpos=adr&7;
  if (bitpos == 0) ch=fgetc(in); // через каждые обработанные 8 бит загружаем очередной байт
  bit=(ch>>bitpos)&1;            // выделяем очередной бит
  fprintf(out,"\n%04x: %1i;",adr,bit); // и выводим в mif его значение
}
// MIF-терминатор
fprintf(out,"\nEND;\n");
}
