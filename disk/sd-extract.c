#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include "devtable.h"

void main(int argc, char* argv[]) {
    

FILE* card;
FILE* out;

char devname[100];
char* buf;
char* outbuf;
int bank=0,unit=0;
char filename[100];
int id;
int i;
uint32_t boffset, doffset, usize, cardoffset, realsize;
char DD[4];
int cyl;
int sec;
int adr;
int outpos=0;

if (argc < 3) {
    printf("Извлечение образа диска с SD-карты\n\
Формат командной строки:\n\
%s DEV TYPE [unit [bank [file]]]\n\n\
DEV   - имя устройства Sd-карты, например /dev/sdd\n\
TYPE  - тип диска, RK, DW, DX\n\
unit  - номер устройства для многодисковых устройств, по умолчанию 0\n\
bank  - номер банка дисков, по умолчанию 0\n\
file  - имя файла для записи извлекаемого образа, по умолчанию B#-DD#.DSK\n\n\
 Пример: %s /dev/sdc RK 2 1 - извлечение образа RK2: из банка 1\n\n",argv[0],argv[0]);
    return;
}

DD[0]=toupper((argv[2][0]));
DD[1]=toupper((argv[2][1]));
DD[3]=0;

// поиск устройства
for(id=0;;id++) {
    if (devtable[id].name == 0) {
        printf("Неверный тип устройства - %s\n",DD);
        return;
    }
    if (strncmp(DD,devtable[id].name,2) == 0) {
         doffset=devtable[id].doffset;
         usize=devtable[id].usize;
         realsize=devtable[id].realsize;
         break;
    }     
}
if (argc>3) unit=atoi(argv[3]);
if (argc>4) bank=atoi(argv[4]);                      
if (argc>5) strcpy(filename,argv[5]);
else sprintf(filename,"B%i-%s%i.dsk",bank,devtable[id].name,unit);

if (unit > devtable[id].maxdev) {
    printf("Недопустимый номер устройства - %i, максимально допустимый = %i",unit, devtable[id].maxdev);
    return;
}    

out=fopen(filename,"w");
if (out == 0) {
    printf("- ошибка открытия выходного файла %s\n",filename);
    return;
}
card=fopen(argv[1],"r");
if (card == 0) {
    printf("- ошибка открытия SD-карты %s\n",argv[1]);
    return;
}

buf=malloc(usize*512);
if (buf == 0) {
    printf("- недостаточно памяти под буфер\n");
    return;
}

cardoffset=bank*banksize+doffset+unit*usize;
printf("* Стартовый блок: %xh\n",cardoffset);
// вычитываем образ диска с карты
fseek(card,(cardoffset)*512,SEEK_SET);
fread(buf,512,usize,card);
fclose(card);

// Для дисков DX - делаем преобразование буфера
if (strncmp(DD,"DX",2) == 0) {
    outbuf=malloc(502*512);
    if (outbuf == 0) {
        printf("- недостаточно памяти под буфер\n");
        return;
    }
    for (cyl=0;cyl<77;cyl++) {
        for(sec=1;sec<27;sec++) {
            adr=((cyl<<5) + sec)*512;
            for(i=0;i<256;i+=2) outbuf[outpos++]=*(buf+adr+i);
        }
    }

free(buf);
buf=outbuf; // заменяем исходный буфер на преобразованный
}
printf("* Размер образа : %xh\n",usize);

// записываем образ диска в выходной файл
fwrite(buf,512,realsize,out);
fclose(out);
free(buf);
}

