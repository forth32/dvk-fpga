#include <stdio.h>
#include <stdint.h>
void main (int argc, char* argv[]) {

uint16_t rbuf[4096];    
uint32_t sum=0;
int i;

if (argc<2) return;
FILE* rom=fopen(argv[1],"r+");
if (rom == 0) return;

fread(rbuf,1,8192,rom);
for(i=0;i<013776/2;i++) {
    sum+=rbuf[i];
    if ((sum&0x10000) != 0) {
        sum&=0xffff;
        sum++;
    }
}
printf("sum = %06o\n",sum);
rbuf[013776/2]=sum;
fseek(rom,0,SEEK_SET);
fwrite(rbuf,1,8192,rom);
fclose(rom);
}
