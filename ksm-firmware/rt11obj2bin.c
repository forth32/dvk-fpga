#include <string.h>
#include <stdio.h>
#include <stdint.h>

//*******************************************
//* Преобразование Radix-50 в строку ASCII  *
//*******************************************

char* rad2ascii(uint16_t rad,char* buf) {

// таблица кодов RADIX-50  
char code[] = {' ','A','B','C','D','E','F','G','H','I','J','K','L',
               'M','N','O','P','Q','R','S','T','U','V','W','X','Y',
	       'Z','$','.','%','0','1','2','3','4','5','6','7','8','9'};
 
unsigned int d1,d2,d3,t;

d1=rad/1600;
t=rad-d1*1600;
d2=t/40;
d3=t-d2*40;
buf[0]=code[d1];
buf[1]=code[d2];
buf[2]=code[d3];
return buf;
}


//***********************
//* Дамп области памяти *
//***********************

void dump(char buffer[],int len,uint32_t base) {
unsigned int i,j;
char ch;
uint16_t val;

for (i=0;i<len;i+=16) {
  printf("%06o: ",base+i);
  for (j=0;j<16;j+=2){
   if ((i+j) < len) {
     val=*((uint16_t*)&buffer[i+j]);
     printf("%06o ",val);
   }  
   else printf("       ");
  }
  printf(" *");
  for (j=0;j<16;j++) {
   if ((i+j) < len) {
    ch=buffer[i+j];
    if ((ch < 0x20)|(ch > 0x80)) ch='.';
    putchar(ch);
   }
   else putchar(' ');
  }
  printf("*\n");
}
}

//*********************************
//*  Обработка записей типа GSD    
//*********************************
void process_gsd(uint8_t* buf, int len) {

struct gsdrecord {
  uint16_t name[2];
  uint8_t flags;
  uint8_t type;
  uint16_t value;
};  
char sname[7];
char* rtypes[]={"Module name","CSECT","Internal symbol name","Transfer Address","Global symbol name","PSECT","IDENT","VSECT"};


struct gsdrecord* gsdptr;  
  
int nr;
for(nr=0;(nr*sizeof(struct gsdrecord))<len;nr++) {
 gsdptr=(struct gsdrecord*)(buf+nr*sizeof(struct gsdrecord));
 rad2ascii(gsdptr->name[0],sname);
 rad2ascii(gsdptr->name[1],sname+3);
 sname[6]=0;
 switch(gsdptr->type) {
   case 0:
     printf("\n Module name: %s",sname);
     break;
     
   case 1:
     printf("\n CSECT: %s, len=%i",sname,gsdptr->value);
     break;
     
   case 2:
     printf("\n ISD: %s",sname);
     break;
     
   case 3:
     printf("\n Transfer address: %s+%06o",sname,gsdptr->value);
     break;
     
   case 4:
     printf("\n Global symbol: %s = %06o",sname,gsdptr->value);
     fprintf(stderr,"! Global symbol: %s = %06o\n",sname,gsdptr->value);
     break;
     
   case 5:
     printf("\n PSECT: %s, len=%i",sname,gsdptr->value);
     break;
     
   case 6:
     printf("\n IDENT: %s",sname);
     break;
     
   case 7:
     printf("\n VSECT Mapper array: %s, len=%i",sname,gsdptr->value);     
     break;
 }    
}
}

//*********************************
//*  Обработка записей типа TXT    
//*********************************
uint32_t process_txt(uint8_t* buf, int len, uint8_t* outbuf) {

uint16_t load_addr;

load_addr=*((uint16_t*)buf);
printf("\n Load address: %6o - %04xh\n",load_addr,load_addr);
dump(buf+2,len-2,load_addr);
memcpy(outbuf+load_addr,buf+2,len-2);
return load_addr;
}

//*********************************
//*  Обработка записей типа RLD    
//*********************************
uint32_t process_rld(uint8_t* buf, int len, uint8_t* outbuf, uint32_t lastaddr) {

struct rld_record {
  uint8_t cmd;
  uint8_t disp;
  uint16_t d1;
  uint16_t d2;
  uint16_t d3;
};

char name[7]={0,0,0,0,0,0,0};
struct rld_record* rld;
uint8_t* ptr;
uint32_t addr;
uint32_t reloffset;
uint16_t cval;
//dump(buf,len,0);      
//printf("\n*********\n");

for (ptr=buf;ptr<(buf+len);) {
  rld=(struct rld_record*)ptr;
  addr=lastaddr+rld->disp-4;
  if (addr < 0x10000) cval=*((uint16_t*)(outbuf+addr));
  switch (rld->cmd&0x7f) {

    case 1:
      printf("\n Internal relocation: %06o: %06o (%06o)",addr,rld->d1,cval);
      if (rld->d1 != cval) printf(" --- ! diff ! ---");
      if ((rld->cmd&0x80) != 0) printf(" (B)");
      ptr+=4;
      break;
      
    case 2:
      rad2ascii(rld->d1,name);
      rad2ascii(rld->d2,name+3);
      printf("\n Global relocation: %s: %06o",name,addr);
      if ((rld->cmd&0x80) != 0) printf(" (B)");
      ptr+=6;
      break;
      
    case 3:
      printf("\n Internal displaced relocation:%06o: %06o (%06o)",addr,rld->d1,cval);
      // Патч относительных адресов
        reloffset=rld->d1-addr-2;
	*((uint16_t*)(outbuf+addr))=reloffset;
	printf("\n  * %6o ->%6o",rld->d1,reloffset);
      ptr+=4;
      break;
      
    case 4:
      rad2ascii(rld->d1,name);
      rad2ascii(rld->d2,name+3);
      printf("\n Global displaced relocation: %s: %6o",name,addr);
      if ((rld->cmd&0x80) != 0) printf(" (B)");
      ptr+=4;
      break;

    case 5:
      rad2ascii(rld->d1,name);
      rad2ascii(rld->d2,name+3);
      printf("\n Global additive relocation: %s+%06o: %6o",name,rld->d3,addr);
      if ((rld->cmd&0x80) != 0) printf(" (B)");
      ptr+=8;
      break;

    case 6:
      rad2ascii(rld->d1,name);
      rad2ascii(rld->d2,name+3);
      printf("\n Global additive displaced relocation: %s+%06o: %6o",name,rld->d3,addr);
      if ((rld->cmd&0x80) != 0) printf(" (B)");
      ptr+=8;
      break;

    case 7:
      rad2ascii(rld->d1,name);
      rad2ascii(rld->d2,name+3);
      printf("\n Location counter definition: psect %s: %06o",name,rld->d3);
      if ((rld->cmd&0x80) != 0) printf(" (B)");
      ptr+=8;
      break;

    case 010:
      printf("\n Location counter modification: new addr=%06o",rld->d1);
      if ((rld->cmd&0x80) != 0) printf(" (B)");
      ptr+=4;
      break;
      
    default:
      printf("\n undefined rld: %o",rld->cmd&0x7f);
      return 0;

  }      
}   
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
void main(int argc, char* argv[]) {
  
uint16_t len;
FILE* in;
int res;
int count=0;
uint8_t buf[0x10000];
uint8_t cs,csum;
uint8_t mb;
uint32_t i;
uint8_t rectype;
uint8_t outbuf[0x10000];
uint32_t maxaddr=0;
FILE* out;
char fname[64];
uint32_t lastaddr=0;

char* recnames[]={"UNKNOWN","GSD","ENDGSD","TXT","RLD","ISD","ENDMOD","LIBHD","ENDLIB"};

in=fopen(argv[1],"r");
if (in == 0) return;

strcpy(fname,argv[1]);
strcat(fname,".bin");
out=fopen(fname,"w");

while (!feof(in)) {
  mb=fgetc(in);
  if (feof(in)) break;
  if (mb == 0) continue;
  if (mb != 1) {
    printf("\n Нарушение структуры: magic1 != 1");
    return;
  }  
  mb=fgetc(in);
  if (feof(in)) break;
  if (mb != 0) {
    printf("\n Нарушение структуры: magic2 != 0");
    return;
  }  
  res=fread(&len,1,2,in);
  if (res != 2) break;
  fread(buf,1,len-4,in);
    
  rectype=buf[0];
  
  printf("\n Record %i: %02x(%s)\n",count++,rectype,recnames[rectype]);  
  switch(rectype) {
    case 1:
      process_gsd(buf+2,len-6);
      break;
    
    case 2:
      break;
      
    case 3:
      lastaddr=process_txt(buf+2,len-6,outbuf);
      res=lastaddr+len-8;
      if (res>maxaddr) maxaddr=res;
      break;

    case 4:
      process_rld(buf+2,len-6,outbuf,lastaddr);
      break;
            
    default:
      break;
 //     dump(buf,len-4,0);
  }
  
  cs=fgetc(in);
  csum=1+(len&0xff) + ((len>>8)&0xff);
  for(i=0;i<(len-4);i++) csum+=buf[i];
  csum=~csum+1;
  if (cs != csum)   printf("\n!!! Несоответствие контрольной суммы: %08x  %08x",csum,cs);
  
  printf("\n------------------------------------------------------------------------");
}
fwrite(outbuf,1,maxaddr,out);
}
